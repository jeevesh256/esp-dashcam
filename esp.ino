#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <SD_MMC.h>
#include <esp_camera.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Camera pin configuration for AI Thinker
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

const char* ssid_ap = "ESP32";
const char* password_ap = "12345678";

AsyncWebServer asyncServer(80);   // For file operations
WebServer syncServer(81);         // For camera streaming

File currentVideoFile;
unsigned long lastFileTime = 0;
int fileCounter = 0;

struct StorageInfo {
  uint64_t totalBytes;
  uint64_t usedBytes;
  uint64_t freeBytes;
  int usagePercent;
};

void startCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;  // Reduced from SVGA
    config.jpeg_quality = 15;  // Reduced quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;  // Even smaller for non-PSRAM
    config.jpeg_quality = 20;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}

String getCurrentFileName() {
  return String("/Dashcam_") + String(fileCounter) + ".mjpeg";
}

// Add file completion message
void completeVideoFile() {
  if (currentVideoFile) {
    currentVideoFile.print("\r\n--video--\r\n");  // End MJPEG marker
    currentVideoFile.close();
    Serial.printf("Completed video file: video_%d.mjpeg\n", fileCounter-1);
  }
}

void rotateVideoFile() {
  completeVideoFile();  // Complete current file before rotating
  
  String filename = String("/Dashcam_") + String(fileCounter) + ".mjpeg";
  currentVideoFile = SD_MMC.open(filename.c_str(), FILE_WRITE);
  
  if (currentVideoFile) {
    // Write MJPEG header with proper content type and boundary
    currentVideoFile.print("--video\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n");
    Serial.printf("Started new video file: %s\n", filename.c_str());
    lastFileTime = millis();  // Update timestamp after successful file creation
  } else {
    Serial.println("Failed to create new video file!");
  }
  
  fileCounter++;
}

void handleStream() {
  WiFiClient client = syncServer.client();
  
  if (!client.connected()) {
    return;
  }
  
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Access-Control-Allow-Origin: *\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  client.print(response);

  unsigned long frameStart;
  const int frameDelay = 100; // 10 FPS

  while (client.connected()) {
    frameStart = millis();
    
    // Check if we need to rotate the file (every 60 seconds)
    if (currentVideoFile && (millis() - lastFileTime >= 60000)) {
      rotateVideoFile();
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      delay(10);
      continue;
    }

    // Send frame to client
    client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
    client.write(fb->buf, fb->len);
    client.print("\r\n");

    // Save frame to current video file
    if (currentVideoFile) {
      currentVideoFile.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
      currentVideoFile.write(fb->buf, fb->len);
      currentVideoFile.print("\r\n");
      currentVideoFile.flush();  // Ensure frame is written to disk
    }

    esp_camera_fb_return(fb);

    // Maintain consistent frame rate
    int processingTime = millis() - frameStart;
    if (processingTime < frameDelay) {
      delay(frameDelay - processingTime);
    }
  }

  completeVideoFile();  // Properly close current file when stream ends
}

void handleDownload(AsyncWebServerRequest *request) {
  if (!request->hasParam("file")) {
    request->send(400, "text/plain", "Missing file parameter");
    return;
  }

  String filename = request->getParam("file")->value();
  if (!filename.startsWith("/")) filename = "/" + filename;

  fs::File file = SD_MMC.open(filename);
  if (!file || file.isDirectory()) {
    request->send(404, "text/plain", "File not found or is a directory");
    return;
  }

  size_t fileSize = file.size();
  AsyncWebServerResponse *response = request->beginResponse("application/octet-stream", fileSize,
    [file](uint8_t *buffer, size_t maxLen, size_t index) mutable -> size_t {
      size_t bytesRead = file.read(buffer, maxLen);
      vTaskDelay(1);
      return bytesRead;
    });

  response->addHeader("Content-Disposition", "attachment; filename=\"" + filename.substring(filename.lastIndexOf("/") + 1) + "\"");
  response->addHeader("Content-Length", String(fileSize));
  request->send(response);
}

StorageInfo getStorageInfo() {
  StorageInfo info;
  info.totalBytes = SD_MMC.totalBytes();
  info.usedBytes = SD_MMC.usedBytes();
  info.freeBytes = SD_MMC.totalBytes() - SD_MMC.usedBytes();
  info.usagePercent = (int)((info.usedBytes * 100) / info.totalBytes);
  return info;
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector
  Serial.begin(115200);
  delay(1000);
  Serial.println("Booting...");

  WiFi.softAP(ssid_ap, password_ap);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("SD Card Mount Failed");
    return;
  }

  if (SD_MMC.cardType() == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  startCamera();

  // Start first video file
  rotateVideoFile();

  // Serve UI
  asyncServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP32-CAM Dashboard</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    :root {
      --primary-color: #2196F3;
      --danger-color: #f44336;
      --success-color: #4CAF50;
      --warning-color: #ff9800;
      --border-radius: 8px;
      --shadow: 0 2px 4px rgba(0,0,0,0.1);
    }
    body { 
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      background: #f5f5f5;
      color: #333;
      line-height: 1.6;
    }
    .container { max-width: 1200px; margin: 0 auto; padding: 20px; }
    header {
      background: white;
      padding: 1.5rem;
      box-shadow: var(--shadow);
      border-radius: var(--border-radius);
      margin-bottom: 2rem;
    }
    h1 { 
      color: var(--primary-color); 
      text-align: center;
      font-size: 2rem;
      font-weight: 500;
    }

    /* Navigation */
    .nav-tabs {
      display: flex;
      background: white;
      padding: 0.5rem;
      border-radius: var(--border-radius);
      box-shadow: var(--shadow);
      margin-bottom: 2rem;
    }
    .nav-tab {
      flex: 1;
      padding: 1rem;
      text-align: center;
      border: none;
      background: transparent;
      color: #666;
      font-size: 1rem;
      cursor: pointer;
      transition: all 0.3s;
      border-radius: var(--border-radius);
    }
    .nav-tab.active {
      background: var(--primary-color);
      color: white;
    }
    .nav-tab:hover:not(.active) {
      background: #f0f0f0;
    }

    /* Content Panels */
    .panel {
      display: none;
      background: white;
      border-radius: var(--border-radius);
      box-shadow: var(--shadow);
    }
    .panel.active { display: block; }

    /* Stream Panel */
    #stream-panel.panel.active {
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 2rem;
    }

    /* Files Panel */
    #files-panel {
      display: none;
      height: auto;
    }
    #files-panel.active {
      display: flex;
      flex-direction: column;
    }

    .storage-info {
      padding: 1rem;
      background: white;
      border-bottom: 1px solid #eee;
    }

    .storage-bar {
      width: 100%;
      height: 24px;
      background: #e9ecef;
      border-radius: 4px;
      overflow: hidden;
      margin: 0.5rem 0;
    }

    .storage-progress {
      height: 100%;
      background: var(--success-color);
      color: white;
      text-align: center;
      line-height: 24px;
      transition: width 0.3s ease;
    }

    .file-manager {
      flex: 1;
      display: flex;
      flex-direction: column;
    }

    .file-header {
      background: #f8f9fa;
      padding: 1rem;
      border-bottom: 1px solid #dee2e6;
      display: grid;
      grid-template-columns: minmax(200px, 1fr) 120px 100px;
      font-weight: 500;
      color: #495057;
    }

    .files-list {
      flex: 1;
      overflow-y: auto;
      scrollbar-width: thin;
      scrollbar-color: #90A4AE #CFD8DC;
    }

    .files-list::-webkit-scrollbar {
      width: 8px;
    }

    .files-list::-webkit-scrollbar-track {
      background: #f1f1f1;
      border-radius: 4px;
    }

    .files-list::-webkit-scrollbar-thumb {
      background: #90A4AE;
      border-radius: 4px;
    }

    .files-list::-webkit-scrollbar-thumb:hover {
      background: #78909C;
    }

    .file-item {
      display: grid;
      grid-template-columns: minmax(200px, 1fr) 120px 100px;
      padding: 0.75rem 1rem;
      align-items: center;
      border-bottom: 1px solid #eee;
      transition: background-color 0.2s;
    }

    .file-item:hover {
      background-color: #f8f9fa;
    }

    .file-name {
      display: flex;
      align-items: center;
      gap: 0.5rem;
    }

    .file-icon {
      color: var(--primary-color);
      font-size: 1.2rem;
    }

    .file-actions {
      display: flex;
      gap: 0.5rem;
    }

    .btn {
      padding: 0.4rem 0.8rem;
      border: none;
      border-radius: 4px;
      cursor: pointer;
      font-size: 0.85rem;
      transition: opacity 0.2s;
      text-decoration: none;
      text-align: center;
    }

    .btn-icon {
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 0.25rem;
      padding: 0.6rem 1rem;
      font-size: 1.1rem;
    }

    .storage-info {
      margin-bottom: 1rem;
      padding: 1rem;
      background: white;
      border-radius: var(--border-radius);
      box-shadow: var(--shadow);
    }
  </style>
</head>
<body>
  <div class="container">
    <header>
      <h1>ESP32-CAM Dashboard</h1>
    </header>

    <div class="nav-tabs">
      <button class="nav-tab active" onclick="showPanel('stream-panel')">Live Stream</button>
      <button class="nav-tab" onclick="showPanel('files-panel')">File Manager</button>
    </div>

    <div id="stream-panel" class="panel active">
      <div class="stream-container">
        <div class="stream-wrapper">
          <img id="stream" class="stream-video" alt="Loading stream...">
        </div>
        <div id="stream-status" class="stream-status">
          <span class="status-icon">⌛</span>
          <span class="status-text">Initializing stream...</span>
        </div>
      </div>
    </div>

    <div id="files-panel" class="panel">
      <div class="storage-info">
        <div class="storage-bar">
          <div id="storage-progress" class="storage-progress"></div>
        </div>
        <div id="storage-text" class="storage-text"></div>
      </div>
      <div class="file-manager">
        <div class="file-header">
          <div>Name</div>
          <div>Size</div>
          <div>Actions</div>
        </div>
        <div id="files-list" class="files-list"></div>
      </div>
    </div>
  </div>

  <script>
    function showPanel(panelId) {
      document.querySelectorAll('.panel').forEach(p => p.classList.remove('active'));
      document.querySelectorAll('.nav-tab').forEach(t => t.classList.remove('active'));
      document.getElementById(panelId).classList.add('active');
      event.currentTarget.classList.add('active');

      if (panelId === 'stream-panel') {
        initStream();
      } else {
        // Stop and remove stream when switching to file manager
        const streamImg = document.getElementById('stream');
        const status = document.getElementById('stream-status');
        streamImg.removeAttribute('src');
        streamImg.style.display = 'none';
        status.style.display = 'none';
      }
    }

    function initStream() {
      const img = document.getElementById('stream');
      const status = document.getElementById('stream-status');
      const ip = window.location.hostname;
      
      // Show stream elements
      img.style.display = 'block';
      status.style.display = 'inline-flex';
      
      status.className = 'stream-status';
      status.style.background = 'var(--warning-color)';
      status.style.color = 'white';
      status.innerHTML = '<span class="status-icon">⌛</span><span class="status-text">Connecting to stream...</span>';
      
      img.src = `http://${ip}:81/stream`;
      
      img.onload = () => {
        status.style.background = 'var(--success-color)';
        status.innerHTML = '<span class="status-icon">✅</span><span class="status-text">Stream Connected</span>';
      };
      
      img.onerror = () => {
        status.style.background = 'var(--danger-color)';
        status.innerHTML = '<span class="status-icon">⚠️</span><span class="status-text">Stream Disconnected</span>';
      };
    }

    function formatFileSize(size) {
      if (size < 1024) return size + ' B';
      if (size < 1024 * 1024) return (size / 1024).toFixed(1) + ' KB';
      return (size / (1024 * 1024)).toFixed(1) + ' MB';
    }

    function getFileIcon(filename) {
      const ext = filename.split('.').pop().toLowerCase();
      const icons = {
        'mjpeg': '📹', // Video files
        'jpg': '🖼️',   // Image files
        'jpeg': '🖼️',  // Image files
        'txt': '📄',   // Text files
        'log': '📝',   // Log files
        'json': '📋',  // Data files
        'csv': '📊',   // Spreadsheet files
        'avi': '🎥',   // Video files
        'default': '📁' // Default icon
      };
      return icons[ext] || icons.default;
    }

    function fetchFiles() {
      fetch("/list.json").then(res => res.json()).then(files => {
        const list = document.getElementById("files-list");
        list.innerHTML = "";
        files.forEach(file => {
          const item = document.createElement("div");
          item.className = "file-item";
          item.innerHTML = `
            <div class="file-name">
              <span class="file-icon">${getFileIcon(file.name)}</span>
              ${file.name}
            </div>
            <div class="file-size">${formatFileSize(file.size)}</div>
            <div class="file-actions">
              <a href="/download?file=${file.name}" class="btn btn-primary btn-icon" download>
                <span>⬇️</span>
              </a>
              <button class="btn btn-danger btn-icon" onclick="deleteFile('${file.name}')">
                <span>❌</span>
              </button>
            </div>`;
          list.appendChild(item);
        });
      });
    }

    function deleteFile(filename) {
      if (!confirm(`Delete ${filename}?`)) return;
      fetch(`/delete?file=${filename}`).then(res => res.text()).then(msg => {
        alert(msg);
        fetchFiles();
        updateStorageInfo();
      });
    }

    function updateStorageInfo() {
      fetch("/storage").then(res => res.json()).then(info => {
        const progress = document.getElementById('storage-progress');
        const text = document.getElementById('storage-text');
        progress.style.width = info.percent + '%';
        progress.style.background = info.percent > 90 ? 'var(--danger-color)' : 'var(--success-color)';
        progress.textContent = info.percent + '%';
        text.textContent = `Storage: ${info.used}MB used of ${info.total}MB (${info.free}MB free)`;
      });
    }

    // Remove stream initialization from page load
    fetchFiles();
    updateStorageInfo();
    setInterval(fetchFiles, 10000);
    setInterval(updateStorageInfo, 30000);

    // Initialize stream if we start on stream panel
    if (document.getElementById('stream-panel').classList.contains('active')) {
      initStream();
    }
  </script>
</body>
</html>
)rawliteral");
  });

  asyncServer.on("/list.json", HTTP_GET, [](AsyncWebServerRequest *request) {
    File root = SD_MMC.open("/");
    File file = root.openNextFile();
    String json = "[";
    bool first = true;
    while (file) {
      if (!file.isDirectory()) {
        if (!first) json += ",";
        // Add file size to the JSON response
        json += "{\"name\":\"" + String(file.name()) + 
                "\",\"size\":" + String(file.size()) + "}";
        first = false;
      }
      file = root.openNextFile();
    }
    json += "]";
    request->send(200, "application/json", json);
  });

  asyncServer.on("/download", HTTP_GET, handleDownload);

  asyncServer.on("/delete", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasParam("file")) {
      request->send(400, "text/plain", "Missing file parameter");
      return;
    }
    String filename = request->getParam("file")->value();
    if (!filename.startsWith("/")) filename = "/" + filename;
    if (SD_MMC.remove(filename)) {
      request->send(200, "text/plain", "Deleted: " + filename);
    } else {
      request->send(500, "text/plain", "Delete failed: " + filename);
    }
  });

  asyncServer.on("/storage", HTTP_GET, [](AsyncWebServerRequest *request) {
    StorageInfo info = getStorageInfo();
    String json = "{\"total\":\"" + String((float)info.totalBytes / 1048576.0, 1) + 
                  "\",\"used\":\"" + String((float)info.usedBytes / 1048576.0, 1) + 
                  "\",\"free\":\"" + String((float)info.freeBytes / 1048576.0, 1) + 
                  "\",\"percent\":" + String(info.usagePercent) + "}";
    request->send(200, "application/json", json);
  });

  asyncServer.begin();

  syncServer.on("/stream", handleStream);
  syncServer.begin();
}

void loop() {
  syncServer.handleClient();
}