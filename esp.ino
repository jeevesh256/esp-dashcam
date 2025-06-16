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
    // Write MJPEG header
    currentVideoFile.print("--video\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n");
    Serial.printf("Started new video file: %s\n", filename.c_str());
  } else {
    Serial.println("Failed to create new video file!");
  }
  
  fileCounter++;
  lastFileTime = millis();
}

void handleStream() {
  WiFiClient client = syncServer.client();
  
  if (!client.connected()) {
    return;
  }
  
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Access-Control-Allow-Origin: *\r\n";  // Add CORS header
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  client.print(response);

  while (client.connected()) {
    unsigned long currentTime = millis();
    
    // Check if we need to rotate the file (every 60 seconds)
    if (currentTime - lastFileTime >= 60000) {
      rotateVideoFile();
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      delay(100);
      continue;
    }

    client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
    client.write(fb->buf, fb->len);
    client.print("\r\n");

    // Save frame to current video file with proper MJPEG format
    if (currentVideoFile) {
      currentVideoFile.printf("\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
      currentVideoFile.write(fb->buf, fb->len);
    }

    esp_camera_fb_return(fb);
    delay(100);
  }

  // If client disconnects, complete current file
  completeVideoFile();
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
  <title>ESP32-CAM Stream</title>
  <style>
    body { font-family: sans-serif; background: #f4f4f4; text-align: center; padding: 10px; }
    #stream { width: 320px; border-radius: 10px; margin: 15px auto; box-shadow: 0 0 8px rgba(0,0,0,0.2); }
    .stream-error { border: 2px solid red; }
    table { width: 100%; max-width: 600px; margin: auto; background: white; border-collapse: collapse; border-radius: 6px; overflow: hidden; box-shadow: 0 0 10px rgba(0,0,0,0.1); }
    th, td { padding: 10px; border-bottom: 1px solid #eee; }
    td a { text-decoration: none; color: #007bff; }
    .btn { padding: 6px 12px; margin: 2px; border: none; border-radius: 4px; cursor: pointer; }
    .download { background-color: #28a745; color: white; }
    .delete { background-color: #dc3545; color: white; }
    .status-connected { color: green; }
    .status-connecting { color: orange; }
  </style>
</head>
<body>
  <h2>ESP32-CAM Stream</h2>
  <div id="stream-container">
    <img id="stream" alt="Loading stream..." onerror="handleStreamError(this)"/>
  </div>
  <div id="stream-status" class="status-connecting">Connecting to stream...</div>
  
  <div id="storage-info" style="margin: 15px auto; max-width: 600px;">
    <div style="background: #eee; border-radius: 4px; padding: 2px; margin: 10px 0;">
      <div id="storage-bar" style="background: #4CAF50; height: 20px; border-radius: 3px; width: 0%; transition: width 0.3s ease;">
        <span style="color: white; font-size: 12px; line-height: 20px;"></span>
      </div>
    </div>
    <div id="storage-text" style="font-size: 14px; color: #666;"></div>
  </div>
  
  <h3>Recorded Files</h3>
  <table id="fileTable"><thead><tr><th>File</th><th>Actions</th></tr></thead><tbody></tbody></table>

  <script>
    function initStream() {
      const img = document.getElementById('stream');
      const status = document.getElementById('stream-status');
      const ip = window.location.hostname;
      img.src = `http://${ip}:81/stream`;
      status.textContent = 'Connecting to stream...';
      status.className = 'status-connecting';
      
      img.onload = function() {
        status.textContent = 'Stream Connected';
        status.className = 'status-connected';
      }
    }

    function handleStreamError(img) {
      const status = document.getElementById('stream-status');
      img.classList.add('stream-error');
      status.textContent = 'Stream connection lost. Retrying...';
      status.className = 'status-connecting';
      setTimeout(() => {
        img.src = img.src.split('?')[0] + '?' + new Date().getTime();
        img.classList.remove('stream-error');
      }, 1000);
    }

    function fetchFiles() {
      fetch("/list.json").then(res => res.json()).then(files => {
        const table = document.querySelector("#fileTable tbody");
        table.innerHTML = "";
        files.forEach(file => {
          const row = document.createElement("tr");
          row.innerHTML = `
            <td>${file}</td>
            <td>
              <a href="/download?file=${file}" class="btn download" download>Download</a>
              <button class="btn delete" onclick="deleteFile('${file}')">Delete</button>
            </td>`;
          table.appendChild(row);
        });
      });
    }

    function deleteFile(filename) {
      if (!confirm(`Delete ${filename}?`)) return;
      fetch(`/delete?file=${filename}`).then(res => res.text()).then(msg => {
        alert(msg); fetchFiles();
      });
    }

    function updateStorageInfo() {
      fetch("/storage").then(res => res.json()).then(info => {
        const bar = document.getElementById('storage-bar');
        const text = document.getElementById('storage-text');
        bar.style.width = info.percent + '%';
        bar.style.background = info.percent > 90 ? '#f44336' : '#4CAF50';
        bar.children[0].textContent = info.percent + '%';
        text.textContent = `Storage: ${info.used}MB used of ${info.total}MB (${info.free}MB free)`;
      });
    }

    initStream();
    setInterval(fetchFiles, 10000);
    fetchFiles();
    setInterval(updateStorageInfo, 30000); // Update every 30 seconds
    updateStorageInfo();
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
        json += "\"" + String(file.name()) + "\"";
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