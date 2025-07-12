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
#include "esp_task_wdt.h"

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

#define MONITOR_INTERVAL 5000     // Check system every 5 seconds
#define MIN_HEAP_SIZE 40000       // Minimum heap before recovery
#define MIN_FREE_HEAP 30000       // Minimum free heap for operation
#define MAX_RETRY_COUNT 3         // Max retries before restart
#define RECOVERY_DELAY 1000       // Delay between recovery attempts
#define FREE_SPACE_MIN 5242880    // 5MB minimum free space
#define VIDEO_DURATION 60000     // 60 seconds
#define FRAME_RATE 5             // Reduced to 5fps for smaller files with better quality
#define FRAME_INTERVAL 200000    // Microseconds (1000000/5 for 5fps)
#define FRAMES_PER_VIDEO 300     // Total frames for 1 minute (5fps * 60s)
#define WRITE_BUFFER_SIZE 512    // Smaller write buffer
#define FRAME_BUFFER_SIZE 8192   // Smaller frame buffer
#define STACK_SIZE 4096          // Reduced stack size
#define WDT_TIMEOUT 60           // 1 minute watchdog

// Core assignments
#define RECORDING_CORE 0
#define STREAM_CORE 1

// Add timing control variables
unsigned long videoStartTime = 0;
unsigned long nextFrameTime = 0;
unsigned long frameCount = 0;

// Task handle for continuous recording
TaskHandle_t recordingTaskHandle = NULL;

// Add global variables at the top with other globals
unsigned long recordingStartTime = 0;
unsigned long lastFrameTime = 0;
unsigned long framesInCurrentFile = 0;
unsigned long frameCounter = 0;
bool isRecording = false;
bool isFirstFile = true;

// Add monitoring variables
unsigned long lastMonitorCheck = 0;
unsigned long lastFrameSuccess = 0;
unsigned long failedWrites = 0;
bool systemHealthy = false;

// Add recording status tracking
bool isFirstBoot = true;
bool recordingStarted = false;

static uint8_t* writeBuffer = NULL;
static size_t writeBufferSize = 0;

void initMemory() {
    if (writeBuffer) free(writeBuffer);
    writeBufferSize = WRITE_BUFFER_SIZE;
    writeBuffer = (uint8_t*)malloc(writeBufferSize);
    if (!writeBuffer) {
        Serial.println("Failed to allocate write buffer");
        ESP.restart();
    }
}

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
    config.xclk_freq_hz = 20000000; // Higher clock for better quality
    config.pixel_format = PIXFORMAT_JPEG;

    config.frame_size = FRAMESIZE_VGA;   // VGA (640x480) for good balance
    config.jpeg_quality = 10;            // Much better quality (10 instead of 18)
    config.fb_count = 2;                 // Dual buffer for stability
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
    
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed: %d\n", err);
        ESP.restart();
    }
    
    sensor_t* s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_VGA);   // 640x480 resolution
    s->set_quality(s, 10);                // Better quality for less artifacts
    s->set_brightness(s, 0);              // Neutral brightness
    s->set_saturation(s, 0);              // Normal saturation (no reduction)
    s->set_contrast(s, 0);                // Normal contrast (no reduction)
    s->set_sharpness(s, 0);               // Normal sharpness
    s->set_denoise(s, 1);                 // Enable denoise
    s->set_gainceiling(s, GAINCEILING_2X); // Lower gain ceiling for less noise
    s->set_colorbar(s, 0);                // Disable color bar
    s->set_whitebal(s, 1);                // Enable white balance
    s->set_gain_ctrl(s, 1);               // Enable gain control
    s->set_exposure_ctrl(s, 1);           // Enable exposure control
    s->set_hmirror(s, 0);                 // No horizontal mirror
    s->set_vflip(s, 0);                   // No vertical flip
    s->set_awb_gain(s, 1);                // Enable auto white balance gain
    s->set_agc_gain(s, 0);                // Auto gain control
    s->set_aec_value(s, 300);             // Auto exposure value
    s->set_aec2(s, 0);                    // Disable AEC2
    s->set_dcw(s, 1);                     // Enable downsize
    s->set_bpc(s, 0);                     // Disable bad pixel correction
    s->set_wpc(s, 1);                     // Enable white pixel correction
    s->set_raw_gma(s, 1);                 // Enable gamma correction
    s->set_lenc(s, 1);                    // Enable lens correction
    
    // Additional fixes for color artifacts
    s->set_special_effect(s, 0);          // No special effects
    s->set_wb_mode(s, 0);                 // Auto white balance mode
    
    Serial.println("Camera configured for 5fps with high quality (target: ~4MB)");
}

// Add AVI header structures
struct AviMainHeader {
  uint32_t microSecPerFrame;
  uint32_t maxBytesPerSec;
  uint32_t paddingGranularity;
  uint32_t flags;
  uint32_t totalFrames;
  uint32_t initialFrames;
  uint32_t streams;
  uint32_t suggestedBufferSize;
  uint32_t width;
  uint32_t height;
  uint32_t reserved[4];
} __attribute__((packed));

struct AviStreamHeader {
  char fccType[4];
  char fccHandler[4];
  uint32_t flags;
  uint16_t priority;
  uint16_t language;
  uint32_t initialFrames;
  uint32_t scale;
  uint32_t rate;
  uint32_t start;
  uint32_t length;
  uint32_t suggestedBufferSize;
  uint32_t quality;
  uint32_t sampleSize;
  struct {
    uint16_t left;
    uint16_t top;
    uint16_t right;
    uint16_t bottom;
  } frame;
} __attribute__((packed));

struct BitmapInfoHeader {
  uint32_t size;
  uint32_t width;
  uint32_t height;
  uint16_t planes;
  uint16_t bitCount;
  uint32_t compression;
  uint32_t sizeImage;
  uint32_t xPelsPerMeter;
  uint32_t yPelsPerMeter;
  uint32_t clrUsed;
  uint32_t clrImportant;
} __attribute__((packed));

struct ChunkHeader {
  char id[4];
  uint32_t size;
} __attribute__((packed));

// Global variables for AVI
uint32_t currentFileFrames = 0;
uint32_t totalDataSize = 0;
uint32_t moviListPos = 0;

String getCurrentFileName() {
  return String("/Dashcam_") + String(fileCounter) + ".avi";
}

void writeAviHeader() {
  if (!currentVideoFile) return;
  
  currentVideoFile.seek(0);
  
  // RIFF header
  ChunkHeader riffHeader = {{'R', 'I', 'F', 'F'}, 0}; // Size will be updated later
  currentVideoFile.write((uint8_t*)&riffHeader, sizeof(riffHeader));
  currentVideoFile.write((const uint8_t*)"AVI ", 4);
  
  // LIST hdrl - Fixed size calculation
  uint32_t hdrlSize = 4 + sizeof(AviMainHeader) + 8 + 4 + sizeof(AviStreamHeader) + 8 + sizeof(BitmapInfoHeader) + 8;
  ChunkHeader listHeader = {{'L', 'I', 'S', 'T'}, hdrlSize};
  currentVideoFile.write((uint8_t*)&listHeader, sizeof(listHeader));
  currentVideoFile.write((const uint8_t*)"hdrl", 4);
  
  // avih chunk
  ChunkHeader avihChunk = {{'a', 'v', 'i', 'h'}, sizeof(AviMainHeader)};
  currentVideoFile.write((uint8_t*)&avihChunk, sizeof(avihChunk));
  
  AviMainHeader mainHeader = {};
  mainHeader.microSecPerFrame = 1000000 / FRAME_RATE;
  mainHeader.maxBytesPerSec = 640 * 480 * FRAME_RATE / 3; // Adjusted for better quality
  mainHeader.paddingGranularity = 0;
  mainHeader.flags = 0x10; // AVIF_HASINDEX
  mainHeader.totalFrames = 0; // Will update later
  mainHeader.initialFrames = 0;
  mainHeader.streams = 1;
  mainHeader.suggestedBufferSize = 100000; // Increased for better quality
  mainHeader.width = 640;    // VGA width
  mainHeader.height = 480;   // VGA height
  memset(mainHeader.reserved, 0, sizeof(mainHeader.reserved)); // Clear reserved fields
  currentVideoFile.write((uint8_t*)&mainHeader, sizeof(mainHeader));
  
  // LIST strl - Fixed size calculation
  uint32_t strlSize = 4 + sizeof(AviStreamHeader) + 8 + sizeof(BitmapInfoHeader) + 8;
  ChunkHeader strlHeader = {{'L', 'I', 'S', 'T'}, strlSize};
  currentVideoFile.write((uint8_t*)&strlHeader, sizeof(strlHeader));
  currentVideoFile.write((const uint8_t*)"strl", 4);
  
  // strh chunk
  ChunkHeader strhChunk = {{'s', 't', 'r', 'h'}, sizeof(AviStreamHeader)};
  currentVideoFile.write((uint8_t*)&strhChunk, sizeof(strhChunk));
  
  AviStreamHeader streamHeader = {};
  memcpy(streamHeader.fccType, "vids", 4);
  memcpy(streamHeader.fccHandler, "MJPG", 4);
  streamHeader.flags = 0;
  streamHeader.priority = 0;
  streamHeader.language = 0;
  streamHeader.initialFrames = 0;
  streamHeader.scale = 1;
  streamHeader.rate = FRAME_RATE;
  streamHeader.start = 0;
  streamHeader.length = 0; // Will update later
  streamHeader.suggestedBufferSize = 100000; // Increased for better quality
  streamHeader.quality = 10000; // Better quality setting (not -1)
  streamHeader.sampleSize = 0;
  streamHeader.frame.left = 0;
  streamHeader.frame.top = 0;
  streamHeader.frame.right = 640;  // VGA width
  streamHeader.frame.bottom = 480; // VGA height
  currentVideoFile.write((uint8_t*)&streamHeader, sizeof(streamHeader));
  
  // strf chunk (stream format)
  ChunkHeader strfChunk = {{'s', 't', 'r', 'f'}, sizeof(BitmapInfoHeader)};
  currentVideoFile.write((uint8_t*)&strfChunk, sizeof(strfChunk));
  
  BitmapInfoHeader bitmapHeader = {};
  bitmapHeader.size = sizeof(BitmapInfoHeader);
  bitmapHeader.width = 640;   // VGA width
  bitmapHeader.height = 480;  // VGA height
  bitmapHeader.planes = 1;
  bitmapHeader.bitCount = 24;
  bitmapHeader.compression = 0x47504A4D; // 'MJPG' in little endian
  bitmapHeader.sizeImage = 640 * 480 * 3; // VGA size
  bitmapHeader.xPelsPerMeter = 0;
  bitmapHeader.yPelsPerMeter = 0;
  bitmapHeader.clrUsed = 0;
  bitmapHeader.clrImportant = 0;
  currentVideoFile.write((uint8_t*)&bitmapHeader, sizeof(bitmapHeader));
  
  // LIST movi
  moviListPos = currentVideoFile.position();
  ChunkHeader moviHeader = {{'L', 'I', 'S', 'T'}, 4}; // Size will be updated later
  currentVideoFile.write((uint8_t*)&moviHeader, sizeof(moviHeader));
  currentVideoFile.write((const uint8_t*)"movi", 4);
  
  currentFileFrames = 0;
  totalDataSize = 0;
  
  Serial.println("AVI header written successfully with improved format");
}

void updateAviHeader() {
  if (!currentVideoFile || currentFileFrames == 0) return;
  
  uint32_t currentPos = currentVideoFile.position();
  
  // Update RIFF size
  uint32_t riffSize = currentPos - 8;
  currentVideoFile.seek(4);
  currentVideoFile.write((uint8_t*)&riffSize, 4);
  
  // Update total frames in main header
  currentVideoFile.seek(32); // Position of totalFrames in AviMainHeader
  currentVideoFile.write((uint8_t*)&currentFileFrames, 4);
  
  // Update stream length
  currentVideoFile.seek(84); // Position of length in AviStreamHeader
  currentVideoFile.write((uint8_t*)&currentFileFrames, 4);
  
  // Update movi list size
  uint32_t moviSize = currentPos - moviListPos - 8;
  currentVideoFile.seek(moviListPos + 4);
  currentVideoFile.write((uint8_t*)&moviSize, 4);
  
  // Go back to end of file
  currentVideoFile.seek(currentPos);
  
  Serial.printf("Updated AVI header: %lu frames, %lu bytes\n", currentFileFrames, riffSize);
}

void rotateVideoFile() {
  if (currentVideoFile) {
    updateAviHeader();
    currentVideoFile.close();
    Serial.printf("Completed AVI file %d, frames: %lu, size: %lu bytes\n", 
                 fileCounter, currentFileFrames, totalDataSize);
  }

  if (SD_MMC.totalBytes() - SD_MMC.usedBytes() < FREE_SPACE_MIN) {
    deleteOldestFile();
  }

  String filename = getCurrentFileName();
  currentVideoFile = SD_MMC.open(filename.c_str(), FILE_WRITE);
  
  if (currentVideoFile) {
    Serial.printf("Started new AVI file: %s\n", filename.c_str());
    writeAviHeader();
    videoStartTime = millis();
    frameCounter = 0;
    fileCounter++;
  } else {
    Serial.println("Failed to create new AVI file!");
  }
}

void deleteOldestFile() {
  File root = SD_MMC.open("/");
  if (!root || !root.isDirectory()) return;

  String oldestFile;
  unsigned long oldestNum = ULONG_MAX;

  File file = root.openNextFile();
  while (file) {
    String fname = String(file.name());
    if (fname.startsWith("/Dashcam_") && fname.endsWith(".avi")) {
      unsigned long num = fname.substring(9, fname.length() - 4).toInt();
      if (num < oldestNum) {
        oldestNum = num;
        oldestFile = fname;
      }
    }
    file = root.openNextFile();
  }

  if (oldestFile.length() > 0) {
    SD_MMC.remove(oldestFile);
    Serial.printf("Deleted oldest file: %s\n", oldestFile.c_str());
  }
}

// Update minSize function to handle size_t properly
inline size_t minSize(size_t a, size_t b) {
    return (a < b) ? a : b;
}

void monitorSystem() {
    unsigned long now = millis();
    if (now - lastMonitorCheck >= MONITOR_INTERVAL) {
        lastMonitorCheck = now;
        
        // Check heap
        if (ESP.getFreeHeap() < MIN_HEAP_SIZE) {
            Serial.println("Low memory, recovering...");
            completeVideoFile();
            ESP.restart();
        }
        
        // Check recording status
        if (now - lastFrameSuccess > 10000) {  // No frames for 10 seconds
            Serial.println("Recording stalled, recovering...");
            restartRecording();
        }
        
        // Check file system
        if (!currentVideoFile || failedWrites > 10) {
            Serial.println("File system issues, recovering...");
            restartRecording();
        }
        
        systemHealthy = true;
    }
}

void restartRecording() {
    completeVideoFile();
    delay(100);
    rotateVideoFile();
    failedWrites = 0;
    videoStartTime = millis();
    frameCount = 0;
}

void startRecording() {
    videoStartTime = millis();
    frameCount = 0;
    rotateVideoFile();
    recordingStarted = true;
    isFirstBoot = false;
    Serial.println("Recording started automatically");
}

void completeVideoFile() {
  if (currentVideoFile) {
    updateAviHeader();
    currentVideoFile.close();
    Serial.printf("Completed video file: Dashcam_%d.avi\n", fileCounter-1);
  }
}

void recordingTask(void* parameter) {
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);
    
    // Ensure recording starts immediately
    Serial.println("Recording task started - initializing...");
    delay(2000); // Give system time to stabilize
    
    size_t frameErrors = 0;
    unsigned long videoStartUs = micros();
    unsigned long nextFrameUs = videoStartUs + FRAME_INTERVAL;
    
    while(true) {
        unsigned long currentUs = micros();
        
        // Wait until it's time for the next frame
        if (currentUs < nextFrameUs) {
            unsigned long waitUs = nextFrameUs - currentUs;
            if (waitUs < 50000) { // Only delay if less than 50ms
                delayMicroseconds(waitUs);
            } else {
                vTaskDelay(pdMS_TO_TICKS(waitUs / 1000)); // Use vTaskDelay for longer waits
            }
            esp_task_wdt_reset();
            continue;
        }
        
        if (!currentVideoFile || !recordingStarted) {
            Serial.println("Starting new recording cycle...");
            startRecording();
            videoStartUs = micros();
            nextFrameUs = videoStartUs + FRAME_INTERVAL;
            esp_task_wdt_reset();
            continue;
        }

        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            if (++frameErrors > 5) {
                Serial.println("Too many camera errors, restarting...");
                ESP.restart();
            }
            // Still advance to next frame time even on error
            nextFrameUs += FRAME_INTERVAL;
            esp_task_wdt_reset();
            continue;
        }
        frameErrors = 0;

        if (currentVideoFile && writeBuffer) {
            bool writeOK = true;
            
            // Write AVI chunk header for frame
            ChunkHeader frameChunk = {{'0', '0', 'd', 'c'}, fb->len};
            
            if (currentVideoFile.write((uint8_t*)&frameChunk, sizeof(frameChunk)) != sizeof(frameChunk)) {
                writeOK = false;
                Serial.println("Failed to write frame header");
            }
            
            // Write frame data in chunks
            if (writeOK) {
                size_t pos = 0;
                while (pos < fb->len && writeOK) {
                    size_t toWrite = minSize(writeBufferSize, fb->len - pos);
                    if (currentVideoFile.write(fb->buf + pos, toWrite) != toWrite) {
                        writeOK = false;
                        Serial.println("Frame write failed");
                        break;
                    }
                    pos += toWrite;
                    if (pos % (writeBufferSize * 4) == 0) {
                        esp_task_wdt_reset();
                    }
                }
            }
            
            // Add padding if frame size is odd (AVI requirement)
            if (writeOK && (fb->len % 2) == 1) {
                uint8_t padding = 0;
                currentVideoFile.write(&padding, 1);
            }
            
            if (writeOK) {
                currentVideoFile.flush(); // Ensure data is written
                frameCount++;
                currentFileFrames++;
                totalDataSize += sizeof(ChunkHeader) + fb->len + (fb->len % 2);
                lastFrameSuccess = millis();
                
                // Debug output every 100 frames
                if (frameCount % 100 == 0) {
                    Serial.printf("Recorded %lu frames, file frames: %lu, data size: %lu, frame size: %u\n", 
                                frameCount, currentFileFrames, totalDataSize, fb->len);
                }
                
                // Rotate file after exactly FRAMES_PER_VIDEO frames
                if (frameCount >= FRAMES_PER_VIDEO) {
                    Serial.printf("Completing 1-minute AVI with %lu frames\n", frameCount);
                    rotateVideoFile();  // This will start a new file automatically
                    videoStartUs = micros();
                    frameCount = 0;
                    nextFrameUs = videoStartUs + FRAME_INTERVAL;
                } else {
                    // Calculate next frame time based on start time and frame count
                    nextFrameUs = videoStartUs + (frameCount + 1) * FRAME_INTERVAL;
                }
            } else {
                failedWrites++;
                // Even on write failure, advance to next frame time
                nextFrameUs += FRAME_INTERVAL;
            }
        } else {
            Serial.println("No file handle or write buffer");
            nextFrameUs += FRAME_INTERVAL;
        }

        esp_camera_fb_return(fb);
        esp_task_wdt_reset();
    }
}

void handleDownload(AsyncWebServerRequest *request) {
  if (!request->hasParam("file")) {
    request->send(400, "text/plain", "Missing file parameter");
    return;
  }

  String filename = request->getParam("file")->value();
  if (!filename.startsWith("/")) filename = "/" + filename;

  fs::File file = SD_MMC.open(filename, "r");
  if (!file || file.isDirectory()) {
    request->send(404);
    return;
  }

  AsyncWebServerResponse *response = request->beginResponse("application/octet-stream", 
    file.size(), [file](uint8_t *buffer, size_t maxLen, size_t index) mutable -> size_t {
      return file.read(buffer, maxLen);
  });
  
  response->addHeader("Content-Disposition", "attachment; filename=" + 
                     filename.substring(filename.lastIndexOf("/") + 1));
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

void onAllocError(size_t requested, size_t available, const char* hint) {
    Serial.printf("Memory allocation failed - requested: %u, available: %u\n", requested, available);
    completeVideoFile();
    ESP.restart();
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

  Serial.printf("SD Card Type: %s\n", 
    SD_MMC.cardType() == CARD_MMC ? "MMC" :
    SD_MMC.cardType() == CARD_SD ? "SDSC" :
    SD_MMC.cardType() == CARD_SDHC ? "SDHC" : "UNKNOWN");
  Serial.printf("SD Card Size: %lluMB\n", SD_MMC.cardSize() / (1024 * 1024));

  startCamera();
  initMemory();
  
  // Start recording task with higher priority
  xTaskCreatePinnedToCore(
    recordingTask,
    "Record",
    STACK_SIZE,
    NULL,
    2,  // Higher priority for recording
    &recordingTaskHandle,
    RECORDING_CORE
  );
  
  Serial.println("ESP32-CAM ready - continuous recording will start automatically");
  Serial.printf("Recording %d frames per minute at %dfps (VGA 640x480, high quality, target: ~4MB)\n", FRAMES_PER_VIDEO, FRAME_RATE);
  
  // Serve UI
  asyncServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP32-CAM File Manager</title>
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
      <h1>ESP32-CAM File Manager</h1>
    </header>

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

  <script>
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
        'mjpeg': '📹', // Video files
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

    // Initialize
    fetchFiles();
    updateStorageInfo();
    setInterval(fetchFiles, 10000);
    setInterval(updateStorageInfo, 30000);
  </script>
</body>
</html>
)rawliteral");
  });

  // Keep other routes
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
  Serial.println("Web server started");
}

void loop() {
  // Monitor system health
  monitorSystem();
  
  // Check if recording task is still running
  if (recordingTaskHandle == NULL || eTaskGetState(recordingTaskHandle) == eDeleted) {
    Serial.println("Recording task stopped, restarting...");
    xTaskCreatePinnedToCore(
      recordingTask,
      "Record",
      STACK_SIZE,
      NULL,
      2,
      &recordingTaskHandle,
      RECORDING_CORE
    );
  }
  
  esp_task_wdt_reset();
  vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
}