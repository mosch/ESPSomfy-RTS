#include <WiFi.h>
#include <LittleFS.h>
#include <esp_task_wdt.h>
#include <mbedtls/platform.h>
#include "ConfigSettings.h"
#include "ESPNetwork.h"
#include "Web.h"
#include "Sockets.h"
#include "Utils.h"
#include "Somfy.h"
#include "MQTT.h"
#include "GitOTA.h"
#include "SomfyMatter.h"

ConfigSettings settings;
Web webServer;
SocketEmitter sockEmit;
ESPNetwork net;
rebootDelay_t rebootDelay;
// The shade controller is the largest object (~27KB).  It lives in PSRAM so the internal RAM is available for
// WiFi, TLS and Matter.  The receive interrupt does not touch it.
static SomfyShadeController *newShadeController() {
  void *mem = heap_caps_malloc(sizeof(SomfyShadeController), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if(!mem) mem = malloc(sizeof(SomfyShadeController));
  return new(mem) SomfyShadeController();
}
SomfyShadeController &somfy = *newShadeController();
MQTTClass mqtt;
GitUpdater git;
SomfyMatter matter;

uint32_t oldheap = 0;
static void *mbedtlsCalloc(size_t n, size_t size) {
  void *p = heap_caps_calloc(n, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  return p ? p : heap_caps_calloc(n, size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Startup/Boot....");
  // mbedTLS allocates from internal RAM only.  Matter (startup, pairing sessions) and TLS connections easily
  // exhaust it so prefer PSRAM like CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC would.
  if(psramFound()) mbedtls_platform_set_calloc_free(mbedtlsCalloc, free);
  Serial.println("Mounting File System...");
  if(LittleFS.begin()) Serial.println("File system mounted successfully");
  else Serial.println("Error mounting file system");
  settings.begin();
  if(WiFi.status() == WL_CONNECTED) WiFi.disconnect(true);
  delay(10);
  Serial.println();
  webServer.startup();
  webServer.begin();
  delay(1000);
  net.setup();  
  somfy.begin();
  //git.checkForUpdate();
  esp_task_wdt_config_t wdtConfig = { .timeout_ms = 7000, .idle_core_mask = 0, .trigger_panic = true };
  esp_task_wdt_reconfigure(&wdtConfig); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

}

void loop() {
  // put your main code here, to run repeatedly:
  //uint32_t heap = ESP.getFreeHeap();
  if(rebootDelay.reboot && millis() > rebootDelay.rebootTime) {
    Serial.print("Rebooting after ");
    Serial.print(rebootDelay.rebootTime);
    Serial.println("ms");
    net.end();
    ESP.restart();
    return;
  }
  uint32_t timing = millis();
  
  net.loop();
  if(millis() - timing > 100) Serial.printf("Timing Net: %ldms\n", millis() - timing);
  timing = millis();
  esp_task_wdt_reset();
  somfy.loop();
  if(millis() - timing > 100) Serial.printf("Timing Somfy: %ldms\n", millis() - timing);
  timing = millis();
  esp_task_wdt_reset();
  if(!rebootDelay.reboot) matter.loop();
  if(millis() - timing > 100) Serial.printf("Timing Matter: %ldms\n", millis() - timing);
  timing = millis();
  esp_task_wdt_reset();
  if(net.connected() || net.softAPOpened) {
    if(!rebootDelay.reboot && net.connected() && !net.softAPOpened) {
      git.loop();
      esp_task_wdt_reset();
    }
    webServer.loop();
    esp_task_wdt_reset();
    if(millis() - timing > 100) Serial.printf("Timing WebServer: %ldms\n", millis() - timing);
    esp_task_wdt_reset();
    timing = millis();
    sockEmit.loop();
    if(millis() - timing > 100) Serial.printf("Timing Socket: %ldms\n", millis() - timing);
    esp_task_wdt_reset();
    timing = millis();
  }
  if(rebootDelay.reboot && millis() > rebootDelay.rebootTime) {
    net.end();
    ESP.restart();
  }
  esp_task_wdt_reset();
}
