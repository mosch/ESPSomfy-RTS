#include <Arduino.h>
#include <sdkconfig.h>
#include "ConfigSettings.h"
#include "ESPNetwork.h"
#include "Somfy.h"
#include "SomfyMatter.h"

extern ConfigSettings settings;
extern SomfyShadeController &somfy;
extern ESPNetwork net;

#ifdef CONFIG_ESP_MATTER_ENABLE_DATA_MODEL
#include <Matter.h>

using namespace esp_matter;

// The root node and the aggregator use two of the available endpoints.
#define MATTER_MAX_SHADES (CONFIG_ESP_MATTER_MAX_DYNAMIC_ENDPOINT_COUNT - 2)
#define MATTER_SYNC_INTERVAL 250
#define MATTER_MOVING_POS_INTERVAL 1000
// Controllers send a stream of targets while a slider is dragged.  Only act once the target has settled
// so the motor does not get a burst of up/down commands.
#define MATTER_TARGET_SETTLE 1000

enum class matter_actions_t : uint8_t {
  lift = 0,
  tilt = 1,
  stop = 2
};
struct matter_cmd_t {
  uint8_t shadeId;
  matter_actions_t action;
  uint8_t value;
};
struct matter_shade_t {
  uint8_t shadeId = 0;
  MatterWindowCovering *ep = nullptr;
  bool hasLift = false;
  bool hasTilt = false;
  volatile bool skipLift = false;
  uint16_t pos = 0xFFFF;
  uint16_t target = 0xFFFF;
  uint16_t tiltPos = 0xFFFF;
  uint16_t tiltTarget = 0xFFFF;
  int8_t dir = -2;
  int8_t tiltDir = -2;
  uint32_t lastPos = 0;
  uint32_t lastTiltPos = 0;
  int16_t pendingLift = -1;
  int16_t pendingTilt = -1;
  uint32_t pendingTime = 0;
};
static matter_shade_t matterShades[MATTER_MAX_SHADES];
static uint8_t matterShadeCount = 0;
static QueueHandle_t matterQueue = nullptr;
static TaskHandle_t matterLoopTask = nullptr;

// ArduinoMatter only lets its own endpoint classes create the node.  We need it before
// the first window covering so the aggregator becomes endpoint 1.
struct MatterNode : public ArduinoMatter {
  static void init() { ArduinoMatter::_init(); }
};

static bool coveringType(SomfyShade *shade, MatterWindowCovering::WindowCoveringType_t *type) {
  switch(shade->shadeType) {
    case shade_types::roller:
    case shade_types::blind:
      *type = MatterWindowCovering::ROLLERSHADE;
      break;
    case shade_types::ldrapery:
    case shade_types::rdrapery:
    case shade_types::cdrapery:
      *type = MatterWindowCovering::DRAPERY;
      break;
    case shade_types::awning:
      *type = MatterWindowCovering::AWNING;
      break;
    case shade_types::shutter:
      *type = MatterWindowCovering::ROLLERSHADE_EXTERIOR;
      break;
    default:
      // Garage doors, gates and dry contacts are not exposed.  A controller would otherwise
      // open them with commands like "open all blinds".
      return false;
  }
  if(shade->tiltType == tilt_types::tiltonly) *type = MatterWindowCovering::BLIND_TILT_ONLY;
  else if(shade->tiltType != tilt_types::none) *type = MatterWindowCovering::BLIND_LIFT_AND_TILT;
  return true;
}
static uint16_t toPercent100ths(float pos) {
  if(pos <= 0.0f) return 0;
  if(pos >= 100.0f) return 10000;
  return (uint16_t)roundf(pos * 100.0f);
}
static MatterWindowCovering::OperationalState_t toOperationalState(int8_t dir) {
  if(dir > 0) return MatterWindowCovering::MOVING_DOWN_OR_CLOSE;
  if(dir < 0) return MatterWindowCovering::MOVING_UP_OR_OPEN;
  return MatterWindowCovering::STALL;
}
// Called on the Matter task.  The radio and shade state belong to the loop task so commands are queued.
static bool queueCommand(uint8_t shadeId, matter_actions_t action, uint8_t value) {
  // Attribute updates pushed by syncShades() come back through these callbacks.
  if(xTaskGetCurrentTaskHandle() == matterLoopTask) return true;
  matter_cmd_t cmd = {shadeId, action, value};
  return xQueueSend(matterQueue, &cmd, 0) == pdTRUE;
}
static void addBridgedNode(endpoint_t *aggregator, MatterWindowCovering *ep, SomfyShade *shade) {
  endpoint_t *wc = endpoint::get(node::get(), ep->getEndPointId());
  if(!wc || !aggregator) return;
  endpoint::bridged_node::config_t cfg;
  if(endpoint::bridged_node::add(wc, &cfg) != ESP_OK) {
    Serial.printf("Matter: unable to add bridged node to endpoint %u\n", ep->getEndPointId());
    return;
  }
  cluster_t *info = cluster::get(wc, chip::app::Clusters::BridgedDeviceBasicInformation::Id);
  if(info) {
    namespace bdbi = cluster::bridged_device_basic_information::attribute;
    using namespace chip::app::Clusters::BridgedDeviceBasicInformation::Attributes;
    if(!attribute::get(info, NodeLabel::Id)) bdbi::create_node_label(info, shade->name, strlen(shade->name));
    char uid[33];
    snprintf(uid, sizeof(uid), "%s-%u", settings.serverId, shade->getShadeId());
    if(!attribute::get(info, UniqueID::Id)) bdbi::create_unique_id(info, uid, strlen(uid));
  }
  endpoint::set_parent_endpoint(wc, aggregator);
}
bool SomfyMatter::supported() {
  // Without the espsomfy partition Matter would share the small settings nvs partition and
  // decommissioning (which erases the default nvs partition) would wipe the settings.
  const char *label = SomfyPreferences::partitionLabel();
  return label && strcmp(label, SOMFY_NVS_PARTITION) == 0;
}
bool SomfyMatter::commissioned() { return this->_started && !this->_failed && Matter.isDeviceCommissioned(); }
uint8_t SomfyMatter::endpointCount() { return matterShadeCount; }
void SomfyMatter::start() {
  this->_started = true;
  matterLoopTask = xTaskGetCurrentTaskHandle();
  matterQueue = xQueueCreate(16, sizeof(matter_cmd_t));
  Matter.setSetupPasscode(settings.Matter.passcode);
  Matter.setSetupDiscriminator(settings.Matter.discriminator);
  // The controller is already on the network.  BLE commissioning would only cost RAM.
  Matter.setBLECommissioningEnabled(false);
  Matter.setVendorName("ESPSomfy-RTS");
  Matter.setProductName("ESPSomfy-RTS Bridge");
  Matter.setDeviceName(settings.hostname);
  Matter.setSerialNumber(settings.serverId);
  MatterNode::init();
  endpoint::aggregator::config_t aggConfig;
  endpoint_t *aggregator = endpoint::aggregator::create(node::get(), &aggConfig, ENDPOINT_FLAG_NONE, nullptr);

  // Create the endpoints in shade id order so they keep their endpoint ids across reboots.
  SomfyShade *shades[SOMFY_MAX_SHADES];
  uint8_t count = 0;
  for(uint8_t i = 0; i < SOMFY_MAX_SHADES; i++) {
    SomfyShade *shade = &somfy.shades[i];
    if(shade->getShadeId() == 255) continue;
    uint8_t j = count++;
    for(; j > 0 && shades[j - 1]->getShadeId() > shade->getShadeId(); j--) shades[j] = shades[j - 1];
    shades[j] = shade;
  }
  for(uint8_t i = 0; i < count; i++) {
    SomfyShade *shade = shades[i];
    MatterWindowCovering::WindowCoveringType_t type;
    if(!coveringType(shade, &type)) continue;
    if(matterShadeCount >= MATTER_MAX_SHADES) {
      Serial.printf("Matter: only %d shades can be exposed, skipping %s\n", MATTER_MAX_SHADES, shade->name);
      continue;
    }
    MatterWindowCovering *ep = new MatterWindowCovering();
    uint8_t lift = (uint8_t)roundf(constrain(shade->currentPos, 0.0f, 100.0f));
    uint8_t tilt = (uint8_t)roundf(constrain(shade->currentTiltPos, 0.0f, 100.0f));
    if(type == MatterWindowCovering::BLIND_TILT_ONLY) lift = 0;
    else if(type != MatterWindowCovering::BLIND_LIFT_AND_TILT) tilt = 0;
    if(!ep->begin(lift, tilt, type)) {
      Serial.printf("Matter: unable to create endpoint for %s\n", shade->name);
      delete ep;
      continue;
    }
    matter_shade_t *ms = &matterShades[matterShadeCount++];
    ms->shadeId = shade->getShadeId();
    ms->ep = ep;
    ms->hasLift = type != MatterWindowCovering::BLIND_TILT_ONLY;
    ms->hasTilt = type == MatterWindowCovering::BLIND_TILT_ONLY || type == MatterWindowCovering::BLIND_LIFT_AND_TILT;
    uint8_t shadeId = ms->shadeId;
    ep->onStop([ms, shadeId]() {
      // The library reports StopMotion and then the same change again as a lift target.
      ms->skipLift = true;
      return queueCommand(shadeId, matter_actions_t::stop, 0);
    });
    ep->onGoToLiftPercentage([ms, shadeId](uint8_t percent) {
      if(ms->skipLift) {
        ms->skipLift = false;
        return true;
      }
      return queueCommand(shadeId, matter_actions_t::lift, percent);
    });
    ep->onGoToTiltPercentage([shadeId](uint8_t percent) { return queueCommand(shadeId, matter_actions_t::tilt, percent); });
    addBridgedNode(aggregator, ep, shade);
    Serial.printf("Matter: %s is endpoint %u\n", shade->name, ep->getEndPointId());
  }
  if(matterShadeCount == 0) {
    // Matter will not start without an endpoint beyond the aggregator.  Shades can be added
    // and picked up after a reboot.
    Serial.println("Matter: no shades to expose, not starting");
    this->_failed = true;
    return;
  }
  this->heapBefore = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  Matter.begin();
  this->heapAfter = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  String code = Matter.getManualPairingCode();
  if(code.length() == 0) {
    Serial.println("Matter: failed to start");
    this->_failed = true;
    return;
  }
  Serial.printf("Matter started: commissioned=%d pairing code=%s\n", Matter.isDeviceCommissioned(), code.c_str());
  if(!Matter.isDeviceCommissioned()) Serial.printf("Matter QR code: %s\n", Matter.getOnboardingQRCodeUrl().c_str());
  this->syncShades(true);
}
void SomfyMatter::processCommands() {
  matter_cmd_t cmd;
  while(xQueueReceive(matterQueue, &cmd, 0) == pdTRUE) {
    matter_shade_t *ms = nullptr;
    for(uint8_t i = 0; i < matterShadeCount && !ms; i++)
      if(matterShades[i].shadeId == cmd.shadeId) ms = &matterShades[i];
    SomfyShade *shade = somfy.getShadeById(cmd.shadeId);
    if(!ms || !shade) continue;
    Serial.printf("Matter: shade %u action %u value %u\n", cmd.shadeId, static_cast<uint8_t>(cmd.action), cmd.value);
    switch(cmd.action) {
      case matter_actions_t::lift:
        ms->pendingLift = cmd.value;
        ms->pendingTime = millis();
        break;
      case matter_actions_t::tilt:
        ms->pendingTilt = cmd.value;
        ms->pendingTime = millis();
        break;
      case matter_actions_t::stop:
        ms->pendingLift = ms->pendingTilt = -1;
        // My stops a moving motor but sends an idle one to its favorite position.
        if(shade->direction != 0 || shade->tiltDirection != 0) shade->sendCommand(somfy_commands::My);
        break;
    }
  }
  for(uint8_t i = 0; i < matterShadeCount; i++) {
    matter_shade_t *ms = &matterShades[i];
    if((ms->pendingLift < 0 && ms->pendingTilt < 0) || millis() - ms->pendingTime < MATTER_TARGET_SETTLE) continue;
    SomfyShade *shade = somfy.getShadeById(ms->shadeId);
    if(shade) {
      if(ms->pendingLift >= 0) shade->moveToTarget(ms->pendingLift);
      if(ms->pendingTilt >= 0) shade->moveToTiltTarget(ms->pendingTilt);
    }
    ms->pendingLift = ms->pendingTilt = -1;
  }
}
void SomfyMatter::syncShades(bool force) {
  if(!force && millis() - this->lastSync < MATTER_SYNC_INTERVAL) return;
  this->lastSync = millis();
  for(uint8_t i = 0; i < matterShadeCount; i++) {
    matter_shade_t *ms = &matterShades[i];
    SomfyShade *shade = somfy.getShadeById(ms->shadeId);
    if(!shade) continue;
    MatterWindowCovering *ep = ms->ep;
    int8_t dir = ms->hasLift ? shade->direction : 0;
    int8_t tiltDir = ms->hasTilt ? shade->tiltDirection : 0;
    if(ms->hasLift) {
      uint16_t target = toPercent100ths(shade->target);
      uint16_t pos = toPercent100ths(shade->currentPos);
      if(force || target != ms->target) {
        ep->setTargetLiftPercent100ths(target);
        ms->target = target;
      }
      // Current position is persisted by Matter so only write it once a second while moving.
      if(force || (pos != ms->pos && (dir == 0 || millis() - ms->lastPos >= MATTER_MOVING_POS_INTERVAL))) {
        ep->setCurrentLiftPercent100ths(pos);
        ms->pos = pos;
        ms->lastPos = millis();
      }
      if(force || dir != ms->dir) ep->setOperationalState(MatterWindowCovering::LIFT, toOperationalState(dir));
    }
    if(ms->hasTilt) {
      uint16_t target = toPercent100ths(shade->tiltTarget);
      uint16_t pos = toPercent100ths(shade->currentTiltPos);
      if(force || target != ms->tiltTarget) {
        ep->setTargetTiltPercent100ths(target);
        ms->tiltTarget = target;
      }
      if(force || (pos != ms->tiltPos && (tiltDir == 0 || millis() - ms->lastTiltPos >= MATTER_MOVING_POS_INTERVAL))) {
        ep->setCurrentTiltPercent100ths(pos);
        ms->tiltPos = pos;
        ms->lastTiltPos = millis();
      }
      if(force || tiltDir != ms->tiltDir) ep->setOperationalState(MatterWindowCovering::TILT, toOperationalState(tiltDir));
    }
    if(force || dir != ms->dir || tiltDir != ms->tiltDir)
      ep->setOperationalState(MatterWindowCovering::GLOBAL, toOperationalState(dir != 0 ? dir : tiltDir));
    ms->dir = dir;
    ms->tiltDir = tiltDir;
  }
}
void SomfyMatter::loop() {
  if(!this->_started) {
    if(!settings.Matter.enabled || !this->supported()) return;
    if(!net.connected() || net.softAPOpened) return;
    if(net.connType == conn_types_t::ethernet) {
      // Matter needs an IPv6 address on the ethernet interface before it starts.
      Matter.selectNetwork(MATTER_NETWORK_ETHERNET);
      if(!Matter.waitForNetwork(0)) return;
    }
    this->start();
    return;
  }
  if(this->_failed) return;
  this->processCommands();
  this->syncShades();
}
void SomfyMatter::decommission() {
  // Erases the default nvs partition (fabrics, keys) and restarts.  ESPSomfy settings are
  // in the espsomfy partition and are not affected.
  if(this->supported()) Matter.decommission();
}
void SomfyMatter::toJSON(JsonResponse &json) {
  bool running = this->_started && !this->_failed;
  json.addElem("supported", this->supported());
  json.addElem("enabled", settings.Matter.enabled);
  json.addElem("started", running);
  json.addElem("commissioned", running && Matter.isDeviceCommissioned());
  json.addElem("endpoints", (uint32_t)matterShadeCount);
  json.addElem("manualCode", running ? Matter.getManualPairingCode().c_str() : "");
  json.addElem("qrCodeUrl", running ? Matter.getOnboardingQRCodeUrl().c_str() : "");
  json.addElem("psram", (uint32_t)ESP.getPsramSize());
  json.addElem("psramFree", (uint32_t)ESP.getFreePsram());
  json.addElem("heapFree", (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  json.addElem("heapMinFree", (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
  json.addElem("heapMaxBlock", (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
  json.addElem("heapBeforeStart", this->heapBefore);
  json.addElem("heapAfterStart", this->heapAfter);
}
#else
bool SomfyMatter::supported() { return false; }
bool SomfyMatter::commissioned() { return false; }
uint8_t SomfyMatter::endpointCount() { return 0; }
void SomfyMatter::start() {}
void SomfyMatter::processCommands() {}
void SomfyMatter::syncShades(bool force) {}
void SomfyMatter::loop() {}
void SomfyMatter::decommission() {}
void SomfyMatter::toJSON(JsonResponse &json) {
  json.addElem("supported", false);
  json.addElem("enabled", settings.Matter.enabled);
  json.addElem("started", false);
  json.addElem("commissioned", false);
  json.addElem("endpoints", (uint32_t)0);
  json.addElem("manualCode", "");
  json.addElem("qrCodeUrl", "");
}
#endif
