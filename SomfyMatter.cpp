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

// The root node and the aggregator use two of the available endpoints.  The rest are shades and groups.
#define MATTER_MAX_ENDPOINTS (CONFIG_ESP_MATTER_MAX_DYNAMIC_ENDPOINT_COUNT - 2)
#define MATTER_SYNC_INTERVAL 250
#define MATTER_MOVING_POS_INTERVAL 1000
// Controllers send a stream of targets while a slider is dragged.  The first target is sent right away,
// targets that follow within this time are collapsed into one so the motor does not get a burst of
// up/down commands.
#define MATTER_TARGET_SETTLE 1000

enum class matter_actions_t : uint8_t {
  lift = 0,
  tilt = 1,
  stop = 2
};
struct matter_cmd_t {
  uint8_t index;  // Into matterShades.
  matter_actions_t action;
  uint8_t value;
};
// A shade or a group.  Groups send one frame that all their paired motors receive.
struct matter_shade_t {
  uint8_t id = 0;
  bool isGroup = false;
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
  uint32_t lastTarget = 0;
};
static matter_shade_t matterShades[MATTER_MAX_ENDPOINTS];
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
static bool queueCommand(matter_shade_t *ms, matter_actions_t action, uint8_t value) {
  // Attribute updates pushed by syncShades() come back through these callbacks.
  if(xTaskGetCurrentTaskHandle() == matterLoopTask) return true;
  matter_cmd_t cmd = {(uint8_t)(ms - matterShades), action, value};
  return xQueueSend(matterQueue, &cmd, 0) == pdTRUE;
}
static void addBridgedNode(endpoint_t *aggregator, MatterWindowCovering *ep, const char *name, const char *uid) {
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
    if(!attribute::get(info, NodeLabel::Id)) bdbi::create_node_label(info, (char *)name, strlen(name));
    if(!attribute::get(info, UniqueID::Id)) bdbi::create_unique_id(info, (char *)uid, strlen(uid));
  }
  endpoint::set_parent_endpoint(wc, aggregator);
}
// Groups are only exposed when every linked shade is a window covering.  The group frame would
// otherwise also open a garage door or gate.  Position is the average of the linked shades.
static bool groupState(SomfyGroup *group, float *pos, float *target, int8_t *dir) {
  uint8_t n = 0;
  *pos = *target = 0.0f;
  *dir = 0;
  for(uint8_t i = 0; i < SOMFY_MAX_GROUPED_SHADES; i++) {
    if(group->linkedShades[i] == 0) continue;
    SomfyShade *shade = somfy.getShadeById(group->linkedShades[i]);
    if(!shade) continue;
    MatterWindowCovering::WindowCoveringType_t type;
    if(!coveringType(shade, &type)) return false;
    *pos += constrain(shade->currentPos, 0.0f, 100.0f);
    *target += constrain(shade->target, 0.0f, 100.0f);
    if(*dir == 0) *dir = shade->direction;
    n++;
  }
  if(n == 0) return false;
  *pos /= n;
  *target /= n;
  return true;
}
static matter_shade_t *addEndpoint(endpoint_t *aggregator, const char *name, const char *uid,
    MatterWindowCovering::WindowCoveringType_t type, uint8_t lift, uint8_t tilt) {
  if(matterShadeCount >= MATTER_MAX_ENDPOINTS) {
    Serial.printf("Matter: only %d shades and groups can be exposed, skipping %s\n", MATTER_MAX_ENDPOINTS, name);
    return nullptr;
  }
  MatterWindowCovering *ep = new MatterWindowCovering();
  if(!ep->begin(lift, tilt, type)) {
    Serial.printf("Matter: unable to create endpoint for %s\n", name);
    delete ep;
    return nullptr;
  }
  matter_shade_t *ms = &matterShades[matterShadeCount++];
  ms->ep = ep;
  ms->hasLift = type != MatterWindowCovering::BLIND_TILT_ONLY;
  ms->hasTilt = type == MatterWindowCovering::BLIND_TILT_ONLY || type == MatterWindowCovering::BLIND_LIFT_AND_TILT;
  ep->onStop([ms]() {
    // The library reports StopMotion and then the same change again as a lift target.
    ms->skipLift = true;
    return queueCommand(ms, matter_actions_t::stop, 0);
  });
  ep->onGoToLiftPercentage([ms](uint8_t percent) {
    if(ms->skipLift) {
      ms->skipLift = false;
      return true;
    }
    return queueCommand(ms, matter_actions_t::lift, percent);
  });
  ep->onGoToTiltPercentage([ms](uint8_t percent) { return queueCommand(ms, matter_actions_t::tilt, percent); });
  addBridgedNode(aggregator, ep, name, uid);
  Serial.printf("Matter: %s is endpoint %u\n", name, ep->getEndPointId());
  return ms;
}
static void moveLift(matter_shade_t *ms, uint8_t pos) {
  if(!ms->isGroup) {
    SomfyShade *shade = somfy.getShadeById(ms->id);
    if(!shade) return;
    // Always send open and close, even when the shade is tracked at that position.  A motor at its
    // end stop ignores it and a shade that was moved without us hearing it gets back in sync.
    if(pos == 0) shade->sendCommand(somfy_commands::Up);
    else if(pos == 100) shade->sendCommand(somfy_commands::Down);
    else shade->moveToTarget(pos);
    return;
  }
  SomfyGroup *group = somfy.getGroupById(ms->id);
  if(!group) return;
  if(pos == 0) group->sendCommand(somfy_commands::Up);
  else if(pos == 100) group->sendCommand(somfy_commands::Down);
  // Groups only open and close.  Push the real target back so the controller drops the position.
  else ms->target = 0xFFFF;
}
static void moveTilt(matter_shade_t *ms, uint8_t pos) {
  SomfyShade *shade = ms->isGroup ? nullptr : somfy.getShadeById(ms->id);
  if(shade) shade->moveToTiltTarget(pos);
}
static void stop(matter_shade_t *ms) {
  // My stops a moving motor but sends an idle one to its favorite position.
  if(ms->isGroup) {
    SomfyGroup *group = somfy.getGroupById(ms->id);
    float pos, target;
    int8_t dir;
    if(group && groupState(group, &pos, &target, &dir) && dir != 0) group->sendCommand(somfy_commands::My);
    return;
  }
  SomfyShade *shade = somfy.getShadeById(ms->id);
  if(shade && (shade->direction != 0 || shade->tiltDirection != 0)) shade->sendCommand(somfy_commands::My);
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
    uint8_t lift = (uint8_t)roundf(constrain(shade->currentPos, 0.0f, 100.0f));
    uint8_t tilt = (uint8_t)roundf(constrain(shade->currentTiltPos, 0.0f, 100.0f));
    if(type == MatterWindowCovering::BLIND_TILT_ONLY) lift = 0;
    else if(type != MatterWindowCovering::BLIND_LIFT_AND_TILT) tilt = 0;
    char uid[33];
    snprintf(uid, sizeof(uid), "%s-%u", settings.serverId, shade->getShadeId());
    matter_shade_t *ms = addEndpoint(aggregator, shade->name, uid, type, lift, tilt);
    if(ms) ms->id = shade->getShadeId();
  }
  // Groups come after the shades and are lift only.  Up and down go out as a single group frame.
  for(uint8_t i = 0; i < SOMFY_MAX_GROUPS; i++) {
    SomfyGroup *group = &somfy.groups[i];
    if(group->getGroupId() == 255) continue;
    float pos, target;
    int8_t dir;
    if(!groupState(group, &pos, &target, &dir)) continue;
    char uid[33];
    snprintf(uid, sizeof(uid), "%s-g%u", settings.serverId, group->getGroupId());
    matter_shade_t *ms = addEndpoint(aggregator, group->name, uid, MatterWindowCovering::ROLLERSHADE, (uint8_t)roundf(pos), 0);
    if(ms) {
      ms->id = group->getGroupId();
      ms->isGroup = true;
    }
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
    if(cmd.index >= matterShadeCount) continue;
    matter_shade_t *ms = &matterShades[cmd.index];
    Serial.printf("Matter: %s %u action %u value %u\n", ms->isGroup ? "group" : "shade", ms->id, static_cast<uint8_t>(cmd.action), cmd.value);
    switch(cmd.action) {
      case matter_actions_t::lift:
      case matter_actions_t::tilt: {
        bool settled = millis() - ms->lastTarget >= MATTER_TARGET_SETTLE;
        ms->lastTarget = millis();
        if(settled) {
          if(cmd.action == matter_actions_t::lift) moveLift(ms, cmd.value);
          else moveTilt(ms, cmd.value);
        }
        else if(cmd.action == matter_actions_t::lift) ms->pendingLift = cmd.value;
        else ms->pendingTilt = cmd.value;
        break;
      }
      case matter_actions_t::stop:
        ms->pendingLift = ms->pendingTilt = -1;
        stop(ms);
        break;
    }
  }
  for(uint8_t i = 0; i < matterShadeCount; i++) {
    matter_shade_t *ms = &matterShades[i];
    if((ms->pendingLift < 0 && ms->pendingTilt < 0) || millis() - ms->lastTarget < MATTER_TARGET_SETTLE) continue;
    if(ms->pendingLift >= 0) moveLift(ms, ms->pendingLift);
    if(ms->pendingTilt >= 0) moveTilt(ms, ms->pendingTilt);
    ms->pendingLift = ms->pendingTilt = -1;
  }
}
void SomfyMatter::syncShades(bool force) {
  if(!force && millis() - this->lastSync < MATTER_SYNC_INTERVAL) return;
  this->lastSync = millis();
  for(uint8_t i = 0; i < matterShadeCount; i++) {
    matter_shade_t *ms = &matterShades[i];
    float liftPos, liftTarget;
    int8_t dir = 0, tiltDir = 0;
    SomfyShade *shade = nullptr;
    if(ms->isGroup) {
      SomfyGroup *group = somfy.getGroupById(ms->id);
      if(!group || !groupState(group, &liftPos, &liftTarget, &dir)) continue;
    }
    else {
      shade = somfy.getShadeById(ms->id);
      if(!shade) continue;
      liftPos = shade->currentPos;
      liftTarget = shade->target;
      if(ms->hasLift) dir = shade->direction;
      if(ms->hasTilt) tiltDir = shade->tiltDirection;
    }
    MatterWindowCovering *ep = ms->ep;
    if(ms->hasLift) {
      uint16_t target = toPercent100ths(liftTarget);
      uint16_t pos = toPercent100ths(liftPos);
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
    if(ms->hasTilt && shade) {
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
