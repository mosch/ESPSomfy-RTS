#include <Arduino.h>
#include "WResp.h"
#ifndef SomfyMatter_h
#define SomfyMatter_h

// Exposes the shades as Matter window coverings behind a Matter bridge (aggregator).
// Commissioning happens over the IP network (no BLE) once the controller is connected.
class SomfyMatter {
  protected:
    bool _started = false;
    bool _failed = false;
    uint32_t lastSync = 0;
    uint32_t heapBefore = 0;  // Free internal heap before and after starting the Matter stack.
    uint32_t heapAfter = 0;
    void start();
    void processCommands();
    void syncShades(bool force = false);
  public:
    bool supported();     // Firmware has Matter and the partition table has room for it.
    bool started() { return this->_started; }
    bool commissioned();
    uint8_t endpointCount();
    void loop();
    void decommission();
    void toJSON(JsonResponse &json);
};
#endif
