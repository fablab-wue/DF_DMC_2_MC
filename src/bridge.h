#pragma once

// USB DMC dispatch: hello, motors, GIO, DMX, path play sequencing.

#include "config.h"
#include "mc_client.h"
#include "path_store.h"
#include "status_led.h"

#include <dmc_protocol.h>
#include <dmx_engine.h>
#include <gio_io.h>

namespace sliderdmc {

using namespace dfdmc;

class DmcBridge {
 public:
  void setup();
  void loop();

 private:
  bool motorIndexValid(uint8_t motor) const;
  void sendDmcFrame(uint32_t id, uint16_t type, const std::vector<uint8_t>& payload);
  void sendDmcAck(uint32_t id, uint16_t type, uint32_t status);
  void sendDmcHello(uint32_t id);
  void sendMotorStatus(uint32_t id);
  void sendMotorPositions(uint32_t id);
  void sendGioIn(uint32_t id);
  void maybeSendBootHello();
  void maybeSendPositionReport();
  void maybeUnsolicitedGio();
  void maybeFinishPath();
  void fireBloop(unsigned ms);
  void applyFrameTrigger(int dfFrame);
  void pumpPendingPlay();
  uint32_t psFromFpsX1000(uint32_t fpsX1000) const;
  void handleDmcFrame(const DmcFrame& frame);
  void handleMotorOrRt(const DmcFrame& frame);

  DmcParser dmcParser_;
  McSerialClient mcClient_;
  DmcGio gio_;
  DmxEngine dmx_;
  PathStore path_;
  StatusLed statusLed_;
  uint32_t lastPositionTxMs_ = 0;
  uint32_t pendingPlayAtMs_ = 0;
  bool dfConnected_ = false;
  bool bootHelloSent_ = false;
  bool pendingPlay_ = false;
  bool wasPathActive_ = false;
  int pendingStart_ = 1;
  int pendingEnd_ = 1;
  unsigned pendingBloopMs_ = 0;
  uint32_t pendingPrerollMs_ = 0;
};

}  // namespace sliderdmc
