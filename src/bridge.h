#pragma once

// USB DMC dispatch: hello, motors, GIO, DMX, path play sequencing.

#include "config.h"
#include "mc_client.h"
#include "path_store.h"
#include "status_led.h"

#include <dmc_protocol.h>
#include <dmx_engine.h>
#include <gio_io.h>
#include <rt_support.h>

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
  void sendMotorPositions(uint32_t id, int32_t frameTime = 0);
  void sendGioIn(uint32_t id);
  void maybeSendBootHello();
  void maybeSendPositionReport();
  void maybeUnsolicitedGio();
  void maybeFinishPath();
  void maybeFinishShoot();
  void endShoot(bool notify);
  void cancelLive(bool notify);
  void noteCdc(bool up);
  void sendHardStop(uint8_t reason);
  bool moveToSample(double frameTime);
  uint32_t poseFault(double frameTime, bool extrapolate) const;
  bool rejectRunLimits(const RtRunMove& move, const RtPlaySpan& span, uint32_t id);
  bool inRun(int frame) const;
  void handleShootFrame(const DmcFrame& frame);
  void handleShootFrame2(const DmcFrame& frame);
  void applyFrameTrigger(int dfFrame);
  void applyProgramDmx(int dfFrame);
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
  bool armedPlay_ = false;
  bool syncDmx_ = false;
  bool wasPathActive_ = false;
  int pendingStart_ = 1;
  int pendingEnd_ = 1;
  int runStart_ = 1;
  int runEnd_ = 1;
  double postrollFrame_ = 1;
  BloopOut bloop_;
  LiveShutter shutter_;
  uint32_t lastStopAllMs_ = 0;
  bool cdcWasUp_ = false;
  bool shootNeedsEnd_ = false;
  uint32_t pendingPostrollMs_ = 0;
  uint32_t postrollUntilMs_ = 0;
  bool postrollWaiting_ = false;
  bool postrollMoveSent_ = false;
  bool shootArmed_ = false;
  bool shootGoing_ = false;
  bool shootPendingMf_ = false;
  bool shootShutterOn_ = false;
  uint32_t shootT0_ = 0;
  uint32_t shootMoveMs_ = 0;
  uint32_t shootRampMs_ = 0;
  uint32_t shootDelayMs_ = 0;
  uint32_t shootShutterOpenMs_ = 0;
  uint32_t shootShutterCloseMs_ = 0;
  uint32_t shootDoneMs_ = 0;
  int32_t shootEndSteps_[kMaxAxes]{};
  bool shootBlur_[kMaxAxes]{};
};

}  // namespace sliderdmc
