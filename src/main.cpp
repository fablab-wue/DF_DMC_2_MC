#include "bridge.h"

namespace {
sliderdmc::DmcBridge bridge;
}

void setup() { bridge.setup(); }

void loop() { bridge.loop(); }
