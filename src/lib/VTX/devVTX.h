#pragma once

#include "device.h"

// Call this to trigger sending of Vtx packet
void VtxTriggerSend();
void VtxPitmodeSwitchUpdate();
void VtxAltChSwitchUpdate(); //sebi
extern device_t VTX_device;