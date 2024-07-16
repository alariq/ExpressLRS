#include "targets.h"
#include "common.h"
#include "device.h"

#include "config.h"
#include "CRSF.h"
#include "msp.h"
#include "logging.h"

#include "devButton.h"

#define PITMODE_OFF     0
#define PITMODE_ON      1

// Delay after disconnect to preserve the VTXSS_CONFIRMED status
// Needs to be long enough to reconnect, but short enough to
// reset between the user switching equipment
#define VTX_DISCONNECT_DEBOUNCE_MS (10 * 1000)

extern CRSF crsf;
extern Stream *TxBackpack;
static uint8_t pitmodeAuxState = 0;
static bool sendEepromWrite = true;
//sebi:
// if < 0 means alt channel is not active, so use default
static int8_t vtxAltChannelIndex = -1;
//~

static enum VtxSendState_e
{
  VTXSS_UNKNOWN,   // Status of the remote side is unknown, so we should send immediately if connected
  VTXSS_MODIFIED,  // Config is editied, should always be sent regardless of connect state
  VTXSS_SENDING1, VTXSS_SENDING2, VTXSS_SENDING3,  VTXSS_SENDINGDONE, // Send the config 3x
  VTXSS_CONFIRMED  // Status of remote side is consistent with our config
} VtxSendState;

void VtxTriggerSend()
{
    VtxSendState = VTXSS_MODIFIED;
    devicesTriggerEvent();
}

void VtxPitmodeSwitchUpdate()
{
    if (config.GetVtxPitmode() == PITMODE_OFF)
    {
        return;
    }

    uint8_t auxInverted = config.GetVtxPitmode() % 2;
    uint8_t auxNumber = (config.GetVtxPitmode() / 2) + 3;
    uint8_t currentPitmodeAuxState = CRSF_to_BIT(ChannelData[auxNumber]) ^ auxInverted;

    if (pitmodeAuxState != currentPitmodeAuxState)
    {
        pitmodeAuxState = currentPitmodeAuxState;
        sendEepromWrite = false;
        VtxTriggerSend();
    }
}
//sebi:
void VtxAltChSwitchUpdate()
{
    // if channel swith is not set or no bands setup, then not bother to check for switch value
    bool off = !config.GetVtxAltChSwitch() || !config.GetVtxAltBand(0);
    if(off) {
        return;
    }

    uint8_t auxNumber = config.GetVtxAltChSwitch() + 3;
    // if not off and channel > mid value
    // num alternative channels + 1 standard default
    int8_t currentAltChAuxState = CRSF_to_N(ChannelData[auxNumber], NUM_ALT_VTX_CHANNELS + 1);
    // if zero means use standard vtx channel
    currentAltChAuxState = currentAltChAuxState - 1;

    if (vtxAltChannelIndex != currentAltChAuxState)
    {
        DBGLN("Trigger switch %s alt state", currentAltChAuxState? "to" : "from");
        vtxAltChannelIndex = currentAltChAuxState;
        sendEepromWrite = false;
        VtxTriggerSend();
    }
}
//~

static void eepromWriteToMSPOut()
{
    mspPacket_t packet;
    packet.reset();
    packet.function = MSP_EEPROM_WRITE;

    crsf.AddMspMessage(&packet);
}

static void VtxConfigToMSPOut()
{
    DBGLN("Sending VtxConfig, altVtxChIndex: %d", vtxAltChannelIndex);
    //sebi:
    uint8_t vtxIdx = (config.GetVtxBand()-1) * 8 + config.GetVtxChannel();
    // should be never more than NUM_ALT_VTX_CHANNELS, but just in case...
    const bool b_alt_band_ok = 
        vtxAltChannelIndex >= 0 && vtxAltChannelIndex < NUM_ALT_VTX_CHANNELS && config.GetVtxAltBand(vtxAltChannelIndex)!=0;
    vtxIdx = b_alt_band_ok ? (config.GetVtxAltBand(vtxAltChannelIndex) - 1) * 8 + config.GetVtxAltChannel(vtxAltChannelIndex) : vtxIdx;
    //~

    mspPacket_t packet;
    packet.reset();
    packet.makeCommand();
    packet.function = MSP_SET_VTX_CONFIG;
    packet.addByte(vtxIdx);
    packet.addByte(0);
    if (config.GetVtxPower()) {
        packet.addByte(config.GetVtxPower());

        if (config.GetVtxPitmode() == PITMODE_OFF || config.GetVtxPitmode() == PITMODE_ON)
        {
            packet.addByte(config.GetVtxPitmode());
        }
        else
        {
            packet.addByte(pitmodeAuxState);
        }
    }

    crsf.AddMspMessage(&packet);

//sebi: allow
#if 0
    if (!crsf::IsArmed()) // Do not send while armed.  There is no need to change the video frequency while armed.  It can also cause VRx modules to flash up their OSD menu e.g. Rapidfire.
#endif
    {
        MSP::sendPacket(&packet, TxBackpack); // send to tx-backpack as MSP
    }
}

static void initialize()
{
    registerButtonFunction(ACTION_SEND_VTX, VtxTriggerSend);
}

static int event()
{
    if (VtxSendState == VTXSS_MODIFIED ||
        (VtxSendState == VTXSS_UNKNOWN && connectionState == connected))
    {
        VtxSendState = VTXSS_SENDING1;
        return 1000;
    }

    if (connectionState == disconnected)
    {
        // If the VtxSend has completed, wait before going back to VTXSS_UNKNOWN
        // to ignore a temporary disconnect after saving EEPROM
        if (VtxSendState == VTXSS_CONFIRMED)
        {
            VtxSendState = VTXSS_CONFIRMED;
            return VTX_DISCONNECT_DEBOUNCE_MS;
        }
        VtxSendState = VTXSS_UNKNOWN;
    }
    else if (VtxSendState == VTXSS_CONFIRMED && connectionState == connected)
    {
        return DURATION_NEVER;
    }

    return DURATION_IGNORE;
}

static int timeout()
{
    // 0 = off in the lua Band field
    if (config.GetVtxBand() == 0)
    {
        VtxSendState = VTXSS_CONFIRMED;
        return DURATION_NEVER;
    }

    // Can only get here in VTXSS_CONFIRMED state if still disconnected
    if (VtxSendState == VTXSS_CONFIRMED)
    {
        VtxSendState = VTXSS_UNKNOWN;
        return DURATION_NEVER;
    }

    VtxConfigToMSPOut();

    VtxSendState = (VtxSendState_e)((int)VtxSendState + 1);
    if (VtxSendState < VTXSS_SENDINGDONE)
        return 500; // repeat send in 500ms

    if (connectionState == connected)
    {
        // Connected while sending, assume the MSP got to the RX
        VtxSendState = VTXSS_CONFIRMED;
        if (sendEepromWrite)
            eepromWriteToMSPOut();
        sendEepromWrite = true;
    }
    else
    {
        VtxSendState = VTXSS_UNKNOWN;
        // Never received a connection, clear the queue which now
        // has multiple VTX config packets in it
        crsf.ResetMspQueue();
    }

    return DURATION_NEVER;
}

device_t VTX_device = {
    .initialize = initialize,
    .start = NULL,
    .event = event,
    .timeout = timeout
};
