#include "lpp.h"
#include "uwb.h"
#include "stm32f10x.h"

#define debug(...) // printf(__VA_ARGS__)

void lppHandleShortPacket(char *data, size_t length)
{
  if (length < 1) return;

  int type  = data[0];

  debug("Handling LPP short packet of type %02x, length %d\r\n", type, length);
  debug("Raw data: ");
  for (int i=0; i<length; i++) {
    debug("%02x ", data[i]);
  }
  debug("\r\n");

  switch(type) {
    case LPP_SHORT_ANCHOR_POSITION:
    {
      struct lppShortAnchorPosition_s* newpos = (struct lppShortAnchorPosition_s*)&data[1];

      if (length != 3*sizeof(float) + 1) {
        debug("LPP: Wrong set-anchor position length\r\n");
        break;
      }

      //cfgWriteFP32list(cfgAnchorPos, newpos->position, 3);
      uwbConfig_t *uwbConfig = uwb_get_config();
      uwbConfig->position[0] = newpos->position[0];
      uwbConfig->position[1] = newpos->position[1];
      uwbConfig->position[2] = newpos->position[2];
      uwbConfig->positionEnabled = true;

      debug("Setting new anchor position to %f, %f, %f\r\n", newpos->position[0],
                                                             newpos->position[1],
                                                             newpos->position[2]);
      break;
    }
    case LPP_SHORT_REBOOT:
    {
      struct lppShortReboot_s* rebootInfo = (struct lppShortReboot_s*)&data[1];

      // Set boot flags
      if (rebootInfo->bootMode == LPP_SHORT_REBOOT_TO_BOOTLOADER) {
        //bootmodeSetBootloaderModeFlag();
      } else if (rebootInfo->bootMode == LPP_SHORT_REBOOT_TO_FIRMWARE) {
        //bootmodeClearBootloaderModeFlag();
      }

      // Then resets!
      NVIC_SystemReset();

      break;
    }
  }
}
