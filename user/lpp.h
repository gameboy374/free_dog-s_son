#ifndef __LPP_H__
#define __LPP_H__

#include <stddef.h>
#include <stdint.h>

void lppHandleShortPacket(char *data, size_t length);

#define SHORT_LPP 0xF0

#define LPP_SHORT_ANCHOR_POSITION 0x01
#define LPP_SHORT_REBOOT 0x02



struct lppShortAnchorPosition_s {
  float position[3];
};

#define LPP_SHORT_REBOOT_TO_BOOTLOADER 0x00
#define LPP_SHORT_REBOOT_TO_FIRMWARE 0x01

struct lppShortReboot_s {
  uint8_t bootMode;
} __attribute__((packed));

#endif //__LPP_H__
