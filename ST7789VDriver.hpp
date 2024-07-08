#ifndef __DISPLAY_H
#define __DISPLAY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void touchgfxDisplayDriverTransmitBlock(const uint8_t* pixels, uint16_t x, uint16_t y, uint16_t w, uint16_t h);
int touchgfxDisplayDriverTransmitActive(void);
int touchgfxDisplayDriverShouldTransferBlock(uint16_t bottom);
void DisplayOn(void);
void DisplayReset(void);
void DisplayInit(void);
void DisplayDriver_TransferCompleteCallback(void);
void DisplayFillColor(uint16_t color);
void SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void DisplayBitmap(const uint8_t *bitmap, uint16_t posx, uint16_t posy, uint16_t sizex, uint16_t sizey);

#ifdef __cplusplus
}
#endif

#endif
