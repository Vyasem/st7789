#include "stm32f4xx_hal.h"
#include "dc.h"
#include "main.h"
#include "ST7789VDriver.hpp"
#include <assert.h>
#include <touchgfx/hal/OSWrappers.hpp>
#include "freertos_utils/freertos_utils.h"

extern SPI_HandleTypeDef hspi1;
extern DMA2D_HandleTypeDef hdma2d;
extern TIM_HandleTypeDef htim6;
SemaphoreHandle_t mutex_lcd = NULL;

volatile uint8_t IsTransmittingBlock_ = 0;
volatile uint16_t TE = 0;

//Signal TE interrupt to TouchGFX
void touchgfxSignalVSync(void);
void setVSYNC(void);
void clearVSYNC(void);

static  void SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint16_t Size)
{
	SET_BIT(hspi->Instance->CR1, SPI_CR1_SPE);
	hspi->pTxBuffPtr  = (uint8_t *)pTxData;
	hspi->TxXferCount = Size;

	while(hspi->TxXferCount > 0)
	{
		while(!(hspi->Instance->SR & SPI_FLAG_TXE));

		if(hspi->Init.DataSize == SPI_DATASIZE_16BIT)
		{
			hspi->Instance->DR = *((uint16_t *)hspi->pTxBuffPtr);
			hspi->pTxBuffPtr += sizeof(uint16_t);
		}
		else
		{
			hspi->Instance->DR = *hspi->pTxBuffPtr;
			hspi->pTxBuffPtr += sizeof(uint8_t);
		}

		while(!(hspi->Instance->SR & SPI_FLAG_RXNE));
		(void) hspi->Instance->DR;

		hspi->TxXferCount--;
	}

	while(hspi->Instance->SR & SPI_FLAG_BSY);

	CLEAR_BIT(hspi->Instance->CR1, SPI_CR1_SPE);
}

static void Display_DCS_Send(uint8_t command)
{
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
	SPI_Transmit(&hspi1, &command, sizeof(command));
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

static void Display_DCS_Send_With_Data(uint8_t command, uint8_t* data, uint8_t size)
{
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
	//HAL_SPI_Transmit(&hspi1, &command, sizeof(command), HAL_MAX_DELAY);
	SPI_Transmit(&hspi1, &command, sizeof(command));

	HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
	SPI_Transmit(&hspi1, data, sizeof(data));
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

void DisplayOn(void)
{
	// Display ON
	Display_DCS_Send(LCD_DISPON);
	//touchgfx::OSWrappers::taskDelay(10);
}

void Display_OFF(void)
{
	// Display OFF
	Display_DCS_Send(LCD_DISPOFF);
	touchgfx::OSWrappers::taskDelay(100);
}

static void WriteData(uint8_t *buff, size_t buff_size)
{
	LCD_CS_GPIO_Port->BSRR = LCD_CS_Pin << 16;
	LCD_DC_GPIO_Port->BSRR = LCD_DC_Pin;

	// split data in small chunks because HAL can't send more than 64K at once
	while (buff_size > 0) {
		uint16_t chunk_size = buff_size > 65535 ? 65535 : buff_size;
		HAL_SPI_Transmit(&hspi1, buff, chunk_size, HAL_MAX_DELAY);
		//SPI_Transmit(&hspi1, buff, chunk_size);
		buff += chunk_size;
		buff_size -= chunk_size;
	}

	LCD_CS_GPIO_Port->BSRR = LCD_CS_Pin;
}

static uint16_t old_x0=0xFFFF, old_x1=0xFFFF, old_y0=0xFFFF, old_y1=0xFFFF;

void SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	//HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
	uint16_t x_start = x0, x_end = x1;
	uint16_t y_start = y0, y_end = y1;

	/* Column Address set */
	Display_DCS_Send(LCD_CASET);
	{
		uint8_t data[] = {x_start >> 8, x_start & 0xFF, x_end >> 8, x_end & 0xFF};
		WriteData(data, sizeof(data));
	}
	/* Row Address set */
	Display_DCS_Send(LCD_RASET);
	{
		uint8_t data[] = {y_start >> 8, y_start & 0xFF, y_end >> 8, y_end & 0xFF};
		WriteData(data, sizeof(data));
	}
	/* Write to RAM */
	Display_DCS_Send(LCD_RAMWR);
	//HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}
void DisplayBitmap(const uint8_t *bitmap, uint16_t posx, uint16_t posy, uint16_t sizex, uint16_t sizey)
{
	xSemaphoreTake(mutex_lcd, portMAX_DELAY);
	IsTransmittingBlock_ = 1;
	// Define the display area
	SetAddressWindow(posx, posy, posx+sizex-1, posy+sizey-1);

	CLEAR_BIT(SPI1->CR1, SPI_CR1_SPE); /** Disable SPI */
	SET_BIT(SPI1->CR1, SPI_CR1_DFF); /** Enable 16-bit data frame format */
	SET_BIT(SPI1->CR1, SPI_CR1_SPE); /** Enable SPI */

	hspi1.Init.DataSize = SPI_DATASIZE_16BIT;

	WriteData((uint8_t*)bitmap, (sizex * sizey));

	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;

	CLEAR_BIT(SPI1->CR1, SPI_CR1_SPE); /** Disable SPI */
	CLEAR_BIT(SPI1->CR1, SPI_CR1_DFF); /** Disable 16-bit data frame format */
	SET_BIT(SPI1->CR1, SPI_CR1_SPE); /** Enable SPI */

	IsTransmittingBlock_ = 0;
	DisplayDriver_TransferCompleteCallback();
	xSemaphoreGive(mutex_lcd);
}

void DisplayInit(void)
{
	mutex_lcd = xSemaphoreCreateMutex();
	uint8_t arguments[4];

	//Exit from sleep mode
	Display_DCS_Send(LCD_SLPOUT);
	touchgfx::OSWrappers::taskDelay(10);

	//Display Normal mode
	Display_DCS_Send(LCD_NORON);
	touchgfx::OSWrappers::taskDelay(10);

	//Iddle mode off
	Display_DCS_Send(LCD_IDMOFF);
	touchgfx::OSWrappers::taskDelay(10);

	//Invert mode on
	Display_DCS_Send(LCD_INVON);
	touchgfx::OSWrappers::taskDelay(10);

	// Pixel Format
	arguments[0] = 0x5; // 16-bit color
	Display_DCS_Send_With_Data(LCD_COLMOD, arguments, 1);
	touchgfx::OSWrappers::taskDelay(10);

	// Tearing effect line on
	Display_DCS_Send(LCD_TEON);
	touchgfx::OSWrappers::taskDelay(10);

}

void DisplayReset(void)
{
	HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_RESET);
	touchgfx::OSWrappers::taskDelay(10);
	HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_SET);
	touchgfx::OSWrappers::taskDelay(10);
}

int touchgfxDisplayDriverTransmitActive(void)
{
	return IsTransmittingBlock_;
}

void touchgfxDisplayDriverTransmitBlock(const uint8_t* pixels, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
	DisplayBitmap(pixels, x, y, w, h);
}

int touchgfxDisplayDriverShouldTransferBlock(uint16_t bottom)
{
	//return (bottom < getCurrentLine());
	return (bottom < (TE > 0 ? 0xFFFF : ((uint16_t)htim6.Instance->CNT)));
}

void DisplayFillColor(uint16_t color)
{

	SetAddressWindow(0, 0, 240 - 1, 320 - 1);
	for (int i = 0; i < 240; i++)
		for (int j = 0; j < 320; j++) {
			//WriteData((uint8_t*)&color, sizeof(color)/sizeof(uint16_t));
			uint8_t data[] = {color >> 8, color & 0xFF};
			WriteData(data, sizeof(data));
		}
}
