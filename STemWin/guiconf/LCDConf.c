/*********************************************************************
*          Portions COPYRIGHT 2016 STMicroelectronics                *
*          Portions SEGGER Microcontroller GmbH & Co. KG             *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2015  SEGGER Microcontroller GmbH & Co. KG       *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.32 - Graphical user interface for embedded applications **
All  Intellectual Property rights  in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with the following terms:

The  software has  been licensed  to STMicroelectronics International
N.V. a Dutch company with a Swiss branch and its headquarters in Plan-
les-Ouates, Geneva, 39 Chemin du Champ des Filles, Switzerland for the
purposes of creating libraries for ARM Cortex-M-based 32-bit microcon_
troller products commercialized by Licensee only, sublicensed and dis_
tributed under the terms and conditions of the End User License Agree_
ment supplied by STMicroelectronics International N.V.
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
File        : LCDConf_FlexColor_Template.c
Purpose     : Display controller configuration (single layer)
---------------------------END-OF-HEADER------------------------------
*/

/**
  ******************************************************************************
  * @attention
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

#include "GUI.h"
#include "GUIDRV_FlexColor.h"
#include "main.h"
#include <stdint.h>

extern SPI_HandleTypeDef hspi3;

/*********************************************************************
*
*       Layer configuration (to be modified)
*
**********************************************************************
*/

//
// Physical display size
//
#define XSIZE_PHYS  240 // To be adapted to x-screen size
#define YSIZE_PHYS  320 // To be adapted to y-screen size

/*********************************************************************
*
*       Configuration checking
*
**********************************************************************
*/
#ifndef   VXSIZE_PHYS
  #define VXSIZE_PHYS XSIZE_PHYS
#endif
#ifndef   VYSIZE_PHYS
  #define VYSIZE_PHYS YSIZE_PHYS
#endif
#ifndef   XSIZE_PHYS
  #error Physical X size of display is not defined!
#endif
#ifndef   YSIZE_PHYS
  #error Physical Y size of display is not defined!
#endif
#ifndef   GUICC_565
  #error Color conversion not defined!
#endif
#ifndef   GUIDRV_FLEXCOLOR
  #error No display driver defined!
#endif

/*********************************************************************
*
*       Local functions
*
**********************************************************************
*/
static void setCS(uint8_t value)
{
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, value);
}

static void setA0(uint8_t value)
{
	HAL_GPIO_WritePin(LCD_A0_GPIO_Port, LCD_A0_Pin, value);
}

static void setRESET(uint8_t value)
{
	HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, value);
}

static void sendCmd(uint8_t data)
{
	uint8_t t = data;
	setA0(0);
	//HAL_SPI_Transmit_DMA(&hspi3, &t, 1);
	HAL_SPI_Transmit(&hspi3, &t, 1, 5000);
	//while(hspi3.Instance->SR  & SPI_SR_BSY);
	while(HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_BUSY_TX);
}

static void sendData(uint8_t data)
{
	uint8_t t = data;
	setA0(1);
	//HAL_SPI_Transmit_DMA(&hspi3, &t, 1);
	HAL_SPI_Transmit(&hspi3, &t, 1, 5000);
	//while(hspi3.Instance->SR  & SPI_SR_BSY);
	while(HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_BUSY_TX);
}

void st7735Init(void)
{
	setCS(0);
	HAL_Delay(100);

	// software reset
	sendCmd(0x01);
	HAL_Delay(100);

	// hardware reset
	setRESET(0);
    HAL_Delay(100);
    setRESET(1);
    HAL_Delay(100);

    // wake up
    sendCmd(0x11);
    HAL_Delay(100);

    // color mode 16bit
    sendCmd(0x3A);
    sendData(0x05);

    // direction and color
    sendCmd(0x36);
    sendData(0x14); // RGB
    //sendData(0x1C); // BGR

    sendCmd(0x29); // turn on display

    // setCS(1); ???
}

void st7735SetRect(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
	sendCmd(0x2A);
	sendData(0x00);
	sendData(x1);
	sendData(0x00);
	sendData(x2);

	sendCmd(0x2B);
	sendData(0x00);
	sendData(y1);
	sendData(0x00);
	sendData(y2);
}

void st7735FillRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint16_t color)
{
	if(width == 0)
		width = 1;
	if(height == 0)
		height = 1;

	st7735SetRect(x, y, x+width-1, y+height-1);
	sendCmd(0x2C);
	setA0(1);

	HAL_SPI_DeInit(&hspi3);
	hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
	HAL_SPI_Init(&hspi3);

	//while(HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_RESET);
	HAL_StatusTypeDef result = HAL_SPI_Transmit_DMA(&hspi3, (uint8_t*)(&color), 2*width*height);
	//while(hspi3.Instance->SR  & SPI_SR_BSY);
	while(HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_BUSY_TX);

	//HAL_Delay(100);

	HAL_SPI_DeInit(&hspi3);
	hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
	HAL_SPI_Init(&hspi3);
}

/********************************************************************
*
*       LcdWriteReg
*
* Function description:
*   Sets display register
*/
static void LcdWriteReg(U16 Data) {
  // ... TBD by user
	sendCmd(Data);
}

/********************************************************************
*
*       LcdWriteData
*
* Function description:
*   Writes a value to a display register
*/
static void LcdWriteData(U16 Data) {
  // ... TBD by user
	sendData(Data);
}

/********************************************************************
*
*       LcdWriteDataMultiple
*
* Function description:
*   Writes multiple values to a display register.
*/
static void LcdWriteDataMultiple(U16 * pData, int NumItems) {
  int i = 0;
  if(i==0){
	  i=1;
	  st7735FillRect(10,10,20,20,0x31);
  }
  /*while (NumItems--) {
    // ... TBD by user
	  sendData(&(pData[i++]));
  }*/
}

/********************************************************************
*
*       LcdReadDataMultiple
*
* Function description:
*   Reads multiple values from a display register.
*/
static void LcdReadDataMultiple(U16 * pData, int NumItems) {
  while (NumItems--) {
    // ... TBD by user
  }
}

/*********************************************************************
*
*       Public functions
*
**********************************************************************
*/
/*********************************************************************
*
*       LCD_X_Config
*
* Function description:
*   Called during the initialization process in order to set up the
*   display driver configuration.
*
*/
void LCD_X_Config(void) {
  GUI_DEVICE * pDevice;
  CONFIG_FLEXCOLOR Config = {0};
  GUI_PORT_API PortAPI = {0};
  //
  // Set display driver and color conversion
  //
  //pDevice = GUI_DEVICE_CreateAndLink(GUIDRV_FLEXCOLOR, GUICC_565, 0, 0);

  /* for RGB - GUICC_M565 , for BGR - GUICC_565*/
  pDevice = GUI_DEVICE_CreateAndLink(GUIDRV_FLEXCOLOR, GUICC_M565, 0, 0);

  //
  // Display driver configuration, required for Lin-driver
  //
  LCD_SetSizeEx (0, XSIZE_PHYS , YSIZE_PHYS);
  LCD_SetVSizeEx(0, VXSIZE_PHYS, VYSIZE_PHYS);
  //
  // Orientation
  //
  Config.Orientation = GUI_SWAP_XY | GUI_MIRROR_Y;
  GUIDRV_FlexColor_Config(pDevice, &Config);
  //
  // Set controller and operation mode
  //
  PortAPI.pfWrite16_A0  = LcdWriteReg;
  PortAPI.pfWrite16_A1  = LcdWriteData;
  PortAPI.pfWriteM16_A1 = LcdWriteDataMultiple;
  PortAPI.pfReadM16_A1  = LcdReadDataMultiple;
  //GUIDRV_FlexColor_SetFunc(pDevice, &PortAPI, GUIDRV_FLEXCOLOR_F66708, GUIDRV_FLEXCOLOR_M16C0B16);
  /* GUIDRV_FLEXCOLOR_F66709 - for st7735 */
  /* GUIDRV_FLEXCOLOR_M16C0B8 16bpp, no cache, 8 bit bus - SPI data bus */
  GUIDRV_FlexColor_SetFunc(pDevice, &PortAPI, GUIDRV_FLEXCOLOR_F66709, GUIDRV_FLEXCOLOR_M16C0B8);
}

/*********************************************************************
*
*       LCD_X_DisplayDriver
*
* Function description:
*   This function is called by the display driver for several purposes.
*   To support the according task the routine needs to be adapted to
*   the display controller. Please note that the commands marked with
*   'optional' are not cogently required and should only be adapted if
*   the display controller supports these features.
*
* Parameter:
*   LayerIndex - Index of layer to be configured
*   Cmd        - Please refer to the details in the switch statement below
*   pData      - Pointer to a LCD_X_DATA structure
*
* Return Value:
*   < -1 - Error
*     -1 - Command not handled
*      0 - Ok
*/
int LCD_X_DisplayDriver(unsigned LayerIndex, unsigned Cmd, void * pData) {
  int r;
  (void) LayerIndex;
  (void) pData;
  
  switch (Cmd) {
  case LCD_X_INITCONTROLLER: {
    //
    // Called during the initialization process in order to set up the
    // display controller and put it into operation. If the display
    // controller is not initialized by any external routine this needs
    // to be adapted by the customer...
    //
    // ...
	st7735Init();
    return 0;
  }
  default:
    r = -1;
  }
  return r;
}

/*************************** End of file ****************************/

