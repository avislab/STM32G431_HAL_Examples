/*
 * ssd1315.c
 *
 *  Created on: 27 окт. 2025 г.
 *      Author: andrey
 */

#include "main.h"
#include "ssd1315.h"
#include <string.h>


void oled_send_cmd(SSD1315Struct *ssd1315, uint8_t cmd) {
    uint8_t buf[2] = {0x80, cmd};

	if (HAL_I2C_Master_Transmit(ssd1315->hi2c, (OLED_ADDR << 1), buf, 2, HAL_MAX_DELAY) != HAL_OK) {
		Error_Handler();
	}
}

void oled_send_buf(SSD1315Struct *ssd1315, uint8_t *buf, uint16_t buflen) {
    uint8_t temp_buf[buflen+1];

    for (int i = 1; i < buflen + 1; i++) {
        temp_buf[i] = buf[i - 1];
    }

    temp_buf[0] = 0x40;

	if (HAL_I2C_Master_Transmit(ssd1315->hi2c, (OLED_ADDR << 1), temp_buf, buflen+1, HAL_MAX_DELAY) != HAL_OK) {
		Error_Handler();
	}
}

void LCD_Init(SSD1315Struct *ssd1315, I2C_HandleTypeDef *hi2c, uint8_t Address) {
	ssd1315->hi2c = hi2c;
	ssd1315->Address = Address;

	oled_send_cmd(ssd1315, OLED_SET_DISP | 0x00); // Display Off
	oled_send_cmd(ssd1315, OLED_SET_DISP_CLK_DIV); // Set Display Clock Divide Ratio / Oscillator Frequency
	oled_send_cmd(ssd1315, 0x80); // Default value for display clock divide ratio
	oled_send_cmd(ssd1315, OLED_SET_MUX_RATIO); // Set Multiplex Ratio
	oled_send_cmd(ssd1315, 0x3F); // 1/64 Duty (for 64-pixel high display)
	oled_send_cmd(ssd1315, OLED_SET_DISP_OFFSET); // Set Display Offset
	oled_send_cmd(ssd1315, 0x00); // No offset
	oled_send_cmd(ssd1315, OLED_SET_DISP_START_LINE); // Set Display Start Line (COM0)
	oled_send_cmd(ssd1315, OLED_SET_CHARGE_PUMP); // Charge Pump Setting
	oled_send_cmd(ssd1315, 0x14); // Enable Charge Pump
	oled_send_cmd(ssd1315, OLED_SET_MEM_ADDR); // Set Memory Addressing Mode
	oled_send_cmd(ssd1315, 0x00); // Horizontal Addressing Mode
	oled_send_cmd(ssd1315, OLED_SET_SEG_REMAP | 0x01); // Set Segment Re-map (A0/A1 for normal/reversed)
	oled_send_cmd(ssd1315, OLED_SET_COM_OUT_DIR | 0x08); // Set COM Output Scan Direction (C0/C8 for normal/reversed)
    oled_send_cmd(ssd1315, OLED_SET_COM_PIN_CFG); // Set COM Pins Hardware Configuration
    oled_send_cmd(ssd1315, 0x12); // Sequential COM pins, Disable remap
    oled_send_cmd(ssd1315, OLED_SET_CONTRAST); // Set Contrast Control
    oled_send_cmd(ssd1315, 0xCF); // Default contrast (can be adjusted)
    oled_send_cmd(ssd1315, OLED_SET_PRECHARGE); // Set Pre-charge Period
    oled_send_cmd(ssd1315, 0xF1); // Default pre-charge period
    oled_send_cmd(ssd1315, OLED_SET_VCOM_DESEL); // Set VCOMH Deselect Level
    oled_send_cmd(ssd1315, 0x40); // Default VCOMH
    oled_send_cmd(ssd1315, OLED_SET_ENTIRE_ON); // Entire Display On/Off (0xA4 for normal, 0xA5 for entire display on)
    oled_send_cmd(ssd1315, OLED_SET_NORM_INV); // Set Normal/Inverse Display (0xA6 for normal, 0xA7 for inverse)
    oled_send_cmd(ssd1315, OLED_SET_DISP | 0x01); // Display On

}

void LCD_SetContrast(SSD1315Struct *ssd1315, uint8_t contrast) {
    oled_send_cmd(ssd1315, OLED_SET_CONTRAST);
    oled_send_cmd(ssd1315, contrast);
}

void LCD_Clear(SSD1315Struct *ssd1315) {
    LCD_Cache_Clear(ssd1315);
    LCD_Update(ssd1315);
}

void LCD_Cache_Clear(SSD1315Struct *ssd1315) {
    uint8_t page, column;
    for (page=0; page<LCD_NUM_PAGES; page++) {
       for (column=0; column<LCD_WIDTH; column++) {
    	   ssd1315->LCD_Cache[page][column] = 0x00;
        }
    }
}

void LCD_Update(SSD1315Struct *ssd1315) {
    oled_send_cmd(ssd1315, OLED_SET_COL_ADDR);
    oled_send_cmd(ssd1315, 0);
    oled_send_cmd(ssd1315, LCD_WIDTH-1);

    oled_send_cmd(ssd1315, OLED_SET_PAGE_ADDR);
    oled_send_cmd(ssd1315, 0);
    oled_send_cmd(ssd1315, LCD_NUM_PAGES-1);

    oled_send_buf(ssd1315, (uint8_t *)ssd1315->LCD_Cache, LCD_CACHE_SIZE);
}

//------------------------------

// Draw Pixel
void LCD_DrawPixel (SSD1315Struct *ssd1315, uint8_t x, uint8_t y, uint8_t color) {
	uint8_t temp = 0;
	uint8_t p = 0;

	// Out of range
	if ((x >= LCD_WIDTH) || (y >= LCD_HEIGHT)) {
		return;
    }

	p = y/8;

	temp = ssd1315->LCD_Cache[p][x];

	if (color == PIXEL_INVERT) {
		temp ^= (0x01<<(y-p*8));
	} else {
		if (color == PIXEL_ON) {
            temp |= (0x01<<(y-p*8));
        } else {
            temp &=~(0x01<<(y-p*8));
        }
	}

	ssd1315->LCD_Cache[p][x] = temp;
}

// Draw line
void LCD_DrawLine(SSD1315Struct *ssd1315, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, const uint8_t color) {
    int dx, dy, stepx, stepy, fraction;

    // dy   y2 - y1
    // -- = -------
    // dx   x2 - x1

    dy = y2 - y1;
    dx = x2 - x1;

    if (dy < 0)
    {
        dy    = -dy;
        stepy = -1;
    }
    else
    {
        stepy = 1;
    }

    if (dx < 0)
    {
        dx    = -dx;
        stepx = -1;
    }
    else
    {
        stepx = 1;
    }

    dx <<= 1;
    dy <<= 1;

    // Draw start point
    LCD_DrawPixel(ssd1315, x1, y1, color);

    // Draw next points
    if (dx > dy)
    {
        fraction = dy - ( dx >> 1);
        while (x1 != x2)
        {
            if (fraction >= 0)
            {
                y1 += stepy;
                fraction -= dx;
            }
            x1 += stepx;
            fraction += dy;

            LCD_DrawPixel(ssd1315, x1, y1, color);
        }
    }
    else
    {
        fraction = dx - ( dy >> 1);
        while (y1 != y2)
        {
            if (fraction >= 0)
            {
                x1 += stepx;
                fraction -= dy;
            }
            y1 += stepy;
            fraction += dx;

           LCD_DrawPixel(ssd1315, x1, y1, color);
        }
    }
}


// Draw Circle
void LCD_DrawCircle(SSD1315Struct *ssd1315, uint8_t x, uint8_t y, uint8_t radius, uint8_t color) {
    int8_t xc = 0;
    int8_t yc = 0;
    int8_t p = 0;

    // Out of range
	if ((x >= LCD_WIDTH) || (y >= LCD_HEIGHT)) {
		return;
    }

    yc = radius;
    p = 3 - (radius<<1);
    while (xc <= yc)
    {
        LCD_DrawPixel(ssd1315, x + xc, y + yc, color);
        LCD_DrawPixel(ssd1315, x + xc, y - yc, color);
        LCD_DrawPixel(ssd1315, x - xc, y + yc, color);
        LCD_DrawPixel(ssd1315, x - xc, y - yc, color);
        LCD_DrawPixel(ssd1315, x + yc, y + xc, color);
        LCD_DrawPixel(ssd1315, x + yc, y - xc, color);
        LCD_DrawPixel(ssd1315, x - yc, y + xc, color);
        LCD_DrawPixel(ssd1315, x - yc, y - xc, color);
        if (p < 0){
             p += (xc++ << 2) + 6;
        } else {
            p += ((xc++ - yc--)<<2) + 10;
        }
    }
}

// Draw Rect
void LCD_DrawRect(SSD1315Struct *ssd1315, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color) {
    uint8_t tmpIdx;

    // Out of range
    if ((x1 >= LCD_WIDTH) || (x2 >= LCD_WIDTH) || (y1 >= LCD_HEIGHT) || (y2 >= LCD_HEIGHT)) {
        return;
    }

    if ((x2 > x1) && (y2 > y1))
    {
        // Draw horizontal lines
        for (tmpIdx = x1; tmpIdx <= x2; tmpIdx++)
        {
            LCD_DrawPixel(ssd1315, tmpIdx, y1, color);
            LCD_DrawPixel(ssd1315, tmpIdx, y2, color);
        }

        // Draw vertical lines
        for (tmpIdx = y1; tmpIdx <= y2; tmpIdx++)
        {
            LCD_DrawPixel(ssd1315, x1, tmpIdx, color);
            LCD_DrawPixel(ssd1315, x2, tmpIdx, color);
        }
    }
}

//------------------------
void LCD_SetTextPos(SSD1315Struct *ssd1315, uint8_t x, uint8_t y) {
	ssd1315->LCD_Txt_X = x;
	ssd1315->LCD_Txt_Y = y;
}

void LCD_IncPos(SSD1315Struct *ssd1315, uint8_t fontSize) {
    ssd1315->LCD_Txt_X += fontSize;
    if ((ssd1315->LCD_Txt_X*6 /*+ fontSize*6*/) > LCD_WIDTH) {
        ssd1315->LCD_Txt_X = 0;
        ssd1315->LCD_Txt_Y += fontSize;
        if ((ssd1315->LCD_Txt_Y*8 /*+ fontSize*8*/) > LCD_HEIGHT) {
            ssd1315->LCD_Txt_Y = 0;
        }
    }
}

void LCD_Chr(SSD1315Struct *ssd1315, uint8_t ch, uint8_t fontSize) {
	uint8_t x, y, color;

	// Out of range
	//if ( (LCD_X + 6*size > LCD_X_RES) || ((LCD_Y + 8*size) > LCD_Y_RES) )
    //	return;

    if ( (ch >= 0x20) && (ch <= 0x7F) )
    {
        // offset in symbols table ASCII[0x20-0x7F]
        ch -= 32;
    }
    else if ( (ch >= 0xA5) && (ch <= 0xBF) ) {
        switch (ch)
        {
        case 0xA5:
            ch = 96;
            break;
        case 0xAA:
            ch = 97;
            break;
        case 0xAF:
            ch = 98;
            break;
        case 0xB2:
            ch = 99;
            break;

        case 0xB3:
            ch = 100;
            break;
        case 0xB4:
            ch = 101;
            break;
        case 0xBA:
            ch = 102;
            break;
        case 0xBF:
            ch = 103;
            break;

        default:
            break;
        }
    }
    else if (ch >= 0xC0)
    {
        // offset in symbols table CP1251[0xC0-0xFF] (Cyrillic)
        ch -= 88;
    }
    else
    {
        // Ignore unknown symbols
        ch = 31;
    }

    if (fontSize == 1)
    {
       for (x = 0; x < 5; x++) {
    	   ssd1315->LCD_Cache[ssd1315->LCD_Txt_Y][ssd1315->LCD_Txt_X*6 + x] = FontLookup[ch][x];
       }
    }
    else if (fontSize == 2)
    {
        // The Big numbers defined in BigNumbers array
        // Other symbols will scale
        if ((ch > 15) & (ch < 26)) {
          ch -= 16;
          for (x = 0; x < 10; x++)
          {
        	  ssd1315->LCD_Cache[ssd1315->LCD_Txt_Y][ssd1315->LCD_Txt_X*6 + x] = BigNumbers[ch][x];
        	  ssd1315->LCD_Cache[ssd1315->LCD_Txt_Y+1][ssd1315->LCD_Txt_X*6 + x] = BigNumbers[ch][x+10];
          }
        }
        else {
        	// Scale symbol image
            for (x = 0; x < 5; x++)
            {
            	for (y = 0; y < 8; y++) {
                    if ((FontLookup[ch][x] & (1 << y)) > 0) {
                        color = PIXEL_ON;
                    } else {
                        color = PIXEL_OFF;
                    }
            		LCD_DrawPixel(ssd1315, ssd1315->LCD_Txt_X*6 + x*2, ssd1315->LCD_Txt_Y*8 + y*2, color);
            		LCD_DrawPixel(ssd1315, ssd1315->LCD_Txt_X*6 + x*2, ssd1315->LCD_Txt_Y*8 + y*2+1, color);
            		LCD_DrawPixel(ssd1315, ssd1315->LCD_Txt_X*6 + x*2+1, ssd1315->LCD_Txt_Y*8 + y*2, color);
            		LCD_DrawPixel(ssd1315, ssd1315->LCD_Txt_X*6 + x*2+1, ssd1315->LCD_Txt_Y*8 + y*2+1, color);
            	}
            }
            if ((ch == 14) || (ch == 26)) { // . :
          		ssd1315->LCD_Txt_X -=1;
        	}
        }
    }
    else if (fontSize == 4) {
        // The LARGE numbers defined in LargeNumbers array
        // Other symbols will ignored
        // "+", "-" and "."
        if ((ch > 15) & (ch < 26)) {
          ch -= 16;
        }
        else if (ch == 43-32) { // +
          ch = 10;
        }
        else if (ch == 45-32) { // -
          ch = 11;
        }
        else if (ch == 46-32) { // .
          ch = 12;
        }
        else {
          ch= 255;
        }

        if (ch != 255) {
              for (x = 0; x < 20; x++)
              {
            	  ssd1315->LCD_Cache[ssd1315->LCD_Txt_Y][ssd1315->LCD_Txt_X*6 + x] = LargeNumbers[ch][x];
            	  ssd1315->LCD_Cache[ssd1315->LCD_Txt_Y+1][ssd1315->LCD_Txt_X*6 + x] = LargeNumbers[ch][20+x];
            	  ssd1315->LCD_Cache[ssd1315->LCD_Txt_Y+2][ssd1315->LCD_Txt_X*6 + x] = LargeNumbers[ch][40+x];
            	  ssd1315->LCD_Cache[ssd1315->LCD_Txt_Y+3][ssd1315->LCD_Txt_X*6 + x] = LargeNumbers[ch][60+x];
              }
        }

        if (ch == 12) { // .
          ssd1315->LCD_Txt_X -=1;
        }

    }

    LCD_IncPos(ssd1315, fontSize);
}

// Print a string to display
void LCD_Print(SSD1315Struct *ssd1315, uint8_t dataArray[], uint8_t fontSize) {
    uint8_t tmpIdx = 0;

    while( dataArray[ tmpIdx ] != '\0' )
    {
        if ((ssd1315->LCD_Txt_X*6 + fontSize*6) > LCD_WIDTH) {
            ssd1315->LCD_Txt_X = 0;
            ssd1315->LCD_Txt_Y += fontSize;
            if ((ssd1315->LCD_Txt_Y*8 + fontSize*8) > LCD_HEIGHT) {
                ssd1315->LCD_Txt_Y = 0;
            }
        }

        LCD_Chr(ssd1315, dataArray[ tmpIdx ], fontSize);
        tmpIdx++;
    }
}

// Invert fragment of string line
void LCD_IvertLineFragment(SSD1315Struct *ssd1315, uint8_t line, uint8_t chr_x1, uint8_t chr_x2) {
	uint8_t x, x1, x2;
	x1 = chr_x1*6;
	x2 = chr_x2*6;

	for (x=x1; x<x2; x++) {
		ssd1315->LCD_Cache[line][x] = ~ssd1315->LCD_Cache[line][x];
	}
}

// Invert string line
void LCD_IvertLine(SSD1315Struct *ssd1315, uint8_t line) {
	uint8_t x;

	for (x=0; x<LCD_WIDTH; x++){
		ssd1315->LCD_Cache[line][x] = ~ssd1315->LCD_Cache[line][x];
	}
}

// Draw Image
void LCD_Image(SSD1315Struct *ssd1315, uint8_t *imageData) {
	// Copy data to the cache
	memcpy(ssd1315->LCD_Cache, imageData, LCD_CACHE_SIZE);
}
