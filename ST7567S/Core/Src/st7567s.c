/*
 * st7567s.c
 *
 *  Created on: Oct 1, 2024
 *      Author: andre
 */
#include "main.h"
#include "st7567s.h"
#include "string.h"


void st7567s_cmd(ST7567SStruct *st7567s, uint8_t cmd) {
	uint8_t i2c_tx[2];
	i2c_tx[0] = 0x00;
	i2c_tx[1] = cmd;
	HAL_I2C_Master_Transmit(st7567s->hi2c, ST7567S_ADR, (uint8_t*)i2c_tx, 2, 1000);
}

void st7567s_data(ST7567SStruct *st7567s, uint8_t data) {
	uint8_t i2c_tx[2];
	i2c_tx[0] = 0x40;
	i2c_tx[1] = data;
	HAL_I2C_Master_Transmit(st7567s->hi2c, ST7567S_ADR, (uint8_t*)i2c_tx, 2, 1000);
}

void lst7567s_clear(ST7567SStruct *st7567s) {
  for(int x=0; x<8; x++) {
	  st7567s_cmd(st7567s, 0xb0 + x); // y, page address y=1-0-1-1-y3-y2-y1-y0, 1-page with 8-rows

    // automatically increased by one
    st7567s_cmd(st7567s, 0x10); // x, column address x=0-0-0-0-1-x7-x6-x5-x4
    st7567s_cmd(st7567s, 0x00); // x, column address x=0-0-0-0-0-x3-x2-x1-x0

    for(int i=0; i<128; i++){
    	st7567s_data(st7567s, 0x00); // row=bit0--bit7
    }
  }
}

void LCD_Init(ST7567SStruct *st7567s, I2C_HandleTypeDef *hi2c, uint8_t Address) {
	st7567s->hi2c = hi2c;
	st7567s->Address = Address;

	st7567s_cmd(st7567s, 0xE2);    // Internal reset
	HAL_Delay(10);
	st7567s_cmd(st7567s, 0xA2);    // 0xa2 LCD bias ratio is 1/9; 0xa3 is bias ratio 1/7; 128X64=1/9

    // normal screen
	st7567s_cmd(st7567s, 0xA0);    // (0xa0-0xa1 is ADC direction selection) 0xa0 is segment forward, 0xa1 is segment reverse
	st7567s_cmd(st7567s, 0xC8);    // com output mode drive direction is reverse
	// flip screen
    //st7567s_cmd(st7567s, 0xA1);    // may be bug, loss first 4 bits in each row
    //st7567s_cmd(st7567s, 0xC0);

    //st7567s_cmd(st7567s, 0xA7);    // 0xA6 Normal, 0xA7 Inverse

    // Adjust display brightness
	st7567s_cmd(st7567s, 0x25);    // 0x20-0x27 is the internal Rb/Ra resistance adjustment setting of V5 voltage RR=4.5V
    //st750x3F67s_cmd(0x24);    // only 0x25 and 0x24 is ok
    st7567s_cmd(st7567s, 0x81);    // Electricity Mode Settings
    st7567s_cmd(st7567s, 0x20);

    st7567s_cmd(st7567s, 0x2C);    // Internal Power Supply Control Mode
    st7567s_cmd(st7567s, 0x2E);
    st7567s_cmd(st7567s, 0x2F);

    st7567s_cmd(st7567s, 0xAF);
    st7567s_cmd(st7567s, 0x40);
}

void LCD_Cache_Clear(ST7567SStruct *st7567s) {
    uint8_t page, column;
    for (page=0; page<LCD_NUM_PAGES; page++) {
       for (column=0; column<LCD_WIDTH; column++) {
    	   st7567s->LCD_Cache[page][column] = 0x00;
        }
    }
}

void LCD_Update(ST7567SStruct *st7567s) {
    uint8_t page;
    uint8_t column;

    for (page = 0; page < LCD_NUM_PAGES; page++) {
    	st7567s_cmd(st7567s, 0xb0 + page); //y, page address y=1-0-1-1-y3-y2-y1-y0, 1-page with 8-rows

        // automatically increased by one
        st7567s_cmd(st7567s, 0x10); //x, column address x=0-0-0-0-1-x7-x6-x5-x4
        st7567s_cmd(st7567s, 0x00); //x, column address x=0-0-0-0-0-x3-x2-x1-x0

        for(column=0; column<128; column++){
        	st7567s_data(st7567s, st7567s->LCD_Cache[page][column]); //row=bit0--bit7
        }
    }
}

void LCD_Clear(ST7567SStruct *st7567s) {
    LCD_Cache_Clear(st7567s);
    LCD_Update(st7567s);
}

//------------------------------

// Draw Pixel
void LCD_DrawPixel (ST7567SStruct *st7567s, uint8_t x, uint8_t y, uint8_t color) {
	uint8_t temp = 0;
	uint8_t p = 0;

	// Out of range
	if ((x >= LCD_WIDTH) || (y >= LCD_HEIGHT)) {
		return;
    }

	p = y/8;

	temp = st7567s->LCD_Cache[p][x];

	if (color == PIXEL_INVERT) {
		temp ^= (0x01<<(y-p*8));
	} else {
		if (color == PIXEL_ON) {
            temp |= (0x01<<(y-p*8));
        } else {
            temp &=~(0x01<<(y-p*8));
        }
	}

	st7567s->LCD_Cache[p][x] = temp;
}

// Draw line
void LCD_DrawLine(ST7567SStruct *st7567s, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, const uint8_t color) {
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
    LCD_DrawPixel(st7567s, x1, y1, color);

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

            LCD_DrawPixel(st7567s, x1, y1, color);
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

           LCD_DrawPixel(st7567s, x1, y1, color);
        }
    }
}


// Draw Circle
void LCD_DrawCircle(ST7567SStruct *st7567s, uint8_t x, uint8_t y, uint8_t radius, uint8_t color) {
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
        LCD_DrawPixel(st7567s, x + xc, y + yc, color);
        LCD_DrawPixel(st7567s, x + xc, y - yc, color);
        LCD_DrawPixel(st7567s, x - xc, y + yc, color);
        LCD_DrawPixel(st7567s, x - xc, y - yc, color);
        LCD_DrawPixel(st7567s, x + yc, y + xc, color);
        LCD_DrawPixel(st7567s, x + yc, y - xc, color);
        LCD_DrawPixel(st7567s, x - yc, y + xc, color);
        LCD_DrawPixel(st7567s, x - yc, y - xc, color);
        if (p < 0){
             p += (xc++ << 2) + 6;
        } else {
            p += ((xc++ - yc--)<<2) + 10;
        }
    }
}

// Draw Rect
void LCD_DrawRect(ST7567SStruct *st7567s, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color) {
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
            LCD_DrawPixel(st7567s, tmpIdx, y1, color);
            LCD_DrawPixel(st7567s, tmpIdx, y2, color);
        }

        // Draw vertical lines
        for (tmpIdx = y1; tmpIdx <= y2; tmpIdx++)
        {
            LCD_DrawPixel(st7567s, x1, tmpIdx, color);
            LCD_DrawPixel(st7567s, x2, tmpIdx, color);
        }
    }
}

//------------------------
void LCD_SetTextPos(ST7567SStruct *st7567s, uint8_t x, uint8_t y) {
	st7567s->LCD_Txt_X = x;
	st7567s->LCD_Txt_Y = y;
}

void LCD_IncPos(ST7567SStruct *st7567s, uint8_t fontSize) {
	st7567s->LCD_Txt_X += fontSize;
    if ((st7567s->LCD_Txt_X*6 /*+ fontSize*6*/) > LCD_WIDTH) {
    	st7567s->LCD_Txt_X = 0;
    	st7567s->LCD_Txt_Y += fontSize;
        if ((st7567s->LCD_Txt_Y*8 /*+ fontSize*8*/) > LCD_HEIGHT) {
        	st7567s->LCD_Txt_Y = 0;
        }
    }
}

void LCD_Chr(ST7567SStruct *st7567s, uint8_t ch, uint8_t fontSize) {
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
    	   st7567s->LCD_Cache[st7567s->LCD_Txt_Y][st7567s->LCD_Txt_X*6 + x] = FontLookup[ch][x];
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
        	  st7567s->LCD_Cache[st7567s->LCD_Txt_Y][st7567s->LCD_Txt_X*6 + x] = BigNumbers[ch][x];
        	  st7567s->LCD_Cache[st7567s->LCD_Txt_Y+1][st7567s->LCD_Txt_X*6 + x] = BigNumbers[ch][x+10];
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
            		LCD_DrawPixel(st7567s, st7567s->LCD_Txt_X*6 + x*2, st7567s->LCD_Txt_Y*8 + y*2, color);
            		LCD_DrawPixel(st7567s, st7567s->LCD_Txt_X*6 + x*2, st7567s->LCD_Txt_Y*8 + y*2+1, color);
            		LCD_DrawPixel(st7567s, st7567s->LCD_Txt_X*6 + x*2+1, st7567s->LCD_Txt_Y*8 + y*2, color);
            		LCD_DrawPixel(st7567s, st7567s->LCD_Txt_X*6 + x*2+1, st7567s->LCD_Txt_Y*8 + y*2+1, color);
            	}
            }
            if ((ch == 14) || (ch == 26)) { // . :
            	st7567s->LCD_Txt_X -=1;
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
            	  st7567s->LCD_Cache[st7567s->LCD_Txt_Y][st7567s->LCD_Txt_X*6 + x] = LargeNumbers[ch][x];
            	  st7567s->LCD_Cache[st7567s->LCD_Txt_Y+1][st7567s->LCD_Txt_X*6 + x] = LargeNumbers[ch][20+x];
            	  st7567s->LCD_Cache[st7567s->LCD_Txt_Y+2][st7567s->LCD_Txt_X*6 + x] = LargeNumbers[ch][40+x];
            	  st7567s->LCD_Cache[st7567s->LCD_Txt_Y+3][st7567s->LCD_Txt_X*6 + x] = LargeNumbers[ch][60+x];
              }
        }

        if (ch == 12) { // .
        	st7567s->LCD_Txt_X -=1;
        }

    }

    LCD_IncPos(st7567s, fontSize);
}

// Print a string to display
void LCD_Print(ST7567SStruct *st7567s, uint8_t dataArray[], uint8_t fontSize) {
    uint8_t tmpIdx = 0;

    while( dataArray[ tmpIdx ] != '\0' )
    {
        if ((st7567s->LCD_Txt_X*6 + fontSize*6) > LCD_WIDTH) {
        	st7567s->LCD_Txt_X = 0;
            st7567s->LCD_Txt_Y += fontSize;
            if ((st7567s->LCD_Txt_Y*8 + fontSize*8) > LCD_HEIGHT) {
            	st7567s->LCD_Txt_Y = 0;
            }
        }

        LCD_Chr(st7567s, dataArray[ tmpIdx ], fontSize);
        tmpIdx++;
    }
}

// Invert fragment of string line
void LCD_IvertLineFragment(ST7567SStruct *st7567s, uint8_t line, uint8_t chr_x1, uint8_t chr_x2) {
	uint8_t x, x1, x2;
	x1 = chr_x1*6;
	x2 = chr_x2*6;

	for (x=x1; x<x2; x++) {
		st7567s->LCD_Cache[line][x] = ~st7567s->LCD_Cache[line][x];
	}
}

// Invert string line
void LCD_IvertLine(ST7567SStruct *st7567s, uint8_t line) {
	uint8_t x;

	for (x=0; x<LCD_WIDTH; x++){
		st7567s->LCD_Cache[line][x] = ~st7567s->LCD_Cache[line][x];
	}
}

// Draw Image
void LCD_Image(ST7567SStruct *st7567s, uint8_t *imageData) {
	// Copy data to the cache
	memcpy(st7567s->LCD_Cache, imageData, LCD_CACHE_SIZE);
}
