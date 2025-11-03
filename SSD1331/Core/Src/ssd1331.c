#include "main.h"
#include "ssd1331.h"

void send_data(SSD1331Struct * ssd13321, uint8_t c) {
	HAL_SPI_Transmit(&ssd13321->hspi, &c, 1, 100);
}

void send_cmd(SSD1331Struct * ssd13321, uint8_t c)
{
	HAL_GPIO_WritePin(ssd13321->DCPort, ssd13321->DCTPin, GPIO_PIN_RESET); //DC
	HAL_GPIO_WritePin(ssd13321->CSPort, ssd13321->CSPin, GPIO_PIN_RESET); //CS
	send_data(ssd13321, c);
	HAL_GPIO_WritePin(ssd13321->CSPort, ssd13321->CSPin, GPIO_PIN_SET); //CS
}

void SSD1331_Init(SSD1331Struct * ssd13321)
{
	HAL_GPIO_WritePin(ssd13321->RSRPort, ssd13321->RSTPin, GPIO_PIN_RESET); //RES
	HAL_Delay(1);
	HAL_GPIO_WritePin(ssd13321->RSRPort, ssd13321->RSTPin, GPIO_PIN_SET); //RES

    send_cmd(ssd13321, CMD_DISPLAY_OFF);          //Display Off
    send_cmd(ssd13321, CMD_SET_CONTRAST_A);       //Set contrast for color A
    send_cmd(ssd13321, 0x91);                     //145 (0x91)
    send_cmd(ssd13321, CMD_SET_CONTRAST_B);       //Set contrast for color B
    send_cmd(ssd13321, 0x50);                     //80 (0x50)
    send_cmd(ssd13321, CMD_SET_CONTRAST_C);       //Set contrast for color C
    send_cmd(ssd13321, 0x7D);                     //125 (0x7D)
    send_cmd(ssd13321, CMD_MASTER_CURRENT_CONTROL);//master current control
    send_cmd(ssd13321, 0x06);                     //6
    send_cmd(ssd13321, CMD_SET_PRECHARGE_SPEED_A);//Set Second Pre-change Speed For ColorA
    send_cmd(ssd13321, 0x64);                     //100
    send_cmd(ssd13321, CMD_SET_PRECHARGE_SPEED_B);//Set Second Pre-change Speed For ColorB
    send_cmd(ssd13321, 0x78);                     //120
    send_cmd(ssd13321, CMD_SET_PRECHARGE_SPEED_C);//Set Second Pre-change Speed For ColorC
    send_cmd(ssd13321, 0x64);                     //100
    send_cmd(ssd13321, CMD_SET_REMAP);            //set remap & data format
    send_cmd(ssd13321, 0x72);                     //0x72
    send_cmd(ssd13321, CMD_SET_DISPLAY_START_LINE);//Set display Start Line
    send_cmd(ssd13321, 0x0);
    send_cmd(ssd13321, CMD_SET_DISPLAY_OFFSET);   //Set display offset
    send_cmd(ssd13321, 0x0);
    send_cmd(ssd13321, CMD_NORMAL_DISPLAY);       //Set display mode
    send_cmd(ssd13321, CMD_SET_MULTIPLEX_RATIO);  //Set multiplex ratio
    send_cmd(ssd13321, 0x3F);
    send_cmd(ssd13321, CMD_SET_MASTER_CONFIGURE); //Set master configuration
    send_cmd(ssd13321, 0x8E);
    send_cmd(ssd13321, CMD_POWER_SAVE_MODE);      //Set Power Save Mode
    send_cmd(ssd13321, 0x00);                     //0x00
    send_cmd(ssd13321, CMD_PHASE_PERIOD_ADJUSTMENT);//phase 1 and 2 period adjustment
    send_cmd(ssd13321, 0x31);                     //0x31
    send_cmd(ssd13321, CMD_DISPLAY_CLOCK_DIV);    //display clock divider/oscillator frequency
    send_cmd(ssd13321, 0xF0);
    send_cmd(ssd13321, CMD_SET_PRECHARGE_VOLTAGE);//Set Pre-Change Level
    send_cmd(ssd13321, 0x3A);
    send_cmd(ssd13321, CMD_SET_V_VOLTAGE);        //Set vcomH
    send_cmd(ssd13321, 0x3E);
    send_cmd(ssd13321, CMD_DEACTIVE_SCROLLING);   //disable scrolling
    send_cmd(ssd13321, CMD_NORMAL_BRIGHTNESS_DISPLAY_ON);//set display on
}

void SSD1331_DrawPixel(SSD1331Struct * ssd13321, uint16_t x, uint16_t y, uint16_t color)
{
    if ((x < 0) || (x >= RGB_OLED_WIDTH) || (y < 0) || (y >= RGB_OLED_HEIGHT))
        return;
    //set column point
    send_cmd(ssd13321, CMD_SET_COLUMN_ADDRESS);
    send_cmd(ssd13321, x);
    send_cmd(ssd13321, RGB_OLED_WIDTH-1);
    //set row point
    send_cmd(ssd13321, CMD_SET_ROW_ADDRESS);
    send_cmd(ssd13321, y);
    send_cmd(ssd13321, RGB_OLED_HEIGHT-1);

	HAL_GPIO_WritePin(ssd13321->DCPort, ssd13321->DCTPin, GPIO_PIN_SET); //DC
	HAL_GPIO_WritePin(ssd13321->CSPort, ssd13321->CSPin, GPIO_PIN_RESET); //CS

	send_data(ssd13321, color >> 8);
	send_data(ssd13321, color);

	HAL_GPIO_WritePin(ssd13321->CSPort, ssd13321->CSPin, GPIO_PIN_SET); //CS
}

void SSD1331_DrawLine(SSD1331Struct * ssd13321, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    if((x0 < 0) || (y0 < 0) || (x1 < 0) || (y1 < 0))
        return;

    if (x0 >= RGB_OLED_WIDTH)  x0 = RGB_OLED_WIDTH - 1;
    if (y0 >= RGB_OLED_HEIGHT) y0 = RGB_OLED_HEIGHT - 1;
    if (x1 >= RGB_OLED_WIDTH)  x1 = RGB_OLED_WIDTH - 1;
    if (y1 >= RGB_OLED_HEIGHT) y1 = RGB_OLED_HEIGHT - 1;

    send_cmd(ssd13321, CMD_DRAW_LINE);//draw line
    send_cmd(ssd13321, x0);//start column
    send_cmd(ssd13321, y0);//start row
    send_cmd(ssd13321, x1);//end column
    send_cmd(ssd13321, y1);//end row
    send_cmd(ssd13321, (uint8_t)((color>>11)&0x1F));//R
    send_cmd(ssd13321, (uint8_t)((color>>5)&0x3F));//G
    send_cmd(ssd13321, (uint8_t)(color&0x1F));//B
}

void SSD1331_DrawCircle(SSD1331Struct * ssd13321, uint16_t x, uint16_t y, uint16_t radius, uint16_t color) {
	signed char xc = 0;
	signed char yc = 0;
	signed char p = 0;

    // Out of range
    if (x >= RGB_OLED_WIDTH || y >= RGB_OLED_HEIGHT)
        return;

    yc = radius;
    p = 3 - (radius<<1);
    while (xc <= yc)
    {
    	SSD1331_DrawPixel(ssd13321, x + xc, y + yc, color);
    	SSD1331_DrawPixel(ssd13321, x + xc, y - yc, color);
    	SSD1331_DrawPixel(ssd13321, x - xc, y + yc, color);
    	SSD1331_DrawPixel(ssd13321, x - xc, y - yc, color);
    	SSD1331_DrawPixel(ssd13321, x + yc, y + xc, color);
    	SSD1331_DrawPixel(ssd13321, x + yc, y - xc, color);
    	SSD1331_DrawPixel(ssd13321, x - yc, y + xc, color);
    	SSD1331_DrawPixel(ssd13321, x - yc, y - xc, color);
        if (p < 0) p += (xc++ << 2) + 6;
            else p += ((xc++ - yc--)<<2) + 10;
    }

}

void SSD1331_DrawRect(SSD1331Struct * ssd13321, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t outColor, uint16_t fillColor)
{
    if((x0 < 0) || (y0 < 0) || (x1 < 0) || (y1 < 0))
        return;

    if (x0 >= RGB_OLED_WIDTH)  x0 = RGB_OLED_WIDTH - 1;
    if (y0 >= RGB_OLED_HEIGHT) y0 = RGB_OLED_HEIGHT - 1;
    if (x1 >= RGB_OLED_WIDTH)  x1 = RGB_OLED_WIDTH - 1;
    if (y1 >= RGB_OLED_HEIGHT) y1 = RGB_OLED_HEIGHT - 1;

    send_cmd(ssd13321, CMD_FILL_WINDOW);//fill window
    send_cmd(ssd13321, ENABLE_FILL);
    send_cmd(ssd13321, CMD_DRAW_RECTANGLE);//draw rectangle
    send_cmd(ssd13321, x0);//start column
    send_cmd(ssd13321, y0);//start row
    send_cmd(ssd13321, x1);//end column
    send_cmd(ssd13321, y1);//end row
    send_cmd(ssd13321, (uint8_t)((outColor>>11)&0x1F));//R
    send_cmd(ssd13321, (uint8_t)((outColor>>5)&0x3F));//G
    send_cmd(ssd13321, (uint8_t)(outColor&0x1F));//B
    send_cmd(ssd13321, (uint8_t)((fillColor>>11)&0x1F));//R
    send_cmd(ssd13321, (uint8_t)((fillColor>>5)&0x3F));//G
    send_cmd(ssd13321, (uint8_t)(fillColor&0x1F));//B
}

// Set current position in cache
void SSD1331_SetTextPos(SSD1331Struct * ssd13321, uint8_t x, uint8_t y) {
	ssd13321->CHR_X = x;
	ssd13321->CHR_Y = y;
}

void SSD1331_IncPos(SSD1331Struct * ssd13321, LcdFontSize size) {
	ssd13321->CHR_X += 6*size;
	if (ssd13321->CHR_X + 6*size > RGB_OLED_WIDTH) {
		ssd13321->CHR_X = 0;
		ssd13321->CHR_Y += 8*size;
		if (ssd13321->CHR_Y + 8*size > RGB_OLED_HEIGHT) {
			ssd13321->CHR_Y = 0;
		}
	}
}

void SSD1331_Chr(SSD1331Struct * ssd13321, LcdFontSize size, uint8_t ch, uint16_t chr_color, uint16_t bg_color) {
	uint8_t y, x, sx, sy;
	uint16_t color;
	/////uint16_t cx=CHR_X*6*size;
	uint16_t cx = ssd13321->CHR_X;
	/////uint16_t cy=CHR_Y*8*size;
	uint16_t cy = ssd13321->CHR_Y;

	if ( (cx + 6*size > RGB_OLED_WIDTH) || (cy + 8*size > RGB_OLED_HEIGHT) ) {
		return;
	}

	// CHR
    if ( (ch >= 0x20) && (ch <= 0x7F) )
    {
        // offset in symbols table ASCII[0x20-0x7F]
        ch -= 32;
    }
    else if (ch >= 0xC0)
    {
        // offset in symbols table CP1251[0xC0-0xFF] (Cyrillic)
        ch -= 96;
    }
    else
    {
        // Ignore unknown symbols
        ch = 95;
    }

    if ((size > FONT_1X) & (ch > 15) & (ch < 26)) {
        ch -= 16;
    	for (sy = 0; sy<size; sy++) {
    	for (y = 0; y<8; y++ ) {
    		//set column point
    		send_cmd(ssd13321, CMD_SET_COLUMN_ADDRESS);
    		send_cmd(ssd13321, cx);
    		send_cmd(ssd13321, RGB_OLED_WIDTH-1);
    		//set row point
    		send_cmd(ssd13321, CMD_SET_ROW_ADDRESS);
    		send_cmd(ssd13321, y + cy + sy*8);
    		send_cmd(ssd13321, RGB_OLED_HEIGHT-1);
    		HAL_GPIO_WritePin(ssd13321->DCPort, ssd13321->DCTPin, GPIO_PIN_SET); //dc
    		HAL_GPIO_WritePin(ssd13321->CSPort, ssd13321->CSPin, GPIO_PIN_RESET); //cs
    		for (x = 0; x < 5*size; x++ ) {
    			if ( (((BigNumbers[ch][x+sy*10] >> y) & 0x01 ) & (size == FONT_2X)) |
    				 (((LargeNumbers[ch][x+sy*20] >> y) & 0x01 ) & (size == FONT_4X))

    				) {
    				color = chr_color;
    			}
    			else {
    				color = bg_color;
    			}
				send_data(ssd13321, color >> 8);
				send_data(ssd13321, color);
    		}
    	}
    	}
    }
    else {
    	for (y = 0; y<8; y++ ) {
    		for (sy = 0; sy<size; sy++ ) {
    			//set column point
    			send_cmd(ssd13321, CMD_SET_COLUMN_ADDRESS);
    			send_cmd(ssd13321, cx);
    			send_cmd(ssd13321, RGB_OLED_WIDTH-1);
    			//set row point
    			send_cmd(ssd13321, CMD_SET_ROW_ADDRESS);
    			send_cmd(ssd13321, y*size + sy + cy);
    			send_cmd(ssd13321, RGB_OLED_HEIGHT-1);
    			HAL_GPIO_WritePin(ssd13321->DCPort, ssd13321->DCTPin, GPIO_PIN_SET); //DC
    			HAL_GPIO_WritePin(ssd13321->CSPort, ssd13321->CSPin, GPIO_PIN_RESET); //Cs
    			for (x = 0; x<5; x++ ) {
    				if ((FontLookup[ch][x] >> y) & 0x01) {
    					color = chr_color;
    				}
    				else {
    					color = bg_color;
    				}
    				//SSD1331_drawPixel(x+cx, y+cy, color);
    				for (sx = 0; sx<size; sx++ ) {
    					send_data(ssd13321, color >> 8);
    					send_data(ssd13321, color);
    				}
    			}
    			send_data(ssd13321, bg_color >> 8);
    			send_data(ssd13321, bg_color);
    			HAL_GPIO_WritePin(ssd13321->CSPort, ssd13321->CSPin, GPIO_PIN_SET); //CS
    		}
    	}
    }

    /////CHR_X++;
    //CHR_X += 6*size;
}

// Print a string to display
void SSD1331_Print(SSD1331Struct * ssd13321, LcdFontSize size, uint8_t dataArray[], uint16_t chr_color, uint16_t bg_color) {
    unsigned char tmpIdx=0;

    while( dataArray[ tmpIdx ] != '\0' )
    {
        /*/////
    	if (CHR_X > 15) {
        	CHR_X = 0;
        	CHR_Y++;
        	if (CHR_Y > 7) {
        		CHR_Y = 0;
        	}
        }
        */
    	/*
    	if (CHR_X + 6*size > RGB_OLED_WIDTH) {
        	CHR_X = 0;
        	CHR_Y += 8*size;
        	if (CHR_Y + 8*size > RGB_OLED_HEIGHT) {
        		CHR_Y = 0;
        	}
        }*/

        SSD1331_Chr(ssd13321, size, dataArray[ tmpIdx ], chr_color, bg_color);
        SSD1331_IncPos(ssd13321, size);
        tmpIdx++;
    }
}

// Print a string from the Flash to display
void SSD1331_FPrint(SSD1331Struct * ssd13321, LcdFontSize size, const uint8_t *dataPtr, uint16_t chr_color, uint16_t bg_color) {
	uint8_t c;
    for (c = *( dataPtr ); c; ++dataPtr, c = *( dataPtr ))
    {
        /*
    	if (CHR_X > 15) {
        	CHR_X = 0;
        	CHR_Y++;
        	if (CHR_Y > 7) {
        		CHR_Y = 0;
        	}
        }
        */

        SSD1331_Chr(ssd13321, size, c, chr_color, bg_color);
        SSD1331_IncPos(ssd13321, size);
    }
}

void SSD1331_Image(SSD1331Struct * ssd13321, const uint8_t *img, uint16_t x, uint16_t y, uint16_t width, uint16_t height) {
	uint16_t xx, yy;

	if ( (x + width > RGB_OLED_WIDTH) | (y+height > RGB_OLED_HEIGHT) ){
		return;
	}

	for (yy=0; yy<height; yy++) {
		//set column point
		send_cmd(ssd13321, CMD_SET_COLUMN_ADDRESS);
		send_cmd(ssd13321, x);
		send_cmd(ssd13321, RGB_OLED_WIDTH-1);
		//set row point
		send_cmd(ssd13321, CMD_SET_ROW_ADDRESS);
		send_cmd(ssd13321, y + yy);
		send_cmd(ssd13321, RGB_OLED_HEIGHT-1);
		HAL_GPIO_WritePin(ssd13321->DCPort, ssd13321->DCTPin, GPIO_PIN_SET); //DC
		HAL_GPIO_WritePin(ssd13321->CSPort, ssd13321->CSPin, GPIO_PIN_RESET); //CS

		for (xx=0; xx<width*2; xx++) {
			send_data(ssd13321, img[yy*width*2 + xx]);
		}
	}
}

void SSD1331_CopyWindow(SSD1331Struct * ssd13321, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,uint16_t x2, uint16_t y2)
{
    send_cmd(ssd13321, CMD_COPY_WINDOW);//copy window
    send_cmd(ssd13321, x0);//start column
    send_cmd(ssd13321, y0);//start row
    send_cmd(ssd13321, x1);//end column
    send_cmd(ssd13321, y1);//end row
    send_cmd(ssd13321, x2);//new column
    send_cmd(ssd13321, y2);//new row
}

void SSD1331_DimWindow(SSD1331Struct * ssd13321, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    send_cmd(ssd13321, CMD_DIM_WINDOW);//copy area
    send_cmd(ssd13321, x0);//start column
    send_cmd(ssd13321, y0);//start row
    send_cmd(ssd13321, x1);//end column
    send_cmd(ssd13321, y1);//end row
}

void SSD1331_ClearWindow(SSD1331Struct * ssd13321, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    send_cmd(ssd13321, CMD_CLEAR_WINDOW);//clear window
    send_cmd(ssd13321, x0);//start column
    send_cmd(ssd13321, y0);//start row
    send_cmd(ssd13321, x1);//end column
    send_cmd(ssd13321, y1);//end row
}

void SSD1331_SetScrolling(SSD1331Struct * ssd13321, ScollingDirection direction, uint8_t rowAddr, uint8_t rowNum, uint8_t timeInterval)
{
    uint8_t scolling_horizontal = 0x0;
    uint8_t scolling_vertical = 0x0;
    switch(direction){
        case Horizontal:
            scolling_horizontal = 0x01;
            scolling_vertical = 0x00;
            break;
        case Vertical:
            scolling_horizontal = 0x00;
            scolling_vertical = 0x01;
            break;
        case Diagonal:
            scolling_horizontal = 0x01;
            scolling_vertical = 0x01;
            break;
        default:
            break;
    }
    send_cmd(ssd13321, CMD_CONTINUOUS_SCROLLING_SETUP);
    send_cmd(ssd13321, scolling_horizontal);
    send_cmd(ssd13321, rowAddr);
    send_cmd(ssd13321, rowNum);
    send_cmd(ssd13321, scolling_vertical);
    send_cmd(ssd13321, timeInterval);
    send_cmd(ssd13321, CMD_ACTIVE_SCROLLING);
}

void SSD1331_EnableScrolling(SSD1331Struct * ssd13321, bool enable)
{
    if(enable)
        send_cmd(ssd13321, CMD_ACTIVE_SCROLLING);
    else
        send_cmd(ssd13321, CMD_DEACTIVE_SCROLLING);
}

void SSD1331_SetDisplayMode(SSD1331Struct * ssd13321, DisplayMode mode)
{
    send_cmd(ssd13321, mode);
}

void SSD1331_SetDisplayPower(SSD1331Struct * ssd13321, DisplayPower power)
{
    send_cmd(ssd13321, power);
}
