#include "main.h"
#include "n3310.h"

// Send data or command to display
static void _lcd_send(N3310Struct *n3310, uint8_t data, LcdCmdData cd) {
  // Enable display controller
	RESET_CE(n3310);

  if (cd == LCD_DATA) {
	  SET_DC(n3310);
  }
  else {
	  RESET_DC(n3310);
  }

  // Send data to display (programm SPI)
  for(unsigned int i = 0; i< 8; i++, data = data << 1) {
      if ((data & 0x80) == 0x80) {
    	  SET_SDIN(n3310);
      }
      else {
        RESET_SDIN(n3310);
      }

      SET_SCLK(n3310);
      RESET_SCLK(n3310);
  }

  // Disable display controller
  SET_CE(n3310);
}

// Display Initialization
void LCD_Init(N3310Struct *n3310) {
	SET_RST(n3310);
	RESET_DC(n3310);
	RESET_CE(n3310);
	RESET_SDIN(n3310);
	RESET_SCLK(n3310);

    // Reset
	RESET_RST(n3310);
	SET_RST(n3310);

    // Disable LCD controller
	SET_CE(n3310);
  
    // Send commands
    _lcd_send(n3310, 0x21, LCD_CMD); // LCD Extended Commands
    _lcd_send(n3310, 0xC8, LCD_CMD); // Set Contrast (LCD Vop)
    _lcd_send(n3310, 0x06, LCD_CMD); // Set Temperature coefficent
    _lcd_send(n3310, 0x13, LCD_CMD); // LCD bias mode 1:48
    _lcd_send(n3310, 0x20, LCD_CMD); // LCD Standard Commands and Horizontal addressing mode
    _lcd_send(n3310, 0x0C, LCD_CMD); // LCD in normal mode

    // Clear Display
    LCD_Clear(n3310);
    LCD_Update(n3310);
}

// Update LCD. Upload Cache to display
void LCD_Update(N3310Struct *n3310) {
    int i;

    if (n3310->BottomCacheMark < 0)
    	n3310->BottomCacheMark = 0;
    else if (n3310->BottomCacheMark >= LCD_CACHE_SIZE)
    	n3310->BottomCacheMark = LCD_CACHE_SIZE - 1;

    if (n3310->TopCacheMark < 0)
    	n3310->TopCacheMark = 0;
    else if (n3310->TopCacheMark >= LCD_CACHE_SIZE)
    	n3310->TopCacheMark = LCD_CACHE_SIZE - 1;

    // Set start address
    _lcd_send(n3310, 0x80 | (n3310->BottomCacheMark % LCD_X_RES), LCD_CMD);
    _lcd_send(n3310, 0x40 | (n3310->BottomCacheMark / LCD_X_RES), LCD_CMD);

    // Refresh display
    for (i = n3310->BottomCacheMark; i <= n3310->TopCacheMark; i++)
    {
    	_lcd_send(n3310, n3310->LcdCache[i], LCD_DATA);
    }

    // Reset Cache Marks
    n3310->BottomCacheMark = LCD_CACHE_SIZE - 1;
    n3310->TopCacheMark = 0;
}

// Clear Display
void LCD_Clear(N3310Struct *n3310) {
	// Clear cache
    memset(n3310->LcdCache, 0x00, LCD_CACHE_SIZE);

    // Reset Cache marks
    n3310->BottomCacheMark = 0;
    n3310->TopCacheMark = LCD_CACHE_SIZE - 1;
}

// Copy data to cache
void LCD_WriteToCache(N3310Struct *n3310, int addr, uint8_t data) {
	n3310->LcdCache[addr] = data;
}

// Set contrast 0x00 ... 0x7F
void LCD_Contrast(N3310Struct *n3310, uint8_t contrast) {
    _lcd_send(n3310, 0x21, LCD_CMD);              // LCD Extended Commands
    _lcd_send(n3310, 0x80 | contrast, LCD_CMD);   // Set contrast
    _lcd_send(n3310, 0x20, LCD_CMD);              // LCD Standard Commands
}

//------------------------------

// Draw pixel
uint8_t LCD_DrawPixel(N3310Struct *n3310, uint8_t x, uint8_t y, LcdPixelMode mode) {
    int  index;
    uint8_t  offset;
    uint8_t  data;

    if (x >= LCD_X_RES || y >= LCD_Y_RES) return OUT_OF_BORDER;

    index = ( ( y / 8 ) * 84 ) + x;
    offset  = y - ( ( y / 8 ) * 8 );

    data = n3310->LcdCache[ index ];

    // PIXEL_OFF
    if (mode == PIXEL_OFF)
    {
        data &= ( ~( 0x01 << offset ) );
    }
    // PIXEL_ON
    else if (mode == PIXEL_ON)
    {
        data |= ( 0x01 << offset );
    }
    // PIXEL_XOR
    else if (mode  == PIXEL_XOR)
    {
        data ^= ( 0x01 << offset );
    }

    // Copy result to the cache
    n3310->LcdCache[index] = data;

    if (index < n3310->BottomCacheMark)
    {
        // Set new bottom Cache Mark
    	n3310->BottomCacheMark = index;
    }

    if (index > n3310->TopCacheMark)
    {
    	// Set new top Cache Mark
    	n3310->TopCacheMark = index;
    }
    return OK;
}

// Draw line
uint8_t LCD_DrawLine(N3310Struct *n3310, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, LcdPixelMode mode) {
    int dx, dy, stepx, stepy, fraction;
    uint8_t response;

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
    response = LCD_DrawPixel(n3310, x1, y1, mode);
    if (response)
        return response;

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

            response = LCD_DrawPixel(n3310, x1, y1, mode);
            if(response)
                return response;

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

            response = LCD_DrawPixel(n3310, x1, y1, mode);
            if (response)
                return response;
        }
    }

    return OK;
}

// Draw Circle
uint8_t LCD_DrawCircle(N3310Struct *n3310, uint8_t x, uint8_t y, uint8_t radius, LcdPixelMode mode) {
	int8_t xc = 0;
	int8_t yc = 0;
	int8_t p = 0;

    if (x >= LCD_X_RES || y >= LCD_Y_RES) return OUT_OF_BORDER;

    yc = radius;
    p = 3 - (radius<<1);
    while (xc <= yc)
    {
    	LCD_DrawPixel(n3310, x + xc, y + yc, mode);
    	LCD_DrawPixel(n3310, x + xc, y - yc, mode);
    	LCD_DrawPixel(n3310, x - xc, y + yc, mode);
    	LCD_DrawPixel(n3310, x - xc, y - yc, mode);
    	LCD_DrawPixel(n3310, x + yc, y + xc, mode);
    	LCD_DrawPixel(n3310, x + yc, y - xc, mode);
    	LCD_DrawPixel(n3310, x - yc, y + xc, mode);
    	LCD_DrawPixel(n3310, x - yc, y - xc, mode);
        if (p < 0) p += (xc++ << 2) + 6;
            else p += ((xc++ - yc--)<<2) + 10;
    }

    return OK;
}

// Draw Rect
uint8_t LCD_DrawRect(N3310Struct *n3310, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, LcdPixelMode mode) {
	uint8_t tmpIdx;

    // Return if border offsets
    if ((x1 >= LCD_X_RES) ||  (x2 >= LCD_X_RES) || (y1 >= LCD_Y_RES) || (y2 >= LCD_Y_RES))
        return OUT_OF_BORDER;

    if ((x2 > x1) && (y2 > y1))
    {
        // Draw horizontal lines
        for (tmpIdx = x1; tmpIdx <= x2; tmpIdx++)
        {
        	LCD_DrawPixel(n3310, tmpIdx, y1, mode);
        	LCD_DrawPixel(n3310, tmpIdx, y2, mode);
        }

        // Draw vertical lines
        for (tmpIdx = y1; tmpIdx <= y2; tmpIdx++)
        {
        	LCD_DrawPixel(n3310, x1, tmpIdx, mode);
        	LCD_DrawPixel(n3310, x2, tmpIdx, mode);
        }
    }
    return OK;
}

//------------------------

// Set cursor position (standard font size) x = 0...13, y = 0...5
uint8_t LCD_SetTextPos(N3310Struct *n3310, uint8_t x, uint8_t y) {
    if(x > 13 || y > 5) return OUT_OF_BORDER;

    n3310->LcdCacheIdx = x * 6 + y * 84;
    return OK;
}

// Print a single char into current position
uint8_t LCD_Chr(N3310Struct *n3310, uint8_t ch, LcdFontSize size) {
	uint8_t i, c;
	uint8_t b1, b2;
    int  tmpIdx;

    if (n3310->LcdCacheIdx < n3310->BottomCacheMark)
    {
        // Set bottom Cache Mark
    	n3310->BottomCacheMark = n3310->LcdCacheIdx;
    }

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

    if (size == FONT_1X)
    {
        for (i = 0; i < 5; i++)
        {
            // Copy symbol image into cache
        	n3310->LcdCache[n3310->LcdCacheIdx++] = *( &(FontLookup[ch][i]) ) << 1;
        }
    }
    else if (size == FONT_2X)
    {
        tmpIdx = n3310->LcdCacheIdx - 84;

        if (tmpIdx < n3310->BottomCacheMark)
        {
        	n3310->BottomCacheMark = tmpIdx;
        }

        if (tmpIdx < 0) return OUT_OF_BORDER;

        // The Big numbers defined in BigNumbers array
        // Other symbols will scale
        if ((ch > 15) & (ch < 26)) {
          ch -= 16;
          for (i = 0; i < 10; i++)
          {
        	  n3310->LcdCache[tmpIdx++] = *(&(BigNumbers[ch][i]));
        	  n3310->LcdCache[tmpIdx+83] = *(&(BigNumbers[ch][10+i]));
          }
        }
        else{
          for (i = 0; i < 5; i++)
          {
              // Copy symbol image from the table to the temporart variable
              c = *(&(FontLookup[ch][i])) << 1;
              // Scale image
              // First part
              b1 =  (c & 0x01) * 3;
              b1 |= (c & 0x02) * 6;
              b1 |= (c & 0x04) * 12;
              b1 |= (c & 0x08) * 24;

              c >>= 4;
              // Second part
              b2 =  (c & 0x01) * 3;
              b2 |= (c & 0x02) * 6;
              b2 |= (c & 0x04) * 12;
              b2 |= (c & 0x08) * 24;

              // Copy to the cache
              n3310->LcdCache[tmpIdx++] = b1;
              n3310->LcdCache[tmpIdx++] = b1;
              n3310->LcdCache[tmpIdx + 82] = b2;
              n3310->LcdCache[tmpIdx + 83] = b2;
          }
        }

        // Set new cursor position
        n3310->LcdCacheIdx = (n3310->LcdCacheIdx + 11) % LCD_CACHE_SIZE;
    }
    else if (size == FONT_4X) {
        tmpIdx = n3310->LcdCacheIdx - 84;

        if (tmpIdx < n3310->BottomCacheMark)
        {
        	n3310->BottomCacheMark = tmpIdx;
        }

        if (tmpIdx < 0) return OUT_OF_BORDER;
        
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
          for (i = 0; i < 20; i++)
          {
        	  n3310->LcdCache[tmpIdx++] = *(&(LargeNumbers[ch][i]));
        	  n3310->LcdCache[tmpIdx+83] = *(&(LargeNumbers[ch][20+i]));
        	  n3310->LcdCache[tmpIdx+167] = *(&(LargeNumbers[ch][40+i]));
        	  n3310->LcdCache[tmpIdx+251] = *(&(LargeNumbers[ch][60+i]));
          }
        }

        // Set new cursor position
        n3310->LcdCacheIdx = (n3310->LcdCacheIdx + 20) % LCD_CACHE_SIZE;
        
        if (ch == 12) { // .
        	n3310->LcdCacheIdx -=5;
        }
    }

    if (n3310->LcdCacheIdx > n3310->TopCacheMark)
    {
        // Set top Cache Mark
    	n3310->TopCacheMark = n3310->LcdCacheIdx;
    }

    // Horizontal space between chars
    n3310->LcdCache[n3310->LcdCacheIdx] = 0x00;

    if(n3310->LcdCacheIdx == (LCD_CACHE_SIZE - 1))
    {
    	n3310->LcdCacheIdx = 0;
        return OK_WITH_WRAP;
    }

    n3310->LcdCacheIdx++;
    return OK;
}

// Print a string to display
uint8_t LCD_Print(N3310Struct *n3310, uint8_t dataArray[], LcdFontSize size) {
	uint8_t tmpIdx=0;
	uint8_t response;
    while( dataArray[ tmpIdx ] != '\0' )
    {
        response = LCD_Chr(n3310, dataArray[ tmpIdx ], size);
        if( response == OUT_OF_BORDER)
            return OUT_OF_BORDER;
        tmpIdx++;
    }
    return OK;
}

// Invert fragment of string line
uint8_t  LCD_IvertLineFragment(N3310Struct *n3310, uint8_t line, uint8_t chr_x1, uint8_t chr_x2) {
  uint16_t addr, addr_start, addr_end;
  addr_start = line*LCD_X_RES + chr_x1*6;
  addr_end = line*LCD_X_RES + chr_x2*6;

  for (addr=addr_start; addr<addr_end; addr++) {
	  n3310->LcdCache[addr] = ~n3310->LcdCache[addr];
  }
  return OK;
}

// Invert string line
uint8_t  LCD_IvertLine(N3310Struct *n3310, uint8_t line) {
  uint8_t x;
  uint16_t addr;
  
  addr=line*LCD_X_RES;
  for (x=0; x<LCD_X_RES; x++) {
	  n3310->LcdCache[addr] = ~n3310->LcdCache[addr];
      addr++;
  }
  return OK;
}

// Draw Image
uint8_t LCD_Image(N3310Struct *n3310, const uint8_t *imageData) {
    // Copy data to the cache
    memcpy(n3310->LcdCache, imageData, LCD_CACHE_SIZE);

    n3310->BottomCacheMark = 0;
    n3310->TopCacheMark = LCD_CACHE_SIZE - 1;
    return OK;
}

