/*
 * usb_hid_keyboard_mouse.c
 *
 *  Created on: Oct 3, 2025
 *      Author: andre
 */

#include "main.h"
#include "usb_device.h"
#include "usb_hid_keyboard_mouse.h"


extern USBD_HandleTypeDef hUsbDeviceFS;
KeyboardReport_t keyboard_report;
MouseReport_t mouse_report;

extern uint8_t USBD_CUSTOM_HID_SendReport(USBD_HandleTypeDef  *pdev, uint8_t *report, uint16_t len);

static void USB_HID_charToCode(char ch) {
	// Check if lower or upper case
	if(ch >= 'a' && ch <= 'z')
	{
		keyboard_report.KB_KeyboardKeyboardLeftShift = 0;
		// Convert ch to HID letter, starting at a = 4
		keyboard_report.Keyboard[0] = (uint8_t)(4 + (ch - 'a'));
	}
	else if(ch >= 'A' && ch <= 'Z')
	{
		// Add left shift
		keyboard_report.KB_KeyboardKeyboardLeftShift = 1;
		// Convert ch to lower case
		ch = ch - ('A'-'a');
		// Convert ch to HID letter, starting at a = 4
		keyboard_report.Keyboard[0] = (uint8_t)(4 + (ch - 'a'));
	}
	else if(ch >= '0' && ch <= '9') // Check if number
	{
		keyboard_report.KB_KeyboardKeyboardLeftShift = 0;
		// Convert ch to HID number, starting at 1 = 30, 0 = 39
		if(ch == '0')
		{
			keyboard_report.Keyboard[0] = 39;
		}
		else
		{
			keyboard_report.Keyboard[0] = (uint8_t)(30 + (ch - '1'));
		}
	}
	else // Not a letter nor a number
	{
		switch(ch)
		{
			case ' ':
				keyboard_report.KB_KeyboardKeyboardLeftShift = 0;
				keyboard_report.Keyboard[0] = 44;
				break;
			case '.':
				keyboard_report.KB_KeyboardKeyboardLeftShift = 0;
				keyboard_report.Keyboard[0] = 55;
				break;
			case '\n':
				keyboard_report.KB_KeyboardKeyboardLeftShift = 0;
				keyboard_report.Keyboard[0] = 40;
				break;
			case '!':
				//combination of shift modifier and key
				keyboard_report.KB_KeyboardKeyboardLeftShift = 1;	// shift
				keyboard_report.Keyboard[0] = 30; // number 1
				break;
			case '?':
				//combination of shift modifier and key
				keyboard_report.KB_KeyboardKeyboardLeftShift = 1;	// shift
				keyboard_report.Keyboard[0] = 0x38; // key '/'
				break;
			case '@':
				//combination of shift modifier and key
				keyboard_report.KB_KeyboardKeyboardLeftShift = 1;	// shift
				keyboard_report.Keyboard[0] = 31; // number 2
				break;
			default:
				keyboard_report.KB_KeyboardKeyboardLeftShift = 0;
				keyboard_report.Keyboard[0] = 0;
		}
	}
}


// Send character as a single key press
void USB_HID_Keyboard_SendChar(char ch) {
	keyboard_report.reportId = 0x01;

	// Convert Char to USB code
	USB_HID_charToCode(ch);

	// Press keys
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &keyboard_report, sizeof(KeyboardReport_t));
	HAL_Delay(20);

	// Release keys
	keyboard_report.KB_KeyboardKeyboardLeftShift = 0;
	keyboard_report.Keyboard[0] = 0x00;

	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &keyboard_report, sizeof(KeyboardReport_t));
	HAL_Delay(20);
}


// Send string as letters
void USB_HID_Keyboard_SendString(char * s) {

	uint8_t i = 0;

	while(*(s+i))
	{
		USB_HID_Keyboard_SendChar(*(s+i));
		i++;
	}
}

// Mouse move
void USB_HID_Mouse_Move(int8_t x, int8_t y, int8_t wheel, uint8_t btn1, uint8_t btn2, uint8_t btn3) {
	mouse_report.reportId = 0x02;

	mouse_report.GD_MousePointerX = x;
	mouse_report.GD_MousePointerY = y;
	mouse_report.GD_MousePointerWheel = wheel;

	mouse_report.BTN_MousePointerButton1 = btn1;
	mouse_report.BTN_MousePointerButton2 = btn2;
	mouse_report.BTN_MousePointerButton3 = btn3;

	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &mouse_report, sizeof(MouseReport_t));
	HAL_Delay(20);
}

