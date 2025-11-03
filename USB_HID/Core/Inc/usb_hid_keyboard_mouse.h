/*
 * usb_hid_keyboard_mouse.h
 *
 *  Created on: Oct 3, 2025
 *      Author: andre
 */

#ifndef INC_USB_HID_KEYBOARD_MOUSE_H_
#define INC_USB_HID_KEYBOARD_MOUSE_H_

typedef struct
{
  uint8_t  reportId;                                 // Report ID = 0x01 (1)
                                                     // Collection: CA:Keyboard
  uint8_t  KB_KeyboardKeyboardLeftControl : 1;       // Usage 0x000700E0: Keyboard Left Control, Value = 0 to 1
  uint8_t  KB_KeyboardKeyboardLeftShift : 1;         // Usage 0x000700E1: Keyboard Left Shift, Value = 0 to 1
  uint8_t  KB_KeyboardKeyboardLeftAlt : 1;           // Usage 0x000700E2: Keyboard Left Alt, Value = 0 to 1
  uint8_t  KB_KeyboardKeyboardLeftGui : 1;           // Usage 0x000700E3: Keyboard Left GUI, Value = 0 to 1
  uint8_t  KB_KeyboardKeyboardRightControl : 1;      // Usage 0x000700E4: Keyboard Right Control, Value = 0 to 1
  uint8_t  KB_KeyboardKeyboardRightShift : 1;        // Usage 0x000700E5: Keyboard Right Shift, Value = 0 to 1
  uint8_t  KB_KeyboardKeyboardRightAlt : 1;          // Usage 0x000700E6: Keyboard Right Alt, Value = 0 to 1
  uint8_t  KB_KeyboardKeyboardRightGui : 1;          // Usage 0x000700E7: Keyboard Right GUI, Value = 0 to 1
  uint8_t  pad_2;                                    // Pad
  uint8_t  Keyboard[2];                              // Value = 0 to 101
} KeyboardReport_t;

typedef struct
{
  uint8_t  reportId;                                 // Report ID = 0x02 (2)
                                                     // Collection: CA:Mouse CP:Pointer
  uint8_t  BTN_MousePointerButton1 : 1;              // Usage 0x00090001: Button 1 Primary/trigger, Value = 0 to 1
  uint8_t  BTN_MousePointerButton2 : 1;              // Usage 0x00090002: Button 2 Secondary, Value = 0 to 1
  uint8_t  BTN_MousePointerButton3 : 1;              // Usage 0x00090003: Button 3 Tertiary, Value = 0 to 1
  uint8_t  : 5;                                      // Pad
  int8_t   GD_MousePointerX;                         // Usage 0x00010030: X, Value = -127 to 127
  int8_t   GD_MousePointerY;                         // Usage 0x00010031: Y, Value = -127 to 127
  int8_t   GD_MousePointerWheel;                     // Usage 0x00010038: Wheel, Value = -127 to 127
} MouseReport_t;

void USB_HID_Keyboard_SendChar(char ch);
void USB_HID_Keyboard_SendString(char * s);
void USB_HID_Mouse_Move(int8_t x, int8_t y, int8_t wheel, uint8_t btn1, uint8_t btn2, uint8_t btn3);

#endif /* INC_USB_HID_KEYBOARD_MOUSE_H_ */
