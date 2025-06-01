#ifndef __VGAX_RGB3_LIBRARY__
#define __VGAX_RGB3_LIBRARY__

#include <Arduino.h>

#define VGAX_WIDTH 120
#define VGAX_HEIGHT 60

// 3 bits per pixel => 120 pixels * 3 bits = 360 bits per line => 45 bytes per line
#define VGAX_BWIDTH 45
#define VGAX_BSIZE (VGAX_BWIDTH * VGAX_HEIGHT)

extern volatile uint8_t vgaxfb[VGAX_BSIZE]; // framebuffer

// Pins for RGB output (Arduino UNO PORTD)
#define PIN_BLUE  5 // PD5
#define PIN_RED   6 // PD6
#define PIN_GREEN 7 // PD7

#define COLOR_BLACK   0b000
#define COLOR_BLUE    0b001
#define COLOR_RED     0b010
#define COLOR_MAGENTA 0b011
#define COLOR_GREEN   0b100
#define COLOR_CYAN    0b101
#define COLOR_YELLOW  0b110
#define COLOR_WHITE   0b111

class VGAX_RGB3 {
public:
  static void begin(bool enableTone = true);
  static void end();

  static void clear(uint8_t color = COLOR_BLACK);

  static void putpixel(uint8_t x, uint8_t y, uint8_t color);
  static uint8_t getpixel(uint8_t x, uint8_t y);

  static void tone(unsigned int frequency);
  static void noTone();

  static void delay(int msec);

  static unsigned long millis();
};

#endif
