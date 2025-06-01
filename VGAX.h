/*
VGAX_RGB3 Library - Modified from original VGAX by Sandro Maffiodo
Adds support for 3-bit RGB (8 color) video output on Arduino UNO.

Original VGAX source: https://github.com/smaffer/vgax
Modified by: [Your Name]

Supports 120x60 pixels, 1 byte per pixel (only lower 3 bits used for RGB color).
*/

#ifndef VGAX_RGB3_H
#define VGAX_RGB3_H

#define VGAX_VERSION "RGB3-1.0.0"

#ifdef ARDUINO
#include <Arduino.h>
#endif

#define VGAX_WIDTH 120
#define VGAX_HEIGHT 60
#define VGAX_SIZE (VGAX_WIDTH * VGAX_HEIGHT)  // 7200 bytes

// Framebuffer: 1 pixel per byte (lower 3 bits used for color)
extern byte vgaxfb[VGAX_SIZE];

// A virtual timer that increments with VSYNC
extern unsigned long vtimer;

class VGAX {
public:
  static void begin(bool enableTone = true);
  static void end();

  // Set a pixel at (x, y) to a 3-bit color (0-7)
  static inline void putpixel(byte x, byte y, byte color) {
    if (x >= VGAX_WIDTH || y >= VGAX_HEIGHT) return;
    color &= 0b00000111;
    vgaxfb[y * VGAX_WIDTH + x] = color;
  }

  // Get the 3-bit color of the pixel at (x, y)
  static inline byte getpixel(byte x, byte y) {
    if (x >= VGAX_WIDTH || y >= VGAX_HEIGHT) return 0;
    return vgaxfb[y * VGAX_WIDTH + x] & 0b00000111;
  }

  // Fill entire screen with a single color
  static void clear(byte color);

  // Approximate millisecond counter (based on 60Hz vsync)
  static inline unsigned long millis() {
    return vtimer * 16;
  }

  // Approximate microsecond counter
  static inline unsigned long micros() {
    return vtimer * 16000;
  }

  // Sound functions (optional)
  static void tone(unsigned int frequency);
  static void noTone();
};

#endif
