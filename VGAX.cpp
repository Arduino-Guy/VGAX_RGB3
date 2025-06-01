/*
VGAX_RGB3.cpp - 3-bit RGB VGA video output for Arduino UNO
Modified from the original VGAX library by Sandro Maffiodo
*/

#include "VGAX_RGB3.h"
#include <avr/interrupt.h>

// 3-bit RGB framebuffer: 1 byte per pixel (lower 3 bits used)
byte vgaxfb[VGAX_SIZE];

// Virtual timer (increments once per frame, ~60Hz)
unsigned long vtimer = 0;

void VGAX::begin(bool enableTone) {
  // Set VGA sync and color pins as output
  // Adjust as needed for your resistor DAC setup
  DDRD |= _BV(6) | _BV(5) | _BV(3); // PD6=R, PD5=G, PD3=B
  DDRD |= _BV(7); // PD7 = HSYNC
  DDRB |= _BV(0); // PB0 = VSYNC

  // Timer1: HSYNC generator
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS10); // CTC, no prescale
  OCR1A = 1016; // ~15.7kHz for HSYNC
  TIMSK1 = _BV(OCIE1A); // Enable interrupt

  // Timer2: Pixel clock (fast PWM for output)
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); // Fast PWM
  TCCR2B = _BV(CS20); // No prescale
  OCR2B = 0;

  sei(); // Enable interrupts
}

void VGAX::end() {
  // Stop interrupts and timers
  TIMSK1 = 0;
  TCCR1A = TCCR1B = 0;
  TCCR2A = TCCR2B = 0;
  cli();
}

// Clear framebuffer with a 3-bit color
void VGAX::clear(byte color) {
  color &= 0b00000111;
  for (int i = 0; i < VGAX_SIZE; i++) {
    vgaxfb[i] = color;
  }
}

// Generate a tone on a pin (dummy placeholder)
void VGAX::tone(unsigned int frequency) {
  // Optional: implement a simple square wave on a timer pin
}

// Stop tone
void VGAX::noTone() {
  // Optional
}

// --- Interrupt Service Routine ---

// X scan line (0 to VGAX_HEIGHT-1)
volatile byte scanline = 0;

ISR(TIMER1_COMPA_vect) {
  // VSYNC pulse
  if (scanline == 0) {
    PORTB &= ~_BV(0); // VSYNC LOW
  } else if (scanline == 2) {
    PORTB |= _BV(0); // VSYNC HIGH
  }

  // HSYNC pulse
  PORTD &= ~_BV(7); // HSYNC LOW
  delayMicroseconds(4); // Short pulse
  PORTD |= _BV(7); // HSYNC HIGH

  // If visible scanline
  if (scanline < VGAX_HEIGHT) {
    for (int x = 0; x < VGAX_WIDTH; x++) {
      byte color = vgaxfb[scanline * VGAX_WIDTH + x] & 0b00000111;

      // Send RGB bits (PD6=R, PD5=G, PD3=B)
      PORTD = (PORTD & ~(_BV(6) | _BV(5) | _BV(3))) |
              ((color & 0b001) << 3) |   // B -> PD3
              ((color & 0b010) << 4) |   // G -> PD5
              ((color & 0b100) << 3);    // R -> PD6
      delayMicroseconds(1); // Basic pixel clock
    }

    // Clear RGB outputs after scan
    PORTD &= ~(_BV(6) | _BV(5) | _BV(3));
  }

  // Advance scanline
  scanline++;
  if (scanline >= VGAX_HEIGHT + 45) { // total lines incl. vblank
    scanline = 0;
    vtimer++; // 1 frame = 1 tick
  }
}
