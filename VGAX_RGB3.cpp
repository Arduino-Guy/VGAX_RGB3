#include "VGAX_RGB3.h"
#include <avr/interrupt.h>

volatile uint8_t vgaxfb[VGAX_BSIZE];
volatile unsigned long vtimer = 0;

static volatile uint8_t tone_pin_mask = 0;
static volatile uint16_t tone_divider = 0;

static volatile uint8_t hline_counter = 0;
static volatile uint8_t vline_counter = 0;
static volatile uint16_t pixel_index = 0;

// Framebuffer scanline pointer and bit offset for output
static volatile uint16_t scanline_fb_offset = 0;
static volatile uint8_t scanline_bit_offset = 0;

void VGAX_RGB3::begin(bool enableTone) {
  cli();

  // Setup RGB pins as outputs, initial low
  DDRD |= (1 << PIN_BLUE) | (1 << PIN_RED) | (1 << PIN_GREEN);
  PORTD &= ~((1 << PIN_BLUE) | (1 << PIN_RED) | (1 << PIN_GREEN));

  // Clear framebuffer
  VGAX_RGB3::clear(COLOR_BLACK);

  // Timer0 - horizontal line timer, ~31.5kHz (VGA pixel clock ~25.175 MHz / 4 cycles per pixel approx)
  // Using CTC mode with OCR0A = 63 at 16 MHz clock for ~31.5 kHz interrupt rate
  TCCR0A = (1 << WGM01);
  TCCR0B = (1 << CS00); // No prescaling
  OCR0A = 63;
  TIMSK0 |= (1 << OCIE0A);

  // Timer1 - vertical sync timer, 60 Hz
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // CTC, prescaler 64
  OCR1A = 26666; // 16MHz/(64*26666) ~ 60Hz
  TIMSK1 |= (1 << OCIE1A);

  // Setup tone timer (Timer2) if enabled
  if (enableTone) {
    TCCR2A = 0;
    TCCR2B = 0;
    TIMSK2 = 0;
  }

  sei();
}

void VGAX_RGB3::end() {
  cli();
  TIMSK0 = 0;
  TIMSK1 = 0;
  TIMSK2 = 0;
  sei();
}

void VGAX_RGB3::clear(uint8_t color) {
  // Fill framebuffer with repeated color (3 bits per pixel packed)
  uint32_t pattern = 0;
  for (int i = 0; i < 8; i++) {
    pattern <<= 3;
    pattern |= (color & 0x07);
  }
  uint8_t p0 = (pattern >> 16) & 0xFF;
  uint8_t p1 = (pattern >> 8) & 0xFF;
  uint8_t p2 = pattern & 0xFF;

  for (int y = 0; y < VGAX_HEIGHT; y++) {
    for (int b = 0; b < VGAX_BWIDTH; b += 3) {
      vgaxfb[y * VGAX_BWIDTH + b] = p0;
      vgaxfb[y * VGAX_BWIDTH + b + 1] = p1;
      vgaxfb[y * VGAX_BWIDTH + b + 2] = p2;
    }
  }
}

void VGAX_RGB3::putpixel(uint8_t x, uint8_t y, uint8_t color) {
  if (x >= VGAX_WIDTH || y >= VGAX_HEIGHT) return;

  uint16_t bitpos = y * VGAX_WIDTH * 3 + x * 3;
  uint16_t bytepos = bitpos >> 3;
  uint8_t bit_in_byte = bitpos & 7;

  uint8_t val = color & 0x07;

  uint8_t b0 = vgaxfb[bytepos];
  uint8_t b1 = (bytepos + 1 < VGAX_BSIZE) ? vgaxfb[bytepos + 1] : 0;

  if (bit_in_byte <= 5) {
    uint8_t mask = 0x07 << (5 - bit_in_byte);
    b0 = (b0 & ~mask) | (val << (5 - bit_in_byte));
  } else {
    uint8_t first_bits = 8 - bit_in_byte;
    uint8_t second_bits = 3 - first_bits;

    uint8_t mask0 = ((1 << first_bits) - 1);
    uint8_t mask1 = ((1 << second_bits) - 1) << (8 - second_bits);

    b0 = (b0 & ~mask0) | ((val >> second_bits) & mask0);
    b1 = (b1 & ~mask1) | ((val << (8 - second_bits)) & mask1);
  }

  vgaxfb[bytepos] = b0;
  if (bytepos + 1 < VGAX_BSIZE) vgaxfb[bytepos + 1] = b1;
}

uint8_t VGAX_RGB3::getpixel(uint8_t x, uint8_t y) {
  if (x >= VGAX_WIDTH || y >= VGAX_HEIGHT) return 0;

  uint16_t bitpos = y * VGAX_WIDTH * 3 + x * 3;
  uint16_t bytepos = bitpos >> 3;
  uint8_t bit_in_byte = bitpos & 7;

  uint8_t b0 = vgaxfb[bytepos];
  uint8_t b1 = (bytepos + 1 < VGAX_BSIZE) ? vgaxfb[bytepos + 1] : 0;

  if (bit_in_byte <= 5) {
    uint8_t mask = 0x07 << (5 - bit_in_byte);
    return (b0 & mask) >> (5 - bit_in_byte);
  } else {
    uint8_t first_bits = 8 - bit_in_byte;
    uint8_t second_bits = 3 - first_bits;

    uint8_t mask0 = ((1 << first_bits) - 1);
    uint8_t mask1 = ((1 << second_bits) - 1) << (8 - second_bits);

    uint8_t val0 = b0 & mask0;
    uint8_t val1 = (b1 & mask1) >> (8 - second_bits);

    return (val0 << second_bits) | val1;
  }
}

void VGAX_RGB3::delay(int msec) {
  unsigned long start = millis();
  while ((millis() - start) < (unsigned long)msec);
}

unsigned long VGAX_RGB3::millis() {
  return vtimer * 16;
}

// Tone generation on D3 (Timer2)
void VGAX_RGB3::tone(unsigned int frequency) {
  if (frequency == 0) {
    VGAX_RGB3::noTone();
    return;
  }

  uint32_t ocr = (16000000UL / (2UL * 8UL * frequency)) - 1;
  if (ocr > 255) ocr = 255;

  tone_divider = (uint16_t)ocr;

  DDRD |= (1 << 3);
  tone_pin_mask = (1 << 3);

  TCCR2A = (1 << WGM21);
  TCCR2B = (1 << CS11);
  OCR2A = tone_divider;
  TIMSK2 |= (1 << OCIE2A);
}

void VGAX_RGB3::noTone() {
  TIMSK2 &= ~(1 << OCIE2A);
  tone_pin_mask = 0;
  DDRD &= ~(1 << 3);
  PORTD &= ~(1 << 3);
}

// Timer2 Compare Match ISR for tone generation
ISR(TIMER2_COMPA_vect) {
  if (tone_pin_mask) {
    PORTD ^= tone_pin_mask;
  }
}

// --- VGA output ISR ---

// Timer0 Compare Match Interrupt - horizontal line timing and pixel output
ISR(TIMER0_COMPA_vect) {
  static uint8_t hcount = 0;

  if (vline_counter < VGAX_HEIGHT) {
    // Output one horizontal scanline pixel data (120 pixels)

    // We output 120 pixels * 3 bits = 360 bits per line
    // We need to output 3 bits at a time to pins PD5,6,7 (Blue, Red, Green)

    // We output 1 pixel per interrupt call for simplicity (you can speed this up by unrolling)

    // Calculate bit position in framebuffer
    uint32_t bit_pos = vline_counter * VGAX_WIDTH * 3 + hcount * 3;
    uint16_t byte_pos = bit_pos >> 3;
    uint8_t bit_offset = bit_pos & 7;

    // Read 3 bits for pixel
    uint8_t b0 = vgaxfb[byte_pos];
    uint8_t b1 = (byte_pos + 1 < VGAX_BSIZE) ? vgaxfb[byte_pos + 1] : 0;
    uint8_t pixel_val;

    if (bit_offset <= 5) {
      pixel_val = (b0 >> (5 - bit_offset)) & 0x07;
    } else {
      uint8_t first_bits = 8 - bit_offset;
      uint8_t second_bits = 3 - first_bits;
      pixel_val = ((b0 & ((1 << first_bits) - 1)) << second_bits) |
                  ((b1 >> (8 - second_bits)) & ((1 << second_bits) - 1));
    }

    // Output pixel color bits to PORTD pins 5,6,7
    // Bit0=Blue(PD5), Bit1=Red(PD6), Bit2=Green(PD7)
    uint8_t port_mask = ((pixel_val & 0x01) << PIN_BLUE) |
                        ((pixel_val & 0x02) << (PIN_RED - 1)) |
                        ((pixel_val & 0x04) << (PIN_GREEN - 2));

    // Clear pins PD5,6,7
    PORTD &= ~((1 << PIN_BLUE) | (1 << PIN_RED) | (1 << PIN_GREEN));
    // Set pins according to pixel_val
    PORTD |= port_mask;

    hcount++;
  } else {
    // Horizontal blanking interval - set all RGB pins low
    PORTD &= ~((1 << PIN_BLUE) | (1 << PIN_RED) | (1 << PIN_GREEN));
  }

  hcount++;

  if (hcount >= (VGAX_WIDTH + 40)) {
    // After 120 pixels + ~40 blank pixels, reset hcount, increment vline_counter
    hcount = 0;
    vline_counter++;
  }

  if (vline_counter >= 525) {
    vline_counter = 0;
  }
}

// Timer1 Compare Match Interrupt - vertical sync (mocked)
ISR(TIMER1_COMPA_vect) {
  vtimer++;
}

