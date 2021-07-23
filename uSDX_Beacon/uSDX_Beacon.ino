// Si5351_WSPR_AND_FT8
//
// References:
// - https://github.com/etherkit/JTEncode/blob/master/examples/Si5351JTDemo/Si5351JTDemo.ino
// - https://physics.princeton.edu/pulsar/k1jt/FT4_FT8_QEX.pdf
// - https://github.com/threeme3/QCX-SSB/blob/master/QCX-SSB.ino
// ...
//
// Simple WSPR beacon for Arduino Uno, with the Etherkit Si5351A Breakout
// Board, by Jason Milldrum NT7S.
//
// Original code based on Feld Hell beacon for Arduino by Mark Vandewettering
// K6HX, adapted for the Si5351A by Robert Liesenfeld AK6L <ak6l@ak6l.org>.
// Timer setup code by Thomas Knutsen LA3PNA.
//
// Hardware Requirements
// ---------------------
// This firmware must be run on an Arduino AVR microcontroller
//
// Required Libraries
// ------------------
// Etherkit Si5351 (Library Manager)
// Etherkit JTEncode (Library Manager)
// Time (Library Manager)
// Wire (Arduino Standard Library)
// RTClib
//
// https://www.arduinoslovakia.eu/application/timer-calculator
//
// License (for "some" borrowed code)
// ----------------------------------
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
// ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
// CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <si5351.h>
// #include <JTEncode.h>

#include <int.h>
#include <RTClib.h>
#include <TimeLib.h>
#include <LiquidCrystal.h>

#include "Wire.h"

#include "data.h"

// Mode defines
#define WSPR_TONE_SPACING       146          // ~1.46 Hz
#define FT8_TONE_SPACING        625          // ~6.25 Hz

#define WSPR_DELAY              683          // Delay value for WSPR
#define FT8_DELAY               159          // Delay value for FT8

#define WSPR_SYMBOL_COUNT       162
#define FT8_SYMBOL_COUNT        79

#define OFFSET_HZ               (2060UL)     // Determined using Airspy HF+ Discovery ("SDR")
#define FT8_DEFAULT_FREQ        (14075000UL - OFFSET_HZ)
#define WSPR_DEFAULT_FREQ       (14097104UL - OFFSET_HZ)

enum modes
{
  FT8_MODE,
  WSPR_MODE,
};

enum modes tx_mode = FT8_MODE;  // NOTE NOTE NOTE
// enum modes tx_mode = WSPR_MODE; // NOTE NOTE NOTE - REQUIRES ACTIVE COOLING!

// https://www.qsl.net/yo4hfu/SI5351.html says,
//
// If it is necessary, frequency correction must to be applied. My Si5351
// frequency was too high, correction used 1.787KHz at 10MHz. Open again
// Si5351Example sketch, set CLK_0 to 1000000000 (10MHz). Upload. Connect a
// accurate frequency counter to CLK_0 output pin 10. Correction factor:
// (Output frequency Hz - 10000000Hz) x 100. Example: (10001787Hz - 10000000Hz)
// x 100 = 178700 Note: If output frequency is less than 10MHz, use negative
// value of correction, example -178700.
// #define CORRECTION              209900        // !!! ATTENTION !!! Change this for your reference oscillator

// https://raw.githubusercontent.com/threeme3/QCX-SSB/feature-rx-improved/QCX-SSB.ino
// QCX pin definitions
#define LCD_D4  0         // PD0    (pin 2)
#define LCD_D5  1         // PD1    (pin 3)
#define LCD_D6  2         // PD2    (pin 4)
#define LCD_D7  3         // PD3    (pin 5)
#define LCD_EN  4         // PD4    (pin 6)
#define RX      8         // PB0    (pin 14)
#define KEY_OUT 10        // PB2    (pin 16)
#define LCD_RS  18        // PC4    (pin 27)
#define BUTTONS 17        // PC3/A3 (pin 26)

// Global variables
RTC_DS3231 rtc;
Si5351 si5351;
// JTEncode jtencode;

// Global variables
unsigned long freq;
char message[] = "VU3CER VU3FOE MK68";
char call[] = "VU3FOE";
char loc[] = "MK68";
// uint8_t dbm = 27;
char dbm[] = "27";
uint8_t tx_buffer[255];
uint8_t symbol_count;
uint16_t tone_delay, tone_spacing;

// PWM stuff from https://github.com/threeme3/QCX-SSB/blob/master/QCX-SSB.ino
static uint8_t pwm_min = 0;    // PWM value for which PA reaches its minimum: 29 when C31 installed;   0 when C31 removed;   0 for biasing BS170 directly
// static uint8_t pwm_max = 220;  // PWM value for which PA reaches its maximum: 96 when C31 installed; 255 when C31 removed; 220 for biasing BS170 directly
static uint8_t pwm_max = 110;  // PWM value for which PA reaches its maximum: 96 when C31 installed; 255 when C31 removed; 220 for biasing BS170 directly
// uint8_t lut[256];
uint8_t pwm_value;

// Loop through the string, transmitting one character at a time.
void tx()
{
  uint8_t i;

  // Reset the tone to the base frequency and turn on the output
  si5351.set_clock_pwr(SI5351_CLK2, 1);
  digitalWrite(RX, LOW);  // TX mode!
  OCR1BL = pwm_value;
  TCCR1A |= (1 << COM1B1);  // enable KEY_OUT PWM

  // Now do the rest of the message
  for (i = 0; i < symbol_count; i++)
  {
    si5351.set_freq((freq * 100) + (tx_buffer[i] * tone_spacing), SI5351_CLK2);
    // delay(tone_delay); // for regular CPU clock @ 16 MHz
    delay(tone_delay * 1.25); // adjusted for CPU running @ 20 MHz
  }

  // Turn off the output
  si5351.set_clock_pwr(SI5351_CLK2, 0);
  digitalWrite(RX, HIGH);

  // KEY_OUT PWM disable
  TCCR1A &= ~(1 << COM1B1);
  digitalWrite(KEY_OUT, LOW); // disable KEY_OUT PWM
  OCR1BL = 0; // make sure PWM (KEY_OUT) is set to 0%
}

// From https://github.com/threeme3/QCX-SSB/blob/master/QCX-SSB.ino
class LCD : public Print {  // inspired by: http://www.technoblogy.com/show?2BET
  public:  // LCD1602 display in 4-bit mode, RS is pull-up and kept low when idle to prevent potential display RFI via RS line
#define _dn  0      // PD0 to PD3 connect to D4 to D7 on the display
#define _en  4      // PC4 - MUST have pull-up resistor
#define _rs  4      // PC4 - MUST have pull-up resistor
#define LCD_RS_LO() PORTC &= ~(1 << _rs);        // RS low
#define LCD_RS_HI() PORTC |= (1 << _rs);         // RS high
#define LCD_EN_LO() PORTD &= ~(1 << _en);        // EN low
#define LCD_EN_HI() PORTD |= (1 << _en);         // EN high
#define LCD_PREP_NIBBLE(b) (PORTD & ~(0xf << _dn)) | (b) << _dn | 1 << _en // Send data and enable high
    void begin(uint8_t x = 0, uint8_t y = 0) {       // Send command , make sure at least 40ms after power-up before sending commands
      bool reinit = (x == 0) && (y == 0);
      DDRD |= 0xf << _dn | 1 << _en;                 // Make data, EN outputs
      DDRC |= 1 << _rs;
      delayMicroseconds(50000);                      //
      LCD_RS_LO(); LCD_EN_LO();
      cmd(0x33);                                     // Ensures display is in 8-bit mode
      delayMicroseconds(4500); cmd(0x33); delayMicroseconds(4500); cmd(0x33); delayMicroseconds(150); // * Ensures display is in 8-bit mode
      cmd(0x32);                                     // Puts display in 4-bit mode
      cmd(0x28);                                     // * Function set: 2-line, 5x8
      cmd(0x0c);                                     // Display on
      if (reinit) return;
      cmd(0x01);                                     // Clear display
      delay(3);                                      // Allow to execute Clear on display [https://www.sparkfun.com/datasheets/LCD/HD44780.pdf, p.49, p58]
      cmd(0x06);                                     // * Entrymode: left, shift-dec
    }
    void nib(uint8_t b) {                            // Send four bit nibble to display
      PORTD = LCD_PREP_NIBBLE(b);                    // Send data and enable high
      delayMicroseconds(4);
      LCD_EN_LO();
      delayMicroseconds(60);                         // Execution time
    }
    void cmd(uint8_t b) {
      nib(b >> 4);  // Write command: send nibbles while RS low
      nib(b & 0xf);
    }
    size_t write(uint8_t b) {                        // Write data:    send nibbles while RS high
      uint8_t nibh = LCD_PREP_NIBBLE(b >>  4);       // Prepare high nibble data and enable high
      PORTD = nibh;                                  // Send high nibble data and enable high
      uint8_t nibl = LCD_PREP_NIBBLE(b & 0xf);       // Prepare low nibble data and enable high
      LCD_RS_HI();
      LCD_EN_LO();
      PORTD = nibl;                                  // Send low nibble data and enable high
      LCD_RS_LO();
      LCD_RS_HI();
      LCD_EN_LO();
      LCD_RS_LO();
      delayMicroseconds(60);                         // Execution time  (37+4)*1.25 us
      PORTD |= 0x02;                                 // To support serial-interface keep LCD_D5 high, so that DVM is not pulled-down via D
      return 1;
    }
    void setCursor(uint8_t x, uint8_t y) {
      cmd(0x80 | (x + y * 0x40));
    }
    void cursor() {
      cmd(0x0e);
    }
    void noCursor() {
      cmd(0x0c);
    }
    void noDisplay() {
      cmd(0x08);
    }
    void createChar(uint8_t l, uint8_t glyph[]) {
      cmd(0x40 | ((l & 0x7) << 3));
      for (int i = 0; i != 8; i++) write(glyph[i]);
    }
};

LCD lcd;
int buttonState = 0;

int wspr_encode(char *call, char *grid, char *dBm, uint8_t *symbols);

void setup()
{
  int ret = 0;
  char date[10] = "hh:mm:ss";

  pinMode(KEY_OUT, OUTPUT);
  digitalWrite(KEY_OUT, LOW);  // LOW for safety: to prevent exploding PA MOSFETs, in case there was something still biasing them.
  pinMode(RX, OUTPUT);
  digitalWrite(RX, HIGH);
  pinMode(BUTTONS, INPUT);

  // Mode changing hack
  buttonState = digitalRead(BUTTONS);
  if (buttonState == HIGH) {
    tx_mode = WSPR_MODE;
  }

  lcd.begin(16, 2);
  // Serial.begin(7680); // 9600 baud corrected for F_CPU=20M, keep disabled, causes problems with LCD stuff!

  // RTC stuff
  if (!rtc.begin()) {
    lcd.print("RTC Not Found!");
    abort();
  }
  if (rtc.lostPower()) {
    lcd.print("RTC Lost Power!");
    abort();
  }
  // We don't need the 32K Pin, so disable it
  rtc.disable32K();
  // Serial.print(rtc.getTemperature());

  if (tx_mode == WSPR_MODE) {
    freq = WSPR_DEFAULT_FREQ;
    symbol_count = WSPR_SYMBOL_COUNT; // From the library defines
    tone_spacing = WSPR_TONE_SPACING;
    tone_delay = WSPR_DELAY;
    // jtencode.wspr_encode(call, loc, dbm, tx_buffer); // this is crashing!?
    // memcpy(tx_buffer, wspr_fixed_buffer, WSPR_SYMBOL_COUNT);
    wspr_encode(call, loc, dbm, tx_buffer);
  }
  else if (tx_mode == FT8_MODE) {
    freq = FT8_DEFAULT_FREQ;
    symbol_count = FT8_SYMBOL_COUNT;
    tone_spacing = FT8_TONE_SPACING;
    tone_delay = FT8_DELAY;
    // jtencode.ft8_encode(message, tx_buffer); // note: buggy/limited
    memcpy(tx_buffer, ft8_fixed_buffer, FT8_SYMBOL_COUNT);
  }

  // Initialize the Si5351
  ret = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  if (ret != true) {
    lcd.print("VFO Problem!?");
  }
  si5351.set_freq((freq * 100), SI5351_CLK2);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); // Set for minimum power
  //si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for max power
  delay(5000); // Keep TX on for 5 seconds for tuning purposes.
  si5351.set_clock_pwr(SI5351_CLK0, 0);
  si5351.set_clock_pwr(SI5351_CLK1, 0);
  si5351.set_clock_pwr(SI5351_CLK2, 0);

  // Useful for debugging!
  rtc.now().toString(date);
  // lcd.print(date);
  if (tx_mode == WSPR_MODE) {
    lcd.print("WSPR Mode");
  } else {
    lcd.print("FT8 Mode");
  }

  delay(5000); // for safety!

  pwm_value = (float)255 / ((float)255 / ((float)pwm_max - (float)pwm_min)) + pwm_min;
}

DateTime dt;

// Adjust this for better `DT` results.
// int delay_tx = 300;
int delay_tx = 0;
int startup = 1;  // quick tx check on startup hack

void loop()
{
  dt = rtc.now();

  if (startup && dt.second() % 15 == 14 && tx_mode == FT8_MODE) {
    delay(delay_tx);
    startup = 0;
    tx();
  }
  else if (!startup && dt.second() == 14 && tx_mode == FT8_MODE) { // 25% duty cycle
    delay(delay_tx);
    tx();
  }
  else if ((dt.minute() % 2 == 1) && (dt.second() == 59) && tx_mode == WSPR_MODE) {
    delay(delay_tx);
    tx();
  }

  delay(100);
}
