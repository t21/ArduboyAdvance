/**
 * @file Arduboy2Core.cpp
 * \brief
 * The Arduboy2Core class for Arduboy hardware initilization and control.
 */

#include "ArduboyAdvanceCore.h"

const uint8_t PROGMEM lcdBootProgram[] = {
  // boot defaults are commented out but left here in case they
  // might prove useful for reference
  //
  // Further reading: https://www.adafruit.com/datasheets/SSD1306.pdf
  //
  // Display Off
  // 0xAE,

  // Set Display Clock Divisor v = 0xF0
  // default is 0x80
  0xD5, 0xF0,

  // Set Multiplex Ratio v = 0x3F
  // 0xA8, 0x3F,

  // Set Display Offset v = 0
  // 0xD3, 0x00,

  // Set Start Line (0)
  // 0x40,

  // Charge Pump Setting v = enable (0x14)
  // default is disabled
  0x8D, 0x14,

  // Set Segment Re-map (A0) | (b0001)
  // default is (b0000)
  0xA1,

  // Set COM Output Scan Direction
  0xC8,

  // Set COM Pins v
  // 0xDA, 0x12,

  // Set Contrast v = 0xCF
  0x81, 0xCF,

  // Set Precharge = 0xF1
  0xD9, 0xF1,

  // Set VCom Detect
  // 0xDB, 0x40,

  // Entire Display ON
  // 0xA4,

  // Set normal/inverse display
  // 0xA6,

  // Display On
  0xAF,

  // set display mode = horizontal addressing mode (0x00)
  0x20, 0x00,

  // set col address range
  // 0x21, 0x00, COLUMN_ADDRESS_END,

  // set page address range
  // 0x22, 0x00, PAGE_ADDRESS_END
};


ArduboyAdvanceCore::ArduboyAdvanceCore() { }

void ArduboyAdvanceCore::boot()
{
  #ifdef ARDUBOY_SET_CPU_8MHZ
  // ARDUBOY_SET_CPU_8MHZ will be set by the IDE using boards.txt
  setCPUSpeed8MHz();
  #endif

  // Select the ADC input here so a delay isn't required in initRandomSeed()
  // ADMUX = RAND_SEED_IN_ADMUX;

  bootPins();
  bootSPI();
  bootOLED();
  bootPowerSaving();
}

#ifdef ARDUBOY_SET_CPU_8MHZ
// If we're compiling for 8MHz we need to slow the CPU down because the
// hardware clock on the Arduboy is 16MHz.
// We also need to readjust the PLL prescaler because the Arduino USB code
// likely will have incorrectly set it for an 8MHz hardware clock.
void ArduboyAdvanceCore::setCPUSpeed8MHz()
{
  uint8_t oldSREG = SREG;
  cli();                // suspend interrupts
  PLLCSR = _BV(PINDIV); // dissable the PLL and set prescale for 16MHz)
  CLKPR = _BV(CLKPCE);  // allow reprogramming clock
  CLKPR = 1;            // set clock divisor to 2 (0b0001)
  PLLCSR = _BV(PLLE) | _BV(PINDIV); // enable the PLL (with 16MHz prescale)
  SREG = oldSREG;       // restore interrupts
}
#endif

// Pins are set to the proper modes and levels for the specific hardware.
// This routine must be modified if any pins are moved to a different port
void ArduboyAdvanceCore::bootPins()
{

#ifdef ARDUBOY_20

analogReadResolution(A_READ_BITS); //For joystick

pinMode(PIN_A_BUTTON, INPUT_PULLUP);
pinMode(PIN_B_BUTTON, INPUT_PULLUP);
pinMode(PIN_X_BUTTON, INPUT_PULLUP);
pinMode(PIN_Y_BUTTON, INPUT_PULLUP);
pinMode(PIN_JOY_SEL_BUTTON, INPUT_PULLUP); 
pinMode(PIN_JOY_X_AXIS, INPUT);
pinMode(PIN_JOY_Y_AXIS, INPUT);

#elif defined(ARDUBOY_10)

  // Port B INPUT_PULLUP or HIGH
  PORTB |= (1 << RED_LED_BIT) | (1 << GREEN_LED_BIT) | (1 << BLUE_LED_BIT) |
           (1 << B_BUTTON_BIT);
  // Port B INPUT or LOW (none)
  // Port B inputs
  DDRB &= ~((1 << B_BUTTON_BIT));
  // Port B outputs
  DDRB |= (1 << RED_LED_BIT) | (1 << GREEN_LED_BIT) | (1 << BLUE_LED_BIT) |
          (1 << SPI_MOSI_BIT) | (1 << SPI_SCK_BIT);

  // Port C
  // Speaker: Not set here. Controlled by audio class

  // Port D INPUT_PULLUP or HIGH
  PORTD |= (1 << CS_BIT);
  // Port D INPUT or LOW
  PORTD &= ~((1 << RST_BIT));
  // Port D inputs (none)
  // Port D outputs
  DDRD |= (1 << RST_BIT) | (1 << CS_BIT) | (1 << DC_BIT);

  // Port E INPUT_PULLUP or HIGH
  PORTE |= (1 << A_BUTTON_BIT);
  // Port E INPUT or LOW (none)
  // Port E inputs
  DDRE &= ~((1 << A_BUTTON_BIT));
  // Port E outputs (none)

  // Port F INPUT_PULLUP or HIGH
  PORTF |= (1 << LEFT_BUTTON_BIT) | (1 << RIGHT_BUTTON_BIT) |
           (1 << UP_BUTTON_BIT) | (1 << DOWN_BUTTON_BIT);
  // Port F INPUT or LOW
  PORTF &= ~((1 << RAND_SEED_IN_BIT));
  // Port F inputs
  DDRF &= ~((1 << LEFT_BUTTON_BIT) | (1 << RIGHT_BUTTON_BIT) |
            (1 << UP_BUTTON_BIT) | (1 << DOWN_BUTTON_BIT) |
            (1 << RAND_SEED_IN_BIT));
  // Port F outputs (none)

#elif defined(AB_DEVKIT)

  // Port B INPUT_PULLUP or HIGH
  PORTB |= _BV(LEFT_BUTTON_BIT) | _BV(UP_BUTTON_BIT) | _BV(DOWN_BUTTON_BIT) |
           _BV(BLUE_LED_BIT);
  // Port B INPUT or LOW (none)
  // Port B inputs
  DDRB &= ~(_BV(LEFT_BUTTON_BIT) | _BV(UP_BUTTON_BIT) | _BV(DOWN_BUTTON_BIT));
  // Port B outputs
  DDRB |= _BV(BLUE_LED_BIT) | _BV(SPI_MOSI_BIT) | _BV(SPI_SCK_BIT);

  // Port C INPUT_PULLUP or HIGH
  PORTC |= _BV(RIGHT_BUTTON_BIT);
  // Port C INPUT or LOW (none)
  // Port C inputs
  DDRC &= ~(_BV(RIGHT_BUTTON_BIT));
  // Port C outputs (none)

  // Port D INPUT_PULLUP or HIGH
  PORTD |= _BV(CS_BIT);
  // Port D INPUT or LOW
  PORTD &= ~(_BV(RST_BIT));
  // Port D inputs (none)
  // Port D outputs
  DDRD |= _BV(RST_BIT) | _BV(CS_BIT) | _BV(DC_BIT);

  // Port E (none)

  // Port F INPUT_PULLUP or HIGH
  PORTF |= _BV(A_BUTTON_BIT) | _BV(B_BUTTON_BIT);
  // Port F INPUT or LOW
  PORTF &= ~(_BV(RAND_SEED_IN_BIT));
  // Port F inputs
  DDRF &= ~(_BV(A_BUTTON_BIT) | _BV(B_BUTTON_BIT) | _BV(RAND_SEED_IN_BIT));
  // Port F outputs (none)
  // Speaker: Not set here. Controlled by audio class

#endif
}

void ArduboyAdvanceCore::bootOLED()
{
  // reset the display
  delayShort(5); // reset pin should be low here. let it stay low a while
  // bitSet(RST_PORT, RST_BIT); // set high to come out of reset
  delayShort(5); // wait a while

  // select the display (permanently, since nothing else is using SPI)
  // bitClear(CS_PORT, CS_BIT);

  // run our customized boot-up command sequence against the
  // OLED to initialize it properly for Arduboy
  LCDCommandMode();
  for (uint8_t i = 0; i < sizeof(lcdBootProgram); i++) {
    SPItransfer(pgm_read_byte(lcdBootProgram + i));
  }
  LCDDataMode();
}

void ArduboyAdvanceCore::LCDDataMode()
{
  // bitSet(DC_PORT, DC_BIT);
}

void ArduboyAdvanceCore::LCDCommandMode()
{
  // bitClear(DC_PORT, DC_BIT);
}

// Initialize the SPI interface for the display
void ArduboyAdvanceCore::bootSPI()
{
// master, mode 0, MSB first, CPU clock / 2 (8MHz)
  // SPCR = _BV(SPE) | _BV(MSTR);
  // SPSR = _BV(SPI2X);
}

// Write to the SPI bus (MOSI pin)
void ArduboyAdvanceCore::SPItransfer(uint8_t data)
{
  // SPDR = data;
  /*
   * The following NOP introduces a small delay that can prevent the wait
   * loop form iterating when running at the maximum speed. This gives
   * about 10% more speed, even if it seems counter-intuitive. At lower
   * speeds it is unnoticed.
   */
  asm volatile("nop");
  // while (!(SPSR & _BV(SPIF))) { } // wait
}

void ArduboyAdvanceCore::safeMode()
{
  if (buttonsState() == UP_BUTTON)
  {
    digitalWriteRGB(RED_LED, RGB_ON);

    // prevent the bootloader magic number from being overwritten by timer 0
    // when a timer variable overlaps the magic number location
    // power_timer0_disable();

    while (true) { }
  }
}


/* Power Management */

void ArduboyAdvanceCore::idle()
{
  // set_sleep_mode(SLEEP_MODE_IDLE);
  // sleep_mode();
}

void ArduboyAdvanceCore::bootPowerSaving()
{
  // // disable Two Wire Interface (I2C) and the ADC
  // PRR0 = _BV(PRTWI) | _BV(PRADC);
  // // disable USART1
  // PRR1 = _BV(PRUSART1);
  // // All other bits will be written with 0 so will be enabled
}

// Shut down the display
void ArduboyAdvanceCore::displayOff()
{
  LCDCommandMode();
  SPItransfer(0xAE); // display off
  SPItransfer(0x8D); // charge pump:
  SPItransfer(0x10); //   disable
  delayShort(250);
  // bitClear(RST_PORT, RST_BIT); // set display reset pin low (reset state)
}

// Restart the display after a displayOff()
void ArduboyAdvanceCore::displayOn()
{
  bootOLED();
}

uint8_t ArduboyAdvanceCore::width() { return WIDTH; }

uint8_t ArduboyAdvanceCore::height() { return HEIGHT; }


/* Drawing */

void ArduboyAdvanceCore::paint8Pixels(uint8_t pixels)
{
  SPItransfer(pixels);
}

void ArduboyAdvanceCore::paintScreen(const uint8_t *image)
{
  for (int i = 0; i < (HEIGHT*WIDTH)/8; i++)
  {
    SPItransfer(pgm_read_byte(image + i));
  }
}

// paint from a memory buffer, this should be FAST as it's likely what
// will be used by any buffer based subclass
void ArduboyAdvanceCore::paintScreen(uint8_t image[], bool clear)
{
  uint8_t c;
  int i = 0;

  if (clear)
  {
    // SPDR = image[i]; // set the first SPI data byte to get things started
    image[i++] = 0;  // clear the first image byte
  }
  else
    // SPDR = image[i++];

  // the code to iterate the loop and get the next byte from the buffer is
  // executed while the previous byte is being sent out by the SPI controller
  while (i < (HEIGHT * WIDTH) / 8)
  {
    // get the next byte. It's put in a local variable so it can be sent as
    // as soon as possible after the sending of the previous byte has completed
    if (clear)
    {
      c = image[i];
      // clear the byte in the image buffer
      image[i++] = 0;
    }
    else
      c = image[i++];

    // while (!(SPSR & _BV(SPIF))) { } // wait for the previous byte to be sent

    // put the next byte in the SPI data register. The SPI controller will
    // clock it out while the loop continues and gets the next byte ready
    // SPDR = c;
  }
  // while (!(SPSR & _BV(SPIF))) { } // wait for the last byte to be sent
}

void ArduboyAdvanceCore::blank()
{
  for (int i = 0; i < (HEIGHT*WIDTH)/8; i++)
    SPItransfer(0x00);
}

void ArduboyAdvanceCore::sendLCDCommand(uint8_t command)
{
  LCDCommandMode();
  SPItransfer(command);
  LCDDataMode();
}

// invert the display or set to normal
// when inverted, a pixel set to 0 will be on
void ArduboyAdvanceCore::invert(bool inverse)
{
  sendLCDCommand(inverse ? OLED_PIXELS_INVERTED : OLED_PIXELS_NORMAL);
}

// turn all display pixels on, ignoring buffer contents
// or set to normal buffer display
void ArduboyAdvanceCore::allPixelsOn(bool on)
{
  sendLCDCommand(on ? OLED_ALL_PIXELS_ON : OLED_PIXELS_FROM_RAM);
}

// flip the display vertically or set to normal
void ArduboyAdvanceCore::flipVertical(bool flipped)
{
  sendLCDCommand(flipped ? OLED_VERTICAL_FLIPPED : OLED_VERTICAL_NORMAL);
}

// flip the display horizontally or set to normal
void ArduboyAdvanceCore::flipHorizontal(bool flipped)
{
  sendLCDCommand(flipped ? OLED_HORIZ_FLIPPED : OLED_HORIZ_NORMAL);
}

/* RGB LED */

void ArduboyAdvanceCore::setRGBled(uint8_t red, uint8_t green, uint8_t blue)
{
#ifdef ARDUBOY_10 // RGB, all the pretty colors
  // inversion is necessary because these are common annode LEDs
  analogWrite(RED_LED, 255 - red);
  analogWrite(GREEN_LED, 255 - green);
  analogWrite(BLUE_LED, 255 - blue);
#elif defined(AB_DEVKIT)
  // only blue on DevKit, which is not PWM capable
  (void)red;    // parameter unused
  (void)green;  // parameter unused
  bitWrite(BLUE_LED_PORT, BLUE_LED_BIT, blue ? RGB_ON : RGB_OFF);
#endif
}

void ArduboyAdvanceCore::digitalWriteRGB(uint8_t red, uint8_t green, uint8_t blue)
{
#ifdef ARDUBOY_10
  bitWrite(RED_LED_PORT, RED_LED_BIT, red);
  bitWrite(GREEN_LED_PORT, GREEN_LED_BIT, green);
  bitWrite(BLUE_LED_PORT, BLUE_LED_BIT, blue);
#elif defined(AB_DEVKIT)
  // only blue on DevKit
  (void)red;    // parameter unused
  (void)green;  // parameter unused
  bitWrite(BLUE_LED_PORT, BLUE_LED_BIT, blue);
#endif
}

void ArduboyAdvanceCore::digitalWriteRGB(uint8_t color, uint8_t val)
{
#ifdef ARDUBOY_10
  if (color == RED_LED)
  {
    bitWrite(RED_LED_PORT, RED_LED_BIT, val);
  }
  else if (color == GREEN_LED)
  {
    bitWrite(GREEN_LED_PORT, GREEN_LED_BIT, val);
  }
  else if (color == BLUE_LED)
  {
    bitWrite(BLUE_LED_PORT, BLUE_LED_BIT, val);
  }
#elif defined(AB_DEVKIT)
  // only blue on DevKit
  if (color == BLUE_LED)
  {
    bitWrite(BLUE_LED_PORT, BLUE_LED_BIT, val);
  }
#endif
}

/* Buttons */

uint8_t ArduboyAdvanceCore::buttonsState()
{
  uint8_t buttons;
#ifdef ARDUBOY_20
  uint8_t Abit;
  uint8_t Bbit;
  uint8_t Xbit;
  uint8_t Ybit;
  uint8_t Selbit;

  Abit = ~digitalRead(PIN_A_BUTTON) & 0x01;
  Bbit = ~digitalRead(PIN_B_BUTTON) & 0x01;
  Xbit = ~digitalRead(PIN_X_BUTTON) & 0x01;
  Ybit = ~digitalRead(PIN_Y_BUTTON) & 0x01;
  Selbit = ~digitalRead(PIN_JOY_SEL_BUTTON) & 0x01;

  buttons = (Selbit << 4) | (Abit << 3) | (Bbit << 2) | (Xbit << 1) | (Ybit);

  // using ports here is ~100 bytes smaller than digitalRead()
#elif defined(AB_DEVKIT)
  // down, left, up
  buttons = ((~PINB) & B01110000);
  // right button
  buttons = buttons | (((~PINC) & B01000000) >> 4);
  // A and B
  buttons = buttons | (((~PINF) & B11000000) >> 6);
#elif defined(ARDUBOY_10)
  // down, up, left right
  buttons = ((~PINF) & B11110000);
  // A (left)
  buttons = buttons | (((~PINE) & B01000000) >> 3);
  // B (right)
  buttons = buttons | (((~PINB) & B00010000) >> 2);
#endif

  return buttons;
}

// delay in ms with 16 bit duration
void ArduboyAdvanceCore::delayShort(uint16_t ms)
{
  delay((unsigned long) ms);
}

