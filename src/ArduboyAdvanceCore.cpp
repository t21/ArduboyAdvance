/**
 * @file Arduboy2Core.cpp
 * \brief
 * The Arduboy2Core class for Arduboy hardware initilization and control.
 */

#include "ArduboyAdvanceCore.h"
#include "pins_arduino.h"
// #include "spi_dma.h"
// #include <SPI.h>

// #define USE_SPI_LIBRARY
// #define SET_BIT(port, bitMask) digitalWrite(*(port), HIGH);
// #define CLEAR_BIT(port, bitMask) digitalWrite(*(port), LOW);


// const uint8_t PROGMEM lcdBootProgram[] = {
//   // boot defaults are commented out but left here in case they
//   // might prove useful for reference
//   //
//   // Further reading: https://www.adafruit.com/datasheets/SSD1306.pdf
//   //
//   // Display Off
//   // 0xAE,

//   // Set Display Clock Divisor v = 0xF0
//   // default is 0x80
//   0xD5, 0xF0,

//   // Set Multiplex Ratio v = 0x3F
//   // 0xA8, 0x3F,

//   // Set Display Offset v = 0
//   // 0xD3, 0x00,

//   // Set Start Line (0)
//   // 0x40,

//   // Charge Pump Setting v = enable (0x14)
//   // default is disabled
//   0x8D, 0x14,

//   // Set Segment Re-map (A0) | (b0001)
//   // default is (b0000)
//   0xA1,

//   // Set COM Output Scan Direction
//   0xC8,

//   // Set COM Pins v
//   // 0xDA, 0x12,

//   // Set Contrast v = 0xCF
//   0x81, 0xCF,

//   // Set Precharge = 0xF1
//   0xD9, 0xF1,

//   // Set VCom Detect
//   // 0xDB, 0x40,

//   // Entire Display ON
//   // 0xA4,

//   // Set normal/inverse display
//   // 0xA6,

//   // Display On
//   0xAF,

//   // set display mode = horizontal addressing mode (0x00)
//   0x20, 0x00,

//   // set col address range
//   // 0x21, 0x00, COLUMN_ADDRESS_END,

//   // set page address range
//   // 0x22, 0x00, PAGE_ADDRESS_END
// };


ArduboyAdvanceCore::ArduboyAdvanceCore()
{
}

void ArduboyAdvanceCore::boot()
{
  // #ifdef ARDUBOY_SET_CPU_8MHZ
  // // ARDUBOY_SET_CPU_8MHZ will be set by the IDE using boards.txt
  // setCPUSpeed8MHz();
  // #endif

  // Select the ADC input here so a delay isn't required in initRandomSeed()
  // ADMUX = RAND_SEED_IN_ADMUX;

    // bootPins();
    gpio_init();
    // bootSPI();
    // bootOLED();
    ili9341_init();
    // bootTFT();
  // bootPowerSaving();
}

// #ifdef ARDUBOY_SET_CPU_8MHZ
// // If we're compiling for 8MHz we need to slow the CPU down because the
// // hardware clock on the Arduboy is 16MHz.
// // We also need to readjust the PLL prescaler because the Arduino USB code
// // likely will have incorrectly set it for an 8MHz hardware clock.
// void ArduboyAdvanceCore::setCPUSpeed8MHz()
// {
//   uint8_t oldSREG = SREG;
//   cli();                // suspend interrupts
//   PLLCSR = _BV(PINDIV); // dissable the PLL and set prescale for 16MHz)
//   CLKPR = _BV(CLKPCE);  // allow reprogramming clock
//   CLKPR = 1;            // set clock divisor to 2 (0b0001)
//   PLLCSR = _BV(PLLE) | _BV(PINDIV); // enable the PLL (with 16MHz prescale)
//   SREG = oldSREG;       // restore interrupts
// }
// #endif


void ArduboyAdvanceCore::gpio_init()
{
    // Set up pins for the display
    pinMode(PIN_DISP_RD, OUTPUT);
    pinMode(PIN_DISP_WR, OUTPUT);
    pinMode(PIN_DISP_CD, OUTPUT);
    pinMode(PIN_DISP_CS, OUTPUT);

    CS_IDLE; // Set all control bits to HIGH (idle)

    CD_DATA; // Signals are ACTIVE LOW

    WR_IDLE;

    RD_IDLE;

    pinMode(PIN_DISP_RST, OUTPUT);
    digitalWriteFast(PIN_DISP_RST, HIGH);

    //set up 8 bit parallel port to write mode.
    setWriteDataBus();

    // ToDo: Add the rest of the pins

}



// Pins are set to the proper modes and levels for the specific hardware.
// This routine must be modified if any pins are moved to a different port
void ArduboyAdvanceCore::bootPins()
{

#if defined(ARDUBOY_20) || defined(ARDUBOY_36)

pinMode(PIN_A_BUTTON, INPUT_PULLUP);
pinMode(PIN_B_BUTTON, INPUT_PULLUP);
pinMode(PIN_X_BUTTON, INPUT_PULLUP);
pinMode(PIN_Y_BUTTON, INPUT_PULLUP);
pinMode(PIN_JOY_SEL_BUTTON, INPUT_PULLUP); //Maybe pullup
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


void ArduboyAdvanceCore::ili9341_init(void)
{
    // toggle RST low to reset
    digitalWrite(PIN_DISP_RST, HIGH);
    delay(5);
    digitalWrite(PIN_DISP_RST, LOW);
    delay(20);
    digitalWrite(PIN_DISP_RST, HIGH);
    delay(150);

    writecommand(0xEF);
    writedata(0x03);
    writedata(0x80);
    writedata(0x02);

    writecommand(0xCF);
    writedata(0x00);
    writedata(0XC1);
    writedata(0X30);

    writecommand(0xED);
    writedata(0x64);
    writedata(0x03);
    writedata(0X12);
    writedata(0X81);

    writecommand(0xE8);
    writedata(0x85);
    writedata(0x00);
    writedata(0x78);

    writecommand(0xCB);
    writedata(0x39);
    writedata(0x2C);
    writedata(0x00);
    writedata(0x34);
    writedata(0x02);

    writecommand(0xF7);
    writedata(0x20);

    writecommand(0xEA);
    writedata(0x00);
    writedata(0x00);

    writecommand(ILI9341_PWCTR1);    //Power control
    writedata(0x23);   //VRH[5:0]

    writecommand(ILI9341_PWCTR2);    //Power control
    writedata(0x10);   //SAP[2:0];BT[3:0]

    writecommand(ILI9341_VMCTR1);    //VCM control
    writedata(0x3e); //
    writedata(0x28);

    writecommand(ILI9341_VMCTR2);    //VCM control2
    writedata(0x86);  //--

    writecommand(ILI9341_MADCTL);    // Memory Access Control
    writedata(0x48);

    writecommand(ILI9341_PIXFMT);
    writedata(0x55);

    writecommand(ILI9341_FRMCTR1);
    writedata(0x00);
    writedata(0x18);

    writecommand(ILI9341_DFUNCTR);    // Display Function Control
    writedata(0x08);
    writedata(0x82);
    writedata(0x27);

    writecommand(0xF2);    // 3Gamma Function Disable
    writedata(0x00);

    writecommand(ILI9341_GAMMASET);    //Gamma curve selected
    writedata(0x01);

    writecommand(ILI9341_GMCTRP1);    //Set Gamma
    writedata(0x0F);
    writedata(0x31);
    writedata(0x2B);
    writedata(0x0C);
    writedata(0x0E);
    writedata(0x08);
    writedata(0x4E);
    writedata(0xF1);
    writedata(0x37);
    writedata(0x07);
    writedata(0x10);
    writedata(0x03);
    writedata(0x0E);
    writedata(0x09);
    writedata(0x00);

    writecommand(ILI9341_GMCTRN1);    //Set Gamma
    writedata(0x00);
    writedata(0x0E);
    writedata(0x14);
    writedata(0x03);
    writedata(0x11);
    writedata(0x07);
    writedata(0x31);
    writedata(0xC1);
    writedata(0x48);
    writedata(0x08);
    writedata(0x0F);
    writedata(0x0C);
    writedata(0x31);
    writedata(0x36);
    writedata(0x0F);

    // Rotate screen 90deg
    writecommand(ILI9341_MADCTL);
    writedata(ILI9341_MADCTL_MX | ILI9341_MADCTL_MY |
            ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR);

    writecommand(ILI9341_INVOFF); //Invert Off
    delay(120);
    writecommand(ILI9341_SLPOUT);    //Exit Sleep
    delay(120);
    writecommand(ILI9341_DISPON);    //Display on
}


void ArduboyAdvanceCore::setRotation(uint8_t x) {

  // Call parent rotation func first -- sets up rotation flags, etc.
//   Adafruit_GFX::setRotation(x);

    uint8_t rotation = (x & 3);

    switch(rotation) {
        case 0:
        case 2:
            // _width  = SCREEN_WIDTH;
            // _height = SCREEN_HEIGHT;
            break;
        case 1:
        case 3:
            // _width  = SCREEN_HEIGHT;
            // _height = SCREEN_WIDTH;
            break;
        default:
            break;
    }

  // Then perform hardware-specific rotation operations...

    CS_ACTIVE;

    uint16_t t = 0;

    switch (rotation) {
        case 2:
            t = ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR;
            break;
        case 3:
            t = ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR;
            break;
        case 0:
            t = ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR;
            break;
        case 1:
            t = ILI9341_MADCTL_MX | ILI9341_MADCTL_MY | ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR;
            break;
        default:
            break;
    }

    writecommand(ILI9341_MADCTL);
    writedata(t);
    // For 9341, init default full-screen address window:
    // setAddrWindow(0, 0, _width - 1, _height - 1); // CS_IDLE happens here
}

// void ArduboyAdvanceCore::bootOLED()
// {
//   // reset the display
//   delayShort(5); // reset pin should be low here. let it stay low a while
//   // bitSet(RST_PORT, RST_BIT); // set high to come out of reset
//   delayShort(5); // wait a while

//   // select the display (permanently, since nothing else is using SPI)
//   // bitClear(CS_PORT, CS_BIT);

//   // run our customized boot-up command sequence against the
//   // OLED to initialize it properly for Arduboy
//   LCDCommandMode();
//   for (uint8_t i = 0; i < sizeof(lcdBootProgram); i++) {
//     SPItransfer(pgm_read_byte(lcdBootProgram + i));
//   }
//   LCDDataMode();
// }

// void ArduboyAdvanceCore::bootTFT()
// {

//   Serial.print("W:"); Serial.print(WIDTH); Serial.print(":"); Serial.println(_width);
//   Serial.print("H:"); Serial.print(HEIGHT); Serial.print(":"); Serial.println(_height);
//   Serial.print("_cs:"); Serial.println(_cs);
//   Serial.print("_dc:"); Serial.println(_dc);
//   Serial.print("_rst:"); Serial.println(_rst);
// //   Serial.print("_mosi:"); Serial.println(_mosi);
//   Serial.print("_sclk:"); Serial.println(_sclk);

//   pinMode(_rst, OUTPUT);
//   digitalWrite(_rst, LOW);

//     // Control Pins
//     pinMode(_dc, OUTPUT);
//     digitalWrite(_dc, LOW);
//     pinMode(_cs, OUTPUT);
//     digitalWrite(_cs, HIGH);

// // #ifdef __AVR__
// //   csport    = portOutputRegister(digitalPinToPort(_cs));
// //   dcport    = portOutputRegister(digitalPinToPort(_dc));
// // #endif
// // #if defined(__SAM3X8E__)
// //   csport    = digitalPinToPort(_cs);
// //   dcport    = digitalPinToPort(_dc);
// // #endif
// // #if defined(__arm__) && defined(CORE_TEENSY)
// //   mosiport = &_mosi;
// //   clkport = &_sclk;
// //   rsport = &_rst;
// //   csport    = &_cs;
// //   dcport    = &_dc;
// // #endif

// // mosiport = &_mosi;
// // clkport = &_sclk;
// // rsport = &_rst;
// // csport    = &_cs;
// // dcport    = &_dc;
// //   cspinmask = digitalPinToBitMask(_cs);
// //   dcpinmask = digitalPinToBitMask(_dc);

//   // if(hwSPI) { // Using hardware SPI
//     // SPI.end();
//     // SPI.begin();
// // #ifdef __AVR__
// //     SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
// // #endif
// // #if defined(__SAM3X8E__)
// //     SPI.setClockDivider(11); // 85MHz / 11 = 7.6 MHz (full! speed!)
// // #endif    //SPI.setBitOrder(MSBFIRST);
//     // SPI.setBitOrder(MSBFIRST);
//     // SPI.setDataMode(SPI_MODE0);

//     // spi_dma_init();

//     // SPI.begin();
//     // SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
//     // SPI.beginTransaction(SPISettings(12000000, MSBFIRST, SPI_MODE0));
//   // } else {
//   //   pinMode(_sclk, OUTPUT);
//   //   pinMode(_mosi, OUTPUT);
//   //   pinMode(_miso, INPUT);
// // #ifdef __AVR__
// //     clkport     = portOutputRegister(digitalPinToPort(_sclk));
// //     mosiport    = portOutputRegister(digitalPinToPort(_mosi));
// // #endif
// // #if defined(__SAM3X8E__)
// //     clkport     = digitalPinToPort(_sclk);
// //     mosiport    = digitalPinToPort(_mosi);
// // #endif
//   //   clkpinmask  = digitalPinToBitMask(_sclk);
//   //   mosipinmask = digitalPinToBitMask(_mosi);
//   //   CLEAR_BIT(clkport, clkpinmask);
//   //   CLEAR_BIT(mosiport, mosipinmask);
//   // }

//     // toggle RST low to reset
//     digitalWrite(_rst, HIGH);
//     delay(5);
//     digitalWrite(_rst, LOW);
//     delay(20);
//     digitalWrite(_rst, HIGH);
//     delay(150);

//     SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

// //  /*
//   uint8_t x = readcommand8(ILI9340_RDMODE);
//   Serial.print("\nDisplay Power Mode: 0x"); Serial.println(x, HEX);
//   x = readcommand8(ILI9340_RDMADCTL);
//   Serial.print("\nMADCTL Mode: 0x"); Serial.println(x, HEX);
//   x = readcommand8(ILI9340_RDPIXFMT);
//   Serial.print("\nPixel Format: 0x"); Serial.println(x, HEX);
//   x = readcommand8(ILI9340_RDIMGFMT);
//   Serial.print("\nImage Format: 0x"); Serial.println(x, HEX);
//   x = readcommand8(ILI9340_RDSELFDIAG);
//   Serial.print("\nSelf Diagnostic: 0x"); Serial.println(x, HEX);
// //  */

//   //if(cmdList) commandList(cmdList);

//   writecommand(0xEF);
//   writedata(0x03);
//   writedata(0x80);
//   writedata(0x02);

//   writecommand(0xCF);
//   writedata(0x00);
//   writedata(0XC1);
//   writedata(0X30);

//   writecommand(0xED);
//   writedata(0x64);
//   writedata(0x03);
//   writedata(0X12);
//   writedata(0X81);

//   writecommand(0xE8);
//   writedata(0x85);
//   writedata(0x00);
//   writedata(0x78);

//   writecommand(0xCB);
//   writedata(0x39);
//   writedata(0x2C);
//   writedata(0x00);
//   writedata(0x34);
//   writedata(0x02);

//   writecommand(0xF7);
//   writedata(0x20);

//   writecommand(0xEA);
//   writedata(0x00);
//   writedata(0x00);

//   writecommand(ILI9340_PWCTR1);    //Power control
//   writedata(0x23);   //VRH[5:0]

//   writecommand(ILI9340_PWCTR2);    //Power control
//   writedata(0x10);   //SAP[2:0];BT[3:0]

//   writecommand(ILI9340_VMCTR1);    //VCM control
//   writedata(0x3e); //�Աȶȵ���
//   writedata(0x28);

//   writecommand(ILI9340_VMCTR2);    //VCM control2
//   writedata(0x86);  //--

//   writecommand(ILI9340_MADCTL);    // Memory Access Control
//   writedata(ILI9340_MADCTL_MX | ILI9340_MADCTL_BGR);

//   writecommand(ILI9340_PIXFMT);
//   writedata(0x55);

//   writecommand(ILI9340_FRMCTR1);
//   writedata(0x00);
//   writedata(0x18);

//   writecommand(ILI9340_DFUNCTR);    // Display Function Control
//   writedata(0x08);
//   writedata(0x82);
//   writedata(0x27);

//   writecommand(0xF2);    // 3Gamma Function Disable
//   writedata(0x00);

//   writecommand(ILI9340_GAMMASET);    //Gamma curve selected
//   writedata(0x01);

//   writecommand(ILI9340_GMCTRP1);    //Set Gamma
//   writedata(0x0F);
//   writedata(0x31);
//   writedata(0x2B);
//   writedata(0x0C);
//   writedata(0x0E);
//   writedata(0x08);
//   writedata(0x4E);
//   writedata(0xF1);
//   writedata(0x37);
//   writedata(0x07);
//   writedata(0x10);
//   writedata(0x03);
//   writedata(0x0E);
//   writedata(0x09);
//   writedata(0x00);

//   writecommand(ILI9340_GMCTRN1);    //Set Gamma
//   writedata(0x00);
//   writedata(0x0E);
//   writedata(0x14);
//   writedata(0x03);
//   writedata(0x11);
//   writedata(0x07);
//   writedata(0x31);
//   writedata(0xC1);
//   writedata(0x48);
//   writedata(0x08);
//   writedata(0x0F);
//   writedata(0x0C);
//   writedata(0x31);
//   writedata(0x36);
//   writedata(0x0F);

//   writecommand(ILI9340_SLPOUT);    //Exit Sleep
//   delay(120);
//   writecommand(ILI9340_DISPON);    //Display on

//   SPI.endTransaction();
// }

// void ArduboyAdvanceCore::spiwrite(uint8_t c) {
//   // Serial.print("0x"); Serial.print(c, HEX); Serial.print(", ");
//   SPI.transfer(c);
// }

// void ArduboyAdvanceCore::writecommand(uint8_t c) {
//   // CLEAR_BIT(dcport, dcpinmask);
//   digitalWrite(_dc, LOW);
//   // CLEAR_BIT(clkport, clkpinmask);
//   digitalWrite(_sclk, LOW);
//   // CLEAR_BIT(csport, cspinmask);
//   digitalWrite(_cs, LOW);

//   spiwrite(c);

//   // SET_BIT(csport, cspinmask);
//   digitalWrite(_cs, HIGH);
// }


// void ArduboyAdvanceCore::writedata(uint8_t c) {
//   // SET_BIT(dcport,  dcpinmask);
//   digitalWrite(_dc, HIGH);
//   // CLEAR_BIT(clkport, clkpinmask);
//   digitalWrite(_sclk, LOW);
//   // CLEAR_BIT(csport, cspinmask);
//   digitalWrite(_cs, LOW);

//   spiwrite(c);

//   digitalWrite(_cs, HIGH);
//   // SET_BIT(csport, cspinmask);
// }


void ArduboyAdvanceCore::setWriteDataBus(void) {
    // set the data pins to output mode
    pinMode(PIN_DISP_D0, OUTPUT);
    pinMode(PIN_DISP_D1, OUTPUT);
    pinMode(PIN_DISP_D2, OUTPUT);
    pinMode(PIN_DISP_D3, OUTPUT);
    pinMode(PIN_DISP_D4, OUTPUT);
    pinMode(PIN_DISP_D5, OUTPUT);
    pinMode(PIN_DISP_D6, OUTPUT);
    pinMode(PIN_DISP_D7, OUTPUT);
}


void ArduboyAdvanceCore::setReadDataBus(void) {
    //set the data pins to input mode
    pinMode(PIN_DISP_D0, INPUT);
    pinMode(PIN_DISP_D1, INPUT);
    pinMode(PIN_DISP_D2, INPUT);
    pinMode(PIN_DISP_D3, INPUT);
    pinMode(PIN_DISP_D4, INPUT);
    pinMode(PIN_DISP_D5, INPUT);
    pinMode(PIN_DISP_D6, INPUT);
    pinMode(PIN_DISP_D7, INPUT);
}


// void ArduboyAdvanceCore::LCDDataMode()
// {
//     // bitSet(DC_PORT, DC_BIT);
//     digitalWrite(_dc, HIGH);
//     digitalWrite(_sclk, LOW);
//     digitalWrite(_cs, LOW);
// }

// void ArduboyAdvanceCore::LCDCommandMode()
// {
//     // bitClear(DC_PORT, DC_BIT);
//     digitalWrite(_dc, LOW);
//     digitalWrite(_sclk, LOW);
//     digitalWrite(_cs, LOW);
// }

// // Initialize the SPI interface for the display
// void ArduboyAdvanceCore::bootSPI()
// {
//     SPI.begin();
//     // SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

//   // master, mode 0, MSB first, CPU clock / 2 (8MHz)
//   // SPCR = _BV(SPE) | _BV(MSTR);
//   // SPSR = _BV(SPI2X);
// }

// // Write to the SPI bus (MOSI pin)
// void ArduboyAdvanceCore::SPItransfer(uint8_t data)
// {
//     SPI.beginTransaction(SPISettings(12000000, MSBFIRST, SPI_MODE0));
//     spiwrite(data);
//     digitalWrite(_cs, HIGH);
//     SPI.endTransaction();
// }

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
// ToDo
void ArduboyAdvanceCore::displayOff()
{
  // LCDCommandMode();
  // SPItransfer(0xAE); // display off
  // SPItransfer(0x8D); // charge pump:
  // SPItransfer(0x10); //   disable
  // delayShort(250);
  // // bitClear(RST_PORT, RST_BIT); // set display reset pin low (reset state)
}

// Restart the display after a displayOff()
// ToDo
void ArduboyAdvanceCore::displayOn()
{
  // bootOLED();
}

uint16_t ArduboyAdvanceCore::getWidth() {
    return SCREEN_WIDTH;
}

uint16_t ArduboyAdvanceCore::getHeight() {
    return SCREEN_HEIGHT;
}


/* Drawing */

// // ToDo: Remove again? Comes from ILI9340 driver
// void ArduboyAdvanceCore::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
// {

//      writecommand(ILI9340_CASET); // Column addr set
//      writedata(x0 >> 8);
//      writedata(x0 & 0xFF);     // XSTART
//      writedata(x1 >> 8);
//      writedata(x1 & 0xFF);     // XEND

//      writecommand(ILI9340_PASET); // Row addr set
//      writedata(y0>>8);
//      writedata(y0);     // YSTART
//      writedata(y1>>8);
//      writedata(y1);     // YEND

//      writecommand(ILI9340_RAMWR); // write to RAM
// }

void ArduboyAdvanceCore::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    CS_ACTIVE;

    CD_COMMAND;
    // write8special(ILI9341_CASET); // Column addr set
    write8(ILI9341_CASET); // Column addr set
    CD_DATA;
    // write8special(x0 >> 8);
    // write8special(x0 & 0xFF);     // XSTART
    // write8special(x1 >> 8);
    // write8special(x1 & 0xFF);     // XEND
    write8(x0 >> 8);
    write8(x0 & 0xFF);     // XSTART
    write8(x1 >> 8);
    write8(x1 & 0xFF);     // XEND

    CD_COMMAND;
    // write8special(ILI9341_PASET); // Row addr set
    write8(ILI9341_PASET); // Row addr set
    CD_DATA;
    // write8special(y0 >> 8);
    // write8special(y0);     // YSTART
    // write8special(y1 >> 8);
    // write8special(y1);     // YEND
    write8(y0 >> 8);
    write8(y0);     // YSTART
    write8(y1 >> 8);
    write8(y1);     // YEND

    CD_COMMAND;
    // write8special(ILI9341_RAMWR); // write to RAM
    write8(ILI9341_RAMWR); // write to RAM
}


void ArduboyAdvanceCore::paint8Pixels(uint8_t pixels)
{
  SPItransfer(pixels);
}

// void ArduboyAdvanceCore::paintScreen(const uint16_t *image)
// {
//   Serial.println("ArduboyAdvanceCore::paintScreen(const uint8_t *image)");
//   for (int i = 0; i < (HEIGHT*WIDTH)/8; i++)
//   {
//     SPItransfer(pgm_read_byte(image + i));
//   }
// }

// paint from a memory buffer, this should be FAST as it's likely what
// will be used by any buffer based subclass
void ArduboyAdvanceCore::paintScreen(uint8_t image[], bool clear)
{
    setAddrWindow(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1);

    CD_DATA;

    for (uint32_t i = 0; i < SCREEN_BUF_SIZE; i += 2) {
        write8(image[i]);
        write8(image[i + 1]);
        if (clear) {
            image[i] = 0x00;
            image[i + 1] = 0x00;
        }
    }

    CS_IDLE;

    return;

// //   writecommand(ILI9340_CASET); // Column addr set
// //   writedata(0 >> 8);
// //   writedata(0 & 0xFF);     // XSTART
// //   writedata((WIDTH - 1) >> 8);
// //   writedata((WIDTH - 1) & 0xFF);     // XEND

// //   writecommand(ILI9340_PASET); // Row addr set
// //   writedata(0>>8);
// //   writedata(0);     // YSTART
// //   writedata((HEIGHT - 1) >>8);
// //   writedata(HEIGHT - 1);     // YEND

//   writecommand(ILI9340_RAMWR); // write to RAM

//   SPI.beginTransaction(SPISettings(15000000, MSBFIRST, SPI_MODE0));

//   digitalWrite(_dc, HIGH);
//   digitalWrite(_cs, LOW);

//     // SPI.transfer((uint8_t *)image, (2 * HEIGHT * WIDTH));

//     for (uint32_t i = 0; i < (HEIGHT * WIDTH); i++) {
//         SPI.transfer16(image[i]);
//     }

//   digitalWrite(_cs, HIGH);
//   SPI.endTransaction();

//     if (clear) {
//         for (uint32_t i = 0; i < (WIDTH * HEIGHT); i++) {
//             image[i] = 0;
//         }
//     }

//   return;

// //   uint8_t c;
// //   int i = 0;

// //   if (clear)
// //   {
// //     // SPDR = image[i]; // set the first SPI data byte to get things started
// //     image[i++] = 0;  // clear the first image byte
// //   }
// //   else
// //     // SPDR = image[i++];

// //   // the code to iterate the loop and get the next byte from the buffer is
// //   // executed while the previous byte is being sent out by the SPI controller
// //   while (i < (HEIGHT * WIDTH) / 8)
// //   {
// //     // get the next byte. It's put in a local variable so it can be sent as
// //     // as soon as possible after the sending of the previous byte has completed
// //     if (clear)
// //     {
// //       c = image[i];
// //       // clear the byte in the image buffer
// //       image[i++] = 0;
// //     }
// //     else
// //       c = image[i++];

// //     // while (!(SPSR & _BV(SPIF))) { } // wait for the previous byte to be sent

// //     // put the next byte in the SPI data register. The SPI controller will
// //     // clock it out while the loop continues and gets the next byte ready
// //     // SPDR = c;
// //   }
// //   // while (!(SPSR & _BV(SPIF))) { } // wait for the last byte to be sent
}

// bool ArduboyAdvanceCore::waitForDMA()
// {
//     while (!spi_dma_done()) ;
// }

void ArduboyAdvanceCore::blank()
{
    setAddrWindow(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1);

    CD_DATA;

    for (uint32_t i = 0; i < SCREEN_BUF_SIZE; i += 2) {
        write8(0x00);
        write8(0x00);
    }

    CS_IDLE;
}

// void ArduboyAdvanceCore::sendLCDCommand(uint8_t command)
// {
//   LCDCommandMode();
//   SPItransfer(command);
//   LCDDataMode();
// }

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


void ArduboyAdvanceCore::write8(uint8_t c)
{
    CS_ACTIVE;

    // digitalWriteFast(TFT_WR, LOW);
    WR_ACTIVE;
    *((volatile uint8_t *)(&GPIOC_PDOR)) = c;
    // digitalWriteFast(TFT_WR, HIGH);
    WR_IDLE;
    asm volatile("NOP"); // wait ten ns
    asm volatile("NOP");
    asm volatile("NOP");
    asm volatile("NOP");

    // asm volatile("NOP");
    // asm volatile("NOP");
    // asm volatile("NOP");
    // asm volatile("NOP");

    CS_IDLE;
}


void ArduboyAdvanceCore::writecommand(uint8_t c)
{
    CD_COMMAND;
    write8(c);
}


void ArduboyAdvanceCore::writedata(uint8_t c) {
    CD_DATA;
    write8(c);
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
#if defined(ARDUBOY_20) || defined(ARDUBOY_36)
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


////////// stuff not actively being used, but kept for posterity


// uint8_t ArduboyAdvanceCore::spiread(void) {
//   uint8_t r = 0;

//   if (hwSPI) {
// #ifdef __AVR__
//     SPDR = 0x00;
//     while(!(SPSR & _BV(SPIF)));
//     r = SPDR;
// #endif
// #if defined(USE_SPI_LIBRARY)
//     r = SPI.transfer(0x00);
// #endif
//   } else {

//     for (uint8_t i=0; i<8; i++) {
//       digitalWrite(_sclk, LOW);
//       digitalWrite(_sclk, HIGH);
//       r <<= 1;
//       if (digitalRead(_miso))
// 	r |= 0x1;
//     }
//   }
//   //Serial.print("read: 0x"); Serial.print(r, HEX);

//   return r;
// }

//  uint8_t ArduboyAdvanceCore::readdata(void) {
//    digitalWrite(_dc, HIGH);
//    digitalWrite(_cs, LOW);
//    uint8_t r = SPI.transfer(0x00);
//    digitalWrite(_cs, HIGH);

//    return r;
// }


// uint8_t ArduboyAdvanceCore::readcommand8(uint8_t c) {
//    digitalWrite(PIN_DC, LOW);
//    digitalWrite(PIN_SCK, LOW);
//    digitalWrite(PIN_CS, LOW);
//   //  spiwrite(c);
//    SPI.transfer(c);


//    digitalWrite(PIN_DC, HIGH);
//    uint8_t r = SPI.transfer(0x00);
//    digitalWrite(PIN_CS, HIGH);
//    return r;
// }


uint8_t ArduboyAdvanceCore::read8(void)
{
    RD_ACTIVE;
    delay(5);
    uint8_t temp = 0;
    if(digitalReadFast(PIN_DISP_D0)) {temp |= (1 << 0);} // slow reading but works
    if(digitalReadFast(PIN_DISP_D1)) {temp |= (1 << 1);}
    if(digitalReadFast(PIN_DISP_D2)) {temp |= (1 << 2);}
    if(digitalReadFast(PIN_DISP_D3)) {temp |= (1 << 3);}
    if(digitalReadFast(PIN_DISP_D4)) {temp |= (1 << 4);}
    if(digitalReadFast(PIN_DISP_D5)) {temp |= (1 << 5);}
    if(digitalReadFast(PIN_DISP_D6)) {temp |= (1 << 6);}
    if(digitalReadFast(PIN_DISP_D7)) {temp |= (1 << 7);}
    RD_IDLE;
    delay(5);
    return temp;
}


uint8_t ArduboyAdvanceCore::readcommand8(uint8_t c)
{
    writecommand(c);
    CS_ACTIVE;
    CD_DATA;
    setReadDataBus();
    delay(5);
    //single dummy data
    uint8_t data = read8();
    //real data
    data = read8();
    setWriteDataBus();
    CS_IDLE;
    return data;
}


uint32_t ArduboyAdvanceCore::readID(void)
{
    writecommand(ILI9341_RDDID);

    CS_ACTIVE;
    CD_DATA;
    setReadDataBus();
    uint32_t r = read8();
    r <<= 8;
    r |= read8();
    r <<= 8;
    r |= read8();
    r <<= 8;
    r |= read8();
    setWriteDataBus();
    CS_IDLE;

    return r;
}
