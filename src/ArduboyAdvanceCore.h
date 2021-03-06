/**
 * @file Arduboy2Core.h
 * \brief
 * The Arduboy2Core class for Arduboy hardware initilization and control.
 */

#ifndef ARDUBOY2_CORE_H
#define ARDUBOY2_CORE_H

#include <Arduino.h>
// #include <avr/power.h>
// #include <avr/sleep.h>
#include <limits.h>
// #include "Adafruit_ILI9340_defs.h"
#include "ILI9341_defs.h"




// main hardware compile flags

#if !defined(ARDUBOY_10) && !defined(AB_DEVKIT)
/* defaults to Arduboy Release 1.0 if not using a boards.txt file
 *
 * we default to Arduboy Release 1.0 if a compile flag has not been
 * passed to us from a boards.txt file
 *
 * if you wish to compile for the devkit without using a boards.txt
 * file simply comment out the ARDUBOY_10 define and uncomment
 * the AB_DEVKIT define like this:
 *
 *     // #define ARDUBOY_10
 *     #define AB_DEVKIT
 */
// #define ARDUBOY_36   //< compile for the production Arduboy v2.0
//#define ARDUBOY_10   //< compile for the production Arduboy v1.0
// #define AB_DEVKIT    //< compile for the official dev kit
#define ARDUBOY_20   //< compile for the Arduboy Advance v2.0
#endif

#define RGB_ON HIGH   /**< For digitially setting an RGB LED on using digitalWriteRGB() */
#define RGB_OFF LOW /**< For digitially setting an RGB LED off using digitalWriteRGB() */

// ----- Arduboy 3.6 pins -----
#if defined(ARDUBOY_36)

// Display control signals
#define PIN_DISP_CS     18
#define PIN_DISP_CD     19
#define PIN_DISP_WR     20
#define PIN_DISP_RD     21
#define PIN_DISP_RST    17

// Display data signals
#define PIN_DISP_D0     15
#define PIN_DISP_D1     22
#define PIN_DISP_D2     23
#define PIN_DISP_D3     9
#define PIN_DISP_D4     10
#define PIN_DISP_D5     13
#define PIN_DISP_D6     11
#define PIN_DISP_D7     12
#define DISP_PORT       GPIOC_PSOR

// Macros for setting the display control signals
#define RD_IDLE  	    digitalWriteFast(PIN_DISP_RD, HIGH)
#define RD_ACTIVE     digitalWriteFast(PIN_DISP_RD, LOW)
#define WR_IDLE       digitalWriteFast(PIN_DISP_WR, HIGH)
#define WR_ACTIVE     digitalWriteFast(PIN_DISP_WR, LOW)
#define CD_COMMAND    digitalWriteFast(PIN_DISP_CD, LOW)
#define CD_DATA       digitalWriteFast(PIN_DISP_CD, HIGH)
#define CS_IDLE       digitalWriteFast(PIN_DISP_CS, HIGH)
#define CS_ACTIVE     digitalWriteFast(PIN_DISP_CS, LOW)

// RGB LED
#define RED_LED 25   /**< The pin number for the red color in the RGB LED. */
#define GREEN_LED 26 /**< The pin number for the greem color in the RGB LED. */
#define BLUE_LED 28   /**< The pin number for the blue color in the RGB LED. */

// #define RED_LED_PORT 0 //PORTB
// #define RED_LED_BIT 0 //PORTB6

// #define GREEN_LED_PORT 0 //PORTB
// #define GREEN_LED_BIT 0 //PORTB7

// #define BLUE_LED_PORT 0 //PORTB
// #define BLUE_LED_BIT 0 //PORTB5

// bit values for button states
// these are determined by the buttonsState() function
#define A_BUTTON        0b00001000     /**< The A button value for functions requiring a bitmask */
#define B_BUTTON        0b00000100     /**< The B button value for functions requiring a bitmask */
#define X_BUTTON        0b00000010
#define Y_BUTTON        0b00000001
#define SEL_BUTTON      0b00010000

//Button pins
#define PIN_A_BUTTON 37
#define PIN_B_BUTTON 35
#define PIN_X_BUTTON 38
#define PIN_Y_BUTTON 36
#define PIN_JOY_SEL_BUTTON 30

//Joystick
#define PIN_JOY_X_AXIS A14
#define PIN_JOY_Y_AXIS A15
#define JOY_X_THRESHOLD_LOW 2000
#define JOY_Y_THRESHOLD_LOW 2020
#define JOY_X_THRESHOLD_HIGH 2080
#define JOY_Y_THRESHOLD_HIGH 2060
#define JOY_X_THRESHOLD_SCALED_LOW -3
#define JOY_Y_THRESHOLD_SCALED_LOW -2
#define JOY_X_THRESHOLD_SCALED_HIGH 0
#define JOY_Y_THRESHOLD_SCALED_HIGH 0 //Maybe 1
#define JOY_ANALOG_RESOLUTION 10

// Backlight
#define PIN_BACKLIGHT 29

#define PIN_SPEAKER_1 5  /**< The pin number of the first lead of the speaker */
#define PIN_SPEAKER_2 13 /**< The pin number of the second lead of the speaker */

#define SPEAKER_1_PORT 0 //PORTC
#define SPEAKER_1_DDR 0 //DDRC
#define SPEAKER_1_BIT 0 //PORTC6

#define SPEAKER_2_PORT 0 //PORTC
#define SPEAKER_2_DDR 0 //DDRC
#define SPEAKER_2_BIT 0 //PORTC7

#define RAND_SEED_IN A4 // Open analog input used for noise by initRandomSeed()
#define RAND_SEED_IN_PORTF
#define RAND_SEED_IN_BIT 0 //PORTF1
// Value for ADMUX to read the random seed pin: 2.56V reference, ADC1
#define RAND_SEED_IN_ADMUX ((1 << REFS0) | (1 << REFS1) | (1 << MUX0))

// ----- Arduboy 2.0 pins -----
#elif defined(ARDUBOY_20)

    // Display control signals
    #define PIN_DISP_CS     20
    #define PIN_DISP_CD     A5
    #define PIN_DISP_WR     4
    #define PIN_DISP_RD     21
    #define PIN_DISP_RST    A2

    // Display data signals
    #define PIN_DISP_D0     5
    #define PIN_DISP_D1     24
    #define PIN_DISP_D2     6
    #define PIN_DISP_D3     9
    #define PIN_DISP_D4     10
    #define PIN_DISP_D5     11
    #define PIN_DISP_D6     12
    #define PIN_DISP_D7     13
    #define DISP_PORT_SET   REG_PORT_OUTSET0
    #define DISP_PORT_CLR   REG_PORT_OUTCLR0
    #define DISP_PORT_OUT   REG_PORT_OUT0
    #define DISP_PORT_IN    REG_PORT_IN0

    // Macros for setting the display control signals
    #define RD_IDLE       digitalWrite(PIN_DISP_RD, HIGH)
    #define RD_ACTIVE     digitalWrite(PIN_DISP_RD, LOW)
    // #define WR_IDLE       digitalWrite(PIN_DISP_WR, HIGH)
    #define WR_IDLE       (REG_PORT_OUTSET0 = 0x4000)
    // #define WR_ACTIVE     digitalWrite(PIN_DISP_WR, LOW)
    #define WR_ACTIVE     (REG_PORT_OUTCLR0 = 0x4000)
    #define CD_COMMAND    digitalWrite(PIN_DISP_CD, LOW)
    // #define CD_COMMAND    (REG_PORT_OUTCLR0 = (1 << 6))
    #define CD_DATA       digitalWrite(PIN_DISP_CD, HIGH)
    // #define CD_DATA       (REG_PORT_OUTSET0 = (1 << 6))
    // #define CS_IDLE       digitalWrite(PIN_DISP_CS, HIGH)
    #define CS_IDLE       (REG_PORT_OUTSET0 = 0x1000)
    // #define CS_ACTIVE     digitalWrite(PIN_DISP_CS, LOW)
    #define CS_ACTIVE     (REG_PORT_OUTCLR0 = 0x1000)

    #define CS_WR_ACTIVE    (REG_PORT_OUTCLR0 = 0x5000)
    #define CS_WR_IDLE      (REG_PORT_OUTSET0 = 0x5000)

    // RGB LED
    // #define RED_LED 25   /**< The pin number for the red color in the RGB LED. */
    // #define GREEN_LED 26 /**< The pin number for the greem color in the RGB LED. */
    // #define BLUE_LED 28   /**< The pin number for the blue color in the RGB LED. */
    #define PIN_NEOPIXEL D8

// bit values for button states
// these are determined by the buttonsState() function
#define A_BUTTON        0b00001000     /**< The A button value for functions requiring a bitmask */
#define B_BUTTON        0b00000100     /**< The B button value for functions requiring a bitmask */
#define X_BUTTON        0b00000010
#define Y_BUTTON        0b00000001
#define SEL_BUTTON      0b00010000

    //Button pins
    #define PIN_A_BUTTON 1
    #define PIN_B_BUTTON 0
    #define PIN_X_BUTTON 22
    #define PIN_Y_BUTTON 23

    //Joystick
    #define PIN_JOY_X_AXIS A4
    #define PIN_JOY_Y_AXIS A1
    #define JOY_X_THRESHOLD_LOW 2000
    #define JOY_Y_THRESHOLD_LOW 2020
    #define JOY_X_THRESHOLD_HIGH 2080
    #define JOY_Y_THRESHOLD_HIGH 2060
    #define JOY_X_THRESHOLD_SCALED_LOW -3
    #define JOY_Y_THRESHOLD_SCALED_LOW -2
    #define JOY_X_THRESHOLD_SCALED_HIGH 0
    #define JOY_Y_THRESHOLD_SCALED_HIGH 0 //Maybe 1
    #define JOY_ANALOG_RESOLUTION 10

    // Backlight
    #define PIN_BACKLIGHT A3

    #define PIN_SPEAKER_1 5  /**< The pin number of the first lead of the speaker */
    #define PIN_SPEAKER_2 13 /**< The pin number of the second lead of the speaker */

    #define SPEAKER_1_PORT 0 //PORTC
    #define SPEAKER_1_DDR 0 //DDRC
    #define SPEAKER_1_BIT 0 //PORTC6

    #define SPEAKER_2_PORT 0 //PORTC
    #define SPEAKER_2_DDR 0 //DDRC
    #define SPEAKER_2_BIT 0 //PORTC7

    #define RAND_SEED_IN A4 // Open analog input used for noise by initRandomSeed()
    #define RAND_SEED_IN_PORTF
    #define RAND_SEED_IN_BIT 0 //PORTF1
    // Value for ADMUX to read the random seed pin: 2.56V reference, ADC1
    #define RAND_SEED_IN_ADMUX ((1 << REFS0) | (1 << REFS1) | (1 << MUX0))

    #define SCREEN_WIDTH 320 /**< The width of the display in pixels */
    #define SCREEN_HEIGHT 240 /**< The height of the display in pixels */

    #define SCREEN_BUF_SIZE (2 * SCREEN_WIDTH * SCREEN_HEIGHT)

// -----------------------

// ----- Arduboy 1.0 pins -----
// #elif defined(ARDUBOY_10)

// #define PIN_CS 12       // Display CS Arduino pin number
// #define CS_PORT PORTD   // Display CS port
// #define CS_BIT PORTD6   // Display CS physical bit number

// #define PIN_DC 4        // Display D/C Arduino pin number
// #define DC_PORT PORTD   // Display D/C port
// #define DC_BIT PORTD4   // Display D/C physical bit number

// #define PIN_RST 6       // Display reset Arduino pin number
// #define RST_PORT PORTD  // Display reset port
// #define RST_BIT PORTD7  // Display reset physical bit number

// #define SPI_MOSI_PORT PORTB
// #define SPI_MOSI_BIT PORTB2

// #define SPI_SCK_PORT PORTB
// #define SPI_SCK_BIT PORTB1

// #define RED_LED 10   /**< The pin number for the red color in the RGB LED. */
// #define GREEN_LED 11 /**< The pin number for the greem color in the RGB LED. */
// #define BLUE_LED 9   /**< The pin number for the blue color in the RGB LED. */

// #define RED_LED_PORT PORTB
// #define RED_LED_BIT PORTB6

// #define GREEN_LED_PORT PORTB
// #define GREEN_LED_BIT PORTB7

// #define BLUE_LED_PORT PORTB
// #define BLUE_LED_BIT PORTB5

// // bit values for button states
// // these are determined by the buttonsState() function
// #define LEFT_BUTTON 0b00100000  /**< The Left button value for functions requiring a bitmask */
// // #define LEFT_BUTTON _BV(5)  /**< The Left button value for functions requiring a bitmask */
// #define RIGHT_BUTTON 0b01000000 /**< The Right button value for functions requiring a bitmask */
// // #define RIGHT_BUTTON _BV(6) /**< The Right button value for functions requiring a bitmask */
// #define UP_BUTTON 0b10000000    /**< The Up button value for functions requiring a bitmask */
// // #define UP_BUTTON _BV(7)    /**< The Up button value for functions requiring a bitmask */
// #define DOWN_BUTTON 0b00010000  /**< The Down button value for functions requiring a bitmask */
// // #define DOWN_BUTTON _BV(4)  /**< The Down button value for functions requiring a bitmask */
// #define A_BUTTON 0b00001000     /**< The A button value for functions requiring a bitmask */
// // #define A_BUTTON _BV(3)     /**< The A button value for functions requiring a bitmask */
// #define B_BUTTON 0b00000100     /**< The B button value for functions requiring a bitmask */
// // #define B_BUTTON _BV(2)     /**< The B button value for functions requiring a bitmask */

// #define PIN_LEFT_BUTTON A2
// #define LEFT_BUTTON_PORT PORTF
// #define LEFT_BUTTON_BIT PORTF5

// #define PIN_RIGHT_BUTTON A1
// #define RIGHT_BUTTON_PORT PORTF
// #define RIGHT_BUTTON_BIT PORTF6

// #define PIN_UP_BUTTON A0
// #define UP_BUTTON_PORT PORTF
// #define UP_BUTTON_BIT PORTF7

// #define PIN_DOWN_BUTTON A3
// #define DOWN_BUTTON_PORT PORTF
// #define DOWN_BUTTON_BIT PORTF4

// #define PIN_A_BUTTON 7
// #define A_BUTTON_PORT PORTE
// #define A_BUTTON_BIT PORTE6

// #define PIN_B_BUTTON 8
// #define B_BUTTON_PORT PORTB
// #define B_BUTTON_BIT PORTB4

// #define PIN_SPEAKER_1 5  /**< The pin number of the first lead of the speaker */
// #define PIN_SPEAKER_2 13 /**< The pin number of the second lead of the speaker */

// #define SPEAKER_1_PORT PORTC
// #define SPEAKER_1_DDR DDRC
// #define SPEAKER_1_BIT PORTC6

// #define SPEAKER_2_PORT PORTC
// #define SPEAKER_2_DDR DDRC
// #define SPEAKER_2_BIT PORTC7

// #define RAND_SEED_IN A4 // Open analog input used for noise by initRandomSeed()
// #define RAND_SEED_IN_PORTF
// #define RAND_SEED_IN_BIT PORTF1
// // Value for ADMUX to read the random seed pin: 2.56V reference, ADC1
// #define RAND_SEED_IN_ADMUX (_BV(REFS0) | _BV(REFS1) | _BV(MUX0))
// -----------------------

// ----- DevKit pins -----
// #elif defined(AB_DEVKIT)

// #define PIN_CS 6        // Display CS Arduino pin number
// #define CS_PORT PORTD   // Display CS port
// #define CS_BIT PORTD7   // Display CS physical bit number

// #define PIN_DC 4        // Display D/C Arduino pin number
// #define DC_PORT PORTD   // Display D/C port
// #define DC_BIT PORTD4   // Display D/C physical bit number

// #define PIN_RST 12      // Display reset Arduino pin number
// #define RST_PORT PORTD  // Display reset port
// #define RST_BIT PORTD6  // Display reset physical bit number

// #define SPI_MOSI_PORT PORTB
// #define SPI_MOSI_BIT PORTB2

// #define SPI_SCK_PORT PORTB
// #define SPI_SCK_BIT PORTB1

// // map all LEDs to the single TX LED on DEVKIT
// #define RED_LED 17
// #define GREEN_LED 17
// #define BLUE_LED 17

// #define BLUE_LED_PORT PORTB
// #define BLUE_LED_BIT PORTB0

// // bit values for button states
// // these are determined by the buttonsState() function
// #define LEFT_BUTTON _BV(5)
// #define RIGHT_BUTTON _BV(2)
// #define UP_BUTTON _BV(4)
// #define DOWN_BUTTON _BV(6)
// #define A_BUTTON _BV(1)
// #define B_BUTTON _BV(0)

// // pin values for buttons, probably shouldn't use these
// #define PIN_LEFT_BUTTON 9
// #define LEFT_BUTTON_PORT PORTB
// #define LEFT_BUTTON_BIT PORTB5

// #define PIN_RIGHT_BUTTON 5
// #define RIGHT_BUTTON_PORT PORTC
// #define RIGHT_BUTTON_BIT PORTC6

// #define PIN_UP_BUTTON 8
// #define UP_BUTTON_PORT PORTB
// #define UP_BUTTON_BIT PORTB4

// #define PIN_DOWN_BUTTON 10
// #define DOWN_BUTTON_PORT PORTB
// #define DOWN_BUTTON_BIT PORTB6

// #define PIN_A_BUTTON A0
// #define A_BUTTON_PORT PORTF
// #define A_BUTTON_BIT PORTF7

// #define PIN_B_BUTTON A1
// #define B_BUTTON_PORT PORTF
// #define B_BUTTON_BIT PORTF6

// #define PIN_SPEAKER_1 A2
// #define SPEAKER_1_PORT PORTF
// #define SPEAKER_1_DDR DDRF
// #define SPEAKER_1_BIT PORTF5
// // SPEAKER_2 is purposely not defined for DEVKIT as it could potentially
// // be dangerous and fry your hardware (because of the devkit wiring).
// //
// // Reference: https://github.com/Arduboy/Arduboy/issues/108

// #define RAND_SEED_IN A4 // Open analog input used for noise by initRandomSeed()
// #define RAND_SEED_IN_PORTF
// #define RAND_SEED_IN_BIT PORTF1
// // Value for ADMUX to read the random seed pin: 2.56V reference, ADC1
// #define RAND_SEED_IN_ADMUX (_BV(REFS0) | _BV(REFS1) | _BV(MUX0))

#endif
// --------------------

// OLED hardware (SSD1306)

#define OLED_PIXELS_INVERTED 0xA7 // All pixels inverted
#define OLED_PIXELS_NORMAL 0xA6 // All pixels normal

#define OLED_ALL_PIXELS_ON 0xA5 // all pixels on
#define OLED_PIXELS_FROM_RAM 0xA4 // pixels mapped to display RAM contents

#define OLED_VERTICAL_FLIPPED 0xC0 // reversed COM scan direction
#define OLED_VERTICAL_NORMAL 0xC8 // normal COM scan direction

#define OLED_HORIZ_FLIPPED 0xA0 // reversed segment re-map
#define OLED_HORIZ_NORMAL 0xA1 // normal segment re-map

// -----

// #define SCREEN_WIDTH 240 /**< The width of the display in pixels */
// #define SCREEN_HEIGHT 320 /**< The height of the display in pixels */


/** \brief
 * Lower level functions generally dealing directly with the hardware.
 *
 * \details
 * This class is inherited by Arduboy2Base and thus also Arduboy2, so wouldn't
 * normally be used directly by a sketch.
 *
 * \note
 * A friend class named _Arduboy2Ex_ is declared by this class. The intention
 * is to allow a sketch to create an _Arduboy2Ex_ class which would have access
 * to the private and protected members of the Arduboy2Core class. It is hoped
 * that this may eliminate the need to create an entire local copy of the
 * library, in order to extend the functionality, in most circumstances.
 */
class ArduboyAdvanceCore
{
  friend class Arduboy2Ex;

  public:
    ArduboyAdvanceCore();

    /** \brief
     * Idle the CPU to save power.
     *
     * \details
     * This puts the CPU in _idle_ sleep mode. You should call this as often
     * as you can for the best power savings. The timer 0 overflow interrupt
     * will wake up the chip every 1ms, so even at 60 FPS a well written
     * app should be able to sleep maybe half the time in between rendering
     * it's own frames.
     */
    void static idle();

    /** \brief
     * Put the display into data mode.
     *
     * \details
     * When placed in data mode, data that is sent to the display will be
     * considered as data to be displayed.
     *
     * \note
     * This is a low level function that is not intended for general use in a
     * sketch. It has been made public and documented for use by derived
     * classes.
     *
     * \see LCDCommandMode() SPItransfer()
     */
    // void static LCDDataMode();

    /** \brief
     * Put the display into command mode.
     *
     * \details
     * When placed in command mode, data that is sent to the display will be
     * treated as commands.
     *
     * See the SSD1306 controller and OLED display documents for available
     * commands and command sequences.
     *
     * Links:
     *
     * - https://www.adafruit.com/datasheets/SSD1306.pdf
     * - http://www.buydisplay.com/download/manual/ER-OLED013-1_Series_Datasheet.pdf
     *
     * \note
     * This is a low level function that is not intended for general use in a
     * sketch. It has been made public and documented for use by derived
     * classes.
     *
     * \see LCDDataMode() sendLCDCommand() SPItransfer()
     */
    void static LCDCommandMode();

    /** \brief
     * Transfer a byte to the display.
     *
     * \param data The byte to be sent to the display.
     *
     * \details
     * Transfer one byte to the display over the SPI port and wait for the
     * transfer to complete. The byte will either be interpreted as a command
     * or as data to be placed on the screen, depending on the command/data
     * mode.
     *
     * \see LCDDataMode() LCDCommandMode() sendLCDCommand()
     */
    void static SPItransfer(uint8_t data);

    /** \brief
     * Turn the display off.
     *
     * \details
     * The display will clear and be put into a low power mode. This can be
     * used to extend battery life when a game is paused or when a sketch
     * doesn't require anything to be displayed for a relatively long period
     * of time.
     *
     * \see displayOn()
     */
    void static displayOff();

    /** \brief
     * Turn the display on.
     *
     * \details
     * Used to power up and reinitialize the display after calling
     * `displayOff()`.
     *
     * \note
     * The previous call to `displayOff()` will have caused the display's
     * buffer contents to be lost. The display will have to be re-painted,
     * which is usually done by calling `display()`.
     *
     * \see displayOff()
     */
    void static displayOn();

    /** \brief
     * Get the width of the display in pixels.
     *
     * \return The width of the display in pixels.
     *
     * \note
     * In most cases, the defined value `WIDTH` would be better to use instead
     * of this function.
     */
    uint16_t static getWidth();

    /** \brief
     * Get the height of the display in pixels.
     *
     * \return The height of the display in pixels.
     *
     * \note
     * In most cases, the defined value `HEIGHT` would be better to use instead
     * of this function.
     */
    uint16_t static getHeight();

    /** \brief
     * Get the current state of all buttons as a bitmask.
     *
     * \return A bitmask of the state of all the buttons.
     *
     * \details
     * The returned mask contains a bit for each button. For any pressed button,
     * its bit will be 1. For released buttons their associated bits will be 0.
     *
     * The following defined mask values should be used for the buttons:
     *
     * LEFT_BUTTON, RIGHT_BUTTON, UP_BUTTON, DOWN_BUTTON, A_BUTTON, B_BUTTON
     */
    uint8_t static buttonsState();

    /** \brief
     * Paint 8 pixels vertically to the display.
     *
     * \param pixels A byte whose bits specify a vertical column of 8 pixels.
     *
     * \details
     * A byte representing a vertical column of 8 pixels is written to the
     * display at the current page and column address. The address is then
     * incremented. The page/column address will wrap to the start of the
     * display (the top left) when it increments past the end (lower right).
     *
     * The least significant bit represents the top pixel in the column.
     * A bit set to 1 is lit, 0 is unlit.
     *
     * Example:
     *
     *     X = lit pixels, . = unlit pixels
     *
     *     blank()                          paint8Pixels() 0xFF, 0, 0xF0, 0, 0x0F
     *     v TOP LEFT corner (8x9)          v TOP LEFT corner
     *     . . . . . . . . (page 1)         X . . . X . . . (page 1)
     *     . . . . . . . .                  X . . . X . . .
     *     . . . . . . . .                  X . . . X . . .
     *     . . . . . . . .                  X . . . X . . .
     *     . . . . . . . .                  X . X . . . . .
     *     . . . . . . . .                  X . X . . . . .
     *     . . . . . . . .                  X . X . . . . .
     *     . . . . . . . . (end of page 1)  X . X . . . . . (end of page 1)
     *     . . . . . . . . (page 2)         . . . . . . . . (page 2)
     */
    void static paint8Pixels(uint8_t pixels);

    /** \brief
     * Paints an entire image directly to the display from program memory.
     *
     * \param image A byte array in program memory representing the entire
     * contents of the display.
     *
     * \details
     * The contents of the specified array in program memory is written to the
     * display. Each byte in the array represents a vertical column of 8 pixels
     * with the least significant bit at the top. The bytes are written starting
     * at the top left, progressing horizontally and wrapping at the end of each
     * row, to the bottom right. The size of the array must exactly match the
     * number of pixels in the entire display.
     *
     * \see paint8Pixels()
     */
    void static paintScreen(const uint8_t *image);

    /** \brief
     * Paints an entire image directly to the display from an array in RAM.
     *
     * \param image A byte array in RAM representing the entire contents of
     * the display.
     * \param clear If `true` the array in RAM will be cleared to zeros upon
     * return from this function. If `false` the RAM buffer will remain
     * unchanged. (optional; defaults to `false`)
     *
     * \details
     * The contents of the specified array in RAM is written to the display.
     * Each byte in the array represents a vertical column of 8 pixels with
     * the least significant bit at the top. The bytes are written starting
     * at the top left, progressing horizontally and wrapping at the end of
     * each row, to the bottom right. The size of the array must exactly
     * match the number of pixels in the entire display.
     *
     * If parameter `clear` is set to `true` the RAM array will be cleared to
     * zeros after its contents are written to the display.
     *
     * \see paint8Pixels()
     */
    volatile void static paintScreen(uint8_t image[], bool clear = false);

    /** \brief
     * Blank the display screen by setting all pixels off.
     *
     * \details
     * All pixels on the screen will be written with a value of 0 to turn
     * them off.
     */
    void static blank();

    /** \brief
     * Invert the entire display or set it back to normal.
     *
     * \param inverse `true` will invert the display. `false` will set the
     * display to no-inverted.
     *
     * \details
     * Calling this function with a value of `true` will set the display to
     * inverted mode. A pixel with a value of 0 will be on and a pixel set to 1
     * will be off.
     *
     * Once in inverted mode, the display will remain this way
     * until it is set back to non-inverted mode by calling this function with
     * `false`.
     */
    void static invert(bool inverse);

    /** \brief
     * Turn all display pixels on or display the buffer contents.
     *
     * \param on `true` turns all pixels on. `false` displays the contents
     * of the hardware display buffer.
     *
     * \details
     * Calling this function with a value of `true` will override the contents
     * of the hardware display buffer and turn all pixels on. The contents of
     * the hardware buffer will remain unchanged.
     *
     * Calling this function with a value of `false` will set the normal state
     * of displaying the contents of the hardware display buffer.
     *
     * \note
     * All pixels will be lit even if the display is in inverted mode.
     *
     * \see invert()
     */
    void static allPixelsOn(bool on);

    /** \brief
     * Flip the display vertically or set it back to normal.
     *
     * \param flipped `true` will set vertical flip mode. `false` will set
     * normal vertical orientation.
     *
     * \details
     * Calling this function with a value of `true` will cause the Y coordinate
     * to start at the bottom edge of the display instead of the top,
     * effectively flipping the display vertically.
     *
     * Once in vertical flip mode, it will remain this way until normal
     * vertical mode is set by calling this function with a value of `false`.
     *
     * \see flipHorizontal()
     */
    void static flipVertical(bool flipped);

    /** \brief
     * Flip the display horizontally or set it back to normal.
     *
     * \param flipped `true` will set horizontal flip mode. `false` will set
     * normal horizontal orientation.
     *
     * \details
     * Calling this function with a value of `true` will cause the X coordinate
     * to start at the left edge of the display instead of the right,
     * effectively flipping the display horizontally.
     *
     * Once in horizontal flip mode, it will remain this way until normal
     * horizontal mode is set by calling this function with a value of `false`.
     *
     * \see flipVertical()
     */
    void static flipHorizontal(bool flipped);

    /** \brief
     * Send a single command byte to the display.
     *
     * \param command The command byte to send to the display.
     *
     * \details
     * The display will be set to command mode then the specified command
     * byte will be sent. The display will then be set to data mode.
     * Multi-byte commands can be sent by calling this function multiple times.
     *
     * \note
     * Sending improper commands to the display can place it into invalid or
     * unexpected states, possibly even causing physical damage.
     */
    void static sendLCDCommand(uint8_t command);

    /** \brief
     * Set the light output of the RGB LED.
     *
     * \param red,green,blue The brightness value for each LED.
     *
     * \details
     * The RGB LED is actually individual red, green and blue LEDs placed
     * very close together in a single package. By setting the brightness of
     * each LED, the RGB LED can show various colors and intensities.
     * The brightness of each LED can be set to a value from 0 (fully off)
     * to 255 (fully on).
     *
     * \note
     * \parblock
     * Certain libraries that take control of the hardware timers may interfere
     * with the ability of this function to properly control the RGB LED.
     *_ArduboyPlaytune_ is one such library known to do this.
     * The digitalWriteRGB() function will still work properly in this case.
     * \endparblock
     *
     * \note
     * \parblock
     * Many of the Kickstarter Arduboys were accidentally shipped with the
     * RGB LED installed incorrectly. For these units, the green LED cannot be
     * lit. As long as the green led is set to off, setting the red LED will
     * actually control the blue LED and setting the blue LED will actually
     * control the red LED. If the green LED is turned fully on, none of the
     * LEDs will light.
     * \endparblock
     *
     * \see digitalWriteRGB()
     */
    void static setRGBled(uint8_t red, uint8_t green, uint8_t blue);

    /** \brief
     * Set the RGB LEDs digitally, to either fully on or fully off.
     *
     * \param red,green,blue Use value RGB_ON or RGB_OFF to set each LED.
     *
     * \details
     * The RGB LED is actually individual red, green and blue LEDs placed
     * very close together in a single package. This 3 parameter version of the
     * function will set each LED either on or off, to set the RGB LED to
     * 7 different colors at their highest brightness or turn it off.
     *
     * The colors are as follows:
     *
     *     RED LED   GREEN_LED   BLUE_LED   COLOR
     *     -------   ---------  --------    -----
     *     RGB_OFF    RGB_OFF    RGB_OFF    OFF
     *     RGB_OFF    RGB_OFF    RGB_ON     Blue
     *     RGB_OFF    RGB_ON     RGB_OFF    Green
     *     RGB_OFF    RGB_ON     RGB_ON     Cyan
     *     RGB_ON     RGB_OFF    RGB_OFF    Red
     *     RGB_ON     RGB_OFF    RGB_ON     Magenta
     *     RGB_ON     RGB_ON     RGB_OFF    Yellow
     *     RGB_ON     RGB_ON     RGB_ON     White
     *
     * \note
     * Many of the Kickstarter Arduboys were accidentally shipped with the
     * RGB LED installed incorrectly. For these units, the green LED cannot be
     * lit. As long as the green led is set to off, turning on the red LED will
     * actually light the blue LED and turning on the blue LED will actually
     * light the red LED. If the green LED is turned on, none of the LEDs
     * will light.
     *
     * \see digitalWriteRGB(uint8_t, uint8_t) setRGBled()
     */
    void static digitalWriteRGB(uint8_t red, uint8_t green, uint8_t blue);

    /** \brief
     * Set one of the RGB LEDs digitally, to either fully on or fully off.
     *
     * \param color The name of the LED to set. The value given should be one
     * of RED_LED, GREEN_LED or BLUE_LED.
     *
     * \param val Indicates whether to turn the specified LED on or off.
     * The value given should be RGB_ON or RGB_OFF.
     *
     * \details
     * This 2 parameter version of the function will set a single LED within
     * the RGB LED either fully on or fully off. See the description of the
     * 3 parameter version of this function for more details on the RGB LED.
     *
     * \see digitalWriteRGB(uint8_t, uint8_t, uint8_t) setRGBled()
     */
    void static digitalWriteRGB(uint8_t color, uint8_t val);

    /** \brief
     * Initialize the Arduboy's hardware.
     *
     * \details
     * This function initializes the display, buttons, etc.
     *
     * This function is called by begin() so isn't normally called within a
     * sketch. However, in order to free up some code space, by eliminating
     * some of the start up features, it can be called in place of begin().
     * The functions that begin() would call after boot() can then be called
     * to add back in some of the start up features, if desired.
     * See the README file or documentation on the main page for more details.
     *
     * \see Arduboy2Base::begin()
     */
    void static boot();
    // void boot();

    /** \brief
     * Allow upload when the bootloader "magic number" could be corrupted.
     *
     * \details
     * If the UP button is held when this function is entered, the RGB LED
     * will be lit and timer 0 will be disabled, then the sketch will remain
     * in a tight loop. This is to address a problem with uploading a new
     * sketch, for sketches that interfere with the bootloader "magic number".
     * The problem occurs with certain sketches that use large amounts of RAM.
     *
     * This function should be called after `boot()` in sketches that
     * potentially could cause the problem.
     *
     * It is intended to replace the `flashlight()` function when more
     * program space is required. If possible, it is more desirable to use
     * `flashlight()`, so that the actual flashlight feature isn't lost.
     *
     * \see Arduboy2Base::flashlight() boot()
     */
    void static safeMode();

    /** \brief
     * Delay for the number of milliseconds, specified as a 16 bit value.
     *
     * \param ms The delay in milliseconds.
     *
     * \details
     * This function works the same as the Arduino `delay()` function except
     * the provided value is 16 bits long, so the maximum delay allowed is
     * 65535 milliseconds (about 65.5 seconds). Using this function instead
     * of Arduino `delay()` will save a few bytes of code.
     */
    void static delayShort(uint16_t ms) __attribute__ ((noinline));


    // ToDo: Remove?
    void static setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

    void static spiwrite(uint8_t);

    uint32_t static readID(void);


  protected:
    // internals
    void static setCPUSpeed8MHz();
    void static bootSPI();
    void static bootOLED();
    void static bootPins();
    void static bootPowerSaving();
    void static bootTFT();

    void static setRotation(uint8_t x);


    // void static writecommand(uint8_t c);
    // void static writedata(uint8_t c);
    // uint8_t static readcommand8(uint8_t c);

    // New Parallel functions
    void static gpio_init();
    void static ili9341_init(void);

    void static setWriteDataBus(void);
    void static setReadDataBus(void);

    void static write8(uint8_t c);
    void static writecommand(uint8_t c);
    void static writedata(uint8_t c);

    uint8_t static read8(void);
    uint8_t static readcommand8(uint8_t c);

  };

#endif
