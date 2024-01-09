#include <energyic_SPI.h>
#include <energyic_UART.h>


/**********************************************************************************
 * 
 * Interfacing Arduino with KS0108 Monochrome GLCD.
 * This example is for KS0108 GLCD modules with 128x64 Pixel resolution (two CS pins).
 * This is a free software with NO WARRANTY - Use it at your own risk!
 * https://simple-circuit.com/
 *
***********************************************************************************
 Written by Limor Fried/Ladyada for Adafruit Industries,
 with contributions from the open source community.
 BSD license, check license.txt for more information
 All text above, and the splash screen below must be
 included in any redistribution.
************************************************************************************
* Modified to work with KS0108 monochrome GLCD. More information including circuit
*   diagram on:
* https://simple-circuit.com/
* 
 **********************************************************************************/

#include <Adafruit_GFX.h>   // include adafruit GFX library
#include "KS0108_GLCD.h"    // include KS0108 GLCD library
#include <ATM90E32.h>
#include <Wire.h>
#include <SPI.h>


#define UpperLimitfreq  50000
#define LowLimitfreq  50000
#define voltageA_Limit  50
#define voltageB_Limit  50


/* UART PINS */
#define UART_RX_PIN  5  // Arduino D5
#define UART_TX_PIN  6  // Arduino D6

/* I2C PINS */
#define I2C0_SDA_PIN  A4  // Arduino A4
#define I2C0_SCL_PIN  A5  // Arduino A5
#define I2C1_SDA_PIN  2   // Arduino D2
#define I2C1_SCL_PIN  6   // Arduino D6

/* SPI PINS */
#define SDI_PIN       4   // Arduino D4
#define SDO_PIN       5   // Arduino D5
#define SCLK_PIN      6   // Arduino D6

/* INPUTs */
#define IRQ1_PIN      19  // Arduino D19
#define IRQ0_PIN      18  // Arduino D18
#define BUTTON_PIN    2   // Arduino D2
#define WARMOUT_PIN   3   // Arduino D3
#define ZX2_PIN       4   // Arduino D4
#define ZX1_PIN       5   // Arduino D5
#define ZX0_PIN       A0  // Arduino A0
#define SELF_TEST_PIN 23  // Arduino D23
#define PHASE_2_PIN   A1  // Arduino A1

/* OUTPUTS */
#define ATM_CS_PIN    A3  // Arduino A3
#define ATM_PM1_PIN   A2  // Arduino A2
#define ATM_PM0_PIN   A4  // Arduino A4
#define ENABLE_PIN    3   // Arduino D3
#define SQUARE_WAVE_2 7   // Arduino D7
#define SQUARE_WAVE_1 8   // Arduino D8


/// TIMEOUT BASED SETTINGS and PARAMETERS /// 

#define NUM_OF_TIMEOUTS 5  // Adjust the number of timeouts as needed
typedef struct {
  bool timeout_activated;
  u32 countdown;
  u32 timeout_status;
} timeout_file_t;


timeout_file_t timeouts[NUM_OF_TIMEOUTS];

#define TO_LOADED        0
#define TO_COUNTING_DOWN 1
#define TO_OCCURED       2
#define TO_IDLE          3

volatile u32 system_tick = 0;


// Button BASED SETTINGS and PARAMETERS  /// 

#define UP_BUTTON       0
#define SELF_TEST       1

#define BUTTON_RELEASED  0
#define BUTTON_PRESSED   1

#define TRUE  1
#define FALSE 0

typedef struct {
  uint8_t last_value;
  uint8_t pin;
  uint8_t value;
} button_t;

u8 button_state;
button_t button[2]; // Assuming there are two buttons, adjust the size as needed

// I2C SETTINGS and PARAMETERS //

#define I2C_ADDRESS 0x12  // Replace with the actual I2C address
#define I2C_TIMEOUT 20    // Replace with the actual timeout value in milliseconds


// ADC SETTINGS and PARAMETERS // 
#define MAX_ADC_SAMPLE 15
#define SYSTEM_ADC_REF_V 2500 // 3300 //
#define SYSTEM_BATTARY_REF_V 3800 // 3300 //
#define BATTARY_RESISTOR_RATE 2
#define SF6_AN_ADC 6        //Equal to adc channel number
#define SPT100_AN_ADC 7     //Equal to adc channel number
#define NUM_OF_SENSOR_ADCS 16 
u8 adc_cnter = MAX_ADC_SAMPLE;
u16 temp_values[NUM_OF_SENSOR_ADCS];
volatile bool adc_flag = FALSE;
u32 supplyVoltage[2];


// Submain SETTINGS and PARAMETERS // 

// Define constants
#define CONNECTED 1
#define UNCONNECTED 0
#define NUM_OF_SELF_TEST_CYCLES 10
#define SYSTEM_BATTARY_REF_V 5.0
#define BATTARY_RESISTOR_RATE 2.0
#define ONGOING 0
#define DONE_OK 1
#define DONE_FALSE 2

volatile u8 timer_tick = 0;
volatile u8 i2c_0_tx_timeout_ms = 20;
volatile u8 i2c_0_rx_timeout_ms = 20;
volatile u8 gndProb = CONNECTED;
volatile u8 selfTestResult = 0;
volatile u8 selfTestStatus = 0;
volatile u8 selfTestPhases[NUM_OF_SELF_TEST_CYCLES];
volatile u32 angle_A, angle_B, angle_c, frequency, voltage_A, voltage_B;
char ln1_angle = 0;
char ln2_angle = 0;
u8 cnt_btn = 0;


// ATM90e32 SETTINGS and PARAMETERS // 

#define LineFreq_50  389
#define LineFreq_60 4485
#define PGain_1x 0
#define PGain_2x 21
#define PGain_4x 42
#define ATM_WRITE 0 // WRITE SPI
#define ATM_READ 1 	// READ SPI
#define ATM_DEBUG_SERIAL 1
// Create an instance of the ATM90E32 library
ATM90E32 energyChip{};
unsigned short _lineFreq;
unsigned short _pgagain;
unsigned short _ugain;
unsigned short _igainA;
unsigned short _igainB;
unsigned short _igainC;



// KS0108 GLCD library initialization according to the following connection:
// KS0108_GLCD(DI, RW, E, DB0, DB1, DB2, DB3, DB4, DB5, DB6, DB7, CS1, CS2, RES);
KS0108_GLCD display = KS0108_GLCD(A0, A1, A2, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12);



// 'orion logo black', 128x64px

const unsigned char myBitmap [] PROGMEM = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0x00, 0x03, 0xfc, 0x00, 0x07, 0xe0, 0x00, 0x0e, 0x1e, 0x00, 0x01, 0xe0, 0x00, 0x1f, 
	0xff, 0xfd, 0x00, 0x01, 0xf8, 0x00, 0x03, 0xc0, 0x00, 0x1e, 0x1e, 0x00, 0x00, 0xe0, 0x00, 0x0f, 
	0xff, 0xfc, 0x80, 0x01, 0xf0, 0x00, 0x03, 0x80, 0x00, 0x1c, 0x1c, 0x00, 0x00, 0xe0, 0x00, 0x0f, 
	0xff, 0xf8, 0x40, 0x01, 0xf0, 0x00, 0x03, 0x80, 0x00, 0x1c, 0x3c, 0x00, 0x01, 0xc0, 0x00, 0x0f, 
	0xff, 0xf8, 0x00, 0x01, 0xf0, 0x7f, 0x87, 0x87, 0xff, 0xf8, 0x3c, 0x3f, 0xc1, 0xc1, 0xfe, 0x0f, 
	0xff, 0xf8, 0x20, 0x03, 0xe0, 0xff, 0x07, 0x07, 0xff, 0xf8, 0x38, 0x3f, 0xc1, 0xc3, 0xfe, 0x1f, 
	0xff, 0xf0, 0x10, 0x03, 0xe0, 0xff, 0x07, 0x0f, 0xff, 0xf8, 0x78, 0x7f, 0xc3, 0x83, 0xfc, 0x1f, 
	0xff, 0xf0, 0x00, 0x03, 0xe1, 0xff, 0x0e, 0x0f, 0xff, 0xf0, 0x78, 0x7f, 0x83, 0x87, 0xfc, 0x1f, 
	0xff, 0xf0, 0x08, 0x07, 0xc1, 0xfe, 0x0e, 0x0f, 0xff, 0xf0, 0x70, 0x7f, 0x83, 0x87, 0xfc, 0x3f, 
	0xff, 0xe0, 0x04, 0x07, 0xc1, 0xfe, 0x0e, 0x1f, 0xff, 0xf0, 0xf0, 0xff, 0x87, 0x07, 0xf8, 0x3f, 
	0xff, 0xe0, 0x00, 0x0f, 0xc3, 0xfe, 0x1c, 0x1f, 0xff, 0xe0, 0xe0, 0xff, 0x07, 0x0f, 0xf8, 0x3f, 
	0xff, 0xe0, 0x02, 0x0f, 0x83, 0xfc, 0x1c, 0x1f, 0xff, 0xe0, 0xe0, 0xff, 0x07, 0x0f, 0xf8, 0x7f, 
	0xff, 0xc0, 0x01, 0x0f, 0x83, 0xfc, 0x1c, 0x3f, 0xff, 0xe1, 0xe0, 0xfe, 0x0e, 0x0f, 0xf0, 0x7f, 
	0xff, 0xc0, 0x00, 0x1f, 0x80, 0x00, 0x38, 0x3f, 0xff, 0xc1, 0xc0, 0x00, 0x0e, 0x1f, 0xf0, 0xff, 
	0xff, 0xc0, 0x00, 0x9f, 0x80, 0x00, 0x38, 0x3f, 0xff, 0xc1, 0xc0, 0x00, 0x0c, 0x1f, 0xe0, 0xff, 
	0xff, 0xc0, 0x00, 0x5f, 0x80, 0x00, 0x78, 0x7f, 0xff, 0xc3, 0xc0, 0x00, 0x1c, 0x1f, 0xe0, 0xff, 
	0xff, 0xe0, 0x00, 0x3f, 0xc0, 0x00, 0xf0, 0x7f, 0xff, 0x83, 0xe0, 0x00, 0x3c, 0x3f, 0xe1, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0x80, 0x00, 0xfc, 0x00, 0x01, 0xe0, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xfe, 0x40, 0x00, 0xfc, 0x00, 0x01, 0xc0, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xfe, 0x20, 0x00, 0xf8, 0x00, 0x01, 0xc0, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xfc, 0x20, 0x00, 0xf8, 0x1f, 0x01, 0xc0, 0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xfc, 0x10, 0x00, 0xf8, 0x7f, 0xc3, 0x83, 0xfc, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xfc, 0x08, 0x01, 0xf0, 0x7f, 0x83, 0x87, 0xfc, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xf8, 0x08, 0x01, 0xf0, 0x00, 0x03, 0x80, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xf8, 0x04, 0x01, 0xe0, 0x00, 0x07, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xf8, 0x02, 0x03, 0xe0, 0x00, 0x07, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xf0, 0x02, 0x03, 0xe0, 0x00, 0x0f, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xf0, 0x01, 0x03, 0xc1, 0xff, 0xfe, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xf0, 0x00, 0x87, 0xc1, 0xff, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xe0, 0x00, 0x07, 0xc0, 0x00, 0x1e, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xe0, 0x00, 0x47, 0xc0, 0x00, 0x1c, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xe0, 0x00, 0x2f, 0xc0, 0x00, 0x1c, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xe0, 0x00, 0x1f, 0xc0, 0x00, 0x3e, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};



// 'lightning', 32x32px

const unsigned char lightningBitmap [] PROGMEM = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xff, 
	0xff, 0xf7, 0xbf, 0xff, 0xff, 0xf7, 0xbf, 0xff, 0xff, 0xe7, 0xbf, 0xff, 0xff, 0xe7, 0x3f, 0xff, 
	0xff, 0xef, 0x3f, 0xff, 0xff, 0xef, 0x7f, 0xff, 0xff, 0xef, 0x7f, 0xff, 0xff, 0xcf, 0x01, 0xff, 
	0xff, 0xdf, 0xf9, 0xff, 0xff, 0xdf, 0xfb, 0xff, 0xff, 0xdf, 0xf7, 0xff, 0xff, 0x9f, 0xe7, 0xff, 
	0xff, 0x80, 0xef, 0xff, 0xff, 0xfd, 0xcf, 0xff, 0xff, 0xfd, 0xdf, 0xff, 0xff, 0xfd, 0x9f, 0xff, 
	0xff, 0xf9, 0xbf, 0xff, 0xff, 0xfb, 0x3f, 0xff, 0xff, 0xfb, 0x7f, 0xff, 0xff, 0xfa, 0x7f, 0xff, 
	0xff, 0xf2, 0xff, 0xff, 0xff, 0xf5, 0xff, 0xff, 0xff, 0xf1, 0xff, 0xff, 0xff, 0xe3, 0xff, 0xff, 
	0xff, 0xf3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};



// 'lightning-131983802348435912', 16x16px    calculated by hand.

const unsigned char lightningBitmap16 [] PROGMEM = {
	0x00, 0x40, 0x00, 0xc0, 0x00, 0xc0, 0x01, 0x40, 0x02, 0x40, 0x04, 0x40, 0x08, 0x7c, 0x10, 0x08, 
	0x3e, 0x10, 0x02, 0x20, 0x02, 0x40, 0x02, 0x80, 0x03, 0x00, 0x03, 0x00, 0x02, 0x00, 0x00, 0x00
};



// 32x24px  Not Equal Phase icon Bitmap    

const unsigned char phaseCompBitmap [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0xcc, 0x00, 0x00, 0x01, 0x32, 0x00, 0x00, 0x01, 0x32, 0x00, 0x00, 0x02, 0x31, 0x00, 0x00, 0x02, 0x31, 0x00, 0x00, 
  0x04, 0x48, 0x80, 0x00, 0x04, 0x48, 0x80, 0x00, 0x04, 0x48, 0x80, 0x00, 0x08, 0x84, 0x40, 0x00, 0x08, 0x84, 0x40, 0x00, 0x08, 0x84, 0x40, 0x00, 
  0x00, 0x02, 0x21, 0x10, 0x00, 0x02, 0x21, 0x10, 0x00, 0x02, 0x21, 0x10, 0x00, 0x01, 0x12, 0x20, 0x00, 0x01, 0x12, 0x20, 0x00, 0x01, 0x12, 0x20,
  0x00, 0x00, 0x8c, 0x40, 0x00, 0x00, 0x8c, 0x40, 0x00, 0x00, 0x4c, 0x80, 0x00, 0x00, 0x4c, 0x80, 0x00, 0x00, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00

};



// 32x24px Equal Phase bitmap icon. Calculated by hand. It works fine.

const unsigned char phaseBitmap [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x48, 0x00, 0x00, 0x00, 0x48, 0x00, 0x00, 0x00, 0x84, 0x00, 0x00, 0x00, 0x84, 0x00, 0x00,
  0x01, 0x02, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0x02, 0x01, 0x00, 0x00, 0x02, 0x01, 0x00, 0x00, 0x02, 0x01, 0x00, 0x00,
  0x00, 0x00, 0x80, 0x40, 0x00, 0x00, 0x80, 0x40, 0x00, 0x00, 0x80, 0x40, 0x00, 0x00, 0x40, 0x80, 0x00, 0x00, 0x40, 0x80, 0x00, 0x00, 0x40, 0x80,
  0x00, 0x00, 0x21, 0x00, 0x00, 0x00, 0x21, 0x00, 0x00, 0x00, 0x12, 0x00, 0x00, 0x00, 0x12, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00 

};



// '270px-Check_mark, 16x16px

const unsigned char Check_Mark_Bitmap [] PROGMEM = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xff, 0xf3, 0xff, 0xe7, 0xff, 0xc7, 0xff, 0xcf, 0xff, 0x9f, 
	0xff, 0x1f, 0xff, 0x3f, 0xde, 0x7f, 0xcc, 0x7f, 0xc0, 0xff, 0xe1, 0xff, 0xf3, 0xff, 0xff, 0xff
};



// 'false mark', 16x16px

const unsigned char False_Mark_Bitmap [] PROGMEM = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf3, 0xcf, 0xe1, 0x87, 0xe0, 0x07, 0xf0, 0x0f, 0xf8, 0x1f, 
	0xf8, 0x1f, 0xf0, 0x0f, 0xe0, 0x07, 0xe1, 0x87, 0xf3, 0xcf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};



// PIN Init functions // 

void pin_init() {

  /* Initialize INPUT pins */
  pinMode(IRQ1_PIN, INPUT);
  pinMode(IRQ0_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(WARMOUT_PIN, INPUT);
  pinMode(ZX2_PIN, INPUT);
  pinMode(ZX1_PIN, INPUT);
  pinMode(ZX0_PIN, INPUT);
  pinMode(SELF_TEST_PIN, INPUT);
  pinMode(PHASE_2_PIN, INPUT);

  /* Initialize OUTPUT pins */
  pinMode(ATM_CS_PIN, OUTPUT);
  pinMode(ATM_PM1_PIN, OUTPUT);
  pinMode(ATM_PM0_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(SQUARE_WAVE_2, OUTPUT);
  pinMode(SQUARE_WAVE_1, OUTPUT);

}



// SPI Init functions //

void spi_init() {

  pinMode(SDI_PIN, OUTPUT);
  pinMode(SDO_PIN, INPUT);
  pinMode(SCLK_PIN, OUTPUT);
  
  // Set up SPI data mode and clock frequency
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0); // You can choose the appropriate data mode
  SPI.setClockDivider(SPI_CLOCK_DIV16); // You can adjust the clock frequency
}



// Timer Init function // 

/* Set up timer interrupt */
void setupTimerInterrupt() {
  noInterrupts(); // Disable interrupts temporarily
  
  // Set up a timer interrupt to occur every 1 ms
  // Adjust TIMER_PERIOD_MS and TIMER_PRESCALER as needed
  const int TIMER_PERIOD_MS = 1;
  const int TIMER_PRESCALER = 64;
  int timerValue = (F_CPU / 1000 / TIMER_PRESCALER) * TIMER_PERIOD_MS;
  
  TCCR1A = 0; // Timer/Counter Control Register A
  TCCR1B = 0; // Timer/Counter Control Register B
  TCNT1 = 0;  // Timer/Counter Register
  OCR1A = timerValue - 1;  // Output Compare Register
  TCCR1B |= (1 << WGM12);  // Configure timer in CTC mode
  TCCR1B |= (1 << CS11) | (1 << CS10);  // Set prescaler to 64
  TIMSK1 |= (1 << OCIE1A);  // Enable Timer/Counter1 Output Compare A Match Interrupt

  TCCR0A = 0;  // Clear control register A
  TCCR0B = 0;  // Clear control register B
  TCNT0 = 0;   // Clear the counter
  OCR0A = 199;  // Set the compare value for A0
  OCR0B = 124;  // Set the compare value for B0
  TIMSK0 |= _BV(OCIE0A) | _BV(OCIE0B);  // Enable Timer A0 compare interrupts


  interrupts(); // Re-enable interrupts
}


// Timer interrupt Functions //

void i2c_rx_int_handler_0() {
  // Handle I2C RX interrupt
}

ISR(TIMER0_COMPA_vect) {
  // Timer A0 interrupt service routine
  timer_tick++;

  if (selfTestStatus == ONGOING) {
    if (timer_tick % 18 == ln1_angle) {
      digitalWrite(SQUARE_WAVE_1, digitalRead(SQUARE_WAVE_1) ^ 1);
    }
    if (timer_tick % 18 == ln2_angle) {
      digitalWrite(SQUARE_WAVE_2, digitalRead(SQUARE_WAVE_2) ^ 1);
    }
  }

  if (timer_tick >= 2000) {
    digitalWrite(UART_RX_PIN, HIGH);
    digitalWrite(UART_TX_PIN, HIGH);
  }
}

ISR(TIMER0_COMPB_vect) {
  // Timer B0 interrupt service routine
  startConversionADC();
}

ISR(USI_START_VECTOR) {
  // USI (I2C) interrupt service routine
  // Implement the I2C interrupt handling here
}

/*
ISR(USART_RX_vect) {
  // UART interrupt service routine
  // Implement the UART RX interrupt handling here
}
*/

ISR(ADC_vect) {
  // ADC interrupt service routine
  temp_values[adc_cnter] = ADC;

  if (adc_cnter == 0) {
    adc_flag = true;
    adc_cnter = MAX_ADC_SAMPLE;
  } else {
    adc_cnter--;
  }

  ADCSRA |= _BV(ADSC);  // Start the next ADC conversion
}

ISR(PCINT2_vect) {
  // Pin change interrupt for BUTTON_PIN
  if (digitalRead(BUTTON_PIN) == HIGH) {
    if (++cnt_btn > 1) {
      disableSystemPower();
    }
  } else {
    // Button is LOW
  }
}

ISR(PCINT3_vect) {
  // Pin change interrupt for SELF_TEST_PIN
  if (digitalRead(SELF_TEST_PIN) == HIGH) {
    gndProb = CONNECTED;
  } else {
    gndProb = UNCONNECTED;
  }
}



// Button Init Functions //

void initButtons() {
  button[UP_BUTTON].last_value = BUTTON_RELEASED;
  button[UP_BUTTON].pin = BUTTON_PIN;

  button[SELF_TEST].last_value = BUTTON_RELEASED;
  button[SELF_TEST].pin = SELF_TEST_PIN;

  pinMode(BUTTON_PIN, INPUT_PULLUP);      // Set button pin as input with pull-up resistor
  pinMode(SELF_TEST_PIN, INPUT_PULLUP);   // Set button pin as input with pull-up resistor

}



 /// TIMEOUT Functions ///

/* Timer interrupt function */
void timerInterrupt() {
  system_tick++;
}


/*  Function    : process_timeouts
 *  Inputs      : -
 *  Return      : -
 *  Desc        : process timeout structure status
 */
void process_timeouts() {
  for (u32 i = 0; i < NUM_OF_TIMEOUTS; i++) {
    if (timeouts[i].timeout_activated == true) {
      if (timeouts[i].countdown > 0) {
        timeouts[i].countdown--;
        timeouts[i].timeout_status = TO_COUNTING_DOWN;
        continue;
      }

      if (timeouts[i].countdown == 0) {
        timeouts[i].timeout_status = TO_OCCURED;
        timeouts[i].timeout_activated = false;
      }
    }
  }
}


/*  Function    : start_timeout_ms
 *  Inputs      : -
 *  Return      : -
 *  Desc        : start a timeout
 */
void start_timeout_ms(u8 to_no, u32 to_val_ms) {
  timeouts[to_no].timeout_status = TO_LOADED;
  timeouts[to_no].countdown = to_val_ms;
  timeouts[to_no].timeout_activated = true;
}


/*  Function    : check_timeout
 *  Inputs      : -
 *  Return      : status of TO
 *  Desc        : check timeout and clears the status to IDLE if OCCURED is returned
 */
u32 check_timeout(u8 to_no) {
  u32 *status_ptr;

  status_ptr = &timeouts[to_no].timeout_status;

  if (*status_ptr == TO_OCCURED) {
    *status_ptr = TO_IDLE;
    return TO_OCCURED;
  }

  return *status_ptr;
}


/*  Function    : get_time_ms
 *  Inputs      : -
 *  Return      : -
 *  Desc        :
 */
u32 get_time_ms() {
  return system_tick;
}


/*  Function    : calc_time_diff_ms
 *  Inputs      : -
 *  Return      : -
 *  Desc        :
 */
u32 calc_time_diff_ms(u32 prev_time) {
  return (system_tick - prev_time);
}


/*  Function    : delay_ms
 *  Inputs      : -
 *  Return      : -
 *  Desc        : waits for given ms time
 */
void delay_ms(u32 ms) {
  u32 start_time = get_time_ms();

  while (true) {
    u32 curr_time = get_time_ms();
    u32 lapse = curr_time - start_time;
    if (lapse >= ms)
      break;
  }
}



/// BUTTON Functions ///

/*  Function    : updateButtonState
 *  Inputs      : new_state - New state to be updated
 *  Return      : -
 *  Desc        : Updates the button state in the system_status variable
 */
void updateButtonState(uint8_t new_state) {
  button_state = new_state;
}

/*  Function    : isButtonReleased
 *  Inputs      : button_no - Button number to check
 *  Return      : TRUE if the button is released, FALSE otherwise
 *  Desc        : Should be called continuously
 */
uint8_t isButtonReleased(uint8_t button_no) {
  button[button_no].value = digitalRead(button[button_no].pin);
  if ((button[button_no].value == BUTTON_RELEASED) && (button[button_no].last_value == BUTTON_PRESSED)) {
    button[button_no].last_value = button[button_no].value;
    return TRUE;
  }

  button[button_no].last_value = button[button_no].value;
  return FALSE;
}

/*  Function    : isButtonPressed
 *  Inputs      : button_no - Button number to check
 *  Return      : TRUE if the button is pressed, FALSE otherwise
 *  Desc        : Should be called continuously
 */
uint8_t isButtonPressed(uint8_t button_no) {
  button[button_no].value = digitalRead(button[button_no].pin);
  if ((button[button_no].value == BUTTON_PRESSED) && (button[button_no].last_value == BUTTON_RELEASED)) {
    button[button_no].last_value = button[button_no].value;
    return TRUE;
  }

  button[button_no].last_value = button[button_no].value;
  return FALSE;
}


/*  Function    : isButtonLow
 *  Inputs      : button_no - Button number to check
 *  Return      : TRUE if the button level is low, FALSE otherwise
 *  Desc        : Should be called continuously
 */
uint8_t isButtonLow(uint8_t button_no) {
  button[button_no].last_value = button[button_no].value;
  button[button_no].value = digitalRead(button[button_no].pin);

  if (button[button_no].value == BUTTON_PRESSED) {
    return TRUE;
  }

  return FALSE;
}


/// Enable System POWER Functions /// 

/*  Function    : enableSystemPower
 *  Inputs      : -
 *  Return      : -
 *  Desc        : Necessary pins will be set here to enable power
 */
void enableSystemPower()
{
    digitalWrite(ENABLE_PIN, HIGH);
}

/*  Function    : disableSystemPower
 *  Inputs      : -
 *  Return      : -
 *  Desc        : Necessary pins will be set here to disable power
 */
void disableSystemPower()
{
    digitalWrite(ENABLE_PIN, LOW);
}


/// I2C Functions /// 


void i2cRxStart(u8 slaveAddress, u8 rxBuffer[], u8 count) {
  Wire.beginTransmission(slaveAddress);
  Wire.endTransmission(false);  // Send repeated start
  Wire.requestFrom(slaveAddress, count);

  u8 i = 0;
  while (Wire.available() && i < count) {
    rxBuffer[i++] = Wire.read();
  }
}

u8 i2cRx(u8 slaveAddress, u8 rxBuffer[], u8 count) {
  Wire.beginTransmission(slaveAddress);
  Wire.endTransmission(false);  // Send repeated start
  Wire.requestFrom(slaveAddress, count);

  u32 startTime = millis();
  u32 timeLapse;

  u8 i = 0;
  while (Wire.available() && i < count) {
    rxBuffer[i++] = Wire.read();

    timeLapse = millis() - startTime;
    if (timeLapse > I2C_TIMEOUT) {
      return FALSE;  // Timeout
    }
  }

  return TRUE;
}

u8 i2cTx(u8 slaveAddress, u8 txBuffer[], u8 count) {
  Wire.beginTransmission(slaveAddress);
  Wire.write(txBuffer, count);
  u8 result = Wire.endTransmission();

  return (result == 0) ? TRUE : FALSE;
}

void i2cRxStart(uint8_t slaveAddress) {
u8 byte = 1;

  Wire.beginTransmission(slaveAddress);
  Wire.endTransmission(false);  // Send repeated start
  Wire.requestFrom(slaveAddress, byte); // Request 1 byte

  // Wait for the data to be available
  while (Wire.available() < 1);

  // Read the data, but we don't use it in this example
  uint8_t dummy = Wire.read();
}

void i2cRecoverFromStall() {
  Wire.begin();
  Wire.beginTransmission(0);  // Send a dummy I2C address
  Wire.endTransmission();
}


// ADC Functions // 

void initADC() {
  // Configure ADC
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Set ADC prescaler to 128 (16MHz / 128 = 125kHz)
  ADMUX |= (1 << REFS0);  // Set ADC reference to AVCC
  ADCSRA |= (1 << ADEN);  // Enable ADC
  ADCSRA |= (1 << ADIE);  // Enable ADC conversion complete interrupt
}

void startConversionADC() {
  ADCSRA |= (1 << ADSC);  // Start ADC conversion
}

void adcBufferHandler() {
  // Calculate supply voltage
  supplyVoltage[1] = (SYSTEM_BATTARY_REF_V * (long)analogRead(PHASE_2_PIN)) / 4096;
  supplyVoltage[1] *= BATTARY_RESISTOR_RATE;

  // Reset flag
  adc_flag = FALSE;
}


/// Submain FUNCTIONS ///

void initSoftware() {
  enableSystemPower();        
  energyChip.begin(ATM_CS_PIN, LineFreq_50, PGain_1x, 0, 0, 0, 0);
  initButtons();
  selfTestResult = FALSE;
  adc_flag = FALSE;

    // Start ADC conversion
  startConversionADC();

  timer_tick = 0;
  i2c_0_tx_timeout_ms = 20;
  i2c_0_rx_timeout_ms = 20;
  angle_A = 0;
  angle_B = 0;
  angle_c = 0;
  ln1_angle = 0;
  ln2_angle = 0;
  selfTestStatus = 0;

}



/*  Function    : testProcess
 *  Inputs      : -
 *  Return      : -
 *  Desc        : All test process are here
 */
void testProcess()
{
    switch(gndProb)
    {
        case CONNECTED:

            controlSelfTestStatus();

        break;

        case UNCONNECTED:

            //Write to screen "Probları Kontrol Et"
            checkProbeString ();

            //Write to screen Results of Measurements as scanning
            //Write to screen State of Self Test

            systemTest();

        break;
        default:
        break;

    }

}



/*  Function    : systemTest
 *  Inputs      : -
 *  Return      : -
 *  Desc        : System test process is here
 */
void systemTest()
{
    angle_A = energyChip.GetPhaseA(); //  Arduino equivalent
    angle_B = energyChip.GetPhaseB(); // Arduino equivalent
    frequency = (unsigned long)energyChip.GetFrequency(); //  Arduino equivalent
    voltage_A = energyChip.GetLineVoltageA(); //   Arduino equivalent
    voltage_B = energyChip.GetLineVoltageB(); //  Arduino equivalent

    delay(500); //  Arduino equivalent 

    checkFrequencyAndVoltages(angle_A, angle_B, frequency, voltage_A, voltage_B );    // Sonucu ekrana burda yazdıralım
}



/*  Function    : checkFrequencyAndVoltages
 *  Inputs      : -
 *  Return      : -
 *  Desc        : Is frequency and voltages are in range?
 */
u8 checkFrequencyAndVoltages(u32 angle_A, u32 angle_B, u32 frequency, u32 voltage_A, u32 voltage_B )
{
  
  display.clearDisplay();     // Clear the display


/*
display.setTextSize(1, 2);     // Set pixel size with two axis. 

display.setTextColor(KS0108_ON);
display.setCursor(24, 18);     // Start at top-left corner
display.write(f"angle_A");
*/

if(LowLimitfreq <= frequency <= UpperLimitfreq)
{

  if((voltageA_Limit <= angle_A) && (voltageB_Limit <= angle_B))
  {

    display.setTextSize(1, 2);     // Set pixel size with two axis. 

    display.setTextColor(KS0108_ON);
    display.setCursor(24, 18);     // Start at top-left corner
    display.write("Voltage and Freq OK !");

    // Update the display
    display.display();

    return TRUE;

  }


  else {

    display.setTextSize(1, 2);     // Set pixel size with two axis. 

    display.setTextColor(KS0108_ON);
    display.setCursor(24, 18);     // Start at top-left corner
    display.write("Freq OK Voltage NOT OK!");

    // Update the display
    display.display();

    return FALSE;

  }

}

  else {

  display.setTextSize(1, 2);     // Set pixel size with two axis. 

  display.setTextColor(KS0108_ON);
  display.setCursor(24, 18);     // Start at top-left corner
  display.write("Voltage and Freq ");

  display.setTextColor(KS0108_ON);
  display.setCursor(24, 24);     // Start at top-left corner
  display.write("NOT OK !");

  // Update the display
  display.display();

  return FALSE;


  }


/*
display.setTextSize(1);     // Set text size

display.clearDisplay();     // Clear the display

// Display angle_A
display.setCursor(0, 0);
display.print("Angle A: ");
display.print(angle_A);

// Display angle_B
display.setCursor(0, 16);
display.print("Angle B: ");
display.print(angle_B);

// Display frequency
display.setCursor(0, 32);
display.print("Frequency: ");
display.print(frequency);

// Display voltage_A
display.setCursor(0, 48);
display.print("Voltage A: ");
display.print(voltage_A);

// Display voltage_B
display.setCursor(0, 64);
display.print("Voltage B: ");
display.print(voltage_B);

// Update the display
display.display();
*/


}



/*  Function    : checkAngle
 *  Inputs      : -
 *  Return      : -
 *  Desc        : Control self test angle is true or not.
 */
u8 checkAngle()
{

    angle_A = energyChip.GetPhaseA(); //  Arduino equivalent
    angle_B = energyChip.GetPhaseB(); // Arduino equivalent

    
    if(abs(angle_A) - abs(angle_B) == (ln2_angle * 10) )
    {
          return TRUE;
    }

    else 
    {
          return FALSE;
    }

}



/*  Function    : checkelfTestPhases
 *  Inputs      : -
 *  Return      : -
 *  Desc        : Control self test phases are true or not.
 */
u8 checkelfTestPhases()
{

 char angle_check = 0;

for(ln2_angle = 0; ln2_angle <= NUM_OF_SELF_TEST_CYCLES; ln2_angle++) 
{

    if(selfTestPhases[ln2_angle] == TRUE)
    {
        angle_check += 1;
      
    }

    else 
    {
        angle_check = 0;
        
        return FALSE;
    } 

    if(angle_check == 9) 
    {
      return TRUE;
    }

}


}



/*  Function    : selfTest
 *  Inputs      : -
 *  Return      : -
 *  Desc        : Self test process is here
 */
void selfTest()
{
    angle_A = energyChip.GetPhaseA(); //  Arduino equivalent
    angle_B = energyChip.GetPhaseB(); // Arduino equivalent
    frequency = (unsigned long)energyChip.GetFrequency(); //  Arduino equivalent
    voltage_A = energyChip.GetLineVoltageA(); //  Arduino equivalent
    voltage_B = energyChip.GetLineVoltageB(); //  Arduino equivalent

    delay(500); // Replace with Arduino equivalent delay function

    selfTestPhases[ln2_angle] = checkAngle();

    if (ln2_angle++ >= NUM_OF_SELF_TEST_CYCLES)
    {
        ln2_angle = 0;

        // adcBufferHandler(); // Uncomment if you havie an equivalent functon in Arduino
        if (checkelfTestPhases()) // Assuming checkelfTestPhases is a valid function
        {
            selfTestStatus = DONE_OK;
        }
        else
        {
            selfTestStatus = DONE_FALSE;
        }
    }

    // adcBufferHandler(); // Uncomment if you have an equivalent function in Arduino
}



void controlSelfTestStatus() {
  switch (selfTestStatus) {
    case ONGOING:

      // Write to screen "Self Test -> Cont."

      selfTestContinue();         // Displaying "Self Test -> Cont".

      selfTest();
      delay(1000);  // Add a delay or other relevant code if needed
      break;

    case DONE_OK:
      // Write to screen "Self Test -> OK"
      // Do something here if needed

      selfTestOk();             // Displaying "Self Test -> OK".

      break;

    case DONE_FALSE:
      // Write to screen "Check Probes"
      // Write to screen "Self Test -> NOT OK"
      // Do something here if needed

      checkProbeString ();                       //Write to screen "Probları Kontrol Et"

      selfTestNotOk();                          // Displaying "Self Test -> NOT OK".

      break;

    default:
      // Write to screen "Check Probes"
      // Write to screen "Self Test -> NOT OK"
      // Do something here if needed

      checkProbeString ();                       //Write to screen "Probları Kontrol Et"

      selfTestNotOk();                          // Displaying "Self Test -> NOT OK".

      break;
  }
}



//// General Init Functions ////

void General_Init() {
  

  pin_init();         // Initialization of all required pins.
  selfTestContinue();

  spi_init();         // Initialization of SPI pins. 

  setupTimerInterrupt(); // Set up timer interrupt
  initSoftware();        // Init SW



}

////// MAKE Everything READY  //////

void setup()   {         

  Serial.begin(9600);


  // initialize KS0108 GLCD module with active high CS pins
  if (display.begin(KS0108_CS_ACTIVE_HIGH) == false ) {
    //Serial.println( F("display initialization failed!") );    // lack of RAM space
    while(true);  // stay here forever!
  }

  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!

  display.display();
  delay(2000);

  //selfTestContinue();


  General_Init();
  Wire.begin();
  SPI.begin();

  delay(2000);

  // Configure ADC
  analogReference(DEFAULT);  // Set ADC reference to VCC

  // Start ADC conversion
  startConversionADC();

  delay(2000);

  selfTestOk();

  display.display();
  delay(2000); // Pause for 2 seconds


  //enableSystemPower();


  // display.display() is NOT necessary after every single drawing command,
  // unless that's what you want...rather, you can batch up a bunch of
  // drawing operations and then update the screen all at once by calling
  // display.display(). These examples demonstrate both approaches...

  //test_char_hello();   // Print some string (Orion AR-GE)

  drawbitmap_Orionee_logo();    //Orion EE Logo

  templatedraw();     // template for phase comparator


}



///////////// MAIN LOOP //////////

void loop() 
{

testProcess();

}



///////////////////// LCD Phase Comparator  Interface     ////////////////////


void test_char_hello() {

  display.clearDisplay();

  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(KS0108_ON); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.write("Orion EE");

 display.setTextSize(2);      // Normal 1:1 pixel scale
 display.setTextColor(KS0108_OFF, KS0108_ON); // Draw 'inverse' text
  display.setCursor(0, 18);     // Start at top-left corner
  display.write("Tech-Dev");


  display.setTextColor(KS0108_ON);        // Draw white text
  display.setCursor(10,40);             // Start at top-left corner
  display.println(F("HELLO!"));

  display.display();
  delay(1000);
}



// Orion EE  Logo 

void drawbitmap_Orionee_logo(void) {

display.clearDisplay();

 display.drawBitmap(0, 0,  myBitmap, 128, 64, 1);
 display.display();
 delay(4000);

}



void selfTestContinue() {

display.clearDisplay();

//delay(2000);

display.setTextSize(1);      // Normal 1:1 pixel scale
display.setTextColor(KS0108_ON);
display.setCursor(10, 25);     // Start at top-left corner
display.write("Self Test");


arrow(80,30);                  // Displaying an arrow 

display.setTextSize(1);      // Normal 1:1 pixel scale
display.setTextColor(KS0108_ON);
display.setCursor(90, 25);     // Start at top-left corner
display.write("Con");

display.display();


}


/// Displaying Self Test OK  ///

void selfTestOk() {

display.clearDisplay();

delay(2000);

display.setTextSize(0.5);      // Normal 1:1 pixel scale
display.setTextColor(KS0108_ON);
display.setCursor(60, 25);     // Start at top-left corner
display.write("Self Test");

arrow(90,30);                  // Displaying an arrow 

display.setTextSize(1);      // Normal 1:1 pixel scale
display.setTextColor(KS0108_ON);
display.setCursor(100, 25);     // Start at top-left corner
display.println(F("OK"));

display.display();


}



/// Displaying Self Test NOT OK  ///

void selfTestNotOk() {

display.clearDisplay();

delay(2000);

display.setTextSize(0.5);      // Normal 1:1 pixel scale
display.setCursor(60, 25);     // Start at top-left corner
display.write("Self Test");

arrow(90,30);                  // Displaying an arrow 

display.setTextSize(1);      // Normal 1:1 pixel scale
display.setTextColor(KS0108_ON);
display.setCursor(100, 25);     // Start at top-left corner
display.println(F("NOT OK"));

}




// Drawing an arrow to show Phase difference 

// Default values for drawing an arrow.

// Uzun çizgi için x0 -> 80, y0 -> 15, x1-> 60, y1-> 15
// Kısa yan çizgi üst x0 -> 80, y0 -> 15, x1-> 75, y1-> 10
// Kısa yan çizgi alt x0 -> 80, y0 -> 15, x1-> 75, y1-> 20 


void arrow(int x0, int y0) {

    display.drawLine(x0, y0, x0-20, y0, KS0108_ON);
    display.display();
    
    display.drawLine(x0, y0, x0-10, y0-5, KS0108_ON);
    display.display();

    display.drawLine(x0, y0, x0-10, y0+5, KS0108_ON);
    display.display();

}


// Drawing a Check Mark

void checkMark(int x, int y) {

 display.drawBitmap(x, y,  Check_Mark_Bitmap, 16, 16, 1);
 display.display();

}


// Drawing a False Mark 

void falseMark(int x, int y) {

 display.drawBitmap(x, y,  False_Mark_Bitmap, 16, 16, 1);
 display.display();

}


// Drawing an empty lightning icon for Phase A

void lightning_drawbitmap_PhaseA(void) {

 display.drawBitmap(45, 0,  lightningBitmap16, 16, 16, 1);
 display.display();

}


// Drawing an empty lightning icon for Phase B

void lightning_drawbitmap_PhaseB(void) {

 display.drawBitmap(108, 0,  lightningBitmap16, 16, 16, 1);
 display.display();

}

// Phase Equal sin waves.

void phaseEqual_drawbitmap(void) {

 display.drawBitmap(25, 15,  phaseBitmap, 32, 24, 1);
 display.display();

}


// Phase Not Equal sin waves.

void phaseNotEqual_drawbitmap(void) {

 display.drawBitmap(20, 16,  phaseCompBitmap, 32, 24, 1);
 display.display();

}



void twoPhaseConnected() {

 // We have to fill inside the all lightning icons.

lightning_drawbitmap_PhaseA();      // lightning for phase A
lightning_drawbitmap_PhaseB();      // Lightning for phase B

Lightning_Fill_B();         // filling lightning icon for Phase B.

Lightning_Fill_A();         // filling lightning icon for Phase A. 

}


/// Phase B is connected and shows an filled lightning icon.

void Phase_BConnectedLightning() {

 // We have to fill inside the Phase B lightning icons.

lightning_drawbitmap_PhaseA();      // lightning for phase A

lightning_drawbitmap_PhaseB();      // Lightning for phase B

Lightning_Fill_B();         // filling lightning icon for Phase B.

}


/// Phase A is connected and shows an filled lightning icon.

void Phase_AConnectedLightning() {

 // We have to fill inside the Phase A lightning icons.

lightning_drawbitmap_PhaseA();      // lightning for phase A

lightning_drawbitmap_PhaseB();      // Lightning for phase B

Lightning_Fill_A();         // filling lightning icon for Phase A.

}



/// Fill function for Phase A

void Lightning_Fill_A() {


display.writePixel(53, 3, KS0108_ON);   // Write just one pixel.


display.writePixel(52, 4, KS0108_ON);   // Write just one pixel.
display.writePixel(53, 4, KS0108_ON);   // Write just one pixel.


display.writePixel(51, 5, KS0108_ON);   // Write just one pixel.
display.writePixel(52, 5, KS0108_ON);   // Write just one pixel.
display.writePixel(53, 5, KS0108_ON);   // Write just one pixel.


display.writePixel(50, 6, KS0108_ON);   // Write just one pixel.
display.writePixel(51, 6, KS0108_ON);   // Write just one pixel.
display.writePixel(52, 6, KS0108_ON);   // Write just one pixel.
display.writePixel(53, 6, KS0108_ON);   // Write just one pixel.


display.writePixel(49, 7, KS0108_ON);   // Write just one pixel.
display.writePixel(50, 7, KS0108_ON);   // Write just one pixel.
display.writePixel(51, 7, KS0108_ON);   // Write just one pixel.
display.writePixel(52, 7, KS0108_ON);   // Write just one pixel.
display.writePixel(53, 7, KS0108_ON);   // Write just one pixel.
display.writePixel(54, 7, KS0108_ON);   // Write just one pixel.
display.writePixel(55, 7, KS0108_ON);   // Write just one pixel.
display.writePixel(56, 7, KS0108_ON);   // Write just one pixel.


display.writePixel(52, 8, KS0108_ON);   // Write just one pixel.
display.writePixel(53, 8, KS0108_ON);   // Write just one pixel.
display.writePixel(54, 8, KS0108_ON);   // Write just one pixel.
display.writePixel(55, 8, KS0108_ON);   // Write just one pixel.

display.writePixel(52, 9, KS0108_ON);   // Write just one pixel.
display.writePixel(53, 9, KS0108_ON);   // Write just one pixel.
display.writePixel(54, 9, KS0108_ON);   // Write just one pixel.

display.writePixel(52, 10, KS0108_ON);   // Write just one pixel.
display.writePixel(53, 10, KS0108_ON);   // Write just one pixel.

display.writePixel(52, 11, KS0108_ON);   // Write just one pixel.

}


/// Fill function for Phase B

void Lightning_Fill_B() {


display.writePixel(116, 3, KS0108_ON);   // Write just one pixel.


display.writePixel(115, 4, KS0108_ON);   // Write just one pixel.
display.writePixel(116, 4, KS0108_ON);   // Write just one pixel.


display.writePixel(114, 5, KS0108_ON);   // Write just one pixel.
display.writePixel(115, 5, KS0108_ON);   // Write just one pixel.
display.writePixel(116, 5, KS0108_ON);   // Write just one pixel.


display.writePixel(113, 6, KS0108_ON);   // Write just one pixel.
display.writePixel(114, 6, KS0108_ON);   // Write just one pixel.
display.writePixel(115, 6, KS0108_ON);   // Write just one pixel.
display.writePixel(116, 6, KS0108_ON);   // Write just one pixel.


display.writePixel(112, 7, KS0108_ON);   // Write just one pixel.
display.writePixel(113, 7, KS0108_ON);   // Write just one pixel.
display.writePixel(114, 7, KS0108_ON);   // Write just one pixel.
display.writePixel(115, 7, KS0108_ON);   // Write just one pixel.
display.writePixel(116, 7, KS0108_ON);   // Write just one pixel.
display.writePixel(117, 7, KS0108_ON);   // Write just one pixel.
display.writePixel(118, 7, KS0108_ON);   // Write just one pixel.
display.writePixel(119, 7, KS0108_ON);   // Write just one pixel.


display.writePixel(115, 8, KS0108_ON);   // Write just one pixel.
display.writePixel(116, 8, KS0108_ON);   // Write just one pixel.
display.writePixel(117, 8, KS0108_ON);   // Write just one pixel.
display.writePixel(118, 8, KS0108_ON);   // Write just one pixel.

display.writePixel(115, 9, KS0108_ON);   // Write just one pixel.
display.writePixel(116, 9, KS0108_ON);   // Write just one pixel.
display.writePixel(117, 9, KS0108_ON);   // Write just one pixel.

display.writePixel(115, 10, KS0108_ON);   // Write just one pixel.
display.writePixel(116, 10, KS0108_ON);   // Write just one pixel.

display.writePixel(115, 11, KS0108_ON);   // Write just one pixel.

}


// Function for self test pending message downside of the screen.

void selfTestPending() {

  display.drawRect(0, 43, 128, 10, KS0108_ON);
  display.display(); // Update screen with each newly-drawn rectangle

  display.setTextSize(0.5);      // Normal 1:1 pixel scale
  display.setCursor(5, 44);     // Start at top-left corner
  display.write("Self Test Pending");

}


// Function to printing 50 hz in downside of the screen.

void FiftyHz() {

  display.drawRect(0, 53, 64, 10, KS0108_ON);
  display.display(); // Update screen with each newly-drawn rectangle

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setCursor(10, 55);     // Start at top-left corner
  display.write("50 HZ");
}



// Function to printing 60 hz in downside of the screen.

void SixtyHz() {

  display.drawRect(0, 53, 64, 10, KS0108_ON);
  display.display(); // Update screen with each newly-drawn rectangle

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setCursor(10, 55);     // Start at top-left corner
  display.write("60 HZ");

}


// Function to create empty box on bottom of the screen.

void empty_box_Downside() {

  display.drawRect(64, 53, 64, 10, KS0108_ON);
  display.display(); // Update screen with each newly-drawn rectangle

}


// Battery charge rect with rounded and empty

void battery_icon() {

  display.drawRect(85, 55, 15, 7, KS0108_ON);
  display.display(); // Update screen with each newly-drawn rectangle

}

 // Writing PhaseA string.

void PhaseA_String(){

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(KS0108_ON);
  display.setCursor(3, 3);     // Start at top-left corner
  display.println(F("Phase A"));

}

 // Writing PhaseB string.

void PhaseB_String() {

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(KS0108_ON);
  display.setCursor(66, 3);     // Start at top-left corner
  display.println(F("Phase B"));
}

// Writing check probe string

void checkProbeString () {

//display.setTextSize(1);      // Normal 1:1 pixel scale
display.setTextSize(1, 2);     // Set pixel size with two axis. 

display.setTextColor(KS0108_ON);
display.setCursor(24, 18);     // Start at top-left corner
display.write("Check Probe");

}


void phaseDiff_B_overA() {

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(KS0108_ON);
  display.setCursor(60, 25);     // Start at top-left corner
  display.println(F("B"));

  arrow(90,30);

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(KS0108_ON);
  display.setCursor(100, 25);     // Start at top-left corner
  display.println(F("A"));

}


///// Two Phase are connected. No Phase difference.

void twoPhaseConnected_NoDiff () {

 display.clearDisplay();

// 1 st part of the begining 

  delay(2000);

  PhaseB_String();            // Writing Phase B string

// 2 nd Part of the begining

  PhaseA_String();            // Writing Phase A string

  twoPhaseConnected();          // Filled lightning icon.

  //// Middle of the Screen.

  phaseEqual_drawbitmap();         // Phase equal  

  checkMark(75,20);        // Add check mark (using bitmap if it is needed).

////// Downside of the screen.  //////

// Part of the 1st downside

  selfTestPending();          // Self test pending 

// Part of the 2nd downside's 1st part

  FiftyHz();

// Part of the 2nd downside's 2nd part

  empty_box_Downside();

// Battery charge rect with rounded and empty

  battery_icon();

  // Showing battery level.

  display.fillRoundRect(85, 55, 10, 7, 1, KS0108_ON);
  display.display();

  delay(4000);

}



// Two Phase are connected.  Phase difference.

void twoPhaseConnected_Diff () {

 display.clearDisplay();

// Rect parts of the template

// 1 st part of the begining 

  delay(2000);

  display.clearDisplay();

  PhaseB_String();                     // Writing Phase B string
   
// 2 nd Part of the begining

  PhaseA_String();                     // Writing Phase A string


  // When two phase are connected. Add a lightning icon which is filled.

  twoPhaseConnected();          // Filled lightning icon.


  //// Middle of the Screen.

  phaseNotEqual_drawbitmap();   // Phase difference


  //// Add phase difference between B -> A. 

  phaseDiff_B_overA();

////// Downside of the screen.  //////


// Part of the 1st downside

  selfTestPending();          // Self test pending 

// Part of the 2nd downside's 1st part

  FiftyHz();

// Part of the 2nd downside's 2nd part

  empty_box_Downside();

// Battery charge rect with rounded and empty

   battery_icon();

  // Showing battery level.

  display.fillRoundRect(85, 55, 10, 7, 1, KS0108_ON);
  display.display();

  delay(4000);

}




// Template for check probe connection screen. No phase is connected.

void checkProbe_NoPhase() {

  display.clearDisplay();

// 1 st part of the begining 

  delay(2000);

  display.display();

  delay(1000);

  PhaseB_String();
  
  
  lightning_drawbitmap_PhaseB();        /// Drawing lightning icon


// 2 nd Part of the begining


  PhaseA_String();


  lightning_drawbitmap_PhaseA();        /// Drawing lightning icon


///// Middle of the screen.  /////

  checkProbeString();

////// Downside of the screen.  //////


// Part of the 1st downside

  selfTestPending();          // Self test pending 

// Part of the 2nd downside's 1st part

  FiftyHz();

// Part of the 2nd downside's 2nd part

  empty_box_Downside();

// Battery charge rect with rounded and empty

  battery_icon();

  // Showing battery level.

  display.fillRoundRect(85, 55, 10, 7, 1, KS0108_ON);
  display.display();

  delay(4000);

}



// Template for check probe connection screen. Phase A is connected.

void Phase_A_Connected() {

  display.clearDisplay();

// 1 st part of the begining 

  delay(2000);

  display.clearDisplay();


  PhaseB_String();


// 2 nd Part of the begining


  PhaseA_String();


  Phase_AConnectedLightning();

///// Middle of the screen.  /////

  checkProbeString();


////// Downside of the screen.  //////


// Part of the 1st downside

  selfTestPending();          // Self test pending 

// Part of the 2nd downside's 1st part

  FiftyHz();

// Part of the 2nd downside's 2nd part

  empty_box_Downside();

// Battery charge rect with rounded and empty

  battery_icon();

  // Showing battery level.

  display.fillRoundRect(85, 55, 10, 7, 1, KS0108_ON);
  display.display();

  delay(4000);

}



// Template for check probe connection screen. Phase B is connected.

void Phase_B_Connected() {

  display.clearDisplay();

// 1 st part of the begining 

  delay(2000);

  display.clearDisplay();

  PhaseB_String();                     // Phase B string 

// 2 nd Part of the begining

  PhaseA_String();                     // Phase A string 


  Phase_BConnectedLightning();         // Phase B lightning filled icon.

///// Middle of the screen.  /////
 
  checkProbeString();                 // Writing  "check probe" string.


////// Downside of the screen.  //////

// Part of the 1st downside

  selfTestPending();          // Self test pending 

// Part of the 2nd downside's 1st part

  FiftyHz();

// Part of the 2nd downside's 2nd part

  empty_box_Downside();

// Battery charge rect with rounded and empty

  battery_icon();

  // Showing battery level.

  display.fillRoundRect(85, 55, 10, 7, 1, KS0108_ON);
  display.display();

  delay(4000);

}


// First page of the self test page.

void selfTest_False_firstPage() {

display.clearDisplay();

delay(2000);

display.drawRect(0, 0, 128, 10, KS0108_ON);
display.display(); // Update screen with each newly-drawn rectangle

display.setTextSize(0.5);      // Normal 1:1 pixel scale
display.setCursor(40, 2);     // Start at top-left corner
display.write("Self Test");


display.setTextSize(0.5);      // Normal 1:1 pixel scale

display.setTextColor(KS0108_ON);
display.setCursor(0, 15);     // Start at top-left corner
display.write("A Voltage");

falseMark(95,10);


display.setTextSize(0.5);      // Normal 1:1 pixel scale

display.setTextColor(KS0108_ON);
display.setCursor(0, 30);     // Start at top-left corner
display.write("B Voltage");

falseMark(95,25);

display.setTextSize(0.5);      // Normal 1:1 pixel scale

display.setTextColor(KS0108_ON);
display.setCursor(0, 45);     // Start at top-left corner
display.write("Correct Phase");

falseMark(95,40);

delay(2000);

}


// Second page of the self test page.

void selfTest_False_secondPage() {

display.clearDisplay();

delay(2000);

display.drawRect(0, 0, 128, 10, KS0108_ON);
display.display(); // Update screen with each newly-drawn rectangle

display.setTextSize(0.5);      // Normal 1:1 pixel scale
display.setCursor(40, 2);     // Start at top-left corner
display.write("Self Test");


display.setTextSize(0.5);      // Normal 1:1 pixel scale

display.setTextColor(KS0108_ON);
display.setCursor(0, 15);     // Start at top-left corner
display.write("Incorrect Phase");

falseMark(95,10);

/*
display.setTextSize(0.5);      // Normal 1:1 pixel scale

display.setTextColor(KS0108_ON);
display.setCursor(0, 30);     // Start at top-left corner
display.write("Relationship");

falseMark(95,25);
*/

display.setTextSize(0.5);      // Normal 1:1 pixel scale

display.setTextColor(KS0108_ON);
display.setCursor(0, 30);     // Start at top-left corner
display.write("No Voltage");

falseMark(95,25);

delay(2000);

}



// Self Test false main function

void selfTest_False_Main() {

selfTest_False_firstPage();
delay(2000);
selfTest_False_secondPage();

}


// First page of True self test page.

void selfTest_True_firstPage() {

display.clearDisplay();

delay(2000);

display.drawRect(0, 0, 128, 10, KS0108_ON);
display.display(); // Update screen with each newly-drawn rectangle

display.setTextSize(0.5);      // Normal 1:1 pixel scale
display.setCursor(40, 2);     // Start at top-left corner
display.write("Self Test");

display.setTextSize(0.5);      // Normal 1:1 pixel scale

display.setTextColor(KS0108_ON);
display.setCursor(0, 15);     // Start at top-left corner
display.write("A Voltage");

checkMark(95,10);


display.setTextSize(0.5);      // Normal 1:1 pixel scale

display.setTextColor(KS0108_ON);
display.setCursor(0, 30);     // Start at top-left corner
display.write("B Voltage");

checkMark(95,25);


display.setTextSize(0.5);      // Normal 1:1 pixel scale

display.setTextColor(KS0108_ON);
display.setCursor(0, 45);     // Start at top-left corner
display.write("Correct Phase");

checkMark(95,40);

delay(2000);

}



// Second page of True self test page.

void selfTest_True_secondPage() {

display.clearDisplay();

delay(2000);

display.drawRect(0, 0, 128, 10, KS0108_ON);
display.display(); // Update screen with each newly-drawn rectangle

display.setTextSize(0.5);      // Normal 1:1 pixel scale
display.setCursor(40, 2);     // Start at top-left corner
display.write("Self Test");


display.setTextSize(0.5);      // Normal 1:1 pixel scale

display.setTextColor(KS0108_ON);
display.setCursor(0, 15);     // Start at top-left corner
display.write("Incorrect Phase");

checkMark(95,10);

/*
display.setTextSize(0.5);      // Normal 1:1 pixel scale

display.setTextColor(KS0108_ON);
display.setCursor(0, 30);     // Start at top-left corner
display.write("Relationship");

checkMark(95,25);
*/

display.setTextSize(0.5);      // Normal 1:1 pixel scale

display.setTextColor(KS0108_ON);
display.setCursor(0, 30);     // Start at top-left corner
display.write("No Voltage");

checkMark(95,25);

delay(2000);

}



// Self Test True main function

void selfTest_True_Main() {

selfTest_True_firstPage();
delay(2000);
selfTest_True_secondPage();

}




// Template for Phase Comparator

void templatedraw(void) {

  display.clearDisplay();

  delay(2000);

  checkProbe_NoPhase();

  Phase_A_Connected();

  Phase_B_Connected();

  twoPhaseConnected_Diff();

  twoPhaseConnected_NoDiff();

  selfTest_False_Main();

  selfTest_True_Main();

  delay(4000);

  display.clearDisplay();


}





