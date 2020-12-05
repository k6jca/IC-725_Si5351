/*********************
* Revision Log, k6jca
* 
* 200929  Disable 30.72 MHz (clk2)
*         (now generated w/external TCXO)
* 
********************* */
// Code to control an SI5351 Clock Generator
// to generate clockfor an ICOM IC-725 transceiver.
//
// Two clocks are generated from the SI5351's 25 MHz Xtal input:
//    1) CLK0:  9.01 MHz VFO (LSB, USB, CW(TX), CW(RX))
//    2) CLK2:  30.72 MHz (Note: disable per Rev 200929).
//
// Based, in part, on the uBitx Si5351 routines written by Jerry Gaffke, KE7ER.
//
// Frequencies for IC-725 BFO:
//
//  USB    9.013 MHz
//  LSB    9.010 MHz
//  CW RX  9.0098 MHz
//  CW TX  9.0106

#include <Wire.h>
#include <EEPROM.h>

// define pin numbers
#define RX_PIN  3
#define USB_PIN 4
#define CW_PIN  5
#define LSB_PIN 6

// define operating modes
#define NO_BFO 0
#define LSB    1
#define USB    2
#define CW     3

// define Tx/Rx
#define RX 0
#define TX 1

// Define the a, b, and c constants for the PLL frequency divider.
// These values were calculated using SI's "Clockbuilder Desktop"
// (see description in Adafruit's datasheet for their Si5351 board)
// assuming a 25 MHz clock

// CW (RX):  BFO = 9.0098 MHz, Set VCO to 702.7644 MHz
#define A_CW_RX 28L
#define B_CW_RX 41467L   // was (6911L)
#define C_CW_RX 375000L  // was (62500L)

// CW (TX):  BFO = 9.0106 MHz, Set VCO to 702.7644 MHz
#define A_CW_TX 28L
#define B_CW_TX 42402L   // was (406L)
#define C_CW_TX 375000L  // was (3125L)

// USB:  BFO = 9.0136 MHz, Set VCO to 703.14 MHz
#define A_USB 28L
#define B_USB 45216L     // was (1507L)
#define C_USB 375000L    // was (12500L)

// LSB:  BFO = 9.010 MHz, Set VCO to 702.78 MHz
#define A_LSB 28L
#define B_LSB 41700L     // was (139L)
#define C_LSB 375000     // was (1250L)

#define A_MAX (28L)   // These are just for testing, to see
#define B_MAX (9L)    // how long it takes to change the output
#define C_MAX (10L)   // frequency

#define A_30M72 34L    // a, b, and c for 30.72 MHz clock
#define B_30M72 304800L  // 152400L
#define C_30M72 750000L  // 375000L


//#define BFO_CORRECTION  -129L // correct PLLA frequency for xtal error. 0.855 Hz/count
//#define OSC_CORRECTION  -309L  // correct PLLB frequency for xtal error. 1.19 Hz/count
#define BFO_CORRECTION  0L  // correct PLLA frequency for TCXO error. 0.855 Hz/count
#define OSC_CORRECTION  0L  // correct PLLB frequency for TCXO error. 1.19 Hz/count

//#define BFO_CORRECTION  -158L // correct PLLA frequency for xtal error. 0.855 Hz/count
//#define OSC_CORRECTION  -200L  // correct PLLB frequency for xtal error



// Set Clock Output Dividers to be EVEN INTEGERS:
#define SI5351_CLK0_OUT_DIVIDE  78 
#define SI5351_CLK2_OUT_DIVIDE  28 

              
// From the uBitx Si5351 routines (written by Jerry Gaffke, KE7ER).
// Divide the lower 24 bits of a 32 bit word into 3 bytes
#define BB0(x) ((uint8_t)x)             // Bust int32 into Bytes
#define BB1(x) ((uint8_t)(x>>8))
#define BB2(x) ((uint8_t)(x>>16))

#define SI5351_ADDR 0x60              // I2C address of Si5351   (typical)
#define SI5351_XTALPF 2               // 1:6pf  2:8pf  3:10pf

#define SI5351_XTAL 25000000          // Crystal freq in Hz


uint32_t si5351_vcoa = 0;
uint32_t si5351_vcob = 0;
uint8_t  si5351_rdiv = 0;             // 0-7, CLK pin sees fout/(2**rdiv)
uint8_t  si5351_drive[3] = {1, 1, 1}; // 0=2ma 1=4ma 2=6ma 3=8ma for CLK 0,1,2
uint8_t  si5351_clken = 0xFF;         // 0xFF =  all CLK output buffers off
int32_t  clk_9MHz_cal = 0;
int32_t  clk_30MHz_cal = 0;

uint8_t clknum;
uint32_t  msa, msb, msc, msxp1, msxp2, msxp3p2top;
uint8_t reg;  
uint8_t vals[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// define 8 byte arrays representing the register contents
// for PLLA's frequency divider (PLLA will generate the BFO frequencies).
// Note that each mode has its own array.  Thus, we can
// calculate these values once, rather than recalculating
// on the fly whenever we change modes.
uint8_t USB_array[8] = {0,0,0,0,0,0,0,0};
uint8_t LSB_array[8] = {0,0,0,0,0,0,0,0};
uint8_t CW_TX_array[8] = {0,0,0,0,0,0,0,0};
uint8_t CW_RX_array[8] = {0,0,0,0,0,0,0,0};
uint8_t MAX_array[8] = {0,0,0,0,0,0,0,0};
uint8_t CLK_30M72_array[8] = {0,0,0,0,0,0,0,0};

uint8_t mode;
uint8_t new_mode;
uint8_t tx;
uint8_t inval;


void setup()
{
  Serial.begin(38400);
  Serial.flush();  

  // Use the LED pin (D13) as a test pin for timing tests.
  // Initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);

  // Input pins from IC-725
  pinMode(RX_PIN,INPUT);
  pinMode(USB_PIN,INPUT);
  pinMode(CW_PIN,INPUT);
  pinMode(LSB_PIN,INPUT);

//  attachInterrupt(digitalPinToInterrupt(RX_PIN), tx_rx_transition, CHANGE);

  
  tx = RX;       // init tx/rx to Receive
  mode = NO_BFO; // init mode to no bfo
  
/*
  // read NANO pins to find actual mode and tx/rx state
  if (digitalRead(RX_PIN)) {
    tx = RX;  // if RX pin is high, set TX to 0 (RX)
  }
  else tx = TX;
    
  if (digitalRead(USB_PIN)) {
    mode = USB;
  }
  else {
    if (digitalRead(LSB_PIN)){
      mode = LSB;        
    }
    else {
      if (digitalRead(CW_PIN)) {
        mode = CW;
      }
      else {
        mode = NO_BFO;
      }
    }
  }
  new_mode = mode;
  */
  // Calculate si5351A PLLA register values for modes:
  // Note that add correction factor to the numerator of the fraction
  build_fdiv_array(A_USB, B_USB+BFO_CORRECTION, C_USB, USB_array);
  build_fdiv_array(A_LSB, B_LSB+BFO_CORRECTION, C_LSB, LSB_array);
  build_fdiv_array(A_CW_TX, B_CW_TX+BFO_CORRECTION, C_CW_TX, CW_TX_array);
  build_fdiv_array(A_CW_RX, B_CW_RX+BFO_CORRECTION, C_CW_RX, CW_RX_array);
  build_fdiv_array(A_MAX, B_MAX, C_MAX, MAX_array);  // an extra "test" mode
  
  // Calculate si5351A PLLB register values for modes:
  // Note that add correction factor to the numerator of the fraction
  build_fdiv_array(A_30M72, B_30M72+OSC_CORRECTION, C_30M72, CLK_30M72_array);

  Wire.begin();  
  Wire.setClock(400000);              // set i2c clk to 400kbps

  // Initialize Si5351 (general parameters):  
  i2cWrite(149, 0);                   // SpreadSpectrum off
  i2cWrite(3, si5351_clken);          // Disable all CLK output drivers
  i2cWrite(183, SI5351_XTALPF << 6);  // Set 25mhz crystal load capacitance
  i2cWrite(187, 0);                   // No fannout of clkin, xtal, ms0, ms4
  for (reg=16; reg<=23; reg++) i2cWrite(reg, 0x80);  // Powerdown all CLK's

  // Set Si5351 CLK0 Output Divider
  // (Note that the value is integer, not fractional,
  // so msxp2=0, msxp3=1).
  // These are set once and never changed.
  msxp1 = 128*SI5351_CLK0_OUT_DIVIDE-512;  // and msxp2=0, msxp3=1, not fractional
  vals[0] = 0;
  vals[1] = 1;
  vals[2] = BB2(msxp1);
  vals[3] = BB1(msxp1);
  vals[4] = BB0(msxp1);
  vals[5] = 0;
  vals[6] = 0;
  vals[7] = 0;
  i2cWriten(42, vals, 8);                 // Write to 8 CLK0 regs
  i2cWrite(16, 0x4C | si5351_drive[0]); // pwr-up; INT mode; src= PLLA,multisynth.

/*
// *** This next section is for TESTING *******
  // Set Si5351 CLK1 Output Divider
  // (Note that the value is integer, not fractional,
  // so msxp2=0, msxp3=1).
  // These are set once and never changed.
  msxp1 = 128*SI5351_CLK0_OUT_DIVIDE-512;  // and msxp2=0, msxp3=1, not fractional
  vals[0] = 0;
  vals[1] = 1;
  vals[2] = BB2(msxp1);
  vals[3] = BB1(msxp1);
  vals[4] = BB0(msxp1);
  vals[5] = 0;
  vals[6] = 0;
  vals[7] = 0;
  i2cWriten(50, vals, 8);                 // Write to 8 CLK1 regs
  i2cWrite(17, 0x4C | si5351_drive[0]); // pwr-up; INT mode; src= PLLA,multisynth.
*/

  // Set Si5351 CLK2 Output Divider 
  // (Note that the value is integer, not fractional,
  // so msxp2=0, msxp3=1)
  // These are set once and never changed.
  msxp1 = 128*SI5351_CLK2_OUT_DIVIDE-512;
  vals[0] = 0;
  vals[1] = 1;
  vals[2] = BB2(msxp1);
  vals[3] = BB1(msxp1);
  vals[4] = BB0(msxp1);
  vals[5] = 0;
  vals[6] = 0;
  vals[7] = 0;
  i2cWriten(58, vals, 8);                 // Write to 8 CLK0 regs
  i2cWrite(18, 0x6C | si5351_drive[0]); // pwr-up; INT mode; src= PLLB,multisynth.
//  i2cWrite(18, 0x4C | si5351_drive[2]); // pwr-up; INT mode; src= PLLB,multisynth.

/*
  // Set PLLA Frequency Divider
  // (Note that for this app PLLA only controls the CLK0 output,
  // so this call will also disable/re-enable CLK0 out
  // when PLLA is changed).
  // This PLL frequency will change whenever the XCVR's
  // MODE changes between LSB, USB, and CW (RX or TX).
  si5351_write_plla(LSB_array, 8); // Load the PLLA vco freq-divider for LSB
*/
  
  // Set PLLB Frequency Divider
  // (Note that for this app PLLA only controls the CLK2 output,
  // so this call will also disable/re-enable CLK2 out
  // when PLLB is changed).
  // This PLL frequency is set once to 30.72 MHz 
  // and never changed thereafter.
  si5351_write_pllb(CLK_30M72_array, 8); // Load the PLLB vco freq-divider

  // read mode bits on NANO input pins
  inval = (PIND & 0x78) >> 3;
  // bit 0 = Rx/Tx
  // bit 1 = USB
  // bit 2 = CW
  // bit 3 = LSB
  if ((inval & 0x01) == 0x01) tx = RX;
  else tx = TX;
  if ((inval & 0x0E) == 0) mode = NO_BFO;  //neither usb, cw, or lsb
  else {
    if ((inval & 0x02) != 0) {
      mode = USB;
    }
    else {
      if ((inval & 0x04) != 0) {
        mode = CW;
      }
      else {
        if ((inval & 0x08) != 0) {
          mode = LSB;
        }
      }
    }
  }
  // Set PLLA Frequency Divider
  // (Note that for this app PLLA only controls the CLK0 output,
  // so this call will also disable/re-enable CLK0 out
  // when PLLA is changed).
  // This PLL frequency will change whenever the XCVR's
  // MODE changes between LSB, USB, and CW (RX or TX).
  switch (mode) {
    case USB:
      si5351_write_plla(USB_array, 8); // Load the PLLA vco freq-divider
    break;

    case LSB:
      si5351_write_plla(LSB_array, 8); // Load the PLLA vco freq-divider
    break;
    
    case CW:
      if (tx == TX) si5351_write_plla(CW_TX_array, 8); // Load the PLLA vco freq-divider
      else si5351_write_plla(CW_RX_array, 8); // Load the PLLA vco freq-divider for LSB

    break;

    default:  // no bfo
      disable_clk0();
    break;
  }
//  si5351_write_plla(LSB_array, 8); // Load the PLLA vco freq-divider for LSB
      

  
  
}

void loop() {
  // read mode pins on NANO input pins
  inval = (PIND & 0x78) >> 3; // mask of digital bits 3-6, then shift right
  // bit 0 = Rx/Tx
  // bit 1 = USB
  // bit 2 = CW
  // bit 3 = LSB
  if ((inval & 0x01) == 0x01) {  // true = receive
    if (tx == TX) {    // changing from TX to RX
      digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level)
      tx = RX;         // so update tx bit and change freq if CW
      if (mode == CW) si5351_write_plla(CW_RX_array, 8);
    }
  }
  else {  // false = transmit
    if (tx == RX) { // changing from RX to TX
      digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
      tx = TX;
      if (mode == CW) si5351_write_plla(CW_TX_array, 8);
    }
  }
  // now check if changing mode.  If yes, update BFO freq.
  if ((inval & 0x0E) == 0) {//neither usb, cw, or lsb
    if (mode != NO_BFO) disable_clk0();  // turn of BFO
    mode = NO_BFO;  
  }
  else {
    if ((inval & 0x02) != 0) { // USB mode
      if (mode != USB) si5351_write_plla(USB_array, 8);
      mode = USB;
    }
    else {
      if ((inval & 0x04) != 0) { // CW mode
        if (mode != CW) {
          if (tx == RX) si5351_write_plla(CW_RX_array, 8);
          else si5351_write_plla(CW_TX_array, 8);
        }
        mode = CW;
      }
      else {
        if ((inval & 0x08) != 0) { // LSB mode
          if (mode != LSB) si5351_write_plla(LSB_array, 8);
          mode = LSB;
        }
      }
    }
  }
        
/*
    
  if (digitalRead(USB_PIN)) {
    new_mode = USB;
    if (new_mode != mode) {
       si5351_write_plla(USB_array, 8);      
    }
  }
  else {
    if (digitalRead(LSB_PIN)){
      new_mode = LSB;        
      if (new_mode != mode) {
         si5351_write_plla(LSB_array, 8);  
      }    
    }
    else {
      if (digitalRead(CW_PIN)) {
        new_mode = CW;
        if (new_mode != mode) {
          if (tx == RX) si5351_write_plla(CW_RX_array, 8);  
          else si5351_write_plla(CW_TX_array, 8); 
        } 
      }    
      else {
        new_mode = NO_BFO;
        if (new_mode != mode) {
          disable_clk0();   // turn off BFO
        }
      }
    }
  }
  mode = new_mode;
  */
/*  
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
//  si5351_write_plla(MAX_array, 8);

  delay(500);              // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
//  si5351_write_plla(LSB_array, 8);

  delay(500);              // wait for a second
*/
}

void si5351_write_plla(uint8_t ms_vals[], uint8_t num) {
  // Write PLLA's Frequency Multisynth Divider
  // Note that PLLA only drives CLK0
  si5351_clken |= 0x01;        // Set bit to disable CLK0 output...
  i2cWrite(3, si5351_clken);   // ...Disable clock
  i2cWriten(26, ms_vals, num); // Write to 8 PLLA freq-divide regs
  i2cWrite(177, 0x20);         // Reset PLLA 
  si5351_clken &= 0xFE;        // Clear bit to enable CLK0 output...
  i2cWrite(3, si5351_clken);   // ...Enable clock
  delay(5);                    // delay 5 ms before change frequency again
                               // to essentially "debounce" the input tx/rx line 
                               // and mode lines.
}

void si5351_write_pllb(uint8_t ms_vals[], uint8_t num) {
  // Write PLLB's Frequency Multisynt Divider
  // Note that PLLB only drives CLK2
  si5351_clken |= 0x04;        // Set bit to disable CLK2 output...
  i2cWrite(3, si5351_clken);   // ...Disable clock
  i2cWriten(34, ms_vals, num); // Write to 8 PLLB freq-divide regs
  i2cWrite(177, 0x80);         // Reset PLLB 
//  Rev 200929 Comment out the next two statements...
//  si5351_clken &= 0xFB;        // Clear to enable CLK2 output...
//  i2cWrite(3, si5351_clken);   // ...Enable clock
}


void i2cWrite(uint8_t reg, uint8_t val) {   // write reg via i2c
  // uBitx code
  Wire.beginTransmission(SI5351_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void i2cWriten(uint8_t reg, uint8_t *vals, uint8_t vcnt) {  // write array
  // uBitx code
  Wire.beginTransmission(SI5351_ADDR);
  Wire.write(reg);
  while (vcnt--) Wire.write(*vals++);
  Wire.endTransmission();
}

void build_fdiv_array(uint32_t msa,uint32_t msb,uint32_t msc, uint8_t (& myarray) [8]) {
   // k6jca - build array for pll frequency divider using passed a, b, and c values
  
    msxp1 = (128 * msa + 128 * msb / msc - 512);
    msxp2 = 128 * msb - 128 * msb / msc * msc; // msxp3 == msc;
    msxp3p2top = (((msc & 0x0F0000) << 4) | msxp2);     // 2 top nibbles

    myarray[0] = BB1(msc);
    myarray[1] = BB0(msc);
    myarray[2] = BB2(msxp1);
    myarray[3] = BB1(msxp1);
    myarray[4] = BB0(msxp1);
    myarray[5] = BB2(msxp3p2top);
    myarray[6] = BB1(msxp2);
    myarray[7] = BB0(msxp2);
   
}

void disable_clk0() {
  // this routine disable clk0's output
  // Use whenever there should be NO BFO
  // (e.g. AM, FM)
  si5351_clken |= 0x01;        // Set bit to disable CLK0 output...
  i2cWrite(3, si5351_clken);   // ...Disable clock
}

void tx_rx_transition() {
  if (mode == CW) {
    if (digitalRead(RX_PIN)) {
      digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level)
      si5351_write_plla(CW_RX_array, 8);  
      tx = RX;  // if RX pin is high, set TX to 0 (RX)
    }
    else {
      digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
      si5351_write_plla(CW_TX_array, 8);  
      tx = TX;
    }
  }
}
