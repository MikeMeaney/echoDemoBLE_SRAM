/*********************************************************************
This is an example for using nRF8001 Bluetooth Low Energy Breakout
from Adafruit.com with MicroChip's 23A256 Series SPI SRAM ICs. 

  Pick one up a nRF8001 BLE Low Energy Breakout in the adafruit shop!
  ------> http://www.adafruit.com/products/1697

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Kevin Townsend/KTOWN  for Adafruit Industries.

This sketch is made only for the Newer SPI (as of 11/24/14)
(http://www.pjrc.com/teensy/td_libs_SPI.html)
------------------&---------------------------
Adafruit BLE Libraries
(https://github.com/PaulStoffregen/Adafruit_nRF8001)
written by Paul Stoffregen/DorkBot

The piddly-litte SPI SRAM bits @ the end were added by Mike Meaney/Meaneymiked,

The process is:
1) A bytes comes in from BLE ACI ->
2) That byte is written to an arbitray address on the SPI SRAM (0x5A) ->
3) The value of the abitrary address (0x5A) is read back

Deciphering Serial Output:
Values not in square brackets are bytes straight from the BLE ACI,
while values in square brackets have been stored and read back from SPI SRAM
  H  = The char "H" sent over BLE
 [H] = The char "H" saved and read back from SRAM

MIT license, check LICENSE for more information
All text above, and the splash screen below must be included in any redistribution
*********************************************************************/

// This version uses the internal data queing so you can treat it like Serial (kinda)!

#include <SPI.h>
#include "Adafruit_BLE_UART.h"

// Connect CLK/MISO/MOSI to hardware SPI for BLE and SRAM


//SPI SRAM features in testing
#define SS_PIN 2
SPISettings SRAMsettings(2000000, MSBFIRST, SPI_MODE0);  //SPI SRAM works only in MODE0, BLE is MODE1. <--The Problem
                                                         //Gaol to put both on same SPI bus.
//As Per the MicroChip Data Sheet
const uint8_t WRITE_BYTE = B00000010; // This must be sent to trigger SRAM IC to store data
const uint8_t READ_BYTE =  B00000011; // This must be sent to trigger SRAM IC to recite data

// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 3     // This should be an interrupt pin, on Arduino thats #2 or #3
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
/**************************************************************************/
/*!
    Configure the Arduino and start advertising with the radio
*/
/**************************************************************************/
void setup(void)
{ 
  Serial.begin(9600);
  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Print echo demo"));

  // BTLEserial.setDeviceName("CATXCO"); /* 7 characters max! */
  
  //SS on the SPI SRAM must be set as an output
  pinMode(SS_PIN, OUTPUT);
  //YAY, BLE!!!
  BTLEserial.begin();
}

/**************************************************************************/
/*!
    Constantly checks for new events on the nRF8001
*/
/**************************************************************************/
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop()
{
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
    }
    // OK while we still have something to read, get a character and print it out
    while (BTLEserial.available()) {
      char c = BTLEserial.read();
      extSRAM_writeByte(0x5A, c);
      
      //Print out the byte from BLE
      Serial.print(c);
      Serial.print(F("["));
      Serial.print(char(extSRAM_readByte(0x5A)));
      Serial.print(F("]"));

    } 
  }
}


// SPI SRAM Functions

//Read a byte at an address and return it's value
uint8_t extSRAM_readByte(uint16_t address){
 
 //Begin SPI Transaction
  SPI.beginTransaction(SRAMsettings);
  digitalWrite(SS_PIN, LOW); // Pull CS Low to start
  uint8_t read_byte;
  SPI.transfer(READ_BYTE);
  SPI.transfer((char)(address >> 8));
  SPI.transfer((char)(address));
  read_byte = SPI.transfer(0);  
  digitalWrite(SS_PIN, HIGH); //Pull CS high to end
  SPI.endTransaction();
  //SPI Transaction Ended
  
 return read_byte;
}

//Write a byte to an address. return the written value
uint8_t extSRAM_writeByte(uint16_t address, uint8_t data){ 
  //Begin SPI Transaction
  SPI.beginTransaction(SRAMsettings);
  digitalWrite(SS_PIN, LOW); // Pull CS Low to start
  SPI.transfer(WRITE_BYTE);
  SPI.transfer((char)(address>> 8));
  SPI.transfer((char)(address));
  SPI.transfer(data);
  digitalWrite(SS_PIN, HIGH); //Pull CS high to end
  SPI.endTransaction();
  //SPI Transaction ended

  //Return a read of the newly written value
  return extSRAM_readByte(address);
}
