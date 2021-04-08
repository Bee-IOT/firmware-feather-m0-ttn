/*******************************************************************************
   Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   This example sends a valid LoRaWAN packet with payload "Hello,
   world!", using frequency and encryption settings matching those of
   the The Things Network.

   This uses OTAA (Over-the-air activation), where where a DevEUI and
   application key is configured, which are used in an over-the-air
   activation procedure where a DevAddr and session keys are
   assigned/generated for use with all further communication.

   Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
   g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
   violated by this sketch when left running for longer)!

   To use this sketch, first register your application and device with
   the things network, to set or generate an AppEUI, DevEUI and AppKey.
   Multiple devices can use the same AppEUI, but each device has its own
   DevEUI and AppKey.

   Do not forget to define the radio type correctly in config.h.

 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Adafruit_SleepyDog.h>
#include <CayenneLPP.h>

// debug via serial
#define SERIALDEBUG
 
// use Watchdog Sleep Timer
#define SLEEP       true

// Schedule TX  (might become longer due to duty cycle limitations).
uint8_t CYCLE_TIMES = 5;  // minutes for 1 cycle to end (including sleep)
uint16_t SLEEP_TIME = 60*(CYCLE_TIMES-1) + 50; // seconds sleep time 

// do not change
const unsigned TX_INTERVAL = 1;                // seconds of transmit cycle

// A dummy sensor publishing static values of temperatures, weight and battery voltage.
// Please turn this on when working without any sensor hardware.
#define SENSOR_DUMMY       false

// otherwise enable sensors
#if !SENSOR_DUMMY

// enable weight scale sensors

// enable 1-Wire Dallas Temperature sensors
#define SENSOR_DS18B20_OUT false
#define SENSOR_DS18B20_IN  false

// Battery voltage measured at an analog pin
#define SENSOR_BATTERY_VOLTAGE    true
#define VBATPIN A0

#endif

// ----------
// TTN config
// ----------

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

// ---------------
// General - debug
// ---------------

#ifdef SERIALDEBUG
  #define SERIALDEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define SERIALDEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define SERIALDEBUG_PRINT(...)
  #define SERIALDEBUG_PRINTLN(...)
#endif

// -------------------
// HX711: Scale sensor
// -------------------
const int HX711_PIN_DOUT = 12;
const int HX711_PIN_PDSCK = 13;

// ---------------
// General - scale
// ---------------

// Properly using the load cell requires definition of individual configuration values.
// This is not just specific to the *type* of the load cell as even
// load cells of the *same* type / model have individual characteristics.

// To measure these values, please have a look at the firmwares for load cell adjustment:
// https://hiveeyes.org/docs/arduino/firmware/scale-adjust/README.html


// This is the raw sensor value for "0 kg".
// Write down the sensor value of the scale sensor with no load and adjust it here.
#define LOADCELL_ZERO_OFFSET    93618.0f

// This is the raw sensor value for a load weighing exactly 1 kg.
// Add a load with known weight in kg to the scale sensor, note the
// sensor value, calculate the value for a 1 kg load and adjust it here.
#define LOADCELL_KG_DIVIDER     12820

// Note: Use value * 0.5 for asymmetric / single side measurement,
// so that 1 kg is displayed as 2 kg.adc_level
//#define LOADCELL_KG_DIVIDER     11026

// Set temperature compensation factor
float LOADCELL_TEMP_COMP = 0.00035;

// --------------------------
// DS18B20: Temperature sensors and arrays
// --------------------------

// Number of temperature devices on 1-Wire busses
const int ds18b20_outside_device_count = 1;
const int ds18b20_inside_device_count  = 1;

// Order of the physical array of temperature devices,
// the order is normally defined by the device id hardcoded in
// the device.
// You can physically arrange the DS18B20 in case you
// know the ID and use here {1,2,3,4 ...} or you can try out what
// sensor is on which position and adjust it here accordingly.
const int ds18b20_outside_device_order[ds18b20_outside_device_count] = {0};
const int ds18b20_inside_device_order[ds18b20_inside_device_count] = {0};

// For more DS18B20 devices on a sinbgle onwire bus, configure them like...
//const int ds18b20_inside_device_order[ds18b20_inside_device_count] = {0,1};

// Resolution for all devices (9, 10, 11 or 12 bits)
const int ds18b20_precision = 12;

// 1-Wire pin for the temperature sensors
// Note: Pin 5 corresponds to the physical pin D1 of ESP8266 MCUs
const int ds18b20_outside_onewire_pin = 10;
const int ds18b20_inside_onewire_pin = 11;

// =========
// Libraries
// =========

// -----------------
// CayenneLPP object
// -----------------
CayenneLPP lpp(51);

// HX711 weight scale sensor
#if SENSOR_HX711

// HX711 library
#include "HX711.h"

// Create HX711 object
HX711 scale;
// Intermediate data structure for reading the sensor values
float weight;
#endif

//#if SENSOR_DS18B20_OUT

// 1-Wire library
#include <OneWire.h>

// DS18B20/DallasTemperature library
#include <DallasTemperature.h>

// For communicating with any 1-Wire device (not just DS18B20)
OneWire oneWire_out(ds18b20_outside_onewire_pin);

// Initialize DallasTemperature library with reference to 1-Wire object
DallasTemperature ds18b20_sensor_out(&oneWire_out);

// Arrays for device addresses
uint8_t ds18b20_out_addresses[1][8];

// Intermediate data structure for reading the sensor values
float ds18b20_temperature_out[1];

//#endif

//#if SENSOR_DS18B20_IN

// 1-Wire library
#include <OneWire.h>

// DS18B20/DallasTemperature library
#include <DallasTemperature.h>

// For communicating with any 1-Wire device (not just DS18B20)
OneWire oneWire_in(ds18b20_inside_onewire_pin);

// Initialize DallasTemperature library with reference to 1-Wire object
DallasTemperature ds18b20_sensor_in(&oneWire_in);

// Arrays for device addresses
uint8_t ds18b20_in_addresses[ds18b20_inside_device_count][8];

// Intermediate data structure for reading the sensor values
float ds18b20_temperature_in[ds18b20_inside_device_count];

//#endif

#if SENSOR_BATTERY_VOLTAGE
float lipo_voltage;
#endif


// ============
// Main program
// ============

// --------------------
// Forward declarations
// --------------------

void read_sensors();
void read_battery_voltage();

// Battery level
#if SENSOR_BATTERY_VOLTAGE
float battery_level;
#endif

// Pin mapping for Adafruit Feather M0 LoRa, etc.
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 8000000,
};

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        SERIALDEBUG_PRINT('0');
    SERIALDEBUG_PRINT(v, HEX);
}

void onEvent (ev_t ev) {
//  int SLEEP_CYCLES;
  SERIALDEBUG_PRINT(os_getTime());
  SERIALDEBUG_PRINT(F(": "));
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      SERIALDEBUG_PRINTLN(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      SERIALDEBUG_PRINTLN(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      SERIALDEBUG_PRINTLN(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      SERIALDEBUG_PRINTLN(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      SERIALDEBUG_PRINTLN(F("EV_JOINING"));
      // Join with SF10 and let ADR adapt data rate later
      // LMIC_setDrTxpow(DR_SF10,14);
      break;
    case EV_JOINED:
      SERIALDEBUG_PRINTLN(F("EV_JOINED"));
      {
       u4_t netid = 0;
       devaddr_t devaddr = 0;
       u1_t nwkKey[16];
       u1_t artKey[16];
       LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
       SERIALDEBUG_PRINT("netid: ");
       SERIALDEBUG_PRINTLN(netid, DEC);
       SERIALDEBUG_PRINT("devaddr: ");
       SERIALDEBUG_PRINTLN(devaddr, HEX);
       SERIALDEBUG_PRINT("AppSKey: ");
       for (size_t i=0; i<sizeof(artKey); ++i) {
         if (i != 0)
            SERIALDEBUG_PRINT("-");
            printHex2(artKey[i]);
         }
       SERIALDEBUG_PRINTLN("");
       SERIALDEBUG_PRINT("NwkSKey: ");
       for (size_t i=0; i<sizeof(nwkKey); ++i) {
            if (i != 0)
               SERIALDEBUG_PRINT("-");
               printHex2(nwkKey[i]);
       }
       SERIALDEBUG_PRINTLN();
      }

      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      // LMIC_setLinkCheckMode(0);
      break;
    case EV_JOIN_FAILED:
      SERIALDEBUG_PRINTLN(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      SERIALDEBUG_PRINTLN(F("EV_REJOIN_FAILED"));
      break;
      break;
    case EV_TXCOMPLETE:
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
      SERIALDEBUG_PRINTLN(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        SERIALDEBUG_PRINTLN(F("Received ack"));

      if (LMIC.dataLen) {
        SERIALDEBUG_PRINT(F("Received "));
        SERIALDEBUG_PRINT(LMIC.dataLen);
        SERIALDEBUG_PRINT(F(" bytes of payload : 0x"));
        for (int i = 0; i < LMIC.dataLen; i++) {
          if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
            SERIALDEBUG_PRINT(F("0"));
          }
          SERIALDEBUG_PRINT(LMIC.frame[LMIC.dataBeg + i], HEX);
        }
        SERIALDEBUG_PRINTLN();
        SLEEP_TIME = 60 * (LMIC.frame[LMIC.dataBeg] - 1) + 50;
      }

      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
      
      #if SLEEP
      SERIALDEBUG_PRINT(F("Going to sleep for "));
      SERIALDEBUG_PRINT(SLEEP_TIME);
      SERIALDEBUG_PRINT(F(" seconds ..."));
      
      // wait for messages to get written before going to sleep
      delay(100);

      // lmic library sleeps automatically after transmission has been completed
      for(int i= 0; i < SLEEP_TIME / 16; i++) {
        Watchdog.sleep(16000); // maximum seems to be 16 seconds
      }
      if (SLEEP_TIME % 16) {
        Watchdog.sleep((SLEEP_TIME % 16)*1000);
      }
      SERIALDEBUG_PRINTLN(F(""));
      SERIALDEBUG_PRINTLN(F("... woke up again"));
      #endif

      break;
    case EV_LOST_TSYNC:
      SERIALDEBUG_PRINTLN(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      SERIALDEBUG_PRINTLN(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      SERIALDEBUG_PRINTLN(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      SERIALDEBUG_PRINTLN(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      SERIALDEBUG_PRINTLN(F("EV_LINK_ALIVE"));
      break;
    case EV_TXSTART:
      SERIALDEBUG_PRINTLN(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      SERIALDEBUG_PRINTLN(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
       /* do not print anything -- it wrecks timing */
       break;
    case EV_JOIN_TXCOMPLETE:
       SERIALDEBUG_PRINTLN(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
        break;
    default:
      SERIALDEBUG_PRINTLN(F("Unknown event"));
      SERIALDEBUG_PRINTLN((unsigned) ev);
      break;
  }
}

void do_send(osjob_t* j) {
  float weight_comp;
  read_sensors();

  lpp.reset();

  #if SENSOR_DUMMY
  lpp.addLoad(5, 54.4);
  // lpp.addLoad(6, 55.7);
  lpp.addTemperature(0, 12.4);
  lpp.addTemperature(10, 34.9);
  lpp.addTemperature(11, 35.2);
  lpp.addVoltage(0, 3.86);
  #else
  
  #if SENSOR_HX711
  lpp.addLoad(5, weight);
  #if SENSOR_DS18B20_OUT
  if (ds18b20_temperature_out[0] > -30 && ds18b20_temperature_out[0] < 50) {
    weight_comp = weight * ( 1. - LOADCELL_TEMP_COMP * (20. - ds18b20_temperature_out[0]));
    lpp.addLoad(6, weight_comp);
  }
  #endif
  #endif

  //SERIALDEBUG_PRINTLN(LOADCELL_TEMP_COMP,5);

  #if SENSOR_DS18B20_OUT
  // Iterate all inside DS18B20 devices and read temperature values
  for (int i = 0; i < ds18b20_outside_device_count; i++) {
    if (ds18b20_temperature_out[i] == -127.0) {
      SERIALDEBUG_PRINT(F("Inside sensor seems not to be connected. Device : "));
      SERIALDEBUG_PRINTLN(i);
    } else {
      // for inside sensors start with CayenneLPP channel 0
      lpp.addTemperature(i, ds18b20_temperature_out[i]);
    }
  }
  #endif

  #if SENSOR_DS18B20_IN
  // Iterate all inside DS18B20 devices and read temperature values
  for (int i = 0; i < ds18b20_inside_device_count; i++) {
    if (ds18b20_temperature_in[i] == -127.0) {
      SERIALDEBUG_PRINT(F("Inside sensor seems not to be connected. Device : "));
      SERIALDEBUG_PRINTLN(i);
    } else {
      // for inside sensors start with CayenneLPP channel 10
      lpp.addTemperature(10 + i, ds18b20_temperature_in[i]);
    }
  }
  #endif

  #if SENSOR_BATTERY_VOLTAGE
  lpp.addVoltage(0, lipo_voltage);
  #endif

  #endif // DUMMY

  SERIALDEBUG_PRINT(F("CayenneLPP Payload Size : "));
  SERIALDEBUG_PRINTLN(lpp.getSize());
  SERIALDEBUG_PRINT(F("CayenneLPP Payload : "));
  for (size_t i=0; i<lpp.getSize(); ++i) {
//    if (i != 0)
      //SERIALDEBUG_PRINT("-");
      printHex2(lpp.getBuffer()[i]);
  }
  SERIALDEBUG_PRINTLN();

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    SERIALDEBUG_PRINTLN(F("OP_TXRXPEND, not sending"));
  } else {
    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
    SERIALDEBUG_PRINTLN(F("Packet queued"));
  }

  digitalWrite(LED_BUILTIN, HIGH);    // turn the LED on by making the voltage HIGH
  // Next TX is scheduled after TX_COMPLETE event.

}

void setup_sensors() {

  // Setup scale sensor (single HX711)
  #if SENSOR_HX711

  // Initialize the hardware driver with the appropriate pins
  scale.begin(HX711_PIN_DOUT, HX711_PIN_PDSCK);

  // These values are obtained by calibrating the scale sensor with known weights.
  // See the README for further details and - for obtaining them:
  // https://hiveeyes.org/docs/arduino/firmware/scale-adjust/README.html
  scale.set_scale(LOADCELL_KG_DIVIDER);
  scale.set_offset(LOADCELL_ZERO_OFFSET);

  #endif

  // Setup temperature array (multiple DS18B20 sensors)
  #if SENSOR_DS18B20_OUT

  // Start DallasTemperature library
  ds18b20_sensor_out.begin();

  // Set resolution for all devices
  ds18b20_sensor_out.setResolution(ds18b20_precision);

  // Search for devices on the bus and assign based on an index. Ideally,
  // you would do this to initially discover addresses on the bus and then
  // use those addresses and manually assign them (see above) once you know
  // the devices on your bus (and assuming they don't change).

  for (int i = 0; i < ds18b20_inside_device_count; i++) {
    // Get address of single device
    if (!ds18b20_sensor_out.getAddress(ds18b20_out_addresses[ds18b20_outside_device_order[i]], i)) {
      if (SERIALDEBUG_PRINTLN(F("Unable to find address for outside temperature device"))) {
        SERIALDEBUG_PRINT(F("Device #"));
        SERIALDEBUG_PRINTLN(i);
      }
    }
  }
  #endif

  #if SENSOR_DS18B20_IN
  // Start DallasTemperature library
  ds18b20_sensor_in.begin();

  // Set resolution for all devices
  ds18b20_sensor_in.setResolution(ds18b20_precision);

  // Search for devices on the bus and assign based on an index. Ideally,
  // you would do this to initially discover addresses on the bus and then
  // use those addresses and manually assign them (see above) once you know
  // the devices on your bus (and assuming they don't change).

  for (int i = 0; i < ds18b20_inside_device_count; i++) {
    // Get address of single device
    if (!ds18b20_sensor_in.getAddress(ds18b20_in_addresses[ds18b20_inside_device_order[i]], i)) {
      if (SERIALDEBUG_PRINTLN(F("Unable to find address for inside temperature array device"))) {
        SERIALDEBUG_PRINT(F("Device #"));
        SERIALDEBUG_PRINTLN(i);
      }
    }
  }
  #endif
}

// DS18B20: Outside Temperature
#if SENSOR_DS18B20_OUT
void read_temperature_outside() {

  //SERIALDEBUG_PRINTLN(F("read outside temperature"));

  // Make it asynchronous
  ds18b20_sensor_out.setWaitForConversion(false);

  // Initiate temperature retrieval
  ds18b20_sensor_out.requestTemperatures();

  // Wait at least 750 ms for conversion
  delay(1000);

  // Iterate all DS18B20 devices and read temperature values
  for (int i = 0; i < ds18b20_outside_device_count; i++) {

    // Read single device
    float temperatureC = ds18b20_sensor_out.getTempC(ds18b20_out_addresses[i]);
    ds18b20_temperature_out[i] = temperatureC;

    SERIALDEBUG_PRINT(F("temp outside (DS18B20)"));
    SERIALDEBUG_PRINT(i);
    SERIALDEBUG_PRINT(F(" : "));
    SERIALDEBUG_PRINT(ds18b20_temperature_out[i]);
    SERIALDEBUG_PRINTLN(F("°C"));
  }
}
#endif

// DS18B20: Inside Temperature
#if SENSOR_DS18B20_IN
void read_temperature_inside() {

  //SERIALDEBUG_PRINTLN(F("read inside temperature"));

  // Make it asynchronous
  ds18b20_sensor_in.setWaitForConversion(false);

  // Initiate temperature retrieval
  ds18b20_sensor_in.requestTemperatures();

  // Wait at least 750 ms for conversion
  delay(1000);

  // Iterate all DS18B20 devices and read temperature values
  for (int i = 0; i < ds18b20_inside_device_count; i++) {

    // Read single device
    float temperatureC = ds18b20_sensor_in.getTempC(ds18b20_in_addresses[i]);
    ds18b20_temperature_in[i] = temperatureC;

    SERIALDEBUG_PRINT(F("temp inside  (DS18B20)"));
    SERIALDEBUG_PRINT(i);
    SERIALDEBUG_PRINT(F(" : "));
    SERIALDEBUG_PRINT(ds18b20_temperature_in[i]);
    SERIALDEBUG_PRINTLN(F("°C"));
  }
}
#endif

#if SENSOR_BATTERY_VOLTAGE
void read_battery_voltage() {
  int adc_lvl = analogRead(VBATPIN);
  // for 2x200kOhm voltage divider
  lipo_voltage = float(adc_lvl) * 2  * 3.3 / 1024; // last multiplicator to adjust 2*200kOhm voltage divider inaccuracy;

  SERIALDEBUG_PRINT(F("Battery voltage : "));
  SERIALDEBUG_PRINT(lipo_voltage);
  SERIALDEBUG_PRINTLN(F("V"));
  
}
#endif

#if SENSOR_HX711
void read_weight() {
  // HX711 weight scale sensor
  scale.power_up();
  weight = scale.get_units(10);

  SERIALDEBUG_PRINT(F("Weight (HX711)  : "));
  SERIALDEBUG_PRINT(weight,3);
  SERIALDEBUG_PRINTLN(F("kg"));
  
  // Put the ADC to sleep mode
  scale.power_down();
}
#endif

// Read all sensors in sequence
void read_sensors() {
  #if SENSOR_HX711
  read_weight();
  #endif
  #if SENSOR_DS18B20_OUT
  read_temperature_outside();
  #endif
  #if SENSOR_DS18B20_IN
  read_temperature_inside();
  #endif
  #if SENSOR_BATTERY_VOLTAGE
  read_battery_voltage();
  #endif
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW

  delay(3000);
  
  // never ever remove. Device becomes available to serial programming
  Serial.begin(9600);
  
  delay(2000);

  SERIALDEBUG_PRINTLN(F("Starting"));
  
  setup_sensors();

// For Pinoccio Scout boards
  #ifdef VCC_ENABLE
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
  #endif

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  
  LMIC_setAdrMode(1); // turn ADR on
  LMIC_setLinkCheckMode(1); // enable link check mode on
  
  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void loop() {
  //while (!Serial && millis() < 10000);
  //while (!Serial);
  os_runloop_once();
}
