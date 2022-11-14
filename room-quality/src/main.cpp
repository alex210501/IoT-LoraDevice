#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>

#include <DHT.h>

#include "modules/room_gps.h"

#define ENABLE_SENSOR (true)
#define PAYLOAD_SIZE (7)
#define DATA_TIMEOUT (3000)
#define DHTPIN 12 // define data pin on Adafruit, we used 2
#define DHTTYPE DHT11   // define type of sensor

#if ENABLE_SENSOR
RoomGps gps(DATA_TIMEOUT);
DHT dht(DHTPIN, DHTTYPE); //define properties of sensor
#endif

static const u1_t PROGMEM APPEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = {0x9B, 0x74, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};

void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = {0xBB, 0x32, 0x31, 0xC6, 0xCD, 0x36, 0x68, 0x3E, 0xD6, 0x4E, 0x48, 0x80, 0x7D, 0x8E, 0x2E, 0x83};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

static osjob_t sendjob;
static osjob_t doWorkJob;

uint32_t doWorkIntervalSeconds = 3;

    // Schedule TX every this many seconds (might become longer due to duty
    // cycle limitations).
const unsigned TX_INTERVAL = 5;

// Pin mapping
//
// Adafruit BSPs are not consistent -- m0 express defs ARDUINO_SAMD_FEATHER_M0,
// m0 defs ADAFRUIT_FEATHER_M0
//
#if defined(ARDUINO_SAMD_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0)
// Pin mapping for Adafruit Feather M0 LoRa, etc.
// /!\ By default Adafruit Feather M0's pin 6 and DIO1 are not connected.
// Please ensure they are connected.
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 8000000,
};
#elif defined(ARDUINO_AVR_FEATHER32U4)
// Pin mapping for Adafruit Feather 32u4 LoRa, etc.
// Just like Feather M0 LoRa, but uses SPI at 1MHz; and that's only
// because MCCI doesn't have a test board; probably higher frequencies
// will work.
// /!\ By default Feather 32u4's pin 6 and DIO1 are not connected. Please 
// ensure they are connected.
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {7, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,              // LBT cal for the Adafruit Feather 32U4 LoRa, in dB
    .spi_freq = 1000000,
};
#elif defined(ARDUINO_CATENA_4551)
// Pin mapping for Murata module / Catena 4551
const lmic_pinmap lmic_pins = {
        .nss = 7,
        .rxtx = 29,
        .rst = 8,
        .dio = { 25,    // DIO0 (IRQ) is D25
                 26,    // DIO1 is D26
                 27,    // DIO2 is D27
               },
        .rxtx_rx_active = 1,
        .rssi_cal = 10,
        .spi_freq = 8000000     // 8MHz
};
#else
# error "Unknown target"
#endif

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
#if ENABLE_SENSOR
        // Prepare upstream data transmission at the next possible time.
        // dht.compute();
        gps.compute();

        float temperature = dht.readTemperature();
        float humidity = dht.readHumidity();
        double longitude = gps.getLongitude();
        double latitude = gps.getLatitude();
#else
        float temperature = 26.0;
        float humidity = 50.0;
        double longitude = 4.45;
        double latitude = 50.84;
#endif
        Serial.println(dht.readHumidity());  //save the value of humidity to hum variable
        Serial.println(dht.readTemperature()); // save the value of temperature to temp variable  
        uint8_t payload[PAYLOAD_SIZE] = { 0 };

        payload[0] = (uint8_t)temperature;
        payload[1] = (uint8_t)((temperature -(int)temperature) * 10);
        payload[2] = (uint8_t)humidity;
        payload[3] = (uint8_t)latitude;
        payload[4] = (uint8_t)((latitude - (int)latitude) * 100);
        payload[5] = (uint8_t)longitude;
        payload[6] = (uint8_t)((longitude - (int)longitude) * 100);

        for (int i = 0; i < PAYLOAD_SIZE; i++) {
            Serial.print(payload[i], HEX);
            Serial.print(", ");
        }
        Serial.println();

        LMIC_setTxData2(1, payload, PAYLOAD_SIZE, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void processWork(ostime_t doWorkJobTimeStamp)
{
    // This function is called from the doWorkCallback()
    // callback function when the doWork job is executed.

    // Uses globals: payloadBuffer and LMIC data structure.

    // This is where the main work is performed like
    // reading sensor and GPS data and schedule uplink
    // messages if anything needs to be transmitted.

    // Skip processWork if using OTAA and still joining.
    if (LMIC.devaddr == 0)
    {
        // For simplicity LMIC-node will try to send an uplink
        // message every time processWork() is executed.

        // Schedule uplink message if possible
        if (LMIC.opmode & OP_TXRXPEND)
        {
// TxRx is currently pending, do not send.
#ifdef USE_SERIAL
            printEvent(timestamp, "Uplink not scheduled because TxRx pending", PrintTarget::Serial);
#endif
#ifdef USE_DISPLAY
            printEvent(timestamp, "UL not scheduled", PrintTarget::Display);
#endif
        }
        else
        {

            // Prepare uplink payload.
            // dht.compute();
            // gps.compute();

            uint8_t payload[PAYLOAD_SIZE] = {0};

            float temperature = 50.1; // dht.getTemperature();
            float humidity = 50.2; // dht.getHumidity();
            double longitude = 0.0; // gps.getLongitude();
            double latitude = 0.0;  // gps.getLatitude();

            payload[0] = (uint8_t)temperature;
            payload[1] = (uint8_t)((temperature - (int)temperature) * 10);
            payload[2] = (uint8_t)humidity;
            payload[3] = (uint8_t)latitude;
            payload[4] = (uint8_t)((latitude - (int)latitude) * 100);
            payload[5] = (uint8_t)longitude;
            payload[6] = (uint8_t)((longitude - (int)longitude) * 100);
            Serial.println("here");

            for (int i = 0; i < PAYLOAD_SIZE; i++)
            {
                Serial.print(payload[i], HEX);
                Serial.print(", ");
            }
            Serial.println();

            LMIC_setTxData2(1, payload, PAYLOAD_SIZE, 0);
        }
    }
}

static void doWorkCallback(osjob_t *job)
{
    // Event hander for doWorkJob. Gets called by the LMIC scheduler.
    // The actual work is performed in function processWork() which is called below.

    ostime_t timestamp = os_getTime();
#ifdef USE_SERIAL
    serial.println();
    printEvent(timestamp, "doWork job started", PrintTarget::Serial);
#endif

    // Do the work that needs to be performed.
    processWork(timestamp);

    // This job must explicitly reschedule itself for the next run.
    ostime_t startAt = timestamp + sec2osticks((int64_t)doWorkIntervalSeconds);
    os_setTimedCallback(&doWorkJob, startAt, doWorkCallback);
}

void setup() {
    delay(5000);
    
    Serial.begin(9600);
    Serial.println(F("Starting"));

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setLinkCheckMode(0);
    LMIC_setDrTxpow(DR_SF7,14);
    // os_clearCallback(&doWorkJob);
    // os_setCallback(&doWorkJob, doWorkCallback);
    delay(1000); 

#if ENABLE_SENSOR
    gps.begin();
    dht.begin();
#endif

    delay(2000);
    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
   os_runloop_once();
}