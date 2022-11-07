#include <Arduino.h>
#include <LoRa.h>
#include <lmic.h>
#include <hal/hal.h>

const long frequency = 868E6; // LoRa Frequency

const int csPin = 8;   // LoRa radio chip select
const int resetPin = 4; // LoRa radio reset
const int irqPin = 7;   // change for your board; must be a hardware interrupt pin

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.

// #define REVERSE_ARRAY(ARRAY, SIZE)(              \
//     do                                          \
//     {                                           \
//         int mid = (SIZE) / 2;                   \
//         for (int i = 0; i <= mid; i++)          \
//         {                                       \
//             int tmp = (ARRAY[i]);               \
//             (ARRAY)[i] = (ARRAY)[SIZE - 1 - i]; \
//             (ARRAY)[SIZE - 1 - i] = tmp; \
//         } \
//     } while(0))

// void ReverseArray(u1_t array, size_t size) {
//     int mid = size / 2;

//     for (int i = 0; i < mid; i++) {
//         int tmp = array[i];
//         array[i] = array[size - i - 1];
//         array[size - i - 1] = tmp;
//     }
// }

static const u1_t PROGMEM APPEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little endian format, see above.
//tatic const u1_t PROGMEM DEVEUI[8] = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x05, 0x63, 0x9A};
static const u1_t PROGMEM DEVEUI[8] = {0xA4, 0x63, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};

void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied as-is.
// static const u1_t PROGMEM APPKEY[16] = {0x98, 0x4F, 0xB6, 0x4F, 0xA9, 0xBA, 0x1E, 0x0E, 0xD9, 0x80, 0x57, 0x9A, 0x2B, 0x06, 0x0A, 0xD9};
static const u1_t PROGMEM APPKEY[16] = {0x98, 0x4F, 0xB6, 0x4F, 0xA9, 0xBA, 0x1E, 0x0E, 0xD9, 0x80, 0x57, 0x9A, 0x2B, 0x06, 0x0A, 0xD9};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

const unsigned TX_INTERVAL = 60;

const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8, // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 8000000,
};

void do_send(osjob_t *j)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else
    {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void printHex2(unsigned v)
{
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev)
    {
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
            for (size_t i = 0; i < sizeof(artKey); ++i)
            {
                if (i != 0)
                    Serial.print("-");
                printHex2(artKey[i]);
            }
            Serial.println("");
            Serial.print("NwkSKey: ");
            for (size_t i = 0; i < sizeof(nwkKey); ++i)
            {
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
        if (LMIC.dataLen)
        {
            Serial.println(F("Received "));
            Serial.println(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }
        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
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
        Serial.println((unsigned)ev);
        break;
    }
}

void LoRa_rxMode()
{
    LoRa.enableInvertIQ(); // active invert I and Q signals
    LoRa.receive();        // set receive mode
}

void LoRa_txMode()
{
    LoRa.idle();            // set standby mode
    LoRa.disableInvertIQ(); // normal mode
}

void onReceive(int packetSize)
{
    String message = "";

    while (LoRa.available())
    {
        message += (char)LoRa.read();
    }

    Serial.print("Node Receive: ");
    Serial.println(message);
}

void onTxDone()
{
    Serial.println("TxDone");
    LoRa_rxMode();
}



void setup()
{
    pinMode(13, OUTPUT);
    pinMode(3, OUTPUT);
    Serial.begin(9600);

    // LoRa.setPins(csPin, resetPin, irqPin);

    // if (!LoRa.begin(frequency))
    // {
    //     Serial.println("LoRa init failed. Check your connections.");
    //     while (true)
    //          ; // if failed, do nothing
    // }
    // else {
    //     Serial.println("LoRa init succeeded.");
    //     Serial.println();
    //     Serial.println("LoRa Simple Node");
    //     Serial.println("Only receive messages from gateways");
    //     Serial.println("Tx: invertIQ disable");
    //     Serial.println("Rx: invertIQ enable");
    //     Serial.println();
    // }


    // LoRa.onReceive(onReceive);
    // LoRa.onTxDone(onTxDone);
    // LoRa_rxMode();

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setLinkCheckMode(0);
    LMIC_setDrTxpow(DR_SF7, 14);
    LMIC_selectSubBand(1);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop()
{
    digitalWrite(2, HIGH);
    delay(1);
    digitalWrite(2, LOW);
    delay(10000);
    // os_runloop_once();

    // digitalWrite(13, HIGH);
    // delay(1000);
    // digitalWrite(13, LOW);
    // delay(1000);
}