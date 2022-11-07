
#include <TinyGPSPlus.h>
#include "modules/room_gps.h"

float lat = 28.5458, lon = 77.1703; // create variable for latitude and longitude object
RoomGps gps(1000);

/* void displayInfo()
{
    Serial.print(F("Location: "));
    if (gps.location.isValid())
    {
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print(gps.location.lng(), 6);
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.print(F("  Date/Time: "));
    if (gps.date.isValid())
    {
        Serial.print(gps.date.month());
        Serial.print(F("/"));
        Serial.print(gps.date.day());
        Serial.print(F("/"));
        Serial.print(gps.date.year());
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.print(F(" "));
    if (gps.time.isValid())
    {
        if (gps.time.hour() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        if (gps.time.minute() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        if (gps.time.second() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(F("."));
        if (gps.time.centisecond() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.centisecond());
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.println();
} */

void setup()
{
    Serial.begin(9600);
    gps.begin();
}

void loop()
{
    gps.compute();
    Serial.print("Lat: ");
    Serial.println(gps.getLatitude());
    Serial.print("Long: ");
    Serial.println(gps.getLongitude());
    Serial.println();
    /* while (Serial1.available())
    {                                   // check for gps data
        if (gps.encode(Serial1.read())) // encode gps data
        {
            displayInfo();
            // display position
        }
    } */
    delay(1000);
}