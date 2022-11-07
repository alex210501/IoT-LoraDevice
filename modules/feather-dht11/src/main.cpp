#include <Arduino.h>
#include "modules/room_dht11.h"

RoomDht11 dht11(5, 1000);

void setup() {
    Serial.begin(9600);
    dht11.begin();
}

void loop() {
    dht11.compute();
    Serial.print("Temperature: ");
    Serial.println(dht11.getTemperature());
    Serial.print("Humidity: ");
    Serial.println(dht11.getHumidity());
    
    delay(1000);
}