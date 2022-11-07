#include "room_dht11.h"

void RoomDht11::begin(void) 
{
    _dht = new DHT_Unified(_pin, DHT11);

    sensor_t sensor;
    _dht->temperature().getSensor(&sensor);
    _dht->humidity().getSensor(&sensor);
}

void RoomDht11::compute(void)
{
    if ((millis() - _timeElapsed) > _timeout) {
        sensors_event_t event;
        _dht->temperature().getEvent(&event);
        if (isnan(event.temperature)) {
            Serial.println("Error reading temperature!");
        } else {
            _temperature = event.temperature;
        }

        _dht->humidity().getEvent(&event);
        if (isnan(event.relative_humidity)) {
            Serial.println("Error reading humidity!");
        } else {
            _humidity = event.relative_humidity;
        }

        _timeElapsed = millis();
    }
}

float RoomDht11::getTemperature(void) 
{
    return _temperature;
}

float RoomDht11::getHumidity(void) 
{
    return _humidity;
}

unsigned long RoomDht11::getTimeout(void) 
{
    return _timeout;
}

void RoomDht11::setTimeout(unsigned long timeout) 
{
    _timeout = timeout;
}