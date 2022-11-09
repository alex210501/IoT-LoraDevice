#include "room_dht11.h"

void RoomDht11::begin(void)
{
    _dht = new DHT(_pin, DHT11);
}

void RoomDht11::compute(void)
{
    if ((millis() - _timeElapsed) > _timeout)
    {
        _temperature = _dht->readTemperature();
        _humidity = _dht->readHumidity();
        Serial.println(_temperature);
        Serial.println(_humidity);

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