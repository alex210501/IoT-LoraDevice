#define GPS_SERIAL Serial1

#include "room_gps.h"

RoomGps::RoomGps(unsigned long timeout) : _timeout(timeout) {}

void RoomGps::begin(void) 
{
    GPS_SERIAL.begin(9600);
}

    void RoomGps::compute()
{
    if ((millis() - _timeElapsed) > _timeout) {
        while (GPS_SERIAL.available()) {
            if (_gps.encode(GPS_SERIAL.read())) {
                _timeElapsed = millis();
                
                if (_gps.location.isValid()) {
                    _latitude = _gps.location.lat();
                    _latitude = _gps.location.lng();
                }
            }
        }
    }
}

double RoomGps::getLatitude(void) 
{
    return _latitude;
}

double RoomGps::getLongitude(void) 
{
    return _longitude;
}

unsigned long RoomGps::getTimeout(void) 
{
    return _timeout;
}

void RoomGps::setTimeout(unsigned long timeout)
{
    _timeout = timeout;
}