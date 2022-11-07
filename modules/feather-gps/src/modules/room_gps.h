#include <TinyGPSPlus.h>


class RoomGps {
    double _latitude = 0;
    double _longitude = 0;
    unsigned long _timeout = 0;
    unsigned long _timeElapsed = 0;
    TinyGPSPlus _gps;

public:
    RoomGps(unsigned long);

    void begin(void);
    void compute(void);
    double getLatitude(void);
    double getLongitude(void);
    unsigned long getTimeout(void);
    void setTimeout(unsigned long);
};