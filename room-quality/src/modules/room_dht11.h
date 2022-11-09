#include <DHT.h>

class RoomDht11
{
    DHT *_dht;
    const int _pin = 0;
    float _temperature = 0.0;
    float _humidity = 0.0;
    unsigned long _timeout = 0;
    unsigned long _timeElapsed = 0;

public:
    RoomDht11(int pin, unsigned long timeout) : _pin(pin), _timeout(timeout) {}

    void begin(void);
    void compute(void);
    float getTemperature(void);
    float getHumidity(void);
    unsigned long getTimeout(void);
    void setTimeout(unsigned long);
};