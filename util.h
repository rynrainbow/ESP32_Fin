#include <WiFi.h>
#include <vector>

class WIFIinfo{
  public:
    IPAddress* local_IP;
    IPAddress* gateway;
    IPAddress* subnet;  
    int32_t channel;
    char* ssid = "X508";  // "" SHAW-C18FD0  X508  OnePlus 9 Pro
    char* pwd = "Xlabrouter508";  // "" 25116A077922  Xlabrouter508 lb123456
    uint8_t bssid[6] = {0x54, 0xaf, 0x97, 0x59, 0x9e, 0xa3};

    WIFIinfo() : local_IP(new IPAddress(192, 168, 0, 114)), gateway(new IPAddress(192, 168, 0, 1)), 
    subnet(new IPAddress(255, 255, 255, 0)), channel(4) {}
};

void append2vector(uint8_t*, size_t, std::vector<uint8_t>*);
void vector2array(uint8_t*, size_t*, std::vector<uint8_t>*);