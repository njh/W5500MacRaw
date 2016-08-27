#include <SPI.h>
#include "w5100.h"
#include "socket.h"

const int sockNum = 0;

void setup() {
    // Setup serial port for debugging
    Serial.begin(38400);
    Serial.println("[W5100MacRaw]");

    byte mac_address[] = {
        0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
    };

    // Initialise the basic info
    W5100.init();
    SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
    W5100.setMACAddress(mac_address);
    SPI.endTransaction();

    socket(sockNum, SnMR::MACRAW, 0, 0);

    Serial.print("socketStatus=0x");
    Serial.println(socketStatus(sockNum), HEX);
}


uint8_t buffer[800];

void loop() {
    uint16_t data_len = 0;
    uint16_t port;
    uint8_t addr[16];

    int16_t ret = recvAvailable(sockNum);
    if ( ret > 0 ) {
        ret = recvfrom(sockNum, buffer, sizeof(buffer), addr, &port);
        Serial.print("ret=");
        Serial.println(ret, DEC);
    }
}
