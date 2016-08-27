#include <SPI.h>
#include "w5100.h"
#include "socket.h"

const int sockNum = 0;


void printPaddedHex(uint8_t byte)
{
    char str[2];
    str[0] = (byte >> 4) & 0x0f;
    str[1] = byte & 0x0f;

    for (int i=0; i<2; i++) {
        // base for converting single digit numbers to ASCII is 48
        // base for 10-16 to become lower-case characters a-f is 87
        if (str[i] > 9) str[i] += 39;
        str[i] += 48;
        Serial.print(str[i]);
    }
}

void printMACAddress(const uint8_t address[6])
{
    for (uint8_t i = 0; i < 6; ++i) {
        printPaddedHex(address[i]);
        if (i < 5)
            Serial.print(':');
    }
    Serial.println();
}

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
        int16_t len = recvfrom(sockNum, buffer, sizeof(buffer), addr, &port);
        Serial.print("len=");
        Serial.println(ret, DEC);

        Serial.print("Dest=");
        printMACAddress(&buffer[0]);
        Serial.print("Src=");
        printMACAddress(&buffer[6]);

        // 0x0800 = IPv4
        // 0x0806 = ARP
        // 0x86DD = IPv6
        Serial.print("Type=0x");
        printPaddedHex(buffer[12]);
        printPaddedHex(buffer[13]);
        Serial.println();

        Serial.println();
    }
}
