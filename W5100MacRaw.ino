#include <SPI.h>
#include "w5100.h"

const int sockNum = 0;

W5100Class w5100;


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

int read(uint8_t *buffer, uint16_t bufsize)
{
    uint16_t ptr=0;

    SPI.beginTransaction(SPI_ETHERNET_SETTINGS);

    int16_t ret = w5100.getRXReceivedSize(sockNum);
    if (ret > 0) {
        uint8_t head[8];
        uint16_t data_len=0;

        ptr = w5100.readSnRX_RD(sockNum);

        w5100.read_data(sockNum, ptr, head, 2);
        ptr+=2;
        data_len = head[0];
        data_len = (data_len<<8) + head[1] - 2;

        w5100.read_data(sockNum, ptr, buffer, data_len);
        ptr += data_len;
        w5100.writeSnRX_RD(sockNum, ptr);
        w5100.execCmdSn(sockNum, Sock_RECV);
    }

    SPI.endTransaction();

    return ret;
}


void setup() {
    // Setup serial port for debugging
    Serial.begin(38400);
    Serial.println("[W5100MacRaw]");

    byte mac_address[] = {
        0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
    };

    // Initialise the basic info
    w5100.init();
    SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
    w5100.writeSHAR(mac_address);
    SPI.endTransaction();

    SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
    w5100.writeSnMR(sockNum, SnMR::MACRAW);
    w5100.execCmdSn(sockNum, Sock_OPEN);
    SPI.endTransaction();

    SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
    uint8_t socketStatus = w5100.readSnSR(sockNum);
    SPI.endTransaction();
    Serial.print("socketStatus=0x");
    Serial.println(socketStatus, HEX);

    SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
    uint8_t retryCount = w5100.readRCR();
    SPI.endTransaction();

    Serial.print("retryCount=");
    Serial.println(retryCount, DEC);
}


uint8_t buffer[800];

void loop() {

    uint16_t len = read(buffer, sizeof(buffer));
    if ( len > 0 ) {
        Serial.print("len=");
        Serial.println(len, DEC);

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
