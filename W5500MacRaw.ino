#include "w5500.h"


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



const byte mac_address[] = {
    0x52, 0xff, 0xee, 0x1b, 0x44, 0x55
};

Wiznet5500 w5500;

void setup() {
    // Setup serial port for debugging
    Serial.begin(115200);
    Serial.println("[W5500MacRaw]");
    printMACAddress(mac_address);

    w5500.begin(mac_address);
}


uint8_t buffer[800];
uint8_t send_count=0;

void loop() {

    uint16_t len = w5500.readFrame(buffer, sizeof(buffer));
    if ( len > 0 ) {
        Serial.print("Len=");
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

        // Reply to the 0x88B5 Local Experimental Ethertype
        if (buffer[12] == 0x88 && buffer[13] == 0xB5) {
            Serial.print("Byte 15=");
            Serial.println(buffer[15], DEC);

            memcpy(&buffer[0], &buffer[6], 6);   // Set Destination to Source
            memcpy(&buffer[6], mac_address, 6);  // Set Source to our MAC address
            buffer[14] = send_count++;
            w5500.sendFrame(buffer, len);
        }

        Serial.println();
    }
}
