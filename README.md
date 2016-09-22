W5500MacRaw
===========

This Arduino sketch demonstrates reading and writing raw Ethernet frames using the [Wiznet W5500] ethernet controller.

* When a packet is received it prints the Ethernet headers to Serial.
* If the packet is of the special [EtherType] 0x88B5, then it sends a reply back to the sender, while incrementing an 8-bit counter

It is based on the [ioLibrary Driver] code from [Wiznet]. All code that does not relate to Socket 0 and sending and receiving Ethernet frames has been stripped out for size.

Also included in the `sendeth` directory, is some Linux code for sending and receiving Ethernet frames to the Arduino.

The code is licenses under the [3-clause BSD license], the same as the ioLibrary driver.



[Wiznet]:                  http://www.wiznet.co.kr/
[Wiznet W5500]:            http://www.wiznet.co.kr/product-item/w5500/
[ioLibrary Driver]:        http://github.com/wiznet/ioLibrary_Driver
[EtherType]:               http://en.wikipedia.org/wiki/EtherType
[3-clause BSD license]:    http://opensource.org/licenses/BSD-3-Clause
