# PortableNTP
A Portable GPS based NTP Server for Amateur Radio Use

It utilises a ESP32 and a GPS board for the time source with a RTC to provide a time source when a satellite lock in unavailable

The GPS board needs to supply a 1 HZ PPS output. Some Blox boards do not have this available at the pin headers but it is available off a specific resistor which drives an onboard LED as PPS is part of the chipset

Battery backup is included

I have based it on the work of Christano Monteiro
https://www.linkedin.com/pulse/iot-maker-tale-stratum-1-time-server-built-from-scratch-monteiro/
https://github.com/Montecri/GNSSTimeServer - Christano's source code

Also Brett Olivers work : [his site really is worth a visi for some great projects he has built]
http://www.brettoliver.org.uk/GPS_Time_Server/GPS_Time_Server.htm

http://w8bh.net/avr/clock2.pdf - GPS clock work and algorithms
https://forum.arduino.cc/t/ntp-time-server/192816 - NTP Time Server in Arduino
