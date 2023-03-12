# Beaglebone Blue Heading NMEA UDP Sender
Reads data from a Beaglebone Blue magnetometer, formats it as an NMEA-0183 HDT message, and sends it via UDP. Maybe useful for robotics software that expects that format of heading data. This can also be ingested into `gpsd`, by e.g. adding `udp://0.0.0.0:2021` to its list of sources.

Note that the talker ID used is "GP", so the message will be "GPHDT". This is for compatibility with `gpsd`, which will ignore other talker IDs. If you're not using this with `gpsd`, and have a thing for standards compliance, feel free to change it to e.g. "HE".

Apologies for code quality, it's been a while since I last wrote any C.

`make` does exactly what you expect. `make install` will put it in `/usr/local/bin` and create a systemd service for it to run in the background.

Based on the [MPU example](https://beagleboard.org/static/librobotcontrol/rc_test_mpu_8c-example.html) from the Beaglebone Robot Control Library.
