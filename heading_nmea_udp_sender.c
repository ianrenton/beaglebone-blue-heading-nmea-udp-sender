// Beaglebone Blue Heading NMEA UDP Sender
// by Ian Renton, March 2023
// https://github.com/ianrenton/beaglebone-blue-heading-nmea-udp-sender
// Based on the MPU example from the Beaglebone Robot Control Library.
//
// Reads data from a Beaglebone Blue magnetometer, formats it as an NMEA-0183
// HDT message, and sends it via UDP. Maybe useful for robotics software that
// expects that format of heading data. This can also be ingested into `gpsd`,
// by e.g. adding `udp://0.0.0.0:2021` to its list of sources.
//
// If using this for yourself, you may need to customise the #define values
// near the top of the file to reflect the orientation of your board in your
// robot.

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <rc/mpu.h>
#include <rc/time.h>

// The code treats the Beaglebone Blue's +X direction as the heading of the robot. If your
// board is fitted in a different orientation, or is not exactly lined up, set the
// HEADING_OFFSET value here. This value will be added (i.e. clockwise rotation) to the
// magnetometer-based heading to give the result. So if +X points to the left of your robot,
// this should be 90. If +X points to the right, -90. And if +X points backwards, 180.
#define HEADING_OFFSET 90.0
// If your board is mounted upside-down in your robot, the heading results will go the
// wrong way (i.e. increase anticlockwise). Set BOARD_INVERTED to fix this.
#define BOARD_INVERTED false
// The magnetometer produces readings based on magnetic north, whereas the HDT message
// produced by this code should contain a heading based on true north. Enter your local
// magnetic declination here to apply this offset. Positive declination is when mag
// north is east/clockwise of true north.
#define LOCAL_MAGNETIC_DECLINATION 0.1
// This code sends the heading data to the host and port specified here.
#define UDP_SEND_SERVER "127.0.0.1"
#define UDP_SEND_PORT 2021
// Set the I2C bus on which to communicate with the 9DOF MPU. For the Beaglebone Blue and
// Beaglebone Black Robotics Cape, this is 2.
#define I2C_BUS 2


// interrupt handler to catch ctrl-c
static int running = 0;
static void __signal_handler(__attribute__ ((unused)) int dummy) {
    running = 0;
    return;
}

int main()  {
    // Struct to hold data
    rc_mpu_data_t data;

    // Set up interrupt handler
    signal(SIGINT, __signal_handler);
    running = 1;

    // Set up MPU config
    rc_mpu_config_t conf = rc_mpu_default_config();
    conf.i2c_bus = I2C_BUS;
    conf.enable_magnetometer = 1;

    // Enable MPU, exit on failure
    if (rc_mpu_initialize(&data, conf)){
        fprintf(stderr,"rc_mpu_initialize failed\n");
        return -1;
    }

    // Create UDP sockets, exit on failure
    int udpsocket;
    struct sockaddr_in server;
    if ((udpsocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) >= 0) {
        server.sin_family      = AF_INET;
        server.sin_port        = htons(UDP_SEND_PORT);
        server.sin_addr.s_addr = inet_addr(UDP_SEND_SERVER);
    } else {
        fprintf(stderr,"create socket failed\n");
        return -1;
    }

    // Main run loop
    while (running) {
        // Read X & Y mag data
        rc_mpu_read_mag(&data);
        double xField = data.mag[0];
        double yField = data.mag[1];

        // Calculate heading based on magnetometer reading
        double heading = -atan2(-yField, xField)*(180/M_PI);

        // Apply offsets and investions
        if (BOARD_INVERTED) {
            heading = -heading;
        }
        heading = heading + HEADING_OFFSET;
        heading = heading + LOCAL_MAGNETIC_DECLINATION;

        // Ensure we get a number in the range 0.0<=x<360.0
        while (heading < 0.0) {
            heading = heading + 360.0;
        }
        while (heading >= 360.0) {
            heading = heading - 360.0;
        }

        // Build NMEA message content
        char message[20], messageInner[14];
        sprintf(messageInner, "GPHDT,%03.1f,T", heading);

        // Calculate checksum
        int crc = 0;
        int i;
        for (i = 0; i < 14; i ++) {
            crc ^= messageInner[i];
        }

        // Assemble full message
        sprintf(message, "$%s*%02X\r\n", messageInner, crc);

        // Send packet
        sendto(udpsocket, message, (strlen(message)+1), 0, (struct sockaddr *)&server, sizeof(server));

        // 100ms delay until next time
        rc_usleep(100000);
    }

    // Disable MPU & close sockets
    rc_mpu_power_off();
    close(udpsocket);
    return 0;
}
