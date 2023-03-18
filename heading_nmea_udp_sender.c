// Beaglebone Blue Heading NMEA UDP Sender
// by Ian Renton, March 2023
// https://github.com/ianrenton/beaglebone-blue-heading-nmea-udp-sender
// Based on the MPU example from the Beaglebone Robot Control Library.
//
// Reads data from a Beaglebone Blue motion processor, formats it as an NMEA-0183
// GPHDT message, and sends it via UDP. Maybe useful for robotics software that
// expects that format of heading data. This can also be ingested into `gpsd`,
// by e.g. adding `udp://0.0.0.0:2021` to its list of sources.
//
// If using this for yourself, you may need to customise the #define values
// near the top of the file to reflect the orientation of your board in your
// robot.
//
// Must be run as root.

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
// The magnetometer produces readings based on magnetic north, whereas the HDT message
// produced by this code should contain a heading based on true north. Enter your local
// magnetic declination here to apply this offset. Positive declination is when mag
// north is east/clockwise of true north.
#define LOCAL_MAGNETIC_DECLINATION 0.1
// This code sends the heading data to the host and port specified here.
#define UDP_SEND_SERVER "127.0.0.1"
#define UDP_SEND_PORT 2021
// Set the sample rate between 4 & 200 Hz. HDT messages will be sent at this rate. 10 Hz
// recommended.
#define SAMPLE_RATE_HZ 10
// Set the I2C bus on which to communicate with the 9DOF MPU. For the Beaglebone Blue and
// Beaglebone Black Robotics Cape, this is 2.
#define I2C_BUS 2
// Set a GPIO pin to use for an interrupt, in the DMP mode the MPU controls timing and
// will interrupt us when it has new data
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21


// Globals to pass data between threads
rc_mpu_data_t data;
int udpsocket;
struct sockaddr_in server;

// interrupt handler to catch ctrl-c
static int running = 0;
static void __signal_handler(__attribute__ ((unused)) int dummy) {
    running = 0;
    return;
}

// Handle data function. Called back at a predefined interval by the MPU
// when it has new data.
static void __handle_data(void) {
    // Get a heading value based on filtered compass heading reported by MPU
    // Requires inversion so that clockwise is positive
    double heading = -data.compass_heading * RAD_TO_DEG;

    // Apply offsets
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
}

// Main function
int main()  {
    // Set up interrupt handler
    signal(SIGINT, __signal_handler);
    running = 1;

    // Set up MPU config
    rc_mpu_config_t conf = rc_mpu_default_config();
    conf.i2c_bus = I2C_BUS;
    conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
    conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
    conf.enable_magnetometer = 1;
    conf.dmp_sample_rate = SAMPLE_RATE_HZ;

    // Enable MPU, exit on failure
    if (rc_mpu_initialize_dmp(&data, conf)){
        fprintf(stderr,"rc_mpu_initialize_dmp failed\n");
        return -1;
    }

    // Create UDP sockets, exit on failure
    if ((udpsocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) >= 0) {
        server.sin_family      = AF_INET;
        server.sin_port        = htons(UDP_SEND_PORT);
        server.sin_addr.s_addr = inet_addr(UDP_SEND_SERVER);
    } else {
        fprintf(stderr,"create socket failed\n");
        return -1;
    }

    // Set the DMP callback method - the MPU will control the timing
    // from now on.
    rc_mpu_set_dmp_callback(&__handle_data);

    // Wait until we need to quit, nothing else to do
    while (running) {
        rc_usleep(100000);
    }

    // Disable MPU & close sockets
    rc_mpu_power_off();
    close(udpsocket);
    return 0;
}
