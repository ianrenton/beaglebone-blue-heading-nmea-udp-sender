#include <stdlib.h>
#include <stdio.h>
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

// Bus for Robotics Cape and BeagleboneBlue is 2
#define I2C_BUS 2
// Offset added to magnetometer's heading to produce a heading with respect to
// True North. Should take into account any device calibration offset plus local
// magnetic declination.
#define OFFSET 0.0
// UDP configuration
#define UDP_SEND_SERVER "127.0.0.1"
#define UDP_SEND_PORT_1 2021
#define UDP_SEND_PORT_2 2022

static int running = 0;

// interrupt handler to catch ctrl-c
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
    int socket1;
    struct sockaddr_in server1;
    if ((socket1 = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) >= 0) {
        server1.sin_family      = AF_INET;
        server1.sin_port        = htons(UDP_SEND_PORT_1);
        server1.sin_addr.s_addr = inet_addr(UDP_SEND_SERVER);
    } else {
        fprintf(stderr,"create socket 1 failed\n");
        return -1;
    }
    int socket2;
    struct sockaddr_in server2;
    if ((socket2 = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) >= 0) {
        server2.sin_family      = AF_INET;
        server2.sin_port        = htons(UDP_SEND_PORT_2);
        server2.sin_addr.s_addr = inet_addr(UDP_SEND_SERVER);
    } else {
        fprintf(stderr,"create socket 2 failed\n");
        return -1;
    }
    printf("Sending heading data to local UDP ports %d and %d.\n", UDP_SEND_PORT_1, UDP_SEND_PORT_2);

    // Main run loop
    while (running) {
        // Read X & Y mag data
        rc_mpu_read_mag(&data);
        double xField = data.mag[0];
        double yField = data.mag[1];

        // Translate to orientation of Beaglebone in the boat
        double forwardField = -yField;
        double stbdField = -xField;

        // Calculate heading
        double heading = atan2(-stbdField, forwardField)*(180/M_PI);
        heading = heading + OFFSET;
        if (heading < 0.0) {
            heading = heading + 360.0;
        }
        if (heading > 360.0) {
            heading = heading - 360.0;
        }

        // Build NMEA message content
        char message[20], messageInner[14];
        sprintf(messageInner, "HEHDT,%03.1f,T", heading);

        // Calculate checksum
        int crc = 0;
        int i;
        for (i = 0; i < 14; i ++) {
            crc ^= messageInner[i];
        }

        // Assemble full message
        sprintf(message, "$%s*%02X\r\n", messageInner, crc);

        // Send packets
        sendto(socket1, message, (strlen(message)+1), 0, (struct sockaddr *)&server1, sizeof(server1));
        sendto(socket2, message, (strlen(message)+1), 0, (struct sockaddr *)&server2, sizeof(server2));

        // 100ms delay until next time
        rc_usleep(100000);
    }

    // Disable MPU & close sockets
    rc_mpu_power_off();
    close(socket1);
    close(socket2);
    return 0;
}
