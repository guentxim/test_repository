// main.cpp
// IMU Test Code
// This code initializes the IMU, performs calibration, and continuously reads and prints IMU data (accelerometer, gyroscope, and orientation) to the serial console.

#include "IMU.h"
#include "config.h"
#include "mbed.h"
#include <cstdarg>
#include <cstdio>

static BufferedSerial pc(BBOP_LOG_COM_UART_TX_PIN, BBOP_LOG_COM_UART_RX_PIN, 115200);

static void pc_printf(const char *fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (n > 0) {
        pc.write(buf, n);
    }
}

int main() {
    pc_printf("\n\n=== IMU Test Started ===\n");

    IMU imu(BBOP_IMU_SDA_PIN, BBOP_IMU_SCL_PIN);
    pc_printf("IMU initialized\n");

    pc_printf("Calibrating IMU...\n");
    int calibration_counter = 0;

    while (!imu.isCalibrated()) {
        imu.getImuData();
        calibration_counter++;
        if (calibration_counter % 500 == 0) {
            pc_printf("Calibration progress: %d\n", calibration_counter);
        }
        ThisThread::sleep_for(1ms);   // statt thread_sleep_for
    }

    pc_printf("IMU calibration complete!\n\n");

    while (true) {
        IMU::ImuData data = imu.getImuData();

        pc_printf("ACC [m/s^2]: X=%.3f Y=%.3f Z=%.3f | ",
                  data.acc(0), data.acc(1), data.acc(2));

        pc_printf("GYRO [rad/s]: X=%.3f Y=%.3f Z=%.3f | ",
                  data.gyro(0), data.gyro(1), data.gyro(2));

        pc_printf("RPY [deg]: R=%.1f P=%.1f Y=%.1f\n",
                  data.rpy(0) * 57.2958f, data.rpy(1) * 57.2958f, data.rpy(2) * 57.2958f);

        ThisThread::sleep_for(100ms);
    }
}
