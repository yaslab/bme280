// clang -Wall -Wextra -Wsign-conversion -o bme280 bme280.c main.c

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

#include "bme280.h"

void setup(void) {
    bme280_t bme280 = {};
    if (!bme280_i2c_open(&bme280, 1)) {
        printf("Cannot open device\n");
        exit(1);
    }
    bme280_write_config(&bme280, OSRS_X2, OSRS_X16, OSRS_X1, MODE_NORMAL, SB_0_5, FILTER_16, 0);
    bme280_close(&bme280);
}

void write_log(const char *logfile) {
    bme280_t bme280 = {};
    if (!bme280_i2c_open(&bme280, 1)) {
        printf("Cannot open device\n");
        exit(1);
    }
    bme280_read_calib(&bme280);
    bme280_read_raw_temp(&bme280);
    bme280_read_raw_hum(&bme280);
    bme280_read_raw_press(&bme280);
    double temp = bme280_get_compensate_temp(&bme280);
    double hum = bme280_get_compensate_hum(&bme280);
    double press = bme280_get_compensate_press(&bme280);
    bme280_close(&bme280);

    FILE *fp = fopen(logfile, "a");
    if (!fp) {
        printf("Cannot open logfile: %s\n", logfile);
        exit(1);
    }
    time_t timer = time(NULL);
    fprintf(fp, "%ld,%.2f,%.2f,%.2f\n", timer, temp, hum, press);
    fclose(fp);
}

int main(void) {
    char *logfile = "/home/pi/weather_log.csv";

    setup();
    sleep(5);
    while (1) {
        write_log(logfile);
        sleep(30);
    }

    return 0;
}
