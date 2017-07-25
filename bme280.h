#ifndef BME280_H
#define BME280_H

#include <stdint.h>
#include <stdbool.h>
#include <linux/types.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>

typedef struct {

    int fd;

    int is_spi;
    int speed_hz;
    int delay_usecs;
    int bits;

    int dig_temp[3];
    int dig_press[9];
    int dig_hum[6];

    uint32_t raw_temp;
    uint32_t raw_press;
    uint32_t raw_hum;

    double temp_fine;

} bme280_t;

bool bme280_i2c_open(bme280_t *bme280, int bus);
bool bme280_spi_open(bme280_t *bme280, int bus, int chip_select);
void bme280_close(bme280_t *bme280);

void bme280_write_config(bme280_t *bme280, uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h, uint8_t mode, uint8_t t_sb, uint8_t filter, uint8_t spi3w_en);
void bme280_read_raw_temp(bme280_t *bme280);
void bme280_read_raw_press(bme280_t *bme280);
void bme280_read_raw_hum(bme280_t *bme280);
void bme280_read_calib(bme280_t *bme280);
double bme280_get_compensate_temp(bme280_t *bme280);
double bme280_get_compensate_press(bme280_t *bme280);
double bme280_get_compensate_hum(bme280_t *bme280);
void bme280_dump(bme280_t *bme280);

#define DEFAULT_I2C_SLAVE   0x76

#define DEFAULT_SPI_MODE    SPI_MODE_0
#define DEFAULT_SPI_BITS    8
#define DEFAULT_SPI_SPEED   500000
#define DEFAULT_SPI_DELAY   0

#define OSRS_SKIPPED    0b000 // Skipped (output set to 0x80000)
#define OSRS_X1         0b001 // oversampling 1
#define OSRS_X2         0b010 // oversampling 2
#define OSRS_X4         0b011 // oversampling 4
#define OSRS_X8         0b100 // oversampling 8
#define OSRS_X16        0b101 // oversampling 16

#define MODE_SLEEP      0b00 // Sleep mode
#define MODE_FORCED     0b01 // Forced mode
#define MODE_NORMAL     0b11 // Normal mode

#define SB_0_5          0b000 // 0.5ms
#define SB_62_5         0b001 // 62.5ms
#define SB_125          0b010 // 125ms
#define SB_250          0b011 // 250ms
#define SB_500          0b100 // 500ms
#define SB_1000         0b101 // 1000ms
#define SB_10           0b110 // 10ms
#define SB_20           0b111 // 20ms

#define FILTER_OFF      0b000 // Filter off
#define FILTER_2        0b001 // 2
#define FILTER_4        0b010 // 4
#define FILTER_8        0b011 // 8
#define FILTER_16       0b100 // 16

#endif /* BME280_H */
