#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "bme280.h"

#define ARRAY_SIZE(A)   (sizeof(A)/(sizeof(A[0])))

// ----------------------------------------------------------------------------
// I/O Utility (I2C)

bool bme280_i2c_open(bme280_t *bme280, int bus) {
    char device[128];
    sprintf(device, "/dev/i2c-%d", bus);
    int fd = open(device, O_RDWR);
    if (fd < 0) {
        return false;
    }
    bme280->fd = fd;
    bme280->is_spi = false;
    return bme280_i2c_setup(bme280, DEFAULT_I2C_SLAVE);
}

bool bme280_i2c_setup(bme280_t *bme280, int slave) {
    if (ioctl(bme280->fd, I2C_SLAVE, slave) < 0) {
        return false;
    }
    return true;
}

// ----------------------------------------------------------------------------
// I/O Utility (SPI)

bool bme280_spi_open(bme280_t *bme280, int bus, int chip_select) {
    char device[128];
    sprintf(device, "/dev/spidev%d.%d", bus, chip_select);
    int fd = open(device, O_RDWR);
    if (fd < 0) {
        return false;
    }
    bme280->fd = fd;
    bme280->is_spi = true;
    return bme280_spi_setup(bme280, DEFAULT_SPI_MODE, DEFAULT_SPI_BITS, DEFAULT_SPI_SPEED, DEFAULT_SPI_DELAY);
}

bool bme280_spi_setup(bme280_t *bme280, int mode, int bits, int speed, int delay) {
    if (ioctl(bme280->fd, SPI_IOC_WR_MODE, &mode) < 0) {
        return false;
    }
    if (ioctl(bme280->fd, SPI_IOC_RD_MODE, &mode) < 0) {
        return false;
    }

    if (ioctl(bme280->fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        return false;
    }
    if (ioctl(bme280->fd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) {
        return false;
    }

    if (ioctl(bme280->fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        return false;
    }
    if (ioctl(bme280->fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
        return false;
    }

    bme280->bits = bits;
    bme280->speed_hz = speed;
    bme280->delay_usecs = delay;

    return true;
}

// ----------------------------------------------------------------------------
// I/O Utility (Common)

void bme280_close(bme280_t *bme280) {
    close(bme280->fd);
    bme280->fd = 0;
    bme280->is_spi = false;
}

static inline uint8_t bme280_read(bme280_t *bme280, uint8_t addr) {
    uint8_t tx[3] = {};
    uint8_t rx[3] = {};

    tx[0] = addr;

    if (bme280->is_spi) {
        struct spi_ioc_transfer tr = {};

        tr.tx_buf           = (unsigned long)tx;
        tr.rx_buf           = (unsigned long)rx;

        tr.len              = ARRAY_SIZE(tx);
        tr.speed_hz         = (uint32_t)bme280->speed_hz;

        tr.delay_usecs      = bme280->delay_usecs;
        tr.bits_per_word    = bme280->bits;
        tr.cs_change        = 0;

        if (ioctl(bme280->fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
            // error
        }
        
        return rx[1];
    } else {
        if (write(bme280->fd, tx, 1) != 1) {
            // error
        }
        if (read(bme280->fd, rx, 1) != 1) {
            // error
        }
        return rx[0];
    }
}

static inline int bme280_write(bme280_t *bme280, uint8_t addr, uint8_t data) {
    if (bme280->is_spi) {
        addr &= 0x7f;
    }
    uint8_t tx[2] = {addr, data};
    return write(bme280->fd, tx, 2);
}

// ----------------------------------------------------------------------------
// BME280

void bme280_write_config(bme280_t *bme280, uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h, uint8_t mode, uint8_t t_sb, uint8_t filter, uint8_t spi3w_en) {    
    uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
    uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
    uint8_t ctrl_hum_reg  = osrs_h;
    
    bme280_write(bme280, 0xf2, ctrl_hum_reg);
    bme280_write(bme280, 0xf4, ctrl_meas_reg);
    bme280_write(bme280, 0xf5, config_reg);
}

void bme280_read_raw_temp(bme280_t *bme280) {
    uint32_t temp_msb = bme280_read(bme280, 0xfa);
    uint32_t temp_lsb = bme280_read(bme280, 0xfb);
    uint32_t temp_xsb = bme280_read(bme280, 0xfc);

    bme280->raw_temp = (temp_msb << 12) | (temp_lsb << 4) | (temp_xsb >> 4);
}

void bme280_read_raw_press(bme280_t *bme280) {
    uint32_t press_msb = bme280_read(bme280, 0xf7);
    uint32_t press_lsb = bme280_read(bme280, 0xf8);
    uint32_t press_xsb = bme280_read(bme280, 0xf9);

    bme280->raw_press = (press_msb << 12) | (press_lsb << 4) | (press_xsb >> 4);
}

void bme280_read_raw_hum(bme280_t *bme280) {
    uint32_t hum_msb = bme280_read(bme280, 0xfd);
    uint32_t hum_lsb = bme280_read(bme280, 0xfe);

    bme280->raw_hum = (hum_msb << 8) | hum_lsb;
}

void bme280_read_calib(bme280_t *bme280) {
    int addr, i;
    uint8_t hi, lo;

    addr = 0x88;
    for (i = 0; i < 3; i++) {
        lo = bme280_read(bme280, addr);
        hi = bme280_read(bme280, addr + 1);

        bme280->dig_temp[i] = (hi << 8) | lo;
        addr += 2;
    }

    addr = 0x8e;
    for (i = 0; i < 9; i++) {
        lo = bme280_read(bme280, addr);
        hi = bme280_read(bme280, addr + 1);

        bme280->dig_press[i] = (hi << 8) | lo;
        addr += 2;
    }

    bme280->dig_hum[0] = bme280_read(bme280, 0xa1);
    bme280->dig_hum[1] = (bme280_read(bme280, 0xe2) << 8) | bme280_read(bme280, 0xe1);
    bme280->dig_hum[2] = bme280_read(bme280, 0xe3);
    uint8_t data_e5 = bme280_read(bme280, 0xe5);
    bme280->dig_hum[3] = (bme280_read(bme280, 0xe4) << 4) | (data_e5 & 0x0f);
    bme280->dig_hum[4] = (bme280_read(bme280, 0xe6) << 4) | ((data_e5 >> 4) & 0x0f);
    bme280->dig_hum[5] = bme280_read(bme280, 0xe7);
}

// Returns temperature in DegC, double precision. Output value of "51.23" equals 51.23 DegC.
// t_fine carries fine temperature as global value
double bme280_get_compensate_temp(bme280_t *bme280) {
    double adc_T = bme280->raw_temp;
    double dig_T1 = bme280->dig_temp[0];
    double dig_T2 = bme280->dig_temp[1];
    double dig_T3 = bme280->dig_temp[2];
    
    double var1 = (adc_T / 16384.0 - dig_T1 / 1024.0) * dig_T2;
    double var2 = (adc_T / 131072.0 - dig_T1 / 8192.0) * (adc_T / 131072.0 - dig_T1 / 8192.0) * dig_T3;
    double T = var1 + var2;
    bme280->temp_fine = T;
    return T / 5120.0;
}

// Returns pressure in Pa as double. Output value of "96386.2" equals 96386.2 Pa = 963.862 hPa
double bme280_get_compensate_press(bme280_t *bme280) {
    double adc_P = bme280->raw_press;
    double dig_P1 = bme280->dig_press[0];
    double dig_P2 = bme280->dig_press[1];
    double dig_P3 = bme280->dig_press[2];
    double dig_P4 = bme280->dig_press[3];
    double dig_P5 = bme280->dig_press[4];
    double dig_P6 = bme280->dig_press[5];
    double dig_P7 = bme280->dig_press[6];
    double dig_P8 = bme280->dig_press[7];
    double dig_P9 = bme280->dig_press[8];

    // Update temp_fine
    bme280_get_compensate_temp(bme280);

    double var1 = (bme280->temp_fine / 2.0) - 64000.0;
    double var2 = var1 * var1 * dig_P6 / 32768.0;
    var2 = var2 + var1 * dig_P5 * 2.0;
    var2 = var2 / 4.0 + dig_P4 * 65536.0;
    var1 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * dig_P1;
    if (var1 == 0.0) {
        // avoid exception caused by division by zero
        return 0;
    }
    double p = 1048576.0 - adc_P;
    p = (p - var2 / 4096.0) * 6250.0 / var1;
    var1 = dig_P9 * p * p / 2147483648.0;
    var2 = p * dig_P8 / 32768.0;
    return p + (var1 + var2 + dig_P7) / 16.0;
}

// Returns humidity in %rH as as double. Output value of "46.332" represents 46.332 %rH
double bme280_get_compensate_hum(bme280_t *bme280) {
    double adc_H = bme280->raw_hum;
    double dig_H1 = bme280->dig_hum[0];
    double dig_H2 = bme280->dig_hum[1];
    double dig_H3 = bme280->dig_hum[2];
    double dig_H4 = bme280->dig_hum[3];
    double dig_H5 = bme280->dig_hum[4];
    double dig_H6 = bme280->dig_hum[5];

    // Update temp_fine
    bme280_get_compensate_temp(bme280);

    double var_H = bme280->temp_fine - 76800.0;
    var_H = adc_H - (dig_H4 * 64.0 + dig_H5 / 16384.0 * var_H);
    var_H = var_H * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * var_H * (1.0 + dig_H3 / 67108864.0 * var_H)));
    var_H = var_H * (1.0 - dig_H1 * var_H / 524288.0);
    if (var_H > 100.0) {
        var_H = 100.0;
    } else if (var_H < 0.0) {
        var_H = 0.0;
    }
    return var_H;
}

void bme280_dump(bme280_t *bme280) {
    int i = 0;

    printf("===== DUMP =====\n");
    printf("    raw_temp: %d\n", bme280->raw_temp);
    printf("    raw_press: %d\n", bme280->raw_press);
    printf("    raw_hum: %d\n", bme280->raw_hum);
    printf("    temp_fine: %f\n", bme280->temp_fine);

    printf("    dig_temp:\n");
    printf("        ");
    for (i = 0; i < 3; i++) {
        printf("%04x, ", bme280->dig_temp[i]);
    }
    printf("\n");

    printf("    dig_press:\n");
    printf("        ");
    for (i = 0; i < 9; i++) {
        printf("%04x, ", bme280->dig_press[i]);
    }
    printf("\n");

    printf("    dig_hum:\n");
    printf("        ");
    for (i = 0; i < 6; i++) {
        printf("%04x, ", bme280->dig_hum[i]);
    }
    printf("\n");
}
