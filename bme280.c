#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <time.h>

#define ARRAY_SIZE(A)   (sizeof(A)/(sizeof(A[0])))

#define DEFAULT_MODE    SPI_MODE_0
#define DEFAULT_BITS    8
#define DEFAULT_SPEED   500000
#define DEFAULT_DELAY   0

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

typedef struct {
    int fd;
    int speed_hz;
    int delay_usecs;
    int bits;

    int dig_temp[3];
    int dig_press[9];
    int dig_hum[6];

    uint32_t raw_temp;
    uint32_t raw_press;
    uint32_t raw_hum;

    int temp_fine;
} bme280_t;

// ----------------------------------------------------------------------------
// I/O Utility

int bme280_spi_open(bme280_t *bme280, const char *device) {
    int fd = open(device, O_RDWR);
    if (fd < 0) {
        return -1;
    }
    bme280->fd = fd;
    return 0;
}

void bme280_spi_close(bme280_t *bme280) {
    close(bme280->fd);
    bme280->fd = 0;
}

uint8_t bme280_spi_read(bme280_t *bme280, uint8_t addr) {
    uint8_t tx[3] = {0};
    uint8_t rx[3] = {0};
    struct spi_ioc_transfer tr = {0};

    tx[0] = addr;

    tr.tx_buf        = (unsigned long)tx;
    tr.rx_buf        = (unsigned long)rx;

    tr.len           = ARRAY_SIZE(tx);
    tr.speed_hz      = bme280->speed_hz;

    tr.delay_usecs   = bme280->delay_usecs;
    tr.bits_per_word = bme280->bits;
    tr.cs_change     = 0;

    if (ioctl(bme280->fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        // error
    }
    
    return rx[1];
}

int bme280_spi_write(bme280_t *bme280, uint8_t addr, uint8_t data) {
    uint8_t tx[2] = {addr & 0x7f, data};
    return write(bme280->fd, tx, 2);
}

int bme280_spi_setup(bme280_t *bme280, int mode, int bits, int speed, int delay) {
    if (ioctl(bme280->fd, SPI_IOC_WR_MODE, &mode) < 0) {
        return -1;
    }
    if (ioctl(bme280->fd, SPI_IOC_RD_MODE, &mode) < 0) {
        return -1;
    }

    if (ioctl(bme280->fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        return -1;
    }
    if (ioctl(bme280->fd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) {
        return -1;
    }

    if (ioctl(bme280->fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        return -1;
    }
    if (ioctl(bme280->fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
        return -1;
    }

    bme280->bits = bits;
    bme280->speed_hz = speed;
    bme280->delay_usecs = delay;

    return 0;
}

// ----------------------------------------------------------------------------
// 

void bme280_write_config(bme280_t *bme280, uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h, uint8_t mode, uint8_t t_sb, uint8_t filter, uint8_t spi3w_en) {    
    uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
    uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
    uint8_t ctrl_hum_reg  = osrs_h;
    
    bme280_spi_write(bme280, 0xf2, ctrl_hum_reg);
    bme280_spi_write(bme280, 0xf4, ctrl_meas_reg);
    bme280_spi_write(bme280, 0xf5, config_reg);
}

void bme280_read_raw_temp(bme280_t *bme280) {
    uint8_t temp_msb = bme280_spi_read(bme280, 0xfa);
    uint8_t temp_lsb = bme280_spi_read(bme280, 0xfb);
    uint8_t temp_xsb = bme280_spi_read(bme280, 0xfc);

    bme280->raw_temp = (temp_msb << 12) | (temp_lsb << 4) | (temp_xsb >> 4);
}

void bme280_read_raw_press(bme280_t *bme280) {
    uint8_t press_msb = bme280_spi_read(bme280, 0xf7);
    uint8_t press_lsb = bme280_spi_read(bme280, 0xf8);
    uint8_t press_xsb = bme280_spi_read(bme280, 0xf9);

    bme280->raw_press = (press_msb << 12) | (press_lsb << 4) | (press_xsb >> 4);
}

void bme280_read_raw_hum(bme280_t *bme280) {
    uint8_t hum_msb = bme280_spi_read(bme280, 0xfd);
    uint8_t hum_lsb = bme280_spi_read(bme280, 0xfe);

    bme280->raw_hum = (hum_msb << 8) | hum_lsb;
}

void bme280_read_calib(bme280_t *bme280) {
    int addr, i;
    uint8_t hi, lo;

    addr = 0x88;
    for (i = 0; i < 3; i++) {
        lo = bme280_spi_read(bme280, addr);
        hi = bme280_spi_read(bme280, addr + 1);

        bme280->dig_temp[i] = (hi << 8) | lo;
        addr += 2;
    }

    addr = 0x8e;
    for (i = 0; i < 9; i++) {
        lo = bme280_spi_read(bme280, addr);
        hi = bme280_spi_read(bme280, addr + 1);

        bme280->dig_press[i] = (hi << 8) | lo;
        addr += 2;
    }

    bme280->dig_hum[0] = bme280_spi_read(bme280, 0xa1);
    bme280->dig_hum[1] = (bme280_spi_read(bme280, 0xe2) << 8) | bme280_spi_read(bme280, 0xe1);
    bme280->dig_hum[2] = bme280_spi_read(bme280, 0xe3);
    uint8_t data_e5 = bme280_spi_read(bme280, 0xe5);
    bme280->dig_hum[3] = (bme280_spi_read(bme280, 0xe4) << 4) | (data_e5 & 0x0f);
    bme280->dig_hum[4] = (bme280_spi_read(bme280, 0xe6) << 4) | ((data_e5 >> 4) & 0x0f);
    bme280->dig_hum[5] = bme280_spi_read(bme280, 0xe7);
}

// Returns temperature in DegC, double precision. Output value of "51.23" equals 51.23 DegC.
// t_fine carries fine temperature as global value
double bme280_get_compensate_temp(bme280_t *bme280) { //(BME280_S32_t adc_T) {
    double var1, var2, T;
    int adc_T = bme280->raw_temp;
    int dig_T1 = bme280->dig_temp[0];
    int dig_T2 = bme280->dig_temp[1];
    int dig_T3 = bme280->dig_temp[2];
    
    var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
    var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) * (((double)adc_T)/131072.0 - ((double) dig_T1)/8192.0)) * ((double)dig_T3);
    bme280->temp_fine = (int)(var1 + var2);
    T = (var1 + var2) / 5120.0;
    return T;
}

// Returns pressure in Pa as double. Output value of "96386.2" equals 96386.2 Pa = 963.862 hPa
double bme280_get_compensate_press(bme280_t *bme280) {
    double var1, var2, p;
    int adc_P = bme280->raw_press;
    int dig_P1 = bme280->dig_press[0];
    int dig_P2 = bme280->dig_press[1];
    int dig_P3 = bme280->dig_press[2];
    int dig_P4 = bme280->dig_press[3];
    int dig_P5 = bme280->dig_press[4];
    int dig_P6 = bme280->dig_press[5];
    int dig_P7 = bme280->dig_press[6];
    int dig_P8 = bme280->dig_press[7];
    int dig_P9 = bme280->dig_press[8];

    var1 = ((double)bme280->temp_fine/2.0) - 64000.0;
    var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double)dig_P5) * 2.0;
    var2 = (var2/4.0)+(((double)dig_P4) * 65536.0);
    var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0)*((double)dig_P1);
    if (var1 == 0.0) {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576.0 - (double)adc_P;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double)dig_P9) * p * p / 2147483648.0;
    var2 = p * ((double)dig_P8) / 32768.0;
    p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;
    return p;
}

// Returns humidity in %rH as as double. Output value of "46.332" represents 46.332 %rH
double bme280_get_compensate_hum(bme280_t *bme280) {
    double var_H;
    int adc_H = bme280->raw_hum;
    int dig_H1 = bme280->dig_hum[0];
    int dig_H2 = bme280->dig_hum[1];
    int dig_H3 = bme280->dig_hum[2];
    int dig_H4 = bme280->dig_hum[3];
    int dig_H5 = bme280->dig_hum[4];
    int dig_H6 = bme280->dig_hum[5];

    var_H = (((double)bme280->temp_fine) - 76800.0);
    var_H = (adc_H - (((double)dig_H4) * 64.0 + ((double)dig_H5) / 16384.0 * var_H)) *
        (((double)dig_H2) / 65536.0 * (1.0 + ((double)dig_H6) / 67108864.0 * var_H *
        (1.0 + ((double)dig_H3) / 67108864.0 * var_H)));
    var_H = var_H * (1.0 - ((double)dig_H1) * var_H / 524288.0);
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
    printf("    temp_fine: %d\n", bme280->temp_fine);

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

// ----------------------------------------------------------------------------
// 

void setup(const char *device) {
    bme280_t bme280 = {0};    
    if (bme280_spi_open(&bme280, device) < 0) {
        printf("Cannot open device: %s\n", device);
        exit(-1);
    }
    bme280_spi_setup(&bme280, DEFAULT_MODE, DEFAULT_BITS, DEFAULT_SPEED, DEFAULT_DELAY);
    bme280_write_config(&bme280, OSRS_X2, OSRS_X16, OSRS_X1, MODE_NORMAL, SB_0_5, FILTER_16, 0);
    bme280_spi_close(&bme280);
}

void write_log(const char *device, const char *logfile) {
    bme280_t bme280 = {0};
    if (bme280_spi_open(&bme280, device) < 0) {
        printf("Cannot open device: %s\n", device);
        exit(-1);
    }
    bme280_spi_setup(&bme280, DEFAULT_MODE, DEFAULT_BITS, DEFAULT_SPEED, DEFAULT_DELAY);
    bme280_read_calib(&bme280);
    bme280_read_raw_temp(&bme280);
    bme280_read_raw_hum(&bme280);
    bme280_read_raw_press(&bme280);
    double temp = bme280_get_compensate_temp(&bme280);
    double hum = bme280_get_compensate_hum(&bme280);
    double press = bme280_get_compensate_press(&bme280);
    bme280_spi_close(&bme280);

    time_t timer = time(NULL);
    FILE *fp = fopen(logfile, "a");
    if (!fp) {
        printf("Cannot open logfile: %s\n", logfile);
        exit(-1);
    }
    fprintf(fp, "%ld,%.2f,%.2f,%.2f\n", timer, temp, hum, press);
    fclose(fp);
}

int main(int argc, char *argv[]) {
    char *device = "/dev/spidev0.0";
    char *logfile = "/home/pi/weather_log.csv";

    setup(device);
    sleep(5);
    while (1) {
        write_log(device, logfile);
        sleep(30);
    }
    return 0;
}
