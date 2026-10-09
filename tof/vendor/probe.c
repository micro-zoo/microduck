/*
 * Which sensor is on the bus, before any driver is loaded.
 *
 * Both generations share the paged register map: select bank 0, read the two ID
 * registers, put the bank back. Revision 0x02 is a VL53L5CX, 0x0C a VL53L8CX,
 * and the two need different firmware — so this has to be answered *first*,
 * before a ~90 KB upload the wrong sensor would reject halfway through.
 *
 * Standalone on purpose: it opens its own descriptor and does its own three
 * transactions rather than borrowing a generation's platform layer, because
 * borrowing one would mean picking a generation before knowing which is there.
 * Sixty lines of ioctl beats that chicken-and-egg.
 */

#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

/* Write a 16-bit register address plus one byte. */
static int wr_reg8(int fd, uint8_t addr7, uint16_t reg, uint8_t value)
{
    uint8_t buf[3] = { (uint8_t)(reg >> 8), (uint8_t)reg, value };
    struct i2c_msg msg = { .addr = addr7, .flags = 0, .len = 3, .buf = buf };
    struct i2c_rdwr_ioctl_data data = { .msgs = &msg, .nmsgs = 1 };
    return ioctl(fd, I2C_RDWR, &data) < 0 ? -1 : 0;
}

/* Address a 16-bit register, then read `len` bytes back. */
static int rd_reg(int fd, uint8_t addr7, uint16_t reg, uint8_t *out, uint16_t len)
{
    uint8_t idx[2] = { (uint8_t)(reg >> 8), (uint8_t)reg };
    struct i2c_msg msgs[2] = {
        { .addr = addr7, .flags = 0, .len = 2, .buf = idx },
        { .addr = addr7, .flags = I2C_M_RD, .len = len, .buf = out },
    };
    struct i2c_rdwr_ioctl_data data = { .msgs = msgs, .nmsgs = 2 };
    return ioctl(fd, I2C_RDWR, &data) < 0 ? -1 : 0;
}

/*
 * 0 on success, with the two ID bytes written out. -1 means nothing answered:
 * no sensor at that address, or no bus.
 */
int tof_probe_id(const char *dev_path, uint8_t addr_7bit,
                 uint8_t *device_id, uint8_t *revision_id)
{
    int fd = open(dev_path, O_RDWR);
    if (fd < 0) return -1;

    uint8_t ident[2] = { 0, 0 };
    int rc = 0;
    if (wr_reg8(fd, addr_7bit, 0x7FFF, 0x00)) rc = -1;
    if (!rc && rd_reg(fd, addr_7bit, 0x0000, ident, 2)) rc = -1;
    /* Put the bank back even if the read failed: the next attempt (another
     * address, or a retry) starts from the state the sensor was found in. */
    if (wr_reg8(fd, addr_7bit, 0x7FFF, 0x02) && !rc) rc = -1;

    close(fd);
    if (rc) return -1;
    *device_id = ident[0];
    *revision_id = ident[1];
    return 0;
}

/*
 * SPI: the VL53L8CX on a spidev. ST's settings (STSW-IMG042's Linux platform
 * layer): mode 0, 8-bit words, 2 MHz — below the 8 MHz the device tree allows,
 * and the clock ST's own driver runs it at. The firmware upload is ~90 KB, so
 * this is under half a second of it.
 *
 * Here rather than in a shim because both callers need it before a generation
 * is known: the probe below, and vl8_open_spi once the probe has answered.
 */
#define TOF_SPI_HZ 2000000u

int tof_spi_open(const char *dev_path)
{
    int fd = open(dev_path, O_RDWR);
    if (fd < 0) return -1;
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t hz = TOF_SPI_HZ;
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

/* One access, in the framing platform.c's spi_xfer uses: the 16-bit index with
 * bit 15 set for a write, then the data, in one full-duplex transfer. */
static int spi_access(int fd, uint16_t reg, int write, uint8_t *data, uint16_t len)
{
    uint8_t tx[2 + 4] = { 0 }, rx[2 + 4] = { 0 };
    if (len > 4) return -1;
    uint16_t idx = write ? (uint16_t)(reg | 0x8000) : (uint16_t)(reg & 0x7FFF);
    tx[0] = (uint8_t)(idx >> 8);
    tx[1] = (uint8_t)idx;
    if (write) memcpy(tx + 2, data, len);
    struct spi_ioc_transfer t;
    memset(&t, 0, sizeof t);
    t.tx_buf = (unsigned long)tx;
    t.rx_buf = (unsigned long)rx;
    t.len = (uint32_t)(len + 2);
    if (ioctl(fd, SPI_IOC_MESSAGE(1), &t) < 0) return -1;
    if (!write) memcpy(data, rx + 2, len);
    return 0;
}

/*
 * The same three transactions as tof_probe_id, over SPI. A bus with nothing on
 * it still completes every transfer — SPI has no acknowledge — so "nothing
 * answered" shows up as an ID neither driver knows, and the caller says so.
 */
int tof_probe_id_spi(const char *dev_path, uint8_t *device_id, uint8_t *revision_id)
{
    int fd = tof_spi_open(dev_path);
    if (fd < 0) return -1;

    uint8_t bank0 = 0x00, bank2 = 0x02;
    uint8_t ident[2] = { 0, 0 };
    int rc = 0;
    if (spi_access(fd, 0x7FFF, 1, &bank0, 1)) rc = -1;
    if (!rc && spi_access(fd, 0x0000, 0, ident, 2)) rc = -1;
    if (spi_access(fd, 0x7FFF, 1, &bank2, 1) && !rc) rc = -1;

    close(fd);
    if (rc) return -1;
    *device_id = ident[0];
    *revision_id = ident[1];
    return 0;
}
