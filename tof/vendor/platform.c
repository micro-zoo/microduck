/*
 * Linux i2c-dev and spidev platform layer for ST's VL53L5CX and VL53L8CX ULDs.
 *
 * ONE implementation, compiled once per generation. Two things are supplied by
 * the build (see ../build.rs), because the two ULDs each want the hooks under
 * their own names and with their own platform struct:
 *
 *   TOF_PLATFORM   the struct type this generation's header defines
 *   RdByte, WrByte, RdMulti, WrMulti, SwapBuffer, WaitMs
 *                  renamed per generation, so both sets can link into one
 *                  binary — the ULD sources call them unprefixed and are not
 *                  edited, so the rename has to happen in the preprocessor
 *
 * Without the rename the two generations would define the same six symbols and
 * the link would fail; with it, each ULD calls exactly the platform layer that
 * was compiled against its own struct.
 *
 * ST ships a template with these six hooks left to the integrator; this is the
 * Linux one, carried over from `microduck_runtime` where it was measured. Direct
 * I2C_RDWR ioctls, chunked. Doing the transfers in C rather than through a
 * per-callback trip into a scripting language cuts the per-transaction overhead
 * ~10x: the 90 kB firmware upload takes a few seconds at 400 kHz instead of
 * tens of seconds — and that upload happens on every sensor start.
 */

#include "platform.h"

/* The generation's struct, under one name for the code below. */
typedef TOF_PLATFORM Platform;

#include <errno.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <time.h>

/* One chunk per I2C_RDWR message. The rk3x controller handles large
 * messages fine (FIFO refills under interrupt); 2 KiB keeps each bus
 * hold short enough that the codec never waits long — the TLV320AIC3104
 * shares this bus, and a stalled mixer write is audible. */
#define CHUNK 2048u

static uint8_t wr_buf[CHUNK + 2];

static uint8_t xfer(Platform *p, struct i2c_msg *msgs, int n)
{
    struct i2c_rdwr_ioctl_data data = { .msgs = msgs, .nmsgs = (uint32_t)n };
    return ioctl(p->fd, I2C_RDWR, &data) < 0 ? 255 : 0;
}

/*
 * SPI, for the VL53L8CX on a spidev — the framing of ST's own Linux platform
 * layer (STSW-IMG042, user/platform/platform.c): one full-duplex transfer per
 * access, chip select low throughout, starting with the 16-bit register index
 * big-endian, whose bit 15 is 0 for a read and 1 for a write. A read's data is
 * what comes back after those two bytes. The mode, word size and clock are set
 * once on the descriptor when it is opened (probe.c, tof_spi_open).
 *
 * SPI_CHUNK is spidev's default 4096-byte transfer limit less the two index
 * bytes, as in ST's layer. Nothing else is on this bus, so unlike CHUNK above
 * there is no neighbour to keep short.
 */
#define SPI_CHUNK (4096u - 2u)

static uint8_t spi_tx[SPI_CHUNK + 2];
static uint8_t spi_rx[SPI_CHUNK + 2];

static uint8_t spi_xfer(Platform *p, uint16_t idx, uint32_t n)
{
    struct spi_ioc_transfer t;
    memset(&t, 0, sizeof t);
    t.tx_buf = (unsigned long)spi_tx;
    t.rx_buf = (unsigned long)spi_rx;
    t.len = n + 2;
    spi_tx[0] = (uint8_t)(idx >> 8);
    spi_tx[1] = (uint8_t)idx;
    return ioctl(p->fd, SPI_IOC_MESSAGE(1), &t) < 0 ? 255 : 0;
}

static uint8_t spi_rd(Platform *p, uint16_t reg, uint8_t *values, uint32_t size)
{
    for (uint32_t off = 0; off < size;) {
        uint32_t n = size - off > SPI_CHUNK ? SPI_CHUNK : size - off;
        memset(spi_tx + 2, 0, n);
        if (spi_xfer(p, (uint16_t)((reg + off) & 0x7FFF), n)) return 255;
        memcpy(values + off, spi_rx + 2, n);
        off += n;
    }
    return 0;
}

static uint8_t spi_wr(Platform *p, uint16_t reg, uint8_t *values, uint32_t size)
{
    for (uint32_t off = 0; off < size;) {
        uint32_t n = size - off > SPI_CHUNK ? SPI_CHUNK : size - off;
        memcpy(spi_tx + 2, values + off, n);
        if (spi_xfer(p, (uint16_t)((reg + off) | 0x8000), n)) return 255;
        off += n;
    }
    return 0;
}

uint8_t RdMulti(Platform *p, uint16_t reg, uint8_t *values,
                uint32_t size)
{
    if (p->spi) return spi_rd(p, reg, values, size);
    uint32_t off = 0;
    while (off < size) {
        uint16_t idx = (uint16_t)(reg + off);
        uint32_t n = size - off > CHUNK ? CHUNK : size - off;
        uint8_t idx_buf[2] = { (uint8_t)(idx >> 8), (uint8_t)idx };
        struct i2c_msg msgs[2] = {
            { .addr = (uint16_t)(p->address >> 1), .flags = 0,
              .len = 2, .buf = idx_buf },
            { .addr = (uint16_t)(p->address >> 1), .flags = I2C_M_RD,
              .len = (uint16_t)n, .buf = values + off },
        };
        if (xfer(p, msgs, 2)) return 255;
        off += n;
    }
    return 0;
}

uint8_t WrMulti(Platform *p, uint16_t reg, uint8_t *values,
                uint32_t size)
{
    if (p->spi) return spi_wr(p, reg, values, size);
    uint32_t off = 0;
    while (off < size) {
        uint16_t idx = (uint16_t)(reg + off);
        uint32_t n = size - off > CHUNK ? CHUNK : size - off;
        wr_buf[0] = (uint8_t)(idx >> 8);
        wr_buf[1] = (uint8_t)idx;
        memcpy(wr_buf + 2, values + off, n);
        struct i2c_msg msg = {
            .addr = (uint16_t)(p->address >> 1), .flags = 0,
            .len = (uint16_t)(n + 2), .buf = wr_buf,
        };
        if (xfer(p, &msg, 1)) return 255;
        off += n;
    }
    return 0;
}

uint8_t RdByte(Platform *p, uint16_t reg, uint8_t *value)
{
    return RdMulti(p, reg, value, 1);
}

uint8_t WrByte(Platform *p, uint16_t reg, uint8_t value)
{
    return WrMulti(p, reg, &value, 1);
}

void SwapBuffer(uint8_t *buffer, uint16_t size)
{
    /* Byte-reverse each 32-bit word in place (size is a multiple of 4). */
    for (uint16_t i = 0; i + 3 < size; i += 4) {
        uint8_t a = buffer[i], b = buffer[i + 1];
        buffer[i] = buffer[i + 3];
        buffer[i + 1] = buffer[i + 2];
        buffer[i + 2] = b;
        buffer[i + 3] = a;
    }
}

uint8_t WaitMs(Platform *p, uint32_t ms)
{
    (void)p;
    struct timespec ts = { .tv_sec = ms / 1000,
                           .tv_nsec = (long)(ms % 1000) * 1000000L };
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {}
    return 0;
}
