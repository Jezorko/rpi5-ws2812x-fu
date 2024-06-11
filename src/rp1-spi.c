#include <stdlib.h>

#include "rp1-regs.h"
#include "rp1-spi.h"
#include "rp1-spi-regs.h"

const uint32_t spi_bases[] = {
    RP1_SPI0_BASE,
    RP1_SPI1_BASE,
    RP1_SPI2_BASE,
    RP1_SPI3_BASE,
    RP1_SPI4_BASE,
    RP1_SPI5_BASE,
    RP1_SPI6_BASE,
    RP1_SPI7_BASE,
    RP1_SPI8_BASE
};

bool rp1_spi_create(rp1_t *rp1, uint8_t spinum, rp1_spi_instance_t **spi)
{

    rp1_spi_instance_t *s = (rp1_spi_instance_t *)calloc(1, sizeof(rp1_spi_instance_t));
    if (s == NULL)
        return false;

    s->regbase = rp1->rp1_peripherial_base + spi_bases[spinum];
    s->txdata = (char *)0x0;
    s->rxdata = (char *)0x0;
    s->txcount = 0x0;

    *spi = s;

    return true;
}

uint8_t get_bit(uint8_t array[], short bit) {
    return (array[bit / 8] >> (bit % 8)) & 1;
}

// according to https://wp.josh.com/2014/05/13/ws2812-neopixels-are-not-so-finicky-once-you-get-to-know-them/
// since we transmit 50ns at a time (20MHz) we will need to transmit 3 bytes (24 bits, 1200ns) per bit of data
// most fitting timings would be:
// ON 13 (650ns/50) bits high, 11 (550ns/50) bits low
// 11111111 11111000 00000000
// 0xff     0xf8     0x00
// OFF 7 (350ns/50) bits high, 17 (850ns/50) bits low
// 11111110 00000000 00000000
// 0xfe     0x00     0x00
// and to reset/latch
// RES 120 bits low (6000ns/50) == 15 bytes
uint8_t on[3]  = { 0xff, 0xf8, 0x00 };
uint8_t off[3] = { 0xfe, 0x00, 0x00 };

// we need 256 (possible byte values) * 8 (bits in each byte) * 3 (bytes per bit) for a lookup table == 6144 bytes
uint8_t* byte_lookup_table[255];

void rp1_spi_write_array_blocking(rp1_spi_instance_t *spi, uint8_t data[], int data_length)
{
    while(*(volatile uint32_t *)(spi->regbase + DW_SPI_SR) & DW_SPI_SR_BUSY)
    {
        /* wait until the spi is not busy */;
    }

    while(!(*(volatile uint32_t *)(spi->regbase + DW_SPI_SR) & DW_SPI_SR_TF_NOT_FULL))
    {
       /* spin until we can write to the fifo */;
    }

    // set the CS pin
    *(volatile uint32_t *)(spi->regbase + DW_SPI_SER) = 1 <<0;

    for (int i = 0; i < data_length; ++i) {
        // put the data into the fifo
        *(volatile uint8_t *)(spi->regbase + DW_SPI_DR) = data[i];
        // we now need to pull exactly one byte out of the fifo which would
        // have been clocked in when we wrote the data
        /*uint8_t discard = */*(volatile uint8_t *)(spi->regbase + DW_SPI_DR);
    }
}