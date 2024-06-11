#include <stddef.h>
#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

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

void rp1_spi_write_array_blocking(rp1_spi_instance_t *spi, uint8_t data[], int data_length)
{

    // nope. no difference

    // wait until the spi is not busy
    while(*(volatile uint32_t *)(spi->regbase + DW_SPI_SR) & DW_SPI_SR_BUSY)
    {
        ;
    }

    // spin until we can write to the fifo
    while(!(*(volatile uint32_t *)(spi->regbase + DW_SPI_SR) & DW_SPI_SR_TF_NOT_FULL))
    {
       ;
    }

    // set the CS pin
    *(volatile uint32_t *)(spi->regbase + DW_SPI_SER) = 1 <<0;

    for (int i = 0; i < data_length; ++i) {
        // put the data into the fifo
        *(volatile uint8_t *)(spi->regbase + DW_SPI_DR) = data[i];
        /*uint8_t discard = */*(volatile uint8_t *)(spi->regbase + DW_SPI_DR); // yeah nah it is necessary
    }
}