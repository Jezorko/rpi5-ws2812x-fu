#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "rp1-regs.h"

// only six of the nine SPI peripherals are available on the gpio
// these are:
// SPI0, SPI1, SPI2, SPI3, SPI4, SPI5
// see 3.11 Function Select in the RP1 datasheet (2024-Feb)
// https://datasheets.raspberrypi.com/rp1/rp1-peripherals.pdf

typedef enum {
    SPI_OK = 0,
    SPI_ERROR = 1,
    SPI_BUSY = 2,
    SPI_TIMEOUT = 3,
    SPI_INVALID = 4
} spi_status_t;



bool rp1_spi_create(rp1_t *rp1, uint8_t spinum, rp1_spi_instance_t **spi);
spi_status_t rp1_spi_write_array_blocking(rp1_spi_instance_t *spi, uint8_t data[], int data_length);
