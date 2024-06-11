#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>

#include "rp1-regs.h"
#include "rp1-spi.h"
#include "rp1-spi-regs.h"

void *mapgpio(off_t dev_base, off_t dev_size)
{
    int fd;
    void *mapped;
    
    printf("sizeof(off_t) %d\n", sizeof(off_t));

    if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1)
    {
        printf("Can't open /dev/mem\n");
        return (void *)0;
    }

    mapped = mmap(0, dev_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, dev_base);
    // close(fd);

    printf("base address: %llx, size: %x, mapped: %p\n", dev_base, dev_size, mapped);

    if (mapped == (void *)-1)
    {
        printf("Can't map the memory to user space.\n");
        return (void *)0;
    }

    return mapped;
}



bool create_rp1(rp1_t **rp1, void *base)
{

    rp1_t *r = (rp1_t *)calloc(1, sizeof(rp1_t));
    if (r == NULL)
        return false;

    r->rp1_peripherial_base = base;
    r->gpio_base = base + RP1_IO_BANK0_BASE;
    r->pads_base = base + RP1_PADS_BANK0_BASE;
    r->rio_out = (volatile uint32_t *)(base + RP1_RIO0_BASE + RIO_OUT_OFFSET);
    r->rio_output_enable = (volatile uint32_t *)(base + RP1_RIO0_BASE + RIO_OE_OFFSET);
    r->rio_nosync_in = (volatile uint32_t *)(base + RP1_RIO0_BASE + RIO_NOSYNC_IN_OFFSET);

    *rp1 = r;

    return true;
}

uint8_t mosi_pin_number = 10;

bool create_pin(rp1_t *rp1, uint32_t funcmask)
{
    gpio_pin_t *newpin = calloc(1, sizeof(gpio_pin_t));
    if(newpin == NULL) return false;

    newpin->number = mosi_pin_number;

    // each gpio has a status and control register
    // adjacent to each other. control = status + 4 (uint8_t)
    newpin->status = (uint32_t *)(rp1->gpio_base + 8 * mosi_pin_number);
    newpin->ctrl = (uint32_t *)(rp1->gpio_base + 8 * mosi_pin_number + 4);
    newpin->pad = (uint32_t *)(rp1->pads_base + PADS_BANK0_GPIO_OFFSET + mosi_pin_number * 4);

    // set the function
    *(newpin->ctrl + RP1_ATOM_CLR_OFFSET / 4) = CTRL_MASK_FUNCSEL; // first clear the bits
    *(newpin->ctrl + RP1_ATOM_SET_OFFSET / 4) = funcmask;  // now set the value we need

    rp1->pins[mosi_pin_number] = newpin;
    //printf("pin %d stored in pins array %p\n", mosi_pin_number, rp1->pins[mosi_pin_number]);

    return true;
}

uint8_t get_bit(uint8_t value, short bit) {
    return (value >> (bit % 8)) & 1;
}

int main(void)
{

    int i, j;

    /////////////////////////////////////////////////////////
    // RP1

    // get the peripheral base address
    void *base = mapgpio(RP1_BAR1, RP1_BAR1_LEN);
    if (base == NULL) {
        printf("unable to map base\n");
        return 4;
    } 

    // create a rp1 device
    printf("creating rp1\n");
    rp1_t *rp1;
    if (!create_rp1(&rp1, base))
    {
        printf("unable to create rp1\n");
        return 2;
    }
    
    
    /////////////////////////////////////////////////////////
    // SPI

    // create a spi instance
    rp1_spi_instance_t *spi;
    if (!rp1_spi_create(rp1, 0, &spi))
    {
        printf("unable to create spi\n");
        return 5;
    }

    // disable the SPI
    *(volatile uint32_t *)(spi->regbase + DW_SPI_SSIENR) = 0x0;

    printf("setting up the pins for SPI0\n");
    create_pin(rp1, 0x00); // MOSI

    // set the speed - this is the divisor from 200MHz in the RPi5
    *(volatile uint32_t *)(spi->regbase + DW_SPI_BAUDR) = 10; // 20 MHz
    printf("\nbaudr: %d MHz\n", 200/(*(volatile uint32_t *)(spi->regbase + DW_SPI_BAUDR)));

    // set mode - CPOL = 0, CPHA = 1 (Mode 1)
    printf("Setting SPI to Mode 1\n");
    // read control
    uint32_t reg_ctrlr0 = *(volatile uint32_t *)(spi->regbase + DW_SPI_CTRLR0);
    printf("ctrlr0 before setting: %x\n", reg_ctrlr0);
    reg_ctrlr0 |= DW_PSSI_CTRLR0_SCPHA;
    // update the control reg
    *(volatile uint32_t *)(spi->regbase + DW_SPI_CTRLR0) = reg_ctrlr0;
    reg_ctrlr0 = *(volatile uint32_t *)(spi->regbase + DW_SPI_CTRLR0);
    printf("ctrlr0 after setting (might be the same as before if mode was already set): %x\n", reg_ctrlr0);

    // clear interrupts by reading the interrupt status register
    uint32_t reg_icr = *(volatile uint32_t *)(spi->regbase + DW_SPI_ICR);
    printf("icr: %x\n", reg_icr);

    // mask off interrupts
    // uint32_t reg_imr = *(volatile uint32_t *)(spi->regbase + DW_SPI_IMR);
    // *(volatile uint32_t *)(spi->regbase + DW_SPI_IMR) = reg_imr & 0xFFFFFF00;

    // enable the SPI
    *(volatile uint32_t *)(spi->regbase + DW_SPI_SSIENR) = 0x1;

    // LED data
    uint8_t data[] = {
        // G R B
        0x00, 0xff, 0x00, // red
        0x00, 0x00, 0xff, // blue
        0xff, 0x00, 0x00, // green
        0x00, 0xff, 0x00, // red
        0x00, 0x00, 0xff, // blue
        0xff, 0x00, 0x00, // green
        0x00, 0xff, 0x00, // red
        0x00, 0x00, 0xff, // blue
        0xff, 0x00, 0x00, // green
        0x00, 0xff, 0x00, // red
        0x00, 0x00, 0xff, // blue
        0xff, 0x00, 0x00, // green
        0x00, 0xff, 0x00, // red
        0x00, 0x00, 0xff, // blue
        0xff, 0x00, 0x00, // green
        0x00, 0xff, 0x00, // red
        0x00, 0x00, 0xff, // blue
        0xff, 0x00, 0x00, // green
    };
    int data_length = sizeof(data);

    // according to https://wp.josh.com/2014/05/13/ws2812-neopixels-are-not-so-finicky-once-you-get-to-know-them/
    // since we transmit 50ns at a time (20MHz) we will need to transmit 3 bytes (24 bits, 1200ns) per byte of data
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
    int reset_signal_length_bytes = 15;

    // every bit is transmitted as 24 bits
    // so we need to create a data array with length * 24
    // plus 15 bytes for reset/latch signal
    int transformed_data_length = (data_length * 24) + reset_signal_length_bytes;
    uint8_t data_transformed[transformed_data_length];

    // zero out last 15 bytes
    for (int i = transformed_data_length - 1; i >= transformed_data_length - reset_signal_length_bytes; --i) {
        data_transformed[i] = 0x00;
    }

    for (uint8_t value = 0; ; ++value) {
        // for each bit
        printf("{ %d = ", value);
        for (int bitId = 0; bitId < 8; ++bitId) {
            // get bit value
            uint8_t bit = get_bit(value, bitId);
            if (bit == 1) {
                printf("0x%02x, 0x%02x, 0x%02x", on[0], on[1], on[2]);
            } else {
                printf("0x%02x, 0x%02x, 0x%02x", off[0], off[1], off[2]);
            }
            if (bitId != 7) printf(", ");
        }
        printf(" },\n");
        if (value == 255) break;
    }

    // send all transformed data at once
    // rp1_spi_write_array_blocking(spi, data_transformed, transformed_data_length);

    return 0;
}
