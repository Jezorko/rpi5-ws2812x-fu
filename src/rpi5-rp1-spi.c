#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>


#include "rp1-regs.h"
#include "rp1-spi.h"
#include "rp1-spi-regs.h"

void *mapgpio(off_t dev_base, off_t dev_size)
{
    int fd;
    void *mapped;
    
    // printf("sizeof(off_t) %d\n", sizeof(off_t));

    if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1)
    {
        printf("Can't open /dev/mem\n");
        return (void *)0;
    }

    mapped = mmap(0, dev_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, dev_base);
    // close(fd);

    // printf("base address: %llx, size: %x, mapped: %p\n", dev_base, dev_size, mapped);

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

bool create_mosi_pin(rp1_t *rp1, uint32_t funcmask)
{
    uint8_t mosi_pin_number = 10;
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

rp1_spi_instance_t *spi;

int data_length = 0;
uint8_t* data;

void close_strip()
{
    // disable the SPI
    *(volatile uint32_t *)(spi->regbase + DW_SPI_SSIENR) = 0x0;
    free(data);
}

uint8_t* initialize_strip(int leds_count)
{
    data_length = leds_count * 3;
    // data = (uint8_t*) malloc(leds_count * 3);
    data = calloc(leds_count * 3 + 1250 /* latch time */, sizeof(uint8_t*));
    /////////////////////////////////////////////////////////
    // RP1

    // get the peripheral base address
    void *base = mapgpio(RP1_BAR1, RP1_BAR1_LEN);
    if (base == NULL) {
        printf("unable to map base\n");
    }

    // create a rp1 device
    // printf("creating rp1\n");
    rp1_t *rp1;
    if (!create_rp1(&rp1, base))
    {
        printf("unable to create rp1\n");
    }


    /////////////////////////////////////////////////////////
    // SPI

    // create a spi instance
    if (!rp1_spi_create(rp1, 0, &spi))
    {
        printf("unable to create spi\n");
    }

    close_strip();

    create_mosi_pin(rp1, 0x00); // MOSI is all we need anyways

    // set the speed - this is the divisor from 200MHz in the RPi5
    *(volatile uint32_t *)(spi->regbase + DW_SPI_BAUDR) = 40; // 5 MHz
    // printf("\nbaudr: %d MHz\n", 200/(*(volatile uint32_t *)(spi->regbase + DW_SPI_BAUDR)));

    // set mode - CPOL = 0, CPHA = 1 (Mode 1)
    // printf("Setting SPI to Mode 1\n");
    // read control
    uint32_t reg_ctrlr0 = *(volatile uint32_t *)(spi->regbase + DW_SPI_CTRLR0);
    // printf("ctrlr0 before setting: %x\n", reg_ctrlr0);
    reg_ctrlr0 |= DW_PSSI_CTRLR0_SCPHA;
    // update the control reg
    *(volatile uint32_t *)(spi->regbase + DW_SPI_CTRLR0) = reg_ctrlr0;
    reg_ctrlr0 = *(volatile uint32_t *)(spi->regbase + DW_SPI_CTRLR0);
    // printf("ctrlr0 after setting (might be the same as before if mode was already set): %x\n", reg_ctrlr0);

    // clear interrupts by reading the interrupt status register
    uint32_t reg_icr = *(volatile uint32_t *)(spi->regbase + DW_SPI_ICR);
    // printf("icr: %x\n", reg_icr);

    // enable the SPI
    *(volatile uint32_t *)(spi->regbase + DW_SPI_SSIENR) = 0x1;

    return data;
}

void render_strip() {
    rp1_spi_write_array_blocking(spi, data, data_length);
}

int leds_count = 12;

void set_led(int led_id, uint8_t red, uint8_t green, uint8_t blue) {
    int led_data_id = led_id * 3;
    data[led_data_id + 1] = red;
    data[led_data_id + 0] = green;
    data[led_data_id + 2] = blue;
}

void set_all_leds_to(uint8_t red, uint8_t green, uint8_t blue) {
    for (int led_id = 0; led_id < leds_count; ++led_id) {
        set_led(led_id, red, green, blue);
    }
}

int main(void)
{
    uint8_t* data = initialize_strip(leds_count);

    for (int led_id = 0; led_id < leds_count; ++led_id) {
        if (led_id % 2 == 0) {
            set_led(led_id, 0xff, 0x00, 0x00);
        } else {
            set_led(led_id, 0x00, 0x00, 0xff);
        }
    }

    sleep(2);

    set_all_leds_to(0x00, 0x00, 0x00);

    render_strip();

    close_strip();
    return 0;
}
