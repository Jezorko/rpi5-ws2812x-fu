/*
    SPI access on RP1 through PCI BAR1
    2024 March
    Praktronics
    GPL3
    

    run with sudo or as root for permissions to access to /dev/mem
    to compile
    /rpi5-rp1-spi $ mkdir build
    /rpi5-rp1-spi $ cd build
    /rpi5-rp1-spi/build $ cmake ..
    /rpi5-rp1-spi/build $ cmake --build .

    run with sudo or as root
    /rpi5-rp1-spi/build/bin $ sudo ./rpi5-rp1-spi

*/


#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <time.h>

#include "rp1-regs.h"
#include "rp1-spi.h"
#include "rp1-spi-regs.h"
#include "pi_pico_commands.h"

void delay_ms(int milliseconds)
{
    struct timespec ts;
    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = (milliseconds % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

// pci bar info
// from: https://github.com/G33KatWork/RP1-Reverse-Engineering/blob/master/pcie/hacks.py
#define RP1_BAR1 0x1f00000000
#define RP1_BAR1_LEN 0x400000

// offsets from include/dt-bindings/mfd/rp1.h
// https://github.com/raspberrypi/linux/blob/rpi-6.1.y/include/dt-bindings/mfd/rp1.h
#define RP1_IO_BANK0_BASE 0x0d0000
#define RP1_RIO0_BASE 0x0e0000
#define RP1_PADS_BANK0_BASE 0x0f0000

// the following info is from the RP1 datasheet (draft & incomplete as of 2024-02-18)
// https://datasheets.raspberrypi.com/rp1/rp1-peripherals.pdf
#define RP1_ATOM_XOR_OFFSET 0x1000
#define RP1_ATOM_SET_OFFSET 0x2000
#define RP1_ATOM_CLR_OFFSET 0x3000

#define PADS_BANK0_VOLTAGE_SELECT_OFFSET 0
#define PADS_BANK0_GPIO_OFFSET 0x4

#define RIO_OUT_OFFSET 0x00
#define RIO_OE_OFFSET 0x04
#define RIO_NOSYNC_IN_OFFSET 0x08
#define RIO_SYNC_IN_OFFSET 0x0C
//                           3         2         1
//                          10987654321098765432109876543210
#define CTRL_MASK_FUNCSEL 0b00000000000000000000000000011111
#define PADS_MASK_OUTPUT  0b00000000000000000000000011000000

#define CTRL_FUNCSEL_RIO 0x05


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

bool create_pin(uint8_t pinnumber, rp1_t *rp1)
{
    gpio_pin_t *newpin = calloc(1, sizeof(gpio_pin_t));
    if(newpin == NULL) return false;

    newpin->number = pinnumber;

    // each gpio has a status and control register
    // adjacent to each other. control = status + 4 (uint8_t)
    newpin->status = (uint32_t *)(rp1->gpio_base + 8 * pinnumber);
    newpin->ctrl = (uint32_t *)(rp1->gpio_base + 8 * pinnumber + 4);
    newpin->pad = (uint32_t *)(rp1->pads_base + PADS_BANK0_GPIO_OFFSET + pinnumber * 4);

    // set the function
    *(newpin->ctrl + RP1_ATOM_CLR_OFFSET / 4) = CTRL_MASK_FUNCSEL; // first clear the bits
    *(newpin->ctrl + RP1_ATOM_SET_OFFSET / 4) = CTRL_FUNCSEL_RIO;  // now set the value we need

    rp1->pins[pinnumber] = newpin;
    printf("pin %d stored in pins array %p\n", pinnumber, rp1->pins[pinnumber]);

    return true;
}

bool create_pin_2(uint8_t pinnumber, rp1_t *rp1, uint32_t funcmask)
{
    gpio_pin_t *newpin = calloc(1, sizeof(gpio_pin_t));
    if(newpin == NULL) return false;

    newpin->number = pinnumber;

    // each gpio has a status and control register
    // adjacent to each other. control = status + 4 (uint8_t)
    newpin->status = (uint32_t *)(rp1->gpio_base + 8 * pinnumber);
    newpin->ctrl = (uint32_t *)(rp1->gpio_base + 8 * pinnumber + 4);
    newpin->pad = (uint32_t *)(rp1->pads_base + PADS_BANK0_GPIO_OFFSET + pinnumber * 4);

    // set the function
    *(newpin->ctrl + RP1_ATOM_CLR_OFFSET / 4) = CTRL_MASK_FUNCSEL; // first clear the bits
    *(newpin->ctrl + RP1_ATOM_SET_OFFSET / 4) = funcmask;  // now set the value we need

    rp1->pins[pinnumber] = newpin;
    //printf("pin %d stored in pins array %p\n", pinnumber, rp1->pins[pinnumber]);

    return true;
}

int pin_enable_output(uint8_t pinnumber, rp1_t *rp1)
{

    printf("Attempting to enable output\n");
   
    // first enable the pad to output
    // pads needs to have OD[7] -> 0 (don't disable output)
    // and                IE[6] -> 0 (don't enable input)
    // we use atomic access to the bit clearing alias with a mask
    // divide the offset by 4 since we're doing uint32* math

    volatile uint32_t *writeadd = rp1->pins[pinnumber]->pad + RP1_ATOM_CLR_OFFSET / 4;

    printf("attempting write for %p at %p\n", rp1->pins[pinnumber]->pad, writeadd);

    *writeadd = PADS_MASK_OUTPUT;

    // now set the RIO output enable using the atomic set alias
    *(rp1->rio_output_enable + RP1_ATOM_SET_OFFSET / 4) = 1 << rp1->pins[pinnumber]->number;

    return 0;
}

void pin_on(rp1_t *rp1, uint8_t pin)
{
    *(rp1->rio_out + RP1_ATOM_SET_OFFSET / 4) = 1 << pin;
}
void pin_off(rp1_t *rp1, uint8_t pin)
{
    *(rp1->rio_out + RP1_ATOM_CLR_OFFSET / 4) = 1 << pin;
}


const uint8_t pins[] = {17, 27, 22, 23};

void setup_spi_pins(rp1_t *rp1){

    create_pin_2(8, rp1, 0x00);     // CS0
    create_pin_2(9, rp1, 0x00);     // MISO
    create_pin_2(10, rp1, 0x00);    // MOSI
    create_pin_2(11, rp1, 0x00);    // SCLK

}

uint8_t get_bit(uint8_t array[], short bit) {
    return (array[bit / 8] >> (bit % 8)) & 1;
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
    setup_spi_pins(rp1);

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
        0x00, 0x00, 0x00, 0x00, 0x00 // reset
    };
    int data_length = sizeof(data);

    // we gotta transform
    // ON is 800ns high, 450ns low
    // OFF is 400ns high, 850ns low
    // since we transmit 50ns at a time (20MHz)
    // we need to transmit
    // ON 16 (800ns/50) bits high, 9 (450ns/50) bits low
    // OFF 8 (400ns/50) bits high, 17 (850ns/50) bits low
    // which is 25 bits of transmission per bit transmitted
    // we are bit banging, essentially
    // but 25 bits will fucking suck to re-implement here
    // so we'll try 24 bits (3 bytes per state) first
    // which will be:
    // ON 16 (800ns/50) bits high, 8 (400ns/50) bits low
    // OFF 8 (400ns/50) bits high, 16 (800ns/50) bits low
    //
    // OK, scratch that
    // according to https://wp.josh.com/2014/05/13/ws2812-neopixels-are-not-so-finicky-once-you-get-to-know-them/
    // we can do
    // ON 14 (700ns/50) bits high, 12 (600ns/50) bits low == 26 bits total
    // OFF 7 (350ns/50) bits high, 12 (600ns/50) bits low == 19 bits total
    // but that kinda doesn't add up to 24 bits, so it will kinda suck to push
    // so instead we will do
    // ON 13 (650ns/50) bits high, 11 (550ns/50) bits low == 24 bits / 3 bytes total
    // 11111111 11111000 00000000
    // 0xff     0xf8     0x00
    // OFF 7 (350ns/50) bits high, 17 (850ns/50) bits low == 24 bits / 3 bytes total
    // 11111110 00000000 00000000
    // 0xfe     0x00     0x00
    // RES 120 bits low (6000ns/50)                       == 15 bytes
    //
    // grand success
    uint8_t on[3]  = { 0xff, 0xf8, 0x00 };
    uint8_t off[3] = { 0xfe, 0x00, 0x00 };

    // every bit is transmitted as 24 bits
    // so we need to create a data array with length * 24
    int transformed_data_length = data_length * 24;
    uint8_t data_transformed[transformed_data_length];

    // printf("0x%02x", value); to print value as 0xab

    // for each bit
    for (int byteId = 0; byteId < data_length; ++byteId) {
        for (int bitId = 0; bitId < 8; ++bitId) {
            int totalBitId = byteId * 8 + bitId;
            // get bit value
            uint8_t bit = get_bit(data, totalBitId);
            // assign appropriate value to transformed data
            // TODO: optimize and just send on/off state
            if (bit == 1) {
                data_transformed[(totalBitId * 3) + 0] = on[0];
                data_transformed[(totalBitId * 3) + 1] = on[1];
                data_transformed[(totalBitId * 3) + 2] = on[2];
            } else {
                data_transformed[(totalBitId * 3) + 0] = off[0];
                data_transformed[(totalBitId * 3) + 1] = off[1];
                data_transformed[(totalBitId * 3) + 2] = off[2];
            }
        }
    }
    printf("\n");

    // send all transformed data at once
    spi_status_t res = rp1_spi_write_array_blocking(spi, data_transformed, transformed_data_length);

    // 🤞
    if(res != SPI_OK) {
        printf("error sending\n");
        return 6;
    }

    printf("command sent\n");

    return 0;
}
