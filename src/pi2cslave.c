/*
 * Copyright 2020 EQware Engineering Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#define _GNU_SOURCE

#include <pthread.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>

#include "bcm_low_level.h"

static int mem_fd = -1;
static volatile uint32_t * bsc = NULL;
static volatile uint32_t * gpio_reg = NULL;

#define WRITE_USLEEP_INTERVAL (25)
#define GET_FR_RXFLEVEL()     ((bsc[BSC_FR] & FR_RXFLEVEL) >> FR_RXFLEVEL_OFF)
#define GET_FR_TXFLEVEL()     ((bsc[BSC_FR] & FR_TXFLEVEL) >> FR_TXFLEVEL_OFF)
#define RX_EMPTY()            (bsc[BSC_FR] & FR_RXFE)
#define RX_BUSY()             (bsc[BSC_FR] & FR_RXBUSY)
#define TX_BUSY()             (bsc[BSC_FR] & FR_RXBUSY)

#define TAG                   "pi2cslave"

static void * do_mmap(size_t len, off_t base)
{
    return mmap(0, len, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_LOCKED, mem_fd, base);
}

bool init_bcm_reg_mem()
{
    if ((mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        perror(TAG ": Unable to open /dev/mem");
        return false;
    }

    bsc = do_mmap(BSC_LEN, BSC_BASE);
    if (bsc == NULL) {
        perror(TAG ": Unable to mmap BSC memory");
        close(mem_fd);
        return false;
    }

    gpio_reg = do_mmap(GPIO_LEN, GPIO_BASE);
    if (bsc == NULL) {
        perror(TAG ": Unable to mmap GPIO memory");
        munmap((void*)bsc, BSC_LEN);
        close(mem_fd);
        return false;
    }

    return true;
}

void shutdown_bcm_reg_mem()
{
    if (bsc != NULL) {
        munmap((void *)bsc, BSC_LEN);
        bsc = NULL;
    }
    if (gpio_reg != NULL) {
        munmap((void *)gpio_reg, GPIO_LEN);
        gpio_reg = NULL;
    }
    if (mem_fd >= 0) {
        close(mem_fd);
        mem_fd = -1;
    }
}

static void gpio_set_mode(uint32_t gpio, uint32_t mode)
{
    int reg = gpio / GPIO_FUN_PER_REG;
    int shift = (gpio % GPIO_FUN_PER_REG) * GPIO_FUN_SHIFT;

    gpio_reg[reg] &= ~(GPIO_FUN_MASK << shift);
    gpio_reg[reg] |= mode << shift;
}

bool init_bsc_i2c_slv(uint8_t i2c_addr)
{
    if (!bsc) {
        fprintf(stderr, TAG ": init_bcm_reg_mem() has not been called\n");
        return false;
    }

    // Alternative function 3 is for BSC
    gpio_set_mode(GPIO_SDA, GPIO_FUN_ALT3);
    gpio_set_mode(GPIO_SCL, GPIO_FUN_ALT3);

    bsc[BSC_CR] = CR_BRK; // First reset everything
    bsc[BSC_RSR] = 0;
    bsc[BSC_IMSC] = 0xf;
    bsc[BSC_ICR] = 0xf;

    // Shift addr right one to get 7 bit addr without RW bit.
    bsc[BSC_SLV] = (i2c_addr>>1);
    bsc[BSC_CR] = CR_TXE | CR_RXE | CR_I2C | CR_EN;

    return true;
}

bool bcm_set_gpio_out(int gpio, enum gpio_state state)
{
    if (!gpio_reg) {
        fprintf(stderr, TAG ": init_bcm_reg_mem() has not been called\n");
        return false;
    }
    if (gpio < 0 || gpio >= GPIO_COUNT) {
        fprintf(stderr, TAG ": Invalid GPIO: %d\n", gpio);
        return false;
    }
    switch(state) {
        case GPIO_STATE_FLOAT:
            // Using GPIO input as a way to get the gpio to float.
            // We have to do this because the Raspberry Pi lacks an open drain
            // mode, which is what we would really want for this pin.
            gpio_set_mode(gpio, GPIO_FUN_IN);
            return true;
        case GPIO_STATE_LOW:
            gpio_reg[GPCLR0] |= ((uint32_t)1) << gpio;
            gpio_set_mode(gpio, GPIO_FUN_OUT);
            return true;
        case GPIO_STATE_HIGH:
            gpio_reg[GPSET0] |= ((uint32_t)1) << gpio;
            gpio_set_mode(gpio, GPIO_FUN_OUT);
            return true;
        default:
            fprintf(stderr, TAG ": Invalid GPIO state: %d\n", state);
            return false;
    }
}

void shutdown_bsc_i2c_slv()
{
    bsc[BSC_CR] = 0;
}

bool bsc_i2c_receiving(){
    return bsc[BSC_FR] & FR_RXBUSY;
}

int bsc_i2c_read_poll(uint8_t * buf, size_t len)
{
    pthread_testcancel();
    if (len == 0 || buf == NULL) {
        return 0;
    }

    size_t read = 0;

    // Loop as long as:
    // 1. We have room to receive data.
    // 2. The RX fifo has data.
    // 3. This thread has not been canceled
    for (; len && !RX_EMPTY(); len--) {
        pthread_testcancel();
        if (bsc[BSC_RSR] & RSR_OE) {
            // We overflowed. :-(
            fprintf(stderr, TAG ": Overflow!\n");
            bsc[BSC_RSR] &= ~RSR_OE; // Clear the overflow error
        }
        buf[read] = bsc[BSC_DR] & 0xFF;
        read++;
    }

    return read;
}

int bsc_i2c_write(tx_callback cb, uint16_t addr)
{
    pthread_testcancel();
    int offset = 0;

    // Keep replying as long as the master is not writing to us.
    while (RX_EMPTY()) {
        pthread_testcancel();
        // Keep the TX FIFO full
        while ( !(bsc[BSC_FR] & FR_TXFF)) {
            pthread_testcancel();
            // Check for underflows
            if (bsc[BSC_RSR] & RSR_UE) {
                // We had an underrun happen. :-(
                fprintf(stderr, TAG ": Underrun!\n");
                bsc[BSC_RSR] &= ~RSR_UE; // Clear the underrun error
            }
            uint8_t byte;
            if (cb(addr++, &byte)) {
                bsc[BSC_DR] = byte;
                offset++;
            } else {
                // We have used up all the data this callback has.
                break;
            }
        }
        usleep(WRITE_USLEEP_INTERVAL);
    }

    // Return value is how many bytes we put in the TX FIFO minus the number of
    // bytes that are left in it minus an additional byte which got sucked off
    // the FIFO, ready to be sent, but never was sent.
    int ret = offset - GET_FR_TXFLEVEL() - 1;

    // We need to get the TX FIFO clear, otherwise the next time the master
    // does a read, it will get the unread leftovers from this read.
    //
    // The method here is a bit hacky, but it is the only one I could find. I
    // was unable to get CR_BRK to work at all. It simply would not clear the
    // fifo. As far as I could see, it does nothing. I found numerous places
    // online where people reported the same issue.
    //
    // The method is simple, whenever TX is disabled and re-enabled, it drops
    // the current TX byte and pops the next one out of the FIFO, so we can
    // put this in a while loop until the TX FIFO is empty (TXFE) and then
    // do it one more time to drop the last TX byte.
    //
    // Note: this behavior is undocumented as far as I can tell. The BCM2837
    // ARM Peripherals specification document doesn't mention it. However,
    // that spec is generally known to contain errors and omissions.
    while (!(bsc[BSC_FR] & FR_TXFE)) {
        bsc[BSC_CR] &= ~CR_TXE;
        bsc[BSC_CR] |= CR_TXE;
    }
    bsc[BSC_CR] &= ~CR_TXE;
    bsc[BSC_CR] |= CR_TXE;

    // When this software is first getting running, I have seen some
    // instability. This is just a sanity check.
    return (ret < 0) ? 0 : ret;
}
