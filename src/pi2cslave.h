/*
 * Copyright 2020 EQware Engineering Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file pi2cslave.h
 * @brief Main include file for libPi2cSlave
 * @author Stephen Robinson
 * @version 0.1
 * @date 2020-11-13
 */
#ifndef __PI2CSLAVE_H__
#define __PI2CSLAVE_H__

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Constants and functions relating to the BCM BSC (Broadcom Serial Controller)

#define BCM_IO_BASE (0x3F000000)
#define GPIO_BASE   (BCM_IO_BASE + 0x200000)
#define GPIO_LEN    (0xF4) ///< I think this number is too big...
#define BSC_BASE    (BCM_IO_BASE + 0x214000)
#define BSC_LEN     (0x40)

#define GPIO_COUNT        (28)   ///< Total number of user accessable GPIOs
#define GPSET0            (0x1C) ///< Bits to set GPIOs 0-31
#define GPCLR0            (0x28) ///< Bits to clear GPIOs 0-31

#define GPIO_FUN_IN       (0x0) ///< Pin is an input
#define GPIO_FUN_OUT      (0x1) ///< Pin is an output
#define GPIO_FUN_ALT0     (0x4) ///< Pin takes alternate function 0
#define GPIO_FUN_ALT1     (0x5) ///< Pin takes alternate function 1
#define GPIO_FUN_ALT2     (0x6) ///< Pin takes alternate function 2
#define GPIO_FUN_ALT3     (0x7) ///< Pin takes alternate function 3
#define GPIO_FUN_ALT4     (0x3) ///< Pin takes alternate function 4
#define GPIO_FUN_ALT5     (0x2) ///< Pin takes alternate function 5
#define GPIO_FUN_MASK     (0x7)
#define GPIO_FUN_SHIFT    (3)
#define GPIO_FUN_PER_REG  (10)  ///< Each FUN is 3 bits and a reg is 32. 10 fit.

#define GPIO_SDA    (18) ///< GPIO pin number of BSC I2C slave SDA
#define GPIO_SCL    (19) ///< GPIO pin number of BSC I2C slave SCL

#define BSC_DR      (0)  ///< Data register
#define BSC_RSR     (1)  ///< The operation status register and error clear register
#define BSC_SLV     (2)  ///< The I2C SPI Address Register holds the I2C slave address value
#define BSC_CR      (3)  ///< The Control register is used to configure the I2C or SPI operation
#define BSC_FR      (4)  ///< Flag register
#define BSC_IFLS    (5)  ///< Interrupt fifo level select register
#define BSC_IMSC    (6)  ///< Interrupt Mask Set Clear Register
#define BSC_RIS     (7)  ///< Raw Interrupt Status Register
#define BSC_MIS     (8)  ///< Masked Interrupt Status Register
#define BSC_ICR     (9)  ///< Interrupt Clear Register
#define BSC_DMACR   (10) ///< DMA Control Register
#define BSC_TDR     (11) ///< FIFO Test Data
#define BSC_GPUSTAT (12) ///< GPU Status Register
#define BSC_HCTRL   (13) ///< Host Control Register
#define BSC_DEBUG1  (14) ///< I2C Debug Register
#define BSC_DEBUG2  (15) ///< SPI Debug Register

#define CR_INV_TXF    (1<<13) ///< INV-RX Inverse TX status flags
#define CR_HOSTCTRLEN (1<<12) ///< HOSTCTRLEN Enable Control for Host
#define CR_TESTFIFO   (1<<11) ///< TESTFIFO TEST FIFO
#define CR_INV_RXF    (1<<10) ///< INV-RX Inverse RX status flags
#define CR_RXE        (1<<9)  ///< RXE Receive Enable
#define CR_TXE        (1<<8)  ///< TXE Transmit Enable
#define CR_BRK        (1<<7)  ///< BRK Break current operation
#define CR_ENCTRL     (1<<6)  ///< ENCTRL ENABLE CONTROL 8-bit register
#define CR_ENSTAT     (1<<5)  ///< ENSTAT ENABLE STATUS 8-bit register
#define CR_CPOL       (1<<4)  ///< CPOL Clock Polarity
#define CR_CPHA       (1<<3)  ///< CPHA Clock Phase
#define CR_I2C        (1<<2)  ///< SPI Mode
#define CR_SPI        (1<<1)  ///< SPI Mode
#define CR_EN         (1<<0)  ///< EN Enable Device

#define FR_RXFLEVEL_OFF (11) ///< RXFLEVEL RX FIFO Level
#define FR_TXFLEVEL_OFF (6)  ///< TXFLEVEL TX FIFO Level
#define FR_RXFLEVEL     (0x1F<<FR_RXFLEVEL_OFF)
#define FR_TXFLEVEL     (0x1F<<FR_TXFLEVEL_OFF)
#define FR_RXBUSY       (1<<5)     ///< RXBUSY Receive Busy
#define FR_TXFE         (1<<4)     ///< TXFE TX FIFO Empty
#define FR_RXFF         (1<<3)     ///< RXFE RX FIFO Full
#define FR_TXFF         (1<<2)     ///< TXFF TX FIFO Full
#define FR_RXFE         (1<<1)     ///< RXFE RX FIFO Empty
#define FR_TXBUSY       (1<<0)     ///< TXBUSY Transmit Busy

#define RSR_UE          (1<<1)     ///< TXUE TX Underrun Error
#define RSR_OE          (1<<0)     ///< RXOE RX Overrun Error

#define FIFO_LEN        (16) ///< Experimentally verified. Missing from BCM2537 ARM Peripherals spec.

/**
 * @brief Specify the output state for the GPIOs
 */
enum gpio_state {
    GPIO_STATE_FLOAT,
    GPIO_STATE_LOW,
    GPIO_STATE_HIGH,
};

typedef uint16_t addr_t;

/**
 * @brief Callback for the bsc_i2c_write function
 */
typedef bool (*tx_callback)(addr_t addr, uint8_t * out);

/**
 * @brief Initialize /dev/mem to access hardware registers from userspace.
 * @warning This function must be called before any other function in this module.
 *
 * @return false on error, true otherwise
 */
bool init_bcm_reg_mem();
/**
 * @brief Close the /dev/mem interface.
 * @warning This must be called after shutdown_bsc_i2c_slv()
 */
void shutdown_bcm_reg_mem();

/**
 * @brief Initialize the BCM BSC I2C slave device
 *
 * @warning This must be called after init_bcm_reg_mem();
 * @param i2c_addr Address for BCM I2C slave
 *
 * @return false on error, true otherwise
 */
bool init_bsc_i2c_slv(uint8_t i2c_addr);
/**
 * @brief Deinitialize the BSC I2C slave device
 */
void shutdown_bsc_i2c_slv();

/**
 * @brief Set the output state of a GPIO
 *
 * @param gpio Number of GPIO to set
 * @param state One of gpio_state
 *
 * @return false on error, true otherwise
 */
bool bcm_set_gpio_out(int gpio, enum gpio_state state);

/**
 * @brief Check if master is currently sending us data
 *
 * @return True as long as the master is sending us data, else false
 */
bool bsc_i2c_receiving();

/**
 * @brief Read up to len bytes into buffer. Does not block.
 *
 * If there is no data to read from the master, return immediately.
 *
 * @note This function is a pthread cancellation point
 *
 * @param buf Buffer to read bytes into
 * @param len Length of bufer
 *
 * @return Number of bytes read
 */
int bsc_i2c_read_poll(uint8_t * buf, size_t len);

/**
 * @brief Send bytes to master from tx_callback, incrementing addr each time.
 *
 * Continues trying to send data until the master sends us data. When the
 * master starts sending us data, again, unsent data is cleared from the TX
 * FIFO.
 *
 * @note Having the callback called, does not mean that the data has been sent
 *       to the master. It only means that it has been queued. The return value
 *       must be checked to see how many bytes have been sent.
 *
 * @note This function is a pthread cancellation point
 *
 * @param cb The callback to retrieve data to send. If this returns false, no
 *           more data is sent (but the function does not return).
 * @param addr This is passed to the callback to indicate the byte to send
 *
 * @return Number of bytes sent.
 */
int bsc_i2c_write(tx_callback cb, uint16_t addr);

#endif // ! __PI2CSLAVE_H__
