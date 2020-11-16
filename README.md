# π<sup>2</sup>c Slave -- An i<sup>2</sup>c slave library for the Raspberry Pi

It is a bit uncommon to run an I<sup>2</sup>C interface in slave mode on a SoC
running Linux. So it can be a bit hard to find a good driver or library for
when this is necessary. At [EQware](https://www.eqware.net/) we had a need for
using a Raspberry Pi 3b+ as an I<sup>2</sup>C slave mimicking a specific
peripheral.

The result was we wrote libpi2cslave. This is a simple, single purpose library
to make some I<sup>2</sup>C tasks simpler. It has two simple tasks:

* Reading and sending data over I<sup>2</sup>C as a slave device
* Setting GPIOs as high, low, or float output.

To accomplish this, it uses `mmap` on `/dev/mem` to access BCM hardware
registers directly.

## Limitations

Being a simple, single purpose library, libpi2cslave has some significant
limitations.

* Raspberry Pi 4 is not supported. Currently the library has only been tested
  on a Raspberry Pi 3b+, but prior models should be supported as well.
* libpi2cslave requires exclusive use of some regions on `/dev/mem`, so it
  cannot be used in multiple processes simultaneously.
    * This also means that it cannot used alongside of
      [libpigpio](http://abyz.me.uk/rpi/pigpio/).
* Because it needs write access to `/dev/mem`, it must have root privileges.
* Reading from the I<sup>2</sup>C master is only polled I/O.
* Writing to the I<sup>2</sup>C master is a blocking operation which continues
  until the master starts writing back.
* There is no build system yet. This library is meant to be added directly to
  the project using it.

## Working around the Raspberry Pi BSC Control Reg BRK issue

The Raspberry Pi Broadcom's BSC (Broadcom Serial Control) appears to have an
undocumented issue. The Control Registers BRK bit does not clear the TX FIFO as
the BCM ARM Peripherals doc describes. This issue is
[discussed on various forum posts](https://www.raspberrypi.org/forums/viewtopic.php?p=1061380).

Libpi2cslave implements a workaround for this issue using another undocumented
feature/bug in the BSC. Through experimentation it was found that togging the
Control Reg TX Enable bit off and on pops one byte from the TX FIFO. This TX
toggle can be repeated until the TX FIFO has been cleared. Further
characterization determined that this method seems to work because whenever the
byte the BSC has loaded to send out I<sup>2</sup>C is discarded and another one
is loaded from the FIFO.

## Usage

Here is an example which reads a two byte address over I2C and writes back
simulated data from that address

```c
#include "libpi2cslave.h"
#include <stdint.h>
#include <stdio.h>

bool callback(addr_t, uint8_t * out)
{
    // Simulate data by data[addr] = addr % 0xFF
    *out = addr & 0xFF;
    return true; // We had data
}

int main() 
{
    if (!bsc_i2c_slv_init(I2C_SLAVE_ADDR)) {
        return 1;
    }

    for (;;) {
        uint16_t addr = 0;
        uint8_t addr_buf[2] = {0};
        int addr_got = 0;
        while (addr_got != sizeof(addr_buf)) {
            addr_got += bsc_i2c_read_poll(addr_buf + addr_got, sizeof(addr_buf) - addr_got);
        }

        printf("Got addr 0x%04x\n", addr)

        int written = bsc_i2c_write(read_reg, addr);
        printf("Master read %d bytes from us\n", written)
    }

    return 0;
}
```