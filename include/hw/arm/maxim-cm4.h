#include "hw/arm/armv7m.h"
#include "qom/object.h"
#include "chardev/char-fe.h"
#include "hw/clock.h"
#include "hw/i2c/mxc_i2c.h"
#include "hw/i2c/i2c.h"

#define TYPE_MAXIM_CM4 "maxim-cm4"
OBJECT_DECLARE_SIMPLE_TYPE(MaximCM4State, MAXIM_CM4)

#include "ectf_params.h"

struct MaximCM4State {
    /* private */
    SysBusDevice parent_obj;

    /* public */

    // config
    int id;

    // memory regions
    MemoryRegion main;
    MemoryRegion rom;
    MemoryRegion flash;
    MemoryRegion sram;
    MemoryRegion mmio;

    // peripherals
    MemoryRegion icc0;
    MemoryRegion gpio2;

    // i2c
    MXCI2CInitiatorState *i2c1;

    // uart
    MemoryRegion uart0;
    CharBackend be;
    Chardev *chr;
    uint8_t uart_byte, uart_pending;

    // cpu
    ARMv7MState armv7m;
    Clock *sysclk;
    Clock *refclk;
};

