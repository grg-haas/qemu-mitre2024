#include "hw/arm/armv7m.h"
#include "qom/object.h"
#include "chardev/char-fe.h"
#include "hw/clock.h"

#define TYPE_MAXIM_CM4 "maxim-cm4"
OBJECT_DECLARE_SIMPLE_TYPE(MaximCM4State, MAXIM_CM4)

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
    MemoryRegion i2c1;
    MemoryRegion uart0;

    void *i2c1_regs;

    // uart
    CharBackend be;
    Chardev *chr;
    uint8_t byte, pending;

    // cpu
    ARMv7MState armv7m;
    Clock *sysclk;
    Clock *refclk;
};

