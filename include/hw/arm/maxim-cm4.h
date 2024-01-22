#include "hw/arm/armv7m.h"
#include "qom/object.h"
#include "chardev/char-fe.h"
#include "hw/clock.h"

#define TYPE_MAXIM_CM4 "maxim-cm4"
OBJECT_DECLARE_SIMPLE_TYPE(MaximCM4State, MAXIM_CM4)

#include "ectf_params.h"

typedef enum {
    CM4_I2C_ADDR,
    CM4_I2C_DATA
} MaximCM4I2CState;

typedef struct {
    uint8_t addr;
    uint8_t data[64];
    uint8_t ptr;
} MaximCM4I2CRequest;

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
    MemoryRegion i2c1;
    void *i2c1_regs;

    //  initiator interface
    bool i2c_running;
    MaximCM4I2CState i2c_state;
    MaximCM4I2CRequest i2c_req;
    qemu_irq i2c_irq_comp[COMPONENT_CNT];
    MaximCM4I2CRequest *i2c_req_comp[COMPONENT_CNT];

    //  consumer interface
    qemu_irq i2c_irq;

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

