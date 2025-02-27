
#ifndef MXC_I2C_H
#define MXC_I2C_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define __PROGRAM_START
#define __CMSIS_GENERIC
#include "i2c_reva_regs.h"

#define TYPE_MXC_I2C_INITIATOR  "mxc-i2c-initiator"
#define TYPE_MXC_I2C_TARGET     "mxc-i2c-target"

OBJECT_DECLARE_SIMPLE_TYPE(MXCI2CInitiatorState, MXC_I2C_INITIATOR)
OBJECT_DECLARE_SIMPLE_TYPE(MXCI2CTargetState, MXC_I2C_TARGET)

typedef enum {
    CM4_I2C_ADDR,
    CM4_I2C_DATA
} MaximCM4I2CState;

struct MXCI2CInitiatorState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MXCI2CInitiatorState *initiator;
    MXCI2CTargetState *target;

    QemuMutex lock;

    // memory/system info
    uint64_t base;
    MemoryRegion *mmiomem, iomem;
    qemu_irq irq;
    I2CBus *bus;


    // state machine
    bool writing, reading;
    bool start_pending, stop_pending;
    bool interrupt;
    MaximCM4I2CState state;
    uint8_t addr;
    uint8_t fifo[64], ptr;

    // register state
    mxc_i2c_reva_regs_t regs;
};

struct MXCI2CTargetState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MXCI2CInitiatorState *target;
};

#endif //MXC_I2C_H
