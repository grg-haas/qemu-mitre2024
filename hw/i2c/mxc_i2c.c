#include "qemu/osdep.h"
#include "hw/i2c/mxc_i2c.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "hw/i2c/i2c.h"
#include "qemu/log.h"
#include "trace.h"
#include "qemu/module.h"
#include "hw/qdev-properties.h"

/* Utilities */

#define I2C_OFFS(name) \
    offsetof(mxc_i2c_reva_regs_t, name)

#define NOP_I2C_FIELD(name) \
    case I2C_OFFS(name):

#define SET_I2C_FIELD(name) \
    case I2C_OFFS(name): \
        s->regs.name = value; break;

#define GET_I2C_FIELD(name) \
    case I2C_OFFS(name): \
        return s->regs.name; break;

static void mxc_i2c_target_refresh_interrupt(MXCI2CInitiatorState *controller) {
    // Interrupt should be high if there is a pending flag for an enabled line
    if(controller->regs.intfl0 & controller->regs.inten0) {
        if(!controller->interrupt) {
            controller->interrupt = true;
        }
        qemu_irq_raise(controller->irq);
    } else {
        if(controller->interrupt) {
            controller->interrupt = false;
        }
        qemu_irq_lower(controller->irq);
    }
}

static uint64_t i2c_read_initiator(MXCI2CInitiatorState *s, hwaddr addr)
{
    uint64_t res;
    switch (addr) {
        default:
            return MEMTX_ERROR;

        GET_I2C_FIELD(intfl0)
        GET_I2C_FIELD(mstctrl)

        case I2C_OFFS(ctrl):
            res = s->regs.ctrl;
            if(s->regs.ctrl & MXC_F_I2C_REVA_CTRL_SDA_OUT) {
                res |= MXC_F_I2C_REVA_CTRL_SDA;
            }

            if(s->regs.ctrl & MXC_F_I2C_REVA_CTRL_SCL_OUT) {
                res |= MXC_F_I2C_REVA_CTRL_SCL;
            }

            return res;

        case I2C_OFFS(fifo):
           return i2c_recv(s->bus);

        NOP_I2C_FIELD(status)
        NOP_I2C_FIELD(inten0)
        NOP_I2C_FIELD(intfl1)
        NOP_I2C_FIELD(inten1)
        NOP_I2C_FIELD(fifolen)
        NOP_I2C_FIELD(rxctrl0)
        NOP_I2C_FIELD(rxctrl1)
        NOP_I2C_FIELD(txctrl0)
        NOP_I2C_FIELD(txctrl1)
        NOP_I2C_FIELD(clklo)
        NOP_I2C_FIELD(clkhi)
        NOP_I2C_FIELD(hsclk)
        NOP_I2C_FIELD(timeout)
        NOP_I2C_FIELD(dma)
        NOP_I2C_FIELD(slave)
            return 0;
    }
}

static uint64_t i2c_read_target(MXCI2CInitiatorState *s, hwaddr addr) {
    int i;
    uint64_t res, val;
    switch (addr) {
        default:
            return MEMTX_ERROR;

        GET_I2C_FIELD(inten0)
        GET_I2C_FIELD(status)

        case I2C_OFFS(intfl0):
            if(s->interrupt) {
                // Prioritize interrupts
                res = 0;
                val = (s->regs.intfl0 & s->regs.inten0);

                if(val & MXC_F_I2C_REVA_INTFL0_RD_ADDR_MATCH) {
                    res |= MXC_F_I2C_REVA_INTFL0_RD_ADDR_MATCH;
                } else if(val & (MXC_F_I2C_REVA_INTFL0_WR_ADDR_MATCH | MXC_F_I2C_REVA_INTFL0_TX_LOCKOUT)) {
                    res |= (MXC_F_I2C_REVA_INTFL0_WR_ADDR_MATCH | MXC_F_I2C_REVA_INTFL0_TX_LOCKOUT);
                } else if(val & MXC_F_I2C_REVA_INTFL0_RX_THD) {
                    res |= MXC_F_I2C_REVA_INTFL0_RX_THD;
                } else if(val & MXC_F_I2C_REVA_INTFL0_STOP) {
                    res |= MXC_F_I2C_REVA_INTFL0_STOP;
                }

                return res;
            } else {
                return s->regs.intfl0;
            }

        case I2C_OFFS(ctrl):
            res = s->regs.ctrl;
            if(s->regs.ctrl & MXC_F_I2C_REVA_CTRL_SDA_OUT) {
                res |= MXC_F_I2C_REVA_CTRL_SDA;
            }

            if(s->regs.ctrl & MXC_F_I2C_REVA_CTRL_SCL_OUT) {
                res |= MXC_F_I2C_REVA_CTRL_SCL;
            }

            return res;

        case I2C_OFFS(fifo):
            res = 0;
            qemu_mutex_lock(&s->lock);
            if(s->ptr > 0) {
                res = s->fifo[0];
                for(i = 0; i < s->ptr - 1; i++) {
                    s->fifo[i] = s->fifo[i + 1];
                }

                s->ptr--;
                if(s->ptr == 0) {
                    s->regs.status |= MXC_F_I2C_REVA_STATUS_RX_EM;
                }
            }
            qemu_mutex_unlock(&s->lock);
            return res;

        case I2C_OFFS(rxctrl1):
            res = 0;
            qemu_mutex_lock(&s->lock);
            res = s->ptr;
            qemu_mutex_unlock(&s->lock);
            return res << MXC_F_I2C_REVA_RXCTRL1_LVL_POS;

        NOP_I2C_FIELD(intfl1)
        NOP_I2C_FIELD(inten1)
        NOP_I2C_FIELD(fifolen)
        NOP_I2C_FIELD(rxctrl0)
        NOP_I2C_FIELD(txctrl0)
        NOP_I2C_FIELD(txctrl1)
        NOP_I2C_FIELD(mstctrl)
        NOP_I2C_FIELD(clklo)
        NOP_I2C_FIELD(clkhi)
        NOP_I2C_FIELD(hsclk)
        NOP_I2C_FIELD(timeout)
        NOP_I2C_FIELD(dma)
        NOP_I2C_FIELD(slave)
            return 0;
    }}

static uint64_t mxc_i2c_read(void *opaque, hwaddr addr, unsigned size)
{
    MXCI2CInitiatorState *s = opaque;

    if(s->regs.ctrl & MXC_F_I2C_REVA_CTRL_MST_MODE) {
        return i2c_read_initiator(s, addr);
    } else {
        return i2c_read_target(s, addr);
    }
}

static void i2c_write_initiator(MXCI2CInitiatorState *s, hwaddr addr, uint64_t value)
{
    int i;
    uint64_t diff;

    switch(addr) {
        default:
            break;

        SET_I2C_FIELD(ctrl)
        SET_I2C_FIELD(rxctrl1)

        case I2C_OFFS(fifo):
            switch(s->state) {
                case CM4_I2C_DATA:
                    if(s->writing) {
                        // Send now
                        i2c_send(s->bus, value);
                    } else {
                        // Stash
                        if(s->ptr < sizeof(s->fifo)) {
                            s->fifo[s->ptr] = value;
                            s->ptr++;
                        } else {
                            // Flag that we are out of space
                            s->regs.intfl0 |= MXC_F_I2C_REVA_INTFL0_TX_THD;
                            mxc_i2c_target_refresh_interrupt(s);
                        }
                    }

                    break;

                case CM4_I2C_ADDR:
                    // Stash the address
                    s->addr = value >> 1;
                    if(value & 1) {
                        s->reading = true;

                        // Start the transfer
                        if(i2c_start_recv(s->bus, s->addr)) {
                            // Invalid address
                            s->regs.intfl0 |= MXC_F_I2C_REVA_INTFL0_ADDR_NACK_ERR;
                            s->regs.mstctrl &= ~MXC_F_I2C_REVA_MSTCTRL_START;
                            mxc_i2c_target_refresh_interrupt(s);
                        }
                    } else {
                        s->writing = true;
                    }

                    s->ptr = 0;
                    s->state = CM4_I2C_DATA;
                    break;

                default:
                    break;
            }
            break;

        case I2C_OFFS(mstctrl):
            diff = (value & ~s->regs.mstctrl);
            assert(__builtin_popcount(diff) == 1);
            s->regs.mstctrl = value;
            if(diff & MXC_F_I2C_REVA_MSTCTRL_START) {
                if(s->writing) {
                    // Start the transfer
                    if(i2c_start_send(s->bus, s->addr)) {
                        // Invalid address
                        s->regs.intfl0 |= MXC_F_I2C_REVA_INTFL0_ADDR_NACK_ERR;
                        s->regs.mstctrl &= ~MXC_F_I2C_REVA_MSTCTRL_START;
                        mxc_i2c_target_refresh_interrupt(s);
                    } else {
                        for(i = 0; i < s->ptr; i++) {
                            i2c_send(s->bus, s->fifo[i]);
                        }

                        s->ptr = 0;
                    }
                } else if(s->reading) {

                }
            }
            else if(diff & MXC_F_I2C_REVA_MSTCTRL_RESTART) {
                s->start_pending = true;
                if(s->writing) {
                    s->writing = false;
                    s->state = CM4_I2C_ADDR;
                    s->addr = 0;
                    s->ptr = 0;

                    i2c_end_transfer(s->bus);

                    // Target sets MXC_F_I2C_REVA_INTFL0_DONE when stop condition acknowledged
                    s->regs.intfl0 |= MXC_F_I2C_REVA_INTFL0_STOP;
                    s->regs.mstctrl &= ~MXC_F_I2C_REVA_MSTCTRL_STOP;
                    mxc_i2c_target_refresh_interrupt(s);
                }
            }
            else if(diff & MXC_F_I2C_REVA_MSTCTRL_STOP) {
                if(s->writing) {
                    s->writing = false;
                }

                if(s->reading) {
                    s->reading = false;
                }

                s->state = CM4_I2C_ADDR;
                s->addr = 0;
                s->ptr = 0;

                i2c_end_transfer(s->bus);

                // Target sets MXC_F_I2C_REVA_INTFL0_DONE when stop condition acknowledged
                s->regs.intfl0 |= MXC_F_I2C_REVA_INTFL0_STOP;
                s->regs.mstctrl &= ~MXC_F_I2C_REVA_MSTCTRL_STOP;
                mxc_i2c_target_refresh_interrupt(s);
            }

            break;

        case I2C_OFFS(intfl0):
            s->regs.intfl0 &= ~value;
            if(!s->writing && s->ptr < sizeof(s->fifo)) {
                s->regs.intfl0 |= MXC_F_I2C_REVA_INTFL0_TX_THD;
            }

            mxc_i2c_target_refresh_interrupt(s);
            break;

        NOP_I2C_FIELD(status)
        NOP_I2C_FIELD(inten0)
        NOP_I2C_FIELD(intfl1)
        NOP_I2C_FIELD(inten1)
        NOP_I2C_FIELD(fifolen)
        NOP_I2C_FIELD(rxctrl0)
        NOP_I2C_FIELD(txctrl0)
        NOP_I2C_FIELD(txctrl1)
        NOP_I2C_FIELD(clklo)
        NOP_I2C_FIELD(clkhi)
        NOP_I2C_FIELD(hsclk)
        NOP_I2C_FIELD(timeout)
        NOP_I2C_FIELD(dma)
        NOP_I2C_FIELD(slave)
            break;
    }
}

static void i2c_write_target(MXCI2CInitiatorState *s, hwaddr addr, uint64_t value)
{
    switch(addr) {
        default:
            break;

        SET_I2C_FIELD(ctrl)
        SET_I2C_FIELD(slave)

        case I2C_OFFS(inten0):
            s->regs.inten0 = value;

            if(!(value & MXC_F_I2C_REVA_INTFL0_RD_ADDR_MATCH)) {
                printf("bruh\n");
            }

            mxc_i2c_target_refresh_interrupt(s);
            break;

        case I2C_OFFS(intfl0):
            qemu_mutex_lock(&s->lock);
            if(value & MXC_F_I2C_REVA_INTFL0_STOP) {
                // Signal to the initiator that the transfer is complete
                if(s->initiator) {
                    s->initiator->regs.mstctrl &= ~MXC_F_I2C_REVA_MSTCTRL_START;

                    if(s->initiator->start_pending) {
                        s->initiator->start_pending = false;
                        s->initiator->regs.mstctrl &= ~MXC_F_I2C_REVA_MSTCTRL_RESTART;
                    } else {
                        s->initiator->regs.intfl0 |= MXC_F_I2C_REVA_INTFL0_DONE;
                    }
                }
            }

            s->regs.intfl0 &= ~value;
            mxc_i2c_target_refresh_interrupt(s);
            qemu_mutex_unlock(&s->lock);
            break;

        case I2C_OFFS(fifo):
            qemu_mutex_lock(&s->lock);
            if(s->initiator->regs.rxctrl1 > 0) {
                if(s->ptr < sizeof(s->fifo)) {
                    s->fifo[s->ptr] = value;
                    s->ptr++;

                    if(s->ptr == sizeof(s->fifo)) {
                        s->regs.status |= MXC_F_I2C_REVA_STATUS_TX_FULL;
                    }

                    // Notify controller
                    s->initiator->regs.rxctrl1--;
                    s->initiator->regs.intfl0 |= MXC_F_I2C_REVA_INTFL0_RX_THD;
                } else {
                    // Flag that we are out of space
                    // todo unset this
                    exit(-1);
                }
            }

            qemu_mutex_unlock(&s->lock);
            break;

        NOP_I2C_FIELD(status)
        NOP_I2C_FIELD(intfl1)
        NOP_I2C_FIELD(inten1)
        NOP_I2C_FIELD(fifolen)
        NOP_I2C_FIELD(rxctrl0)
        NOP_I2C_FIELD(rxctrl1)
        NOP_I2C_FIELD(txctrl0)
        NOP_I2C_FIELD(txctrl1)
        NOP_I2C_FIELD(mstctrl)
        NOP_I2C_FIELD(clklo)
        NOP_I2C_FIELD(clkhi)
        NOP_I2C_FIELD(hsclk)
        NOP_I2C_FIELD(timeout)
        NOP_I2C_FIELD(dma)
            break;
    }
}

static void mxc_i2c_write(void *opaque, hwaddr addr, uint64_t data, unsigned size)
{
    MXCI2CInitiatorState *s = opaque;

    if(s->regs.ctrl & MXC_F_I2C_REVA_CTRL_MST_MODE) {
        i2c_write_initiator(s, addr, data);
    } else {
        i2c_write_target(s, addr, data);
    }
}

static const MemoryRegionOps mxc_i2c_ops = {
        .read = mxc_i2c_read,
        .write = mxc_i2c_write,
};

/* Target interface */

static int mxc_i2c_target_send(I2CSlave *target, uint8_t data)
{
    MXCI2CTargetState *mts = MXC_I2C_TARGET(target);
    MXCI2CInitiatorState *controller = mts->target;

    qemu_mutex_lock(&controller->lock);
    if(controller->ptr < sizeof(controller->fifo)) {
        controller->fifo[controller->ptr] = data;
        controller->ptr++;

        controller->regs.intfl0 |= MXC_F_I2C_REVA_INTFL0_RX_THD;
        controller->regs.status &= ~MXC_F_I2C_REVA_STATUS_RX_EM;
        mxc_i2c_target_refresh_interrupt(controller);
    }
    qemu_mutex_unlock(&controller->lock);

    return 0;
}

static uint8_t mxc_i2c_target_recv(I2CSlave *target)
{
    int i;
    uint8_t res = 0;
    MXCI2CTargetState *mts = MXC_I2C_TARGET(target);
    MXCI2CInitiatorState *controller = mts->target;

    qemu_mutex_lock(&controller->lock);
    if(controller->ptr > 0) {
        res = controller->fifo[0];
        for(i = 0; i < controller->ptr - 1; i++) {
            controller->fifo[i] = controller->fifo[i + 1];
        }

        controller->ptr--;
        if(controller->ptr == 0) {
            controller->regs.status |= MXC_F_I2C_REVA_STATUS_TX_EM;
            controller->initiator->regs.status |= MXC_F_I2C_REVA_STATUS_RX_EM;
        }

        // todo unset this
        controller->regs.status &= ~MXC_F_I2C_REVA_STATUS_TX_FULL;
    }

    qemu_mutex_unlock(&controller->lock);

    return res;
}

static int mxc_i2c_target_event(I2CSlave *target, enum i2c_event event)
{
    MXCI2CTargetState *mts = MXC_I2C_TARGET(target);
    MXCI2CInitiatorState *controller = mts->target;

    qemu_mutex_lock(&controller->lock);
    switch(event) {
        case I2C_START_RECV:
            controller->ptr = 0;
            controller->regs.intfl0 |= (MXC_F_I2C_REVA_INTFL0_WR_ADDR_MATCH | MXC_F_I2C_REVA_INTFL0_TX_LOCKOUT);
            mxc_i2c_target_refresh_interrupt(controller);
            break;
        case I2C_START_SEND:
            controller->ptr = 0;
            controller->regs.intfl0 |= MXC_F_I2C_REVA_INTFL0_RD_ADDR_MATCH;
            mxc_i2c_target_refresh_interrupt(controller);
            break;
        case I2C_FINISH:
            controller->regs.intfl0 |= MXC_F_I2C_REVA_INTFL0_STOP;
            mxc_i2c_target_refresh_interrupt(controller);
            break;

        case I2C_START_SEND_ASYNC:
        case I2C_NACK:
            return 1;
    }
    qemu_mutex_unlock(&controller->lock);

    return 0;
}

static void mxc_i2c_target_realize(DeviceState *dev, Error **errp)
{

}

static Property mxc_i2c_target_properties[] = {
        DEFINE_PROP_LINK("target", MXCI2CTargetState, target, TYPE_MXC_I2C_INITIATOR, MXCI2CInitiatorState *),
        DEFINE_PROP_END_OF_LIST(),
};

static void mxc_i2c_target_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);

    dc->realize = mxc_i2c_target_realize;
    dc->desc = "Maxim I2C RevA Controller (Target)";

    sc->send = mxc_i2c_target_send;
    sc->recv = mxc_i2c_target_recv;
    sc->event = mxc_i2c_target_event;

    device_class_set_props(dc, mxc_i2c_target_properties);
}

static const TypeInfo mxc_i2c_target_type_info = {
        .name = TYPE_MXC_I2C_TARGET,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(MXCI2CTargetState),
        .class_init = mxc_i2c_target_class_init,
};

static void mxc_i2c_target_register_types(void) {
    type_register_static(&mxc_i2c_target_type_info);
}

type_init(mxc_i2c_target_register_types)

/* Initiator interface */

static void mxc_i2c_initiator_realize(DeviceState *dev, Error **errp)
{
    MXCI2CInitiatorState *s = MXC_I2C_INITIATOR(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &mxc_i2c_ops, s, TYPE_MXC_I2C_INITIATOR, 0x1000);
    if(s->base && s->mmiomem) {
        memory_region_add_subregion(s->mmiomem, s->base, &s->iomem);
    }

    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    s->bus = i2c_init_bus(dev, "i2c");
    qemu_mutex_init(&s->lock);

    s->interrupt = false;
}

static Property mxc_i2c_initiator_properties[] = {
        DEFINE_PROP_UINT64("base", MXCI2CInitiatorState, base, 0),
        DEFINE_PROP_LINK("mmiomem", MXCI2CInitiatorState, mmiomem, TYPE_MEMORY_REGION, MemoryRegion *),
        DEFINE_PROP_LINK("irq", MXCI2CInitiatorState, irq, TYPE_IRQ, qemu_irq),
        DEFINE_PROP_END_OF_LIST(),
};

static void mxc_i2c_initiator_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = mxc_i2c_initiator_realize;
    dc->desc = "Maxim I2C RevA Controller";

    device_class_set_props(dc, mxc_i2c_initiator_properties);
}

static const TypeInfo mxc_i2c_initiator_type_info = {
        .name = TYPE_MXC_I2C_INITIATOR,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(MXCI2CInitiatorState),
        .class_init = mxc_i2c_initiator_class_init,
};

static void mxc_i2c_initiator_register_types(void) {
    type_register_static(&mxc_i2c_initiator_type_info);
}

type_init(mxc_i2c_initiator_register_types)