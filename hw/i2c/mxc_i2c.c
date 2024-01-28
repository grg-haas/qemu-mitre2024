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

#define SET_I2C_FIELD(ctrl, name) \
    case I2C_OFFS(name): \
        (ctrl)->regs.name = value; break;

#define GET_I2C_FIELD(ctrl, name) \
    case I2C_OFFS(name): \
        return (ctrl)->regs.name; break;

static MXCI2CTargetState *mxc_i2c_target_get_target(MXCI2CInitiatorState *tctrl) {
    int i = 0;
    I2CNode *node;

    // Target controllers should only have one target in their lists
    QLIST_FOREACH(node, &tctrl->targets, next) { i++; }
    assert(i == 1);
    assert(node);
    return MXC_I2C_TARGET(node->elt);
}

static void mxc_i2c_target_refresh_interrupt(MXCI2CInitiatorState *ctrl, MXCI2CTargetState *target) {
    // Interrupt should be high if there is a pending flag for an enabled line
    if(ctrl->regs.intfl0 & ctrl->regs.inten0) {
        if(!target->interrupt) {
            target->interrupt = true;
        }
        qemu_irq_raise(ctrl->irq);
    } else {
        if(target->interrupt) {
            target->interrupt = false;
        }
        qemu_irq_lower(ctrl->irq);
    }
}

/* Read functions */

static uint64_t i2c_read_initiator(MXCI2CInitiatorState *tctrl, hwaddr addr)
{
    uint64_t res;
    switch (addr) {
        default:
            return MEMTX_ERROR;

        GET_I2C_FIELD(tctrl, intfl0)
        GET_I2C_FIELD(tctrl, mstctrl)

        case I2C_OFFS(ctrl):
            res = tctrl->regs.ctrl;
            if(tctrl->regs.ctrl & MXC_F_I2C_REVA_CTRL_SDA_OUT) {
                res |= MXC_F_I2C_REVA_CTRL_SDA;
            }

            if(tctrl->regs.ctrl & MXC_F_I2C_REVA_CTRL_SCL_OUT) {
                res |= MXC_F_I2C_REVA_CTRL_SCL;
            }

            return res;

        case I2C_OFFS(fifo):
           return i2c_recv(tctrl->bus);

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

static uint64_t i2c_read_target(MXCI2CInitiatorState *tctrl, hwaddr addr) {
    int i;
    uint64_t res, val;
    MXCI2CTargetState *target = mxc_i2c_target_get_target(tctrl);

    switch (addr) {
        default:
            return 0;

        GET_I2C_FIELD(tctrl, inten0)
        GET_I2C_FIELD(tctrl, status)
        GET_I2C_FIELD(tctrl, slave)

        case I2C_OFFS(intfl0):
            if(target->interrupt) {
                // Prioritize interrupts
                res = 0;
                val = (tctrl->regs.intfl0 & tctrl->regs.inten0);

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
                return tctrl->regs.intfl0;
            }

        case I2C_OFFS(ctrl):
            res = tctrl->regs.ctrl;
            if(tctrl->regs.ctrl & MXC_F_I2C_REVA_CTRL_SDA_OUT) {
                res |= MXC_F_I2C_REVA_CTRL_SDA;
            }

            if(tctrl->regs.ctrl & MXC_F_I2C_REVA_CTRL_SCL_OUT) {
                res |= MXC_F_I2C_REVA_CTRL_SCL;
            }

            return res;

        case I2C_OFFS(fifo):
            res = 0;
            qemu_mutex_lock(&target->lock);
            if(target->ptr > 0) {
                res = target->fifo[0];
                for(i = 0; i < target->ptr - 1; i++) {
                    target->fifo[i] = target->fifo[i + 1];
                }

                target->ptr--;
                if(target->ptr == 0) {
                    tctrl->regs.status |= MXC_F_I2C_REVA_STATUS_RX_EM;
                }
            }
            qemu_mutex_unlock(&target->lock);
            return res;

        case I2C_OFFS(rxctrl1):
            res = 0;
            qemu_mutex_lock(&target->lock);
            res = target->ptr;
            qemu_mutex_unlock(&target->lock);
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

/* Write functions */

static void i2c_write_initiator(MXCI2CInitiatorState *ictrl, hwaddr addr, uint64_t value)
{
    int i;
    uint64_t diff;
    MXCI2CTargetState *target = mxc_i2c_target_get_target(ictrl);

    switch(addr) {
        default:
            break;

        SET_I2C_FIELD(ictrl, ctrl)
        SET_I2C_FIELD(ictrl, rxctrl1)

        case I2C_OFFS(fifo):
            switch(target->state) {
                case CM4_I2C_DATA:
                    if(target->writing) {
                        // Send now
                        i2c_send(ictrl->bus, value);
                    } else {
                        // Stash
                        if(target->ptr < sizeof(target->fifo)) {
                            target->fifo[target->ptr] = value;
                            target->ptr++;

                            if(target->ptr == sizeof(target->fifo)) {
                                // Flag that we are out of space
                                ictrl->regs.intfl0 |= MXC_F_I2C_REVA_INTFL0_TX_THD;
                                mxc_i2c_target_refresh_interrupt(ictrl, target);
                            }
                        }
                    }

                    break;

                case CM4_I2C_ADDR:
                    // Stash the address
                    target->addr = value >> 1;
                    if(value & 1) {
                        target->reading = true;

                        // Start the transfer
                        if(i2c_start_recv(ictrl->bus, target->addr)) {
                            // Invalid address
                            ictrl->regs.intfl0 |= MXC_F_I2C_REVA_INTFL0_ADDR_NACK_ERR;
                            ictrl->regs.mstctrl &= ~MXC_F_I2C_REVA_MSTCTRL_START;
                            mxc_i2c_target_refresh_interrupt(ictrl, target);
                        }
                    } else {
                        target->writing = true;
                    }

                    target->ptr = 0;
                    target->state = CM4_I2C_DATA;
                    break;

                default:
                    break;
            }
            break;

        case I2C_OFFS(mstctrl):
            diff = (value & ~ictrl->regs.mstctrl);
            assert(__builtin_popcount(diff) == 1);
            ictrl->regs.mstctrl = value;
            if(diff & MXC_F_I2C_REVA_MSTCTRL_START) {
                if(target->writing) {
                    // Start the transfer
                    if(i2c_start_send(ictrl->bus, target->addr)) {
                        // Invalid address
                        ictrl->regs.intfl0 |= MXC_F_I2C_REVA_INTFL0_ADDR_NACK_ERR;
                        ictrl->regs.mstctrl &= ~MXC_F_I2C_REVA_MSTCTRL_START;
                        mxc_i2c_target_refresh_interrupt(ictrl, target);
                    } else {
                        for(i = 0; i < target->ptr; i++) {
                            i2c_send(ictrl->bus, target->fifo[i]);
                        }

                        target->ptr = 0;
                    }
                }
            }
            else if(diff & MXC_F_I2C_REVA_MSTCTRL_RESTART) {
                target->start_pending = true;
                if(target->writing) {
                    target->writing = false;
                    target->state = CM4_I2C_ADDR;
                    target->addr = 0;
                    target->ptr = 0;

                    i2c_end_transfer(ictrl->bus);

                    // Target sets MXC_F_I2C_REVA_INTFL0_DONE when stop condition acknowledged
                    ictrl->regs.intfl0 |= MXC_F_I2C_REVA_INTFL0_STOP;
                    ictrl->regs.mstctrl &= ~MXC_F_I2C_REVA_MSTCTRL_STOP;
                    mxc_i2c_target_refresh_interrupt(ictrl, target);
                }
            }
            else if(diff & MXC_F_I2C_REVA_MSTCTRL_STOP) {
                if(target->writing) {
                    target->writing = false;
                }

                if(target->reading) {
                    target->reading = false;
                }

                target->state = CM4_I2C_ADDR;
                target->addr = 0;
                target->ptr = 0;

                i2c_end_transfer(ictrl->bus);

                // Target sets MXC_F_I2C_REVA_INTFL0_DONE when stop condition acknowledged
                ictrl->regs.intfl0 |= MXC_F_I2C_REVA_INTFL0_STOP;
                ictrl->regs.mstctrl &= ~MXC_F_I2C_REVA_MSTCTRL_STOP;
                mxc_i2c_target_refresh_interrupt(ictrl, target);
            }

            break;

        case I2C_OFFS(intfl0):
            ictrl->regs.intfl0 &= ~value;
            if(!target->writing && target->ptr < sizeof(target->fifo)) {
                ictrl->regs.intfl0 |= MXC_F_I2C_REVA_INTFL0_TX_THD;
            }

            mxc_i2c_target_refresh_interrupt(ictrl, target);
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

static void i2c_write_target(MXCI2CInitiatorState *tctrl, hwaddr addr, uint64_t value)
{
    MXCI2CTargetState *target = mxc_i2c_target_get_target(tctrl);
    switch(addr) {
        default:
            break;

        SET_I2C_FIELD(tctrl, ctrl)

        case I2C_OFFS(slave):
            i2c_slave_set_address(I2C_SLAVE(target), value);
            break;

        case I2C_OFFS(inten0):
            tctrl->regs.inten0 = value;
            mxc_i2c_target_refresh_interrupt(tctrl, target);
            break;

        case I2C_OFFS(intfl0):
            qemu_mutex_lock(&target->lock);
            if(value & MXC_F_I2C_REVA_INTFL0_STOP) {
                // Signal to the initiator that the transfer is complete
                if(target->initiator) {
                    target->initiator->regs.mstctrl &= ~MXC_F_I2C_REVA_MSTCTRL_START;

                    if(target->start_pending) {
                        target->start_pending = false;
                        target->initiator->regs.mstctrl &= ~MXC_F_I2C_REVA_MSTCTRL_RESTART;
                    } else {
                        target->initiator->regs.intfl0 |= MXC_F_I2C_REVA_INTFL0_DONE;
                    }
                }
            }

            tctrl->regs.intfl0 &= ~value;
            mxc_i2c_target_refresh_interrupt(tctrl, target);
            qemu_mutex_unlock(&target->lock);
            break;

        case I2C_OFFS(fifo):
            qemu_mutex_lock(&target->lock);
            if(target->initiator->regs.rxctrl1 > 0) {
                // We shouldn't have gotten here if there is not a byte to push
                assert(target->ptr < sizeof(target->fifo));

                target->fifo[target->ptr] = value;
                target->ptr++;

                if(target->ptr == sizeof(target->fifo)) {
                    tctrl->regs.status |= MXC_F_I2C_REVA_STATUS_TX_FULL;
                }

                // Notify controller
                target->initiator->regs.rxctrl1--;
                target->initiator->regs.intfl0 |= MXC_F_I2C_REVA_INTFL0_RX_THD;
            }

            qemu_mutex_unlock(&target->lock);
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
    MXCI2CInitiatorState *tctrl = mts->target;

    qemu_mutex_lock(&mts->lock);
    if(mts->ptr < sizeof(mts->fifo)) {
        mts->fifo[mts->ptr] = data;
        mts->ptr++;

        tctrl->regs.intfl0 |= MXC_F_I2C_REVA_INTFL0_RX_THD;
        tctrl->regs.status &= ~MXC_F_I2C_REVA_STATUS_RX_EM;
        mxc_i2c_target_refresh_interrupt(tctrl, MXC_I2C_TARGET(target));
    }
    qemu_mutex_unlock(&mts->lock);

    return 0;
}

static uint8_t mxc_i2c_target_recv(I2CSlave *target)
{
    int i;
    uint8_t res = 0;
    MXCI2CTargetState *mts = MXC_I2C_TARGET(target);
    MXCI2CInitiatorState *tctrl = mts->target;
    MXCI2CInitiatorState *ictrl = mts->initiator;

    qemu_mutex_lock(&mts->lock);
    if(mts->ptr > 0) {
        res = mts->fifo[0];
        for(i = 0; i < mts->ptr - 1; i++) {
            mts->fifo[i] = mts->fifo[i + 1];
        }

        mts->ptr--;
        if(mts->ptr == 0) {
            tctrl->regs.status |= MXC_F_I2C_REVA_STATUS_TX_EM;
            ictrl->regs.status |= MXC_F_I2C_REVA_STATUS_RX_EM;
        }

        tctrl->regs.status &= ~MXC_F_I2C_REVA_STATUS_TX_FULL;
    }

    qemu_mutex_unlock(&mts->lock);

    return res;
}

static int mxc_i2c_target_event(I2CSlave *target, enum i2c_event event)
{
    int ret = 0;
    MXCI2CTargetState *mts = MXC_I2C_TARGET(target);
    MXCI2CInitiatorState *tctrl = mts->target;

    qemu_mutex_lock(&mts->lock);
    switch(event) {
        case I2C_START_RECV:
            mts->ptr = 0;
            tctrl->regs.intfl0 |= (MXC_F_I2C_REVA_INTFL0_WR_ADDR_MATCH | MXC_F_I2C_REVA_INTFL0_TX_LOCKOUT);
            mxc_i2c_target_refresh_interrupt(tctrl, MXC_I2C_TARGET(target));
            break;
        case I2C_START_SEND:
            mts->ptr = 0;
            tctrl->regs.intfl0 |= MXC_F_I2C_REVA_INTFL0_RD_ADDR_MATCH;
            mxc_i2c_target_refresh_interrupt(tctrl, MXC_I2C_TARGET(target));
            break;
        case I2C_FINISH:
            tctrl->regs.intfl0 |= MXC_F_I2C_REVA_INTFL0_STOP;
            mxc_i2c_target_refresh_interrupt(tctrl, MXC_I2C_TARGET(target));
            break;

        case I2C_START_SEND_ASYNC:
        case I2C_NACK:
            ret = 1;
    }
    qemu_mutex_unlock(&mts->lock);

    return ret;
}

static void mxc_i2c_target_realize(DeviceState *dev, Error **errp)
{
    MXCI2CTargetState *mts = MXC_I2C_TARGET(dev);

    qemu_mutex_init(&mts->lock);
    mts->interrupt = false;
}

static Property mxc_i2c_target_properties[] = {
        DEFINE_PROP_LINK("target", MXCI2CTargetState, target, TYPE_MXC_I2C_INITIATOR, MXCI2CInitiatorState *),
        DEFINE_PROP_LINK("initiator", MXCI2CTargetState, initiator, TYPE_MXC_I2C_INITIATOR, MXCI2CInitiatorState *),
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
    QLIST_INIT(&s->targets);
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