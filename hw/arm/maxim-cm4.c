#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "hw/arm/boot.h"
#include "hw/arm/maxim-cm4.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-clock.h"
#include "hw/misc/unimp.h"
#include "sysemu/sysemu.h"
#include "chardev/char-fe.h"
#include "hw/loader.h"

#define __PROGRAM_START
#define __CMSIS_GENERIC

#include "max78000.h"
#include "icc_reva_regs.h"
#include "i2c_reva_regs.h"
#include "uart_revb_regs.h"

/* Peripherals */

// uart

static int max78000_uart_can_receive(void *opaque)
{
    MaximCM4State *mstate = opaque;
    if(mstate->uart_pending) {
        return 0;
    } else {
        return 1;
    }
}

static void max78000_uart_receive(void *opaque, const uint8_t *buf, int size)
{
    MaximCM4State *mstate = opaque;
    if(mstate->uart_pending) {
        fprintf(stderr, "uart_receive called when pending\n");
        exit(-1);
    } else if(size != 1) {
        fprintf(stderr, "uart_receive called with too big size\n");
    } else {
        mstate->uart_byte = *buf;
        if(mstate->uart_byte == '\n') {
            mstate->uart_byte = '\r';
        }

        mstate->uart_pending = true;
    }
}

#define UART_OFFS(name) \
    offsetof(mxc_uart_revb_regs_t, name)

#define NOP_UART_FIELD(name) \
    case UART_OFFS(name):

static MemTxResult max78000_uart_read(void *opaque, hwaddr addr,
                                      uint64_t *data, unsigned size,
                                      MemTxAttrs attrs)
{
    MaximCM4State *mstate = opaque;

    switch(addr) {
        default:
            return MEMTX_ERROR;

        case UART_OFFS(status):
            *data = (mstate->uart_pending ? 1 : 0) << MXC_F_UART_REVB_STATUS_RX_LVL_POS;
            return MEMTX_OK;

        case UART_OFFS(fifo):
            if(mstate->uart_pending) {
                qemu_chr_fe_accept_input(mstate->chr->be);
                *((uint8_t *) data) = mstate->uart_byte;
                mstate->uart_byte = 0;
                mstate->uart_pending = false;
            } else {
                *((uint8_t *) data) = 0;
            }
            return MEMTX_OK;

        NOP_UART_FIELD(ctrl)
        NOP_UART_FIELD(int_en)
        NOP_UART_FIELD(int_fl)
        NOP_UART_FIELD(clkdiv)
        NOP_UART_FIELD(osr)
        NOP_UART_FIELD(txpeek)
        NOP_UART_FIELD(pnr)
        NOP_UART_FIELD(dma)
        NOP_UART_FIELD(wken)
        NOP_UART_FIELD(wkfl)
            *data = 0;
            return MEMTX_OK;
    }
}

static MemTxResult max78000_uart_write(void *opaque, hwaddr addr,
                                       uint64_t value, unsigned size,
                                       MemTxAttrs attrs)
{
    MaximCM4State *mstate = opaque;
    uint8_t buf;

    switch(addr) {
        default:
            return MEMTX_ERROR;

        case UART_OFFS(fifo):
            buf = (uint8_t) value & 0xFF;
            qemu_chr_fe_write(mstate->chr->be, &buf, 1);
            return MEMTX_OK;

        NOP_UART_FIELD(ctrl)
        NOP_UART_FIELD(status)
        NOP_UART_FIELD(int_en)
        NOP_UART_FIELD(int_fl)
        NOP_UART_FIELD(clkdiv)
        NOP_UART_FIELD(osr)
        NOP_UART_FIELD(txpeek)
        NOP_UART_FIELD(pnr)
        NOP_UART_FIELD(dma)
        NOP_UART_FIELD(wken)
        NOP_UART_FIELD(wkfl)
            return MEMTX_OK;
    }
}

static const MemoryRegionOps max78000_uart_ops = {
        .read_with_attrs = max78000_uart_read,
        .write_with_attrs = max78000_uart_write,
};

// icc0

#define ICC_OFFS(name) \
    offsetof(mxc_icc_reva_regs_t, name)

#define NOP_ICC_FIELD(name) \
    case ICC_OFFS(name):

static MemTxResult max78000_icc_read(void *opaque, hwaddr addr,
                                     uint64_t *data, unsigned size,
                                     MemTxAttrs attrs)
{
    switch(addr) {
        default:
            return MEMTX_ERROR;

        case ICC_OFFS(ctrl):
            *data = MXC_F_ICC_REVA_CTRL_RDY;
            return MEMTX_OK;

        NOP_ICC_FIELD(info)
        NOP_ICC_FIELD(sz)
        NOP_ICC_FIELD(invalidate)
            *data = 0;
            return MEMTX_OK;
    }
}

static MemTxResult max78000_icc_write(void *opaque, hwaddr addr,
                                      uint64_t value, unsigned size,
                                      MemTxAttrs attrs)
{
    switch(addr) {
        default:
            return MEMTX_ERROR;

        NOP_ICC_FIELD(ctrl)
        NOP_ICC_FIELD(info)
        NOP_ICC_FIELD(sz)
        NOP_ICC_FIELD(invalidate)
            return MEMTX_OK;
    }
}

static const MemoryRegionOps max78000_icc_ops = {
        .read_with_attrs = max78000_icc_read,
        .write_with_attrs = max78000_icc_write,
};

/* Main CPU setup */

static void maxim_cm4_initfn(Object *obj)
{
    MaximCM4State *s = MAXIM_CM4(obj);

    object_initialize_child(obj, "armv7m", &s->armv7m, TYPE_ARMV7M);
    s->sysclk = qdev_init_clock_in(DEVICE(s), "sysclk", NULL, NULL, 0);
    s->refclk = qdev_init_clock_in(DEVICE(s), "refclk", NULL, NULL, 0);
}

// This function basically does the same thing as create_unimplemented_device,
// but avoids adding that device to the get_sysmem main region. We need
// multiple instances of these dummy devices (one per CM4) and this wouldn't
// work if we just used the default function.

static void create_unimplemented_device_mmio(const char *name,
                                             hwaddr addr,
                                             hwaddr size,
                                             MemoryRegion *region)
{
    DeviceState *dev = qdev_new(TYPE_UNIMPLEMENTED_DEVICE);

    qdev_prop_set_string(dev, "name", name);
    qdev_prop_set_uint64(dev, "size", size);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    SysBusDevice *sysdev = SYS_BUS_DEVICE(dev);
    sysdev->mmio[0].addr = addr;
    memory_region_add_subregion(region, addr, sysdev->mmio[0].memory);
}

static void maxim_cm4_realize_mem(MaximCM4State *mstate) {
    char buf[128];
    MemoryRegion *sysmem;

    snprintf(buf, sizeof(buf), "main%i", mstate->id);
    memory_region_init(&mstate->main, OBJECT(mstate), buf, UINT32_MAX);
    sysmem = &mstate->main;

    snprintf(buf, sizeof(buf), "rom%i", mstate->id);
    memory_region_init_rom(&mstate->rom, OBJECT(mstate), buf, 512 * KiB, &error_abort);
    memory_region_add_subregion(sysmem, 0x00000000, &mstate->rom);

    snprintf(buf, sizeof(buf), "flash%i", mstate->id);
    memory_region_init_rom(&mstate->flash, OBJECT(mstate), buf, 512 * KiB, &error_abort);
    memory_region_add_subregion(sysmem, 0x10000000, &mstate->flash);

    snprintf(buf, sizeof(buf), "sram%i", mstate->id);
    memory_region_init_ram(&mstate->sram, OBJECT(mstate), buf, 128 * KiB, &error_abort);
    memory_region_add_subregion(sysmem, 0x20000000, &mstate->sram);

    snprintf(buf, sizeof(buf), "mmio%i", mstate->id);
    memory_region_init(&mstate->mmio, OBJECT(mstate), buf, UINT32_MAX);
    memory_region_add_subregion_overlap(sysmem, 0x0, &mstate->mmio, -1000);
}

static void maxim_cm4_realize_cpu(MaximCM4State *mstate, MemoryRegion *sysmem) {
    DeviceState *armv7m = DEVICE(&mstate->armv7m);
    qdev_prop_set_string(armv7m, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m4"));
    qdev_prop_set_uint32(armv7m, "num-irq", MXC_IRQ_EXT_COUNT);
    qdev_connect_clock_in(armv7m, "cpuclk", mstate->sysclk);
    qdev_connect_clock_in(armv7m, "refclk", mstate->refclk);
    object_property_set_link(OBJECT(&mstate->armv7m), "memory", OBJECT(sysmem), &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(&mstate->armv7m), &error_abort);
}

static void maxim_cm4_realize_peripherals(MaximCM4State *mstate, MemoryRegion *sysmem, MemoryRegion *mmiomem) {
    /* Named peripherals first */

    // I2C
    qemu_irq i2c_irq = qdev_get_gpio_in(DEVICE(&mstate->armv7m), I2C1_IRQn);
    DeviceState *dev = qdev_new(TYPE_MXC_I2C_INITIATOR);
    qdev_prop_set_uint64(dev, "base", MXC_BASE_I2C1);
    object_property_set_link(OBJECT(dev), "mmiomem", OBJECT(mmiomem), &error_abort);
    object_property_set_link(OBJECT(dev), "irq", OBJECT(i2c_irq), &error_abort);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_abort);
    mstate->i2c1 = MXC_I2C_INITIATOR(dev);

    // UART, either real or mocked depending on what's needed
    mstate->chr = serial_hd(mstate->id);
    if(mstate->chr) {
        mstate->uart_pending = false;
        qemu_chr_fe_init(&mstate->be, mstate->chr, &error_abort);
        qemu_chr_fe_set_handlers(&mstate->be, max78000_uart_can_receive, max78000_uart_receive, NULL, NULL, mstate, NULL, true);
        memory_region_init_io(&mstate->uart0, OBJECT(mstate), &max78000_uart_ops, mstate, "uart0", 0x800);
        memory_region_add_subregion(sysmem, MXC_BASE_UART0, &mstate->uart0);
    } else {
        create_unimplemented_device_mmio("uart0", MXC_BASE_UART0, 0x1000, mmiomem);
    }

    // Instruction cache controller needs to pretend its ready
    memory_region_init_io(&mstate->icc0, OBJECT(mstate), &max78000_icc_ops, mstate, "icc0", 0x800);
    memory_region_add_subregion(sysmem, MXC_BASE_ICC0, &mstate->icc0);

    /* Unnamed, dummy peripherals */
    create_unimplemented_device_mmio("gcr", MXC_BASE_GCR, 0x400, mmiomem);
    create_unimplemented_device_mmio("gcr", MXC_BASE_GCR, 0x400, mmiomem);
    create_unimplemented_device_mmio("lpgcr", MXC_BASE_LPGCR, 0x400, mmiomem);

    create_unimplemented_device_mmio("gpio0", MXC_BASE_GPIO0, 0x1000, mmiomem);
    create_unimplemented_device_mmio("gpio1", MXC_BASE_GPIO1, 0x1000, mmiomem);
    create_unimplemented_device_mmio("gpio2", MXC_BASE_GPIO2, 0x1000, mmiomem);

    create_unimplemented_device_mmio("i2c0", MXC_BASE_I2C0, 0x1000, mmiomem);
    create_unimplemented_device_mmio("i2c2", MXC_BASE_I2C2, 0x1000, mmiomem);

    create_unimplemented_device_mmio("simo", MXC_BASE_SIMO, 0x400, mmiomem);
    create_unimplemented_device_mmio("flc0", MXC_BASE_FLC0, 0x400, mmiomem);
}

static void maxim_cm4_realize(DeviceState *dev_soc, Error **errp)
{
    MaximCM4State *mstate = MAXIM_CM4(dev_soc);
    if(mstate->id == -1) {
        error_setg(errp, "CM4 id must be set");
        return;
    }

    clock_set_mul_div(mstate->refclk, 8, 1);
    clock_set_source(mstate->refclk, mstate->sysclk);

    /* Initialize core memory regions */
    maxim_cm4_realize_mem(mstate);

    /* Initialize CPU */
    maxim_cm4_realize_cpu(mstate, &mstate->main);

    /* Initialize peripherals */
    maxim_cm4_realize_peripherals(mstate, &mstate->main, &mstate->mmio);
}

static Property maxim_cm4_properties[] = {
        DEFINE_PROP_INT32("id", MaximCM4State, id, -1),
        DEFINE_PROP_END_OF_LIST(),
};

static void maxim_cm4_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = maxim_cm4_realize;
    device_class_set_props(dc, maxim_cm4_properties);
}

static const TypeInfo  maxim_cm4_info = {
        .name          = TYPE_MAXIM_CM4,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(MaximCM4State),
        .instance_init = maxim_cm4_initfn,
        .class_init    = maxim_cm4_class_init,
};

static void maxim_cm4_types(void)
{
    type_register_static(&maxim_cm4_info);
}

type_init(maxim_cm4_types);