#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "hw/arm/boot.h"
#include "exec/address-spaces.h"
#include "hw/arm/maxim-cm4.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-clock.h"
#include "hw/misc/unimp.h"
#include "sysemu/sysemu.h"
#include "chardev/char-fe.h"
#include "hw/loader.h"
#include "elf.h"

#define __PROGRAM_START
#define __CMSIS_GENERIC

#include "max78000.h"
#include "icc_reva_regs.h"
#include "uart_regs.h"

/* Peripherals */

// uart
static int max78000_uart_can_receive(void *opaque)
{
    MaximCM4State *mstate = opaque;
    if(mstate->pending) {
        return 0;
    } else {
        return 1;
    }
}

static void max78000_uart_receive(void *opaque, const uint8_t *buf, int size)
{
    MaximCM4State *mstate = opaque;
    if(mstate->pending) {
        fprintf(stderr, "uart_receive called when pending\n");
        exit(-1);
    } else if(size != 1) {
        fprintf(stderr, "uart_receive called with too big size\n");
    } else {
        mstate->byte = *buf;
        if(mstate->byte == '\n') {
            mstate->byte = '\r';
        }

        mstate->pending = true;
    }
}

static MemTxResult max78000_uart_read(void *opaque, hwaddr addr,
                                      uint64_t *data, unsigned size,
                                      MemTxAttrs attrs)
{
    MaximCM4State *mstate = opaque;

    switch(addr) {
        case offsetof(mxc_uart_regs_t, status):
            *data = (mstate->pending ? 1 : 0) << MXC_F_UART_STATUS_RX_LVL_POS;
            break;

        case offsetof(mxc_uart_regs_t, fifo):
            if(mstate->pending) {
                qemu_chr_fe_accept_input(mstate->chr->be);
                *((uint8_t *) data) = mstate->byte;
                mstate->byte = 0;
                mstate->pending = false;
            } else {
                *((uint8_t *) data) = 0;
            }
            break;

        default:
            break;
    }

    return MEMTX_OK;
}

static MemTxResult max78000_uart_write(void *opaque, hwaddr addr,
                                       uint64_t value, unsigned size,
                                       MemTxAttrs attrs)
{
    MaximCM4State *mstate = opaque;

    uint8_t buf;
    if(addr == offsetof(mxc_uart_regs_t, fifo)) {
        buf = (uint8_t) value & 0xFF;
        qemu_chr_fe_write(mstate->chr->be, &buf, 1);
    }
    return MEMTX_OK;
}

static const MemoryRegionOps max78000_uart_ops = {
        .read_with_attrs = max78000_uart_read,
        .write_with_attrs = max78000_uart_write,
};

// icc0

static MemTxResult max78000_icc_read(void *opaque, hwaddr addr,
                                     uint64_t *data, unsigned size,
                                     MemTxAttrs attrs)
{
    // Pretend icc is always ready
    if(addr == offsetof(mxc_icc_reva_regs_t, ctrl)) {
        *data = MXC_F_ICC_REVA_CTRL_RDY;
    }

    return MEMTX_OK;
}

static MemTxResult max78000_icc_write(void *opaque, hwaddr addr,
                                      uint64_t value, unsigned size,
                                      MemTxAttrs attrs)
{
    return MEMTX_OK;
}

static const MemoryRegionOps max78000_icc_ops = {
        .read_with_attrs = max78000_icc_read,
        .write_with_attrs = max78000_icc_write,
};


static void maxim_cm4_initfn(Object *obj)
{
    MaximCM4State *s = MAXIM_CM4(obj);

    object_initialize_child(obj, "armv7m", &s->armv7m, TYPE_ARMV7M);
    s->sysclk = qdev_init_clock_in(DEVICE(s), "sysclk", NULL, NULL, 0);
    s->refclk = qdev_init_clock_in(DEVICE(s), "refclk", NULL, NULL, 0);
}

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

static void maxim_cm4_realize(DeviceState *dev_soc, Error **errp)
{
    MaximCM4State *mstate = MAXIM_CM4(dev_soc);
    MemoryRegion *sysmem = get_system_memory(), *c;

    clock_set_mul_div(mstate->refclk, 8, 1);
    clock_set_source(mstate->refclk, mstate->sysclk);

    /* Initialize core memory regions */
    memory_region_init_rom(&mstate->rom, OBJECT(dev_soc), "rom", 512 * KiB, &error_abort);
    memory_region_add_subregion(sysmem, 0x00000000, &mstate->rom);

    memory_region_init_rom(&mstate->flash, OBJECT(dev_soc), "flash", 512 * KiB, &error_abort);
    memory_region_add_subregion(sysmem, 0x10000000, &mstate->flash);

    memory_region_init_ram(&mstate->sram, OBJECT(dev_soc), "sram", 128 * KiB, &error_abort);
    memory_region_add_subregion(sysmem, 0x20000000, &mstate->sram);

    memory_region_init(&mstate->mmio, OBJECT(dev_soc), "mmio", UINT32_MAX);
    memory_region_add_subregion_overlap(sysmem, 0x0, &mstate->mmio, -1000);

    /* Initialize CPU */
    DeviceState *armv7m = DEVICE(&mstate->armv7m);
    qdev_prop_set_string(armv7m, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m4"));
    qdev_connect_clock_in(armv7m, "cpuclk", mstate->sysclk);
    qdev_connect_clock_in(armv7m, "refclk", mstate->refclk);
    object_property_set_link(OBJECT(&mstate->armv7m), "memory", OBJECT(sysmem), &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(&mstate->armv7m), &error_abort);

    /* Initialize peripherals */
    memory_region_init_io(&mstate->icc0, OBJECT(dev_soc), &max78000_icc_ops, mstate, "icc0", 0x800);
    memory_region_add_subregion(sysmem, MXC_BASE_ICC0, &mstate->icc0);

    mstate->chr = serial_hd(0);
    mstate->pending = false;
    qemu_chr_fe_init(&mstate->be, mstate->chr, &error_abort);
    qemu_chr_fe_set_handlers(&mstate->be, max78000_uart_can_receive, max78000_uart_receive, NULL, NULL, mstate, NULL, true);
    memory_region_init_io(&mstate->uart0, OBJECT(dev_soc), &max78000_uart_ops, mstate, "uart0", 0x800);
    memory_region_add_subregion(sysmem, MXC_BASE_UART0, &mstate->uart0);

    c = &mstate->mmio;
    create_unimplemented_device_mmio("gcr", MXC_BASE_GCR, 0x400, c);

    create_unimplemented_device_mmio("gcr", MXC_BASE_GCR, 0x400, c);
    create_unimplemented_device_mmio("lpgcr", MXC_BASE_LPGCR, 0x400, c);

    create_unimplemented_device_mmio("gpio0", MXC_BASE_GPIO0, 0x1000, c);
    create_unimplemented_device_mmio("gpio1", MXC_BASE_GPIO1, 0x1000, c);
    create_unimplemented_device_mmio("gpio2", MXC_BASE_GPIO2, 0x1000, c);
    create_unimplemented_device_mmio("uart0", MXC_BASE_UART0, 0x3000, c);
    create_unimplemented_device_mmio("i2c0", MXC_BASE_I2C0, 0x1000, c);
    create_unimplemented_device_mmio("i2c1", MXC_BASE_I2C1, 0x1000, c);
    create_unimplemented_device_mmio("i2c2", MXC_BASE_I2C2, 0x1000, c);

    create_unimplemented_device_mmio("simo", MXC_BASE_SIMO, 0x400, c);
    create_unimplemented_device_mmio("flc0", MXC_BASE_FLC0, 0x400, c);

//    /* Load firmware */
//    struct elf32_hdr ehdr;
//    bool is_elf64;
//    struct entrypoint *entrypoint;
//
//    load_elf_hdr("/home/grg/Projects/2024-ectf-insecure-example/application_processor/build/max78000.elf", &ehdr, &is_elf64, &error_abort);
//    armv7m_load_kernel(mstate->armv7m.cpu, "/home/grg/Projects/2024-ectf-insecure-example/application_processor/build/max78000.elf", 0, 0);

//    entrypoint = g_malloc(sizeof(struct entrypoint));
//    entrypoint->cpu = CPU(mstate->armv7m.cpu);
//    entrypoint->addr = (vaddr) ehdr.e_entry;
//
//    qemu_register_reset(maxim_max78000_set_entrypoint, (void *) entrypoint);
}


static void maxim_cm4_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = maxim_cm4_realize;
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