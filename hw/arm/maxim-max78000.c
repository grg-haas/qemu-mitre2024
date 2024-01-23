#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-clock.h"
#include "hw/arm/maxim-cm4.h"
#include "hw/arm/boot.h"
#include "elf.h"
#include "hw/loader.h"
#include "sysemu/reset.h"
#include "hw/i2c/i2c.h"

/* Board definition */
#define SYSCLK_FRQ 24000000ULL

struct entrypoint {
    CPUState *cpu;
    vaddr addr;
};

static void maxim_max78000_set_entrypoint(void *opaque) {
    struct entrypoint *entrypoint = opaque;
    cpu_set_pc(entrypoint->cpu, entrypoint->addr);
    g_free(entrypoint);
}

static MaximCM4State *maxim_max78000_init_cpu(Clock *sysclk, int id) {
    DeviceState *dev = qdev_new(TYPE_MAXIM_CM4);
    MaximCM4State *mstate = MAXIM_CM4(dev);

    qdev_connect_clock_in(dev, "sysclk", sysclk);
    qdev_prop_set_int32(dev, "id", id);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_abort);

    return mstate;
}

static void maxim_max78000_load_firmware(ARMCPU *cpu, char *firmware) {
    struct elf32_hdr ehdr;
    bool is_elf64;
    struct entrypoint *entrypoint;

    if(!firmware) {
        return;
    }

    load_elf_hdr(firmware, &ehdr, &is_elf64, &error_abort);
    armv7m_load_kernel(cpu, firmware, 0, 0);

    // armv7m_load_kernel registers a reset of the CPU, which is necessary
    // in order for the system to come up directly. Unfortunately, this would
    // also trash the entrypoint if we were to set it here with cpu_set_pc.
    // Therefore, we need to register our own reset function to run after
    // theirs, so that the CPUs start up at the correct address.

    entrypoint = g_malloc(sizeof(struct entrypoint));
    entrypoint->cpu = CPU(cpu);
    entrypoint->addr = (vaddr) ehdr.e_entry;
    qemu_register_reset(maxim_max78000_set_entrypoint, (void *) entrypoint);
}

static void maxim_max78000_init(MachineState *machine) {
    int i;
    CPUState *cpu;
    MaximCM4State *dev, *ap;
    I2CSlave *target;

    uint32_t component_ids[COMPONENT_CNT] = {COMPONENT_IDS};

    /* Initialize processors */
    Clock *sysclk = clock_new(OBJECT(machine), "SYSCLK");
    clock_set_hz(sysclk, SYSCLK_FRQ);

    for(i = 0; i < 1 + COMPONENT_CNT; i++) {
        dev = maxim_max78000_init_cpu(sysclk, i);

        // Also connect i2c buses
        if(i == 0) {
            ap = dev;
        } else {
            target = i2c_slave_new(TYPE_MXC_I2C_TARGET, component_ids[i - 1] & 0xFF);
            object_property_set_link(OBJECT(target), "target", OBJECT(dev->i2c1), &error_abort);
            qdev_realize_and_unref(DEVICE(target), BUS(ap->i2c1->bus), &error_abort);

            // Hacky...
            dev->i2c1->initiator = ap->i2c1;
        }
    }

    /* Load firmware */
    i = 0;
    CPU_FOREACH(cpu) {
        if(i == 0) {
            // AP uses the "kernel" firmware
            maxim_max78000_load_firmware(ARM_CPU(cpu), machine->kernel_filename);
        } else {
            // Components use the "bios" firmware
            maxim_max78000_load_firmware(ARM_CPU(cpu), machine->firmware);
        }
        i++;
    }
}

static void maxim_max78000_machine_init(MachineClass *mc) {
    static const char *const valid_cpu_types[] = {
            ARM_CPU_TYPE_NAME("cortex-m4"),
            NULL
    };

    mc->desc = "Maxim MAX78000 Board";
    mc->alias = "max78000";
    mc->init = maxim_max78000_init;
    mc->valid_cpu_types = valid_cpu_types;
    mc->max_cpus = 3;
    mc->min_cpus = 3;
}

DEFINE_MACHINE("maxim-max78000", maxim_max78000_machine_init)