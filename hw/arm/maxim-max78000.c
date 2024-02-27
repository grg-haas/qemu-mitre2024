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

/* Entrypoint handling */

// See comment in maxim_max78000_load_firmware

struct entrypoint {
    CPUState *cpu;
    vaddr addr;
};

static void maxim_max78000_set_entrypoint(void *opaque) {
    struct entrypoint *entrypoint = opaque;
    cpu_set_pc(entrypoint->cpu, entrypoint->addr);
}

/* Board definition */

static MaximCM4State *maxim_max78000_init_cpu(Clock *sysclk, int id) {
    DeviceState *dev = qdev_new(TYPE_MAXIM_CM4);
    MaximCM4State *mstate = MAXIM_CM4(dev);

    qdev_connect_clock_in(dev, "sysclk", sysclk);
    qdev_prop_set_int32(dev, "id", id);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_abort);

    return mstate;
}

static void maxim_max78000_load_firmware(ARMCPU *cpu, const char *firmware) {
    struct elf32_hdr ehdr;
    bool is_elf64;
    struct entrypoint *entrypoint;

    if(!firmware) {
        return;
    }

    // Get the header first, so we can set the entrypoint later
    load_elf_hdr(firmware, &ehdr, &is_elf64, &error_abort);

    // Load the firmware
    armv7m_load_kernel(cpu, firmware, 0, 0);

    // armv7m_load_kernel registers a reset of the CPU, which is necessary
    // in order for the system to come up correctly. Unfortunately, this would
    // also trash the entrypoint if we were to set it here with cpu_set_pc.
    // Therefore, we need to register our own reset function to run after
    // theirs, so that the CPUs start up at the correct address.

    entrypoint = g_malloc(sizeof(struct entrypoint));
    entrypoint->cpu = CPU(cpu);
    entrypoint->addr = (vaddr) ehdr.e_entry;
    qemu_register_reset(maxim_max78000_set_entrypoint, (void *) entrypoint);
}

#define FW_STRLEN   256
static char ap_firmware[FW_STRLEN] = {0},
                comp1_firmware[FW_STRLEN] = {0},
                comp2_firmware[FW_STRLEN] = {0};

#define SYSCLK_FRQ 24000000ULL

static void maxim_max78000_init(MachineState *machine) {
    int i;
    const char *firmware = NULL;
    CPUState *cpu;
    MaximCM4State *dev, *ap;
    I2CSlave *target;

    /* Initialize processors */
    Clock *sysclk = clock_new(OBJECT(machine), "SYSCLK");
    clock_set_hz(sysclk, SYSCLK_FRQ);

    for(i = 0; i < 1 + COMPONENT_CNT; i++) {
        dev = maxim_max78000_init_cpu(sysclk, i);

        // Also connect i2c buses
        if(i == 0) {
            ap = dev;
        } else {
            target = i2c_slave_new(TYPE_MXC_I2C_TARGET, -1);
            object_property_set_link(OBJECT(target), "target", OBJECT(dev->i2c1), &error_abort);
            qdev_realize_and_unref(DEVICE(target), BUS(ap->i2c1->bus), &error_abort);

            // Hacky...
            dev->i2c1->initiator = ap->i2c1;
            dev->i2c1->target = MXC_I2C_TARGET(target);
        }
    }

    /* Load firmware */
    i = 0;
    CPU_FOREACH(cpu) {
        firmware = NULL;
        switch(i) {
            case 0:
                firmware = ap_firmware;
                break;

            case 1:
                firmware = comp1_firmware;
                break;

            case 2:
                firmware = comp2_firmware;
                break;

            default:
                fprintf(stderr, "Too many CPUs");
                exit(-1);
        }

        if(!firmware[0]) {
            fprintf(stderr, "No firmware for CPU %i\n", i);
            exit(-1);
        }

        maxim_max78000_load_firmware(ARM_CPU(cpu), firmware);
        i++;
    }
}


static char *get_ap_firmware(Object *obj, Error **errp) {
    return (char *) ap_firmware;
}

static void set_ap_firmware(Object *obj, const char *value, Error **errp) {
    strncpy(ap_firmware, value, FW_STRLEN);
}

static char *get_comp1_firmware(Object *obj, Error **errp) {
    return (char *) comp1_firmware;
}

static void set_comp1_firmware(Object *obj, const char *value, Error **errp) {
    strncpy(comp1_firmware, value, FW_STRLEN);
}

static char *get_comp2_firmware(Object *obj, Error **errp) {
    return (char *) comp2_firmware;
}

static void set_comp2_firmware(Object *obj, const char *value, Error **errp) {
    strncpy(comp2_firmware, value, FW_STRLEN);
}

static void maxim_max78000_machine_init(MachineClass *mc) {
    ObjectClass *oc = OBJECT_CLASS(mc);

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

    // Allow for specifying the separate board firmwares on the command line
    object_class_property_add_str(oc, "ap", get_ap_firmware, set_ap_firmware);
    object_class_property_add_str(oc, "comp1", get_comp1_firmware, set_comp1_firmware);
    object_class_property_add_str(oc, "comp2", get_comp2_firmware, set_comp2_firmware);

}

DEFINE_MACHINE("maxim-max78000", maxim_max78000_machine_init)