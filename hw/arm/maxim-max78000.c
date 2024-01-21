#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-clock.h"
#include "qemu/error-report.h"
#include "hw/arm/maxim-cm4.h"
#include "hw/arm/boot.h"
#include "elf.h"
#include "hw/loader.h"
#include "sysemu/reset.h"

/* Main SYSCLK frequency in Hz (24MHz) */
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

static void maxim_max78000_init(MachineState *machine) {
    DeviceState *dev;
    Clock *sysclk;

    /* This clock doesn't need migration because it is fixed-frequency */
    sysclk = clock_new(OBJECT(machine), "SYSCLK");
    clock_set_hz(sysclk, SYSCLK_FRQ);

    dev = qdev_new(TYPE_MAXIM_CM4);
    qdev_connect_clock_in(dev, "sysclk", sysclk);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    /* Load firmware */
    struct elf32_hdr ehdr;
    bool is_elf64;
    struct entrypoint *entrypoint;

    load_elf_hdr(machine->kernel_filename, &ehdr, &is_elf64, &error_abort);
    armv7m_load_kernel(ARM_CPU(first_cpu), machine->kernel_filename, 0, 0);

    entrypoint = g_malloc(sizeof(struct entrypoint));
    entrypoint->cpu = first_cpu;
    entrypoint->addr = (vaddr) ehdr.e_entry;

    qemu_register_reset(maxim_max78000_set_entrypoint, (void *) entrypoint);
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
}

DEFINE_MACHINE("maxim-max78000", maxim_max78000_machine_init)