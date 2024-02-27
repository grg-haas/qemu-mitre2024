// Copyright (c) 2024 Daniel Pyon

#include "qemu/osdep.h"
#include "cpu.h"
#include "patch.h"
#include "gdbstub/internals.h"
#include "qemu/log.h"
#include "sysemu/runstate.h"
#include "target/arm/patch.h"
#include "include/disas/disas.h"

#include <stdio.h>

static CPUARMState fuzz_cpu_state;

int handle_brk(CPUState *cs, CPUARMState *env) {
    int len;
    char* buf;

    uint8_t syndrome = env->exception.syndrome & 0xff;
    switch (syndrome) {
        case 0:
            // gdb case
            if (gdbserver_state.init) {
                qemu_log_mask(CPU_LOG_INT, "breaking to gdb!\n");
                vm_stop(RUN_STATE_DEBUG);
            }
            break;
        case 1: {
            fprintf(stderr, "fuzzing...\n");

            // write host buffer -> guest buffer

            // get r1 (length of buffer)
            len = env->regs[1];
            buf = (char*)g_malloc(len);

            // for (int i = 0; i < len; i++) buf[i] = random() & 0xff;
            memset(buf, 0x41, len);

            // save state for crash logs
            fuzz_cpu_state.regs[0] = env->regs[0];
            fuzz_cpu_state.regs[1] = len; // TODO: probably unnecessary

            if (cpu_memory_rw_debug(cs, env->regs[0], buf, len, 1) < 0) {
                g_free(buf);
                fprintf(stderr, "cannot access memory\n");
                return -1;
            }

            g_free(buf);
            break;
        }
        default:
            break;
    }

    return 0;
}

int handle_abort(CPUState* cs, CPUARMState* env) {
    // TODO: come up with unique file name
    FILE* dump_file = fopen("crashes/crash.txt", "w");
    if (dump_file == NULL)
        return -1;

    const char *fmt_str = "********* Data\\Instruction abort! *********\n"
                          "FAR = 0x%llx\t ELR = 0x%llx\n"
                          "Fuzz x0 = 0x%llx\t Fuzz x1 = 0x%llx\n";

    fprintf(dump_file, fmt_str, env->exception.vaddress,
                                env->pc,
                                fuzz_cpu_state.regs[0],
                                fuzz_cpu_state.regs[1]);

    fprintf(dump_file, "\n********** CPU State **********\n");
    cpu_dump_state(cs, dump_file, CPU_DUMP_CODE);
    fprintf(dump_file, "\n********** Disassembly **********\n");
    target_disas(dump_file, cs, env->pc-0x20, 0x40);
    fprintf(dump_file, "\n********** Memory Dump **********\n");

    // TODO: implement this
    // dump_extra_reg_data(cs, env, dump_file);

    fprintf(dump_file, "\n********** End of report **********\n");

    fclose(dump_file);

    return 0;
}
