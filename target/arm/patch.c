// Copyright (c) 2024 Daniel Pyon

#include "qemu/osdep.h"
#include "cpu.h"
#include "patch.h"

#include <stdio.h>

int handle_brk(CPUState *cs, CPUARMState *env) {
    // TODO: write this
    fprintf(stderr, "inside handle_brk()\n");
    return 0;
}
