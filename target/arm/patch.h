// Copyright (c) 2024 Daniel Pyon

#ifndef ARM_PATCH_H
#define ARM_PATCH_H

#include "cpu.h"
#include "hw/core/cpu.h"

int handle_brk(CPUState *cs, CPUARMState *env);

#endif
