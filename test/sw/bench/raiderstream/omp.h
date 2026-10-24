// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// RaiderSTREAM's kernels include <omp.h> but call nothing from it. The bare-metal
// toolchain has no OpenMP, and they are compiled without -fopenmp: their
// pragmas are ignored, and each kernel runs as a plain loop on one hart.
