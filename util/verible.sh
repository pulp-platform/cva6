#!/bin/bash

verible-verilog-format --inplace $(find core/ -type f -name '*.sv' ! -path 'core/include/*' ! -path 'core/cache_subsystem/hpdcache/*' ! -path 'core/cvfpu/*') > /dev/null

