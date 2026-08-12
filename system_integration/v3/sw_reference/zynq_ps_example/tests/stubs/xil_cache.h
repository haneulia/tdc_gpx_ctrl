#ifndef TEST_XIL_CACHE_H
#define TEST_XIL_CACHE_H

#include <stdint.h>

void Xil_DCacheInvalidateRange(uintptr_t address, uint32_t length);

#endif
