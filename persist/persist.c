#include <assert.h>

#include "persist.h"
#include "ab_boot/ab_boot.h"

// keep this in sync with the linker script
#define PERSIST_SIZE 1024

extern struct persist_data persist;
static_assert(sizeof(persist) < PERSIST_SIZE);
