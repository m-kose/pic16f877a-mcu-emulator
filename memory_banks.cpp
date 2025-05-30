#include "memory_banks.h"
#include <iostream> // For potential debugging output, not strictly necessary for the class itself

// Constructor definition
MemoryBanks::MemoryBanks() {
    // Initialize program_memory and data_memory arrays to 0 or default values
    for (int i = 0; i < max_mem; ++i) {
        program_memory[i] = 0;
        data_memory[i] = 0;
    }
    // registers vector will be empty by default, can be populated by CPU class
}

// Add other MemoryBanks methods here if there were any.
// For now, it seems the MemoryBanks class in the original code was mostly a data structure.
