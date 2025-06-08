#ifndef MEMORY_BANKS_H
#define MEMORY_BANKS_H

#include <vector>
#include <map>
#include <string>
#include <cstdint> // Required for uint8_t, uint16_t

// Forward declaration of Register class
// class Register;
// It's better to include the header directly if it's small and widely used.
#include "register.h"

using INS = uint16_t;
// Using Byte alias from register.h, so no need to redefine if register.h is included.
// using Byte = uint8_t;
static constexpr uint8_t max_mem = 128; // Changed Byte to uint8_t for clarity, or ensure Byte is defined before this line.


class MemoryBanks {
public:
    // Constructor if needed, e.g., to initialize program_memory and data_memory sizes
    MemoryBanks();
    std::vector<std::map<std::string, Register*>> registers;
    INS program_memory[max_mem];
    uint8_t data_memory[max_mem]; // Changed Byte to uint8_t
};

#endif // MEMORY_BANKS_H
