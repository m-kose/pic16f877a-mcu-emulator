#include <iostream>
#include <vector>
#include <cstdint>

using Byte = uint8_t;

class Register {
public:
    Byte value;
};

class MemoryBanks {
public:
    std::vector<Register> registers;
    uint64_t data;
};

class CPU {
public:
    CPU() {
        initiateRegisters();
        setMemoryBanks();
    }
    
    int getBankData(int bankNo, int registerNo){
        return banks[bankNo].registers[registerNo].value;
    }

private:
    std::vector<Register> registers;
    std::vector<MemoryBanks> banks;

    void initiateRegisters() {
        Register PC;
        Register AC;
        PC.value = 0xFF;
        AC.value = 0xBF;
        registers.push_back(PC);
        registers.push_back(AC);
    }

    void setMemoryBanks() {
        MemoryBanks bank1;
        bank1.registers = registers;
        bank1.data = 0x00000000;
        banks.push_back(bank1);
    }
};

int main() {
    CPU PIC16F87;
    std::cout << +PIC16F87.getBankData(0, 1) << std::endl;
    return 0;
}
