#include <iostream>
// #include <vector> // No longer directly needed by main
// #include <cstdint> // No longer directly needed by main
// #include <bitset> // No longer directly needed by main
// #include <map> // No longer directly needed by main
// "register.h" and "memory_banks.h" are included via "cpu.h"
#include "cpu.h"

// using Byte = uint8_t; // Byte is available via cpu.h -> memory_banks.h -> register.h
// INS and max_mem are available via cpu.h -> memory_banks.h

// CPU class definition and implementation is now in cpu.h and cpu.cpp

int main() {
    CPU PIC16F87;
    int Cycles = 3;

    // Example program for the PIC
    // These opcodes need to match the definitions in cpu.h and be processed by cpu.cpp
    // The constants like ADDLW, MOVWF, CLRF are defined in cpu.h
    // The editProgramMemory function takes INS (uint16_t)
    // Let's assume ADDLW k is 0b111110kkkkkkkk (this was ADDLW = 0b00111110 in header)
    // MOVLW k is 0b110000kkkkkkkk (this was MOVLW = 0b00110000 in header)
    // If MOVWF f is 0b00000001fffffff (MOVWF = 0b00000000 dfffffff, d=0 implies W, d=1 implies f)
    // MOVWF f (Move W to f) is 000000000fffffff if f is the address (from MOVWF = 0b00000000)
    //   If f is addr 0xFF, instruction is 0x00FF.
    // CLRF f is 0b00000011dfffffff (CLRF = 0b00000001). If d=1 (f), instr = 0x01FF for f=0xFF.

    // Original instructions in main:
    // PIC16F87.editProgramMemory(0, 0, 0b0011111000000001); // ADDLW 0x01
    // PIC16F87.editProgramMemory(0, 1, 0b0000000011111111); // MOVWF 0xFF (assuming d=0 means W to f)
    // PIC16F87.editProgramMemory(0, 2, 0b0000000111100000); // CLRF 0xE0 (assuming d=1 means CLRF f)

    // Let's use the constants from cpu.h to build instructions if possible, or ensure these raw values are correct.
    // For ADDLW k (k=1): (ADDLW << 8) | 1 = (0x3E << 8) | 0x01 = 0x3E01
    // For MOVWF f (f=0x7F, assuming d=0 in opcode means W to f, but MOVWF is usually d=0): (MOVWF << 8) | 0x7F = (0x00<<8) | 0x7F = 0x007F
    // For CLRF f (f=0x60, d=1 for f): (CLRF << 8) | (1 << 7) | 0x60 = (0x01 << 8) | 0x80 | 0x60 = 0x01E0

    PIC16F87.editProgramMemory(0, 0, (CPU::ADDLW << 8) | 0x01);      // ADDLW 0x01 -> AC = AC + 0x01
    PIC16F87.editProgramMemory(0, 1, (CPU::MOVWF << 8) | 0x20);      // MOVWF 0x20 (move AC to address 0x20 in current bank)
    PIC16F87.editProgramMemory(0, 2, (CPU::CLRF << 8) | (1 << 7) | 0x20); // CLRF 0x20 (clear address 0x20, d=1 for f)
    // Note: The d bit for CLRF needs to be part of the address byte if opcode is just CLRF=0x01.
    // If CLRF opcode itself implies clearing f, then it's (CLRF<<8) | 0x20.
    // The original code in main.cpp had:
    // 0b0000000111100000 -> CLRF (0x01) for f=0xE0, with d=1 (MSB of f part)
    // This matches (CPU::CLRF << 8) | (1 << 7) | 0x60 = 0x01E0 if f=0x60.
    // For f=0x20, (CPU::CLRF << 8) | (1 << 7) | 0x20 = 0x01A0.

    std::cout << "Executing test program..." << std::endl;
    PIC16F87.Execute(Cycles);

    std::cout << "Execution finished. Checking register 0x20 in Bank 0:" << std::endl;
    // To check value, need a way to read from data_memory via CPU, or make banks public temporarily.
    // Or use getRegisterValue if 0x20 is mapped to a name.
    // For now, let's assume getBankDataInfo can show it.
    PIC16F87.getBankDataInfo(0); // Display data memory for bank 0

    return 0;
}
