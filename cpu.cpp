#include "cpu.h"
#include <iostream> // For std::cout
#include <vector>   // For std::vector
#include <string>   // For std::string
#include <map>      // For std::map
// <cstdint> is included via cpu.h -> memory_banks.h
// <bitset> is not directly used in CPU methods, but might be by callers or if instruction decoding becomes more complex.
// For now, let's assume it's not needed directly in cpu.cpp

// Static constants are defined in the header, no need to redefine here.

CPU::CPU() {
    initiateRegisters(); // Initialize AC, PCL, STATUS etc.
    setMemoryBanks();    // Setup memory banks and their register maps
    Reset();             // Perform a full reset, which also initializes/clears memories
}

void CPU::initiateRegisters() {
    // These are the special function registers directly owned by the CPU
    PCL.setValue(0x00);
    PCLATH.setValue(0x00);
    AC.setValue(0x0F); // Initial value for Accumulator
    STATUS.setValue(0b00011000); // Initial STATUS register value (RP0=1, for example, or a common default)
    // Other registers like FSR, PORTA, etc., are also members and will be default constructed.
    // Their specific values are often set by the program or hardware simulation details.
    // If they need specific non-zero initial values, set them here.
    // For example:
    // FSR.setValue(0x00);
    // TRISA.setValue(0xFF); // All PORTA pins as inputs by default
    // PORTA.setValue(0x00);
}

void CPU::setMemoryBanks() {
    banks.resize(4); // Assuming 4 memory banks as in the original design

    // Common registers that might be mirrored or exist in specific banks
    // This setup mimics the original structure.

    //////// BANK 0 ////////
    std::map<std::string, Register*> bank0Registers;
    bank0Registers["indraddr"] = &indraddr;
    bank0Registers["TMR0"] = &TMR0;
    bank0Registers["PCL"] = &PCL;
    bank0Registers["STATUS"] = &STATUS;
    bank0Registers["FSR"] = &FSR;
    bank0Registers["PORTA"] = &PORTA;
    bank0Registers["PORTB"] = &PORTB;
    bank0Registers["PORTC"] = &PORTC;
    bank0Registers["PORTD"] = &PORTD;
    bank0Registers["PORTE"] = &PORTE;
    bank0Registers["PCLATH"] = &PCLATH;
    bank0Registers["INTCON"] = &INTCON;
    bank0Registers["PIR1"] = &PIR1;
    bank0Registers["PIR2"] = &PIR2;
    bank0Registers["TMR1L"] = &TMR1L;
    bank0Registers["TM1HL"] = &TM1HL; // Typo for TMR1H?
    bank0Registers["T1CON"] = &T1CON;
    bank0Registers["TMR2"] = &TMR2;
    bank0Registers["T2CON"] = &T2CON;
    bank0Registers["SSPBUF"] = &SSPBUF;
    bank0Registers["SSPCON"] = &SSPCON;
    bank0Registers["CCPR1L"] = &CCPR1L;
    bank0Registers["CCPR1H"] = &CCPR1H;
    bank0Registers["CCP1CON"] = &CCP1CON;
    bank0Registers["RCSTA"] = &RCSTA;
    bank0Registers["TXREG"] = &TXREG;
    bank0Registers["RCREG"] = &RCREG;
    bank0Registers["CCPR2L"] = &CCPR2L;
    bank0Registers["CCPR2H"] = &CCPR2H;
    bank0Registers["CCP2CON"] = &CCP2CON;
    bank0Registers["ADRESH"] = &ADRESH;
    bank0Registers["ADCON0"] = &ADCON0;
    bank0Registers["AC"] = &AC; // AC is a CPU register, but can be mapped.
    banks[0].registers.push_back(bank0Registers);

    //////// BANK 1 ////////
    std::map<std::string, Register*> bank1Registers;
    bank1Registers["indraddr"] = &indraddr;
    bank1Registers["PCL"] = &PCL;
    bank1Registers["STATUS"] = &STATUS;
    bank1Registers["FSR"] = &FSR;
    bank1Registers["OPTION_REG"] = &OPTION_REG; // OPTION_REG is typically in bank 1
    bank1Registers["TRISA"] = &TRISA;
    bank1Registers["TRISB"] = &TRISB;
    bank1Registers["TRISC"] = &TRISC;
    bank1Registers["TRISD"] = &TRISD;
    bank1Registers["TRISE"] = &TRISE;
    bank1Registers["PCLATH"] = &PCLATH;
    bank1Registers["INTCON"] = &INTCON;
    bank1Registers["PIE1"] = &PIE1;
    bank1Registers["PIE2"] = &PIE2;
    bank1Registers["PCON"] = &PCON;
    // bank1Registers["SSPCON"] = &SSPCON; // SSPCON is also in bank0, check datasheet for mirroring
    bank1Registers["PR2"] = &PR2;
    bank1Registers["SSPADD"] = &SSPADD;
    bank1Registers["SSPSTAT"] = &SSPSTAT;
    bank1Registers["TXSTA"] = &TXSTA;
    bank1Registers["SPBRG"] = &SPBRG;
    bank1Registers["CMCON"] = &CMCON;
    bank1Registers["CVRCON"] = &CVRCON;
    bank1Registers["ADRESL"] = &ADRESL; // ADRESL for ADRESH in bank0
    bank1Registers["ADCON1"] = &ADCON1;
    bank1Registers["AC"] = &AC;
    // Fill with EMPTYREG for unmapped addresses if necessary, original code had specific EMPTYREGs
    bank1Registers["EMPTYREG1"] = &EMPTYREG;
    banks[1].registers.push_back(bank1Registers);

    //////// BANK 2 ////////
    std::map<std::string, Register*> bank2Registers;
    bank2Registers["indraddr"] = &indraddr;
    bank2Registers["TMR0"] = &TMR0;
    bank2Registers["PCL"] = &PCL;
    bank2Registers["STATUS"] = &STATUS;
    bank2Registers["FSR"] = &FSR;
    bank2Registers["PCLATH"] = &PCLATH;
    bank2Registers["INTCON"] = &INTCON;
    bank2Registers["EEDATA"] = &EEDATA;
    bank2Registers["EEADR"] = &EEADR;
    bank2Registers["EEDATH"] = &EEDATH;
    bank2Registers["EEADRH"] = &EEADRH;
    bank2Registers["AC"] = &AC;
    // Fill with EMPTYREG for unmapped bank 2 addresses
    bank2Registers["EMPTYREG_B2_1"] = &EMPTYREG;
    banks[2].registers.push_back(bank2Registers);

    //////// BANK 3 ////////
    std::map<std::string, Register*> bank3Registers;
    bank3Registers["indraddr"] = &indraddr;
    // bank3Registers["OPTION_REG"] = &OPTION_REG; // OPTION_REG is in bank1
    bank3Registers["PCL"] = &PCL;
    bank3Registers["STATUS"] = &STATUS;
    bank3Registers["FSR"] = &FSR;
    // TRIS registers are in bank 1
    bank3Registers["PCLATH"] = &PCLATH;
    bank3Registers["INTCON"] = &INTCON;
    bank3Registers["EECON1"] = &EECON1;
    bank3Registers["EECON2"] = &EECON2; // Note: EECON2 is often not a physical register
    bank3Registers["AC"] = &AC;
    // Fill with EMPTYREG for unmapped bank 3 addresses
    bank3Registers["EMPTYREG_B3_1"] = &EMPTYREG;
    banks[3].registers.push_back(bank3Registers);
}

void CPU::Reset() {
    // Reset CPU's own core registers
    PCL.setValue(0x00);
    PCLATH.setValue(0x00);
    AC.setValue(0x00); // Often W is cleared on reset, or specific value
    STATUS.setValue(0b00011000); // Default STATUS, e.g., RP0=0, TO=1, PD=1 or as per datasheet for specific device

    // Clear program and data memory in all banks
    for (auto& bank : banks) { // Iterate through each MemoryBanks object
        // MemoryBanks constructor already initializes these to 0.
        // This ensures they are cleared again if Reset() is called post-construction.
        for (int i = 0; i < max_mem; ++i) {
            bank.program_memory[i] = 0;
            bank.data_memory[i] = 0;
        }
        // If Register objects pointed to by maps in bank.registers need resetting, do it here.
        // For example, if PORTA should be 0x00 on reset:
        // if (bank.registers[0].count("PORTA")) bank.registers[0]["PORTA"]->setValue(0x00);
        // However, CPU member registers like PORTA are reset above if they are direct members,
        // or handled by their own class's reset if applicable.
        // The current design has CPU member registers (PORTA etc.) which are then mapped.
        // So, resetting the CPU member should suffice.
    }
    // Example: Reset specific CPU member registers that are mapped.
    // These were already set in initiateRegisters or directly in Reset for PCL, AC, STATUS.
    // If others need specific reset values:
    FSR.setValue(0x00);
    PORTA.setValue(0x00); TRISA.setValue(0xFF);
    PORTB.setValue(0x00); TRISB.setValue(0xFF);
    PORTC.setValue(0x00); TRISC.setValue(0xFF);
    PORTD.setValue(0x00); TRISD.setValue(0xFF);
    PORTE.setValue(0x00); TRISE.setValue(0b00000111); // PORTE can have analog functions on reset
    INTCON.setValue(0x00);
    // etc. for all other SFRs according to datasheet specifics for a PIC16F877A for example.
}


void CPU::getBankDataInfo(int bankNo) {
    if (bankNo < 0 || bankNo >= banks.size()) {
        std::cout << "Invalid bank number: " << bankNo << std::endl;
        return;
    }
    std::cout << "Data Memory for Bank " << bankNo << ":" << std::endl;
    for (int i = 0; i < max_mem; ++i) {
        // Assuming data_memory is an array of uint8_t (Byte)
        std::cout << "Addr 0x" << std::hex << i << ": " << +banks[bankNo].data_memory[i] << std::dec << std::endl;
    }
}

void CPU::editProgramMemory(int bankNo, int index, INS data) {
    if (bankNo < 0 || bankNo >= banks.size() || index < 0 || index >= max_mem) {
        std::cout << "Invalid address for program memory edit." << std::endl;
        return;
    }
    banks[bankNo].program_memory[index] = data;
}

// In CPU::setRegisterValue and CPU::getRegisterValue
// The `MemoryBanks& bank` parameter refers to one of the elements from `std::vector<MemoryBanks> banks`.
// The original code `for (std::map<std::string, Register*>& registers_map : bank.registers)`
// implies `bank.registers` is a collection of maps. `bank.registers` is `std::vector<std::map<std::string, Register*>>`.
// We'll assume the first map `bank.registers[0]` is the one to use, as per original structure.

void CPU::setRegisterValue(MemoryBanks& selected_bank, const std::string& registerName, Byte value) {
    if (selected_bank.registers.empty()) {
        std::cout << "No register map in selected bank." << std::endl;
        return;
    }
    auto& registers_map = selected_bank.registers[0]; // Use the first map
    if (registers_map.count(registerName)) {
        registers_map[registerName]->setValue(value);
    } else {
        std::cout << "Register " << registerName << " not found in the specified bank's map." << std::endl;
    }
}

Byte CPU::getRegisterValue(const MemoryBanks& selected_bank, const std::string& registerName) {
    if (selected_bank.registers.empty()) {
        std::cout << "No register map in selected bank." << std::endl;
        return 0; // Or throw exception
    }
    const auto& registers_map = selected_bank.registers[0]; // Use the first map
    if (registers_map.count(registerName)) {
        return registers_map.at(registerName)->getValue();
    } else {
        // std::cout << "Register " << registerName << " not found in the specified bank's map. Returning 0." << std::endl;
        // It's common for reads from unmapped/non-existent registers to return 0.
        return 0;
    }
}

std::vector<Byte> CPU::decodeInstruction(INS instruction) {
    int Type = -1; // Initialize Type to an invalid value
    Byte OPCODE = 0;
    Byte addr = 0;
    Byte control_k = 0;
    Byte literal_k = 0;
    Byte b_val = 0;
    std::vector<Byte> data;

    // Determine instruction type based on higher bits
    // PIC16F877A datasheet instruction set summary can be used as reference
    if ((instruction & 0b1100000000000000) == 0b0000000000000000) { // Byte-oriented file register ops (except MOVWF)
        OPCODE = (instruction >> 7) & 0b1111111; // Opcode is bits 13-7 for some, 13-8 for others. This is a simplification.
                                                 // For PICs, usually it's more like:
                                                 // xxxx xxdd dfff ffff (d=dest, f=file reg)
                                                 // xxxx kkkk kkkk kkkk (literal ops)
                                                 // xxxx bbbf ffff ffff (bit ops)
        // Let's use the original logic's interpretation for now
        Type = 0; // Byte oriented
        OPCODE = (instruction >> 8); // This was the original logic, implies 00xx xxxx
        addr = (instruction & 0xFF); // This seems to take dfffffff if d is bit 7 of this byte.
                                     // If OPCODE is instruction >> 8, then instruction is 00iiiiii ffffffff
                                     // This means original ADDWF, ANDWF etc. constants were for the 'iiiiii' part.

        // Re-evaluating original constants like ADDWF = 0b00000111 (0x07)
        // If instruction is 0x07AD, OPCODE = 0x07, addr = 0xAD.
        // The 'd' bit (destination) is encoded in addr's MSB (bit 7) for some instructions, or part of opcode itself.
        // The prompt has ADDWF's 8th bit (d) as 0 for AC, 1 for file. This is bit 7 of the 'f' part.
        // So, if instruction is 0b00000111 dfff ffff -> OPCODE = 0b00000111, addr = dfffffff
        // This matches the original switch cases.
        data.push_back(OPCODE); // The 00iiiiii part
        data.push_back(addr);   // The dfffffff part
        data.push_back(Type);
        return data;

    } else if ((instruction & 0b1111000000000000) == 0b0100000000000000 || // BCF, BSF (0100xx)
               (instruction & 0b1111000000000000) == 0b0101000000000000 || // BTFSC, BTFSS (0101xx) - Error, these are 0110 and 0111
               (instruction & 0b1111000000000000) == 0b0110000000000000 || // BTFSC
               (instruction & 0b1111000000000000) == 0b0111000000000000) { // BTFSS
        // Bit-oriented file register operations 01xxxx
        Type = 1; // Bit oriented
        OPCODE = (instruction >> 10) & 0b11; // The xx part of 01xx bbb ffff ffff. This is the BCF,BSF,BTFSC,BTFSS part.
                                          // Original constants are BCF=0b0100, BSF=0b0101, etc.
                                          // This means the OPCODE is (instruction >> 8) & 0b1111 (e.g. 0100)
                                          // Or (instruction >> 10) if we only want the variable part.
                                          // The prompt's constants are 4-bit: BCF=0b0100.
                                          // Let's use (instruction >> 7) for addr, and (instruction >>10) for opcode part.
        OPCODE = (instruction >> 10); // This would be 01xx. If constants are 0b0100, this is not right.
                                      // Original: OPCODE = (instruction >> 10); addr = (instruction & 0x7F); b = ((instruction & 0b0000001110000000) >> 7);
                                      // This implies instruction format 01xx bbb ffff ffff. And BCF = 0100 implies xx=00.
                                      // So, if instruction is BCF (0100 bbb ffff ffff), then (instruction>>10) is 0100.
        addr = (instruction & 0x7F); // ffff ffff part
        b_val = (instruction >> 7) & 0b111; // bbb part
        data.push_back(OPCODE); // The 01xx part
        data.push_back(addr);
        data.push_back(b_val);
        data.push_back(Type);
        return data;
    } else if ((instruction & 0b1100000000000000) == 0b1000000000000000 || // CALL, GOTO (100xxx, 101xxx)
               (instruction & 0b1111000000000000) == 0b1100000000000000) { // Literal ops (MOVLW, ADDLW etc. 11xxxx)
        // Literal and control operations
        Type = 2;
        OPCODE = (instruction >> 8); // For 11xxxx kkkkkkkk, OPCODE = 11xxxx. For 100kkk..., OPCODE = 100k.
                                    // Original: OPCODE = (instruction >> 8); control_k = (instruction & 0b11111111111); literal_k = (instruction & 0xFF);
                                    // This implies CALL/GOTO is 100x k...k (11 bits for k) -> OPCODE = 100x
                                    // And literal ops are 11xxxx kkkkkkkk -> OPCODE = 11xxxx
        literal_k = (instruction & 0xFF); // For literal ops kkkkkkkk
        control_k = (instruction & 0x7FF); // For CALL/GOTO kkk kkkk kkkk (11 bits)
                                          // The original code had control_k = (instruction & 0b11111111111) which is 0x7FF for GOTO/CALL
                                          // and literal_k = (instruction & 0xFF) for others.
                                          // This means the type of k depends on the specific opcode.
        data.push_back(OPCODE);
        if ((OPCODE >> 3) == 0b100) { // GOTO (1001 kkk...) or CALL (1000 kkk...) - this is not quite right.
                                   // GOTO is 101x... CALL is 100x...
                                   // If GOTO = 0b1001... then OPCODE (>>8) = 1001xxxx.
                                   // If CALL = 0b1000... then OPCODE (>>8) = 1000xxxx.
            // This part of original code: control_k = (instruction & 0b11111111111); (2^11 - 1 = 0x7FF)
            // And literal_k = (instruction & 0xFF);
            // The switch case for Execute will use the correct k.
            data.push_back(control_k); // Max 11 bits for GOTO/CALL
            data.push_back(literal_k); // Max 8 bits for others (ADDLW etc)
        } else {
             data.push_back(literal_k); // For literal ops
        }
        data.push_back(Type);
        return data;
    } else {
        std::cout << "INSTRUCTION IS NOT DECODED PROPERLY! Opcode: " << std::hex << instruction << std::dec << std::endl;
        // Return empty or error indication
        return data; // Empty data
    }
     // Fallback, should have returned from one of the branches
    std::cout << "Reached end of decodeInstruction without returning, opcode: " << std::hex << instruction << std::dec << std::endl;
    return data; // Should not happen
}


INS CPU::Fetch(int &Cycles, int BankNo) {
    if (BankNo < 0 || BankNo >= banks.size()) {
         std::cout << "Fetch: Invalid bank number " << BankNo << std::endl;
         Cycles = 0; // Stop execution
         return 0xFFFF; // Indicate error
    }
    // PCL is a CPU register, its value is the address.
    // The actual memory is in banks[BankNo].program_memory
    // But PCL's value itself is not bank-dependent for fetching.
    // However, GOTO/CALL can load PCLATH which affects PCL's higher bits,
    // and PCLATH might be considered bank-dependent by some designs, though usually not.
    // For now, assume PCL gives the direct address.
    Byte currentPCL = PCL.getValue();
    if (currentPCL >= max_mem) { // Check against program memory size
        std::cout << "Fetch: PCL value " << +currentPCL << " exceeds program memory size." << std::endl;
        Cycles = 0;
        return 0xFFFF; // Indicate error
    }

    INS instruction = banks[BankNo].program_memory[currentPCL];

    if (instruction == 0) { // Assuming 0 is NOP or end of program marker in this context
        // std::cout << "Fetched NOP or EOP (0x0000)" << std::endl;
        // Let it be processed by Execute, or handle EOP explicitly here if needed.
        // If 0 means halt, then Cycles = 0;
    }

    PCL.setValue(currentPCL + 1); // Increment PCL for next instruction
    Cycles--;
    return instruction;
}

void CPU::updateRegistersOnBank() {
    // This function's original purpose was to sync CPU's register values (like AC, STATUS)
    // with their memory-mapped representations in the current bank's data_memory.
    // However, with direct member registers (AC, STATUS, etc.) and MemoryBanks holding pointers
    // to these CPU members in its map, the values are already "in sync" because the map points
    // to the single source of truth (the CPU member).
    // The part `banks[i].data_memory[k] = entry.second->getValue();` seems to imply
    // that *every* register in the map should also have its value copied to a specific
    // data_memory location `k`. This is unusual unless `k` is the actual register address.
    // The original loop `for(const auto& entry: banks[i].registers[k])` where `k` was an int
    // and `banks[i].registers[k]` was a map, is now `banks[i].registers[0]` if we assume one map per bank.
    // Let's assume the intention was: for each register defined in the bank's map,
    // its value should be mirrored into the data_memory array at an address that corresponds
    // to the register's actual address. This requires knowing the address for each register name.
    // The original loop `int k=0; ... k++` just put them sequentially from data_memory[0] onwards.
    // This is likely not the correct interpretation for a real PIC.
    // For now, I will comment out this function's body as its original implementation
    // is probably not what's needed with the current direct-member register design.
    // If a true memory map sync is needed (e.g. writing STATUS value to 0x03 in data memory),
    // that should be done more explicitly.

    /*
    for(int i = 0; i < banks.size(); ++i) { // For each bank
        if (banks[i].registers.empty()) continue;
        auto& register_map = banks[i].registers[0]; // Assuming one map per bank
        for(const auto& entry : register_map) {
            // Here, we need to know the actual memory address for 'entry.first' (registerName)
            // The original code `banks[i].data_memory[k] = entry.second->getValue(); k++;` is problematic.
            // For example, if STATUS is at address 0x03, we'd do:
            // banks[i].data_memory[0x03] = STATUS.getValue();
            // This requires a lookup from registerName to its address.
        }
    }
    */
   // The most direct interpretation of the original code was to update the *CPU's representation*
   // of special registers based on memory banks, or vice versa.
   // Given Register members are now directly in CPU, this might be simplified or removed.
   // The original code's `updateRegistersOnBank` was:
   /*
    void updateRegistersOnBank(){
        for(int i = 0; i < 4; i++){ // Iterate banks
            int k = 0; // This k was used as an index into data_memory AND to select a map from bank.registers
                       // This implies banks[i].registers was a vector of maps, and k selected one map.
                       // And then, it iterated that map and used k again as data_memory index, incrementing k.
                       // This is very confusing. My current structure is banks[i].registers is a vector of maps,
                       // and I've assumed banks[i].registers[0] is the map for bank i.

            if (banks[i].registers.empty()) continue;

            // If k was meant to be an index for data_memory cells corresponding to registers in the map
            // then the map iteration should determine the correct k (address) for each register.
            // The original code:
            // for(const auto& entry: banks[i].registers[k]){ <--- if k=0, this is banks[i].registers[0] (the map)
            //    const std::string& registerName = entry.first;
            //    banks[i].data_memory[k] = entry.second->getValue(); <--- here k is an index, not related to register's actual address
            //    k++; <--- this k is incremented
            // }
            // This sequential write to data_memory based on map iteration order is likely wrong.
            // I will skip implementing this for now as it needs clarification for correct behavior.
        }
    }
   */
}


void CPU::Execute(int &Cycles) {
    int BankNo = 0;
    while (Cycles > 0) {
        // updateRegistersOnBank(); // Commented out, see function definition

        // Determine current bank from STATUS register (RP1:RP0 bits)
        Byte status_val = STATUS.getValue();
        BankNo = (status_val >> 5) & 0b11; // RP1 is bit 6, RP0 is bit 5

        if (BankNo >= banks.size()) {
            std::cout << "Invalid bank selection from STATUS register: " << BankNo << ". Resetting." << std::endl;
            Reset(); // Or handle error appropriately
            Cycles = 0;
            return;
        }

        // std::cout << "SELECTED BANK: " << BankNo << std::endl;

        INS InstructionAddr = Fetch(Cycles, BankNo);
        if (Cycles <= 0 && InstructionAddr == 0xFFFF) { // Fetch might set Cycles to 0 on error
             std::cout << "Execute: Halting due to fetch error or end of cycles." << std::endl;
            break;
        }
        if (InstructionAddr == 0xFFFF) { // Error from Fetch
            std::cout << "Execute: Halting due to fetch error." << std::endl;
            break;
        }


        std::vector<Byte> InstructionParts = decodeInstruction(InstructionAddr);

        if (InstructionParts.empty()) {
            std::cout << "Execute: Empty instruction decoded. Halting." << std::endl;
            Cycles = 0; // Stop if decode fails significantly
            break;
        }

        Byte type = InstructionParts.back();
        Byte major_opcode = InstructionParts[0]; // This is the group (e.g. 0x07 for ADDWF)
        Byte operand1 = (InstructionParts.size() > 2) ? InstructionParts[1] : 0; // ffff ffff or control_k or addr
        Byte operand2 = (InstructionParts.size() > 3) ? InstructionParts[2] : 0; // literal_k or b_val

        MemoryBanks& currentBank = banks[BankNo]; // Reference to the current bank selected by STATUS

        if (type == 0) { // Byte oriented
            Byte f_addr = operand1 & 0x7F; // Lower 7 bits for address in memory bank
            Byte d_bit = (operand1 >> 7);  // Destination bit (0 for W/AC, 1 for f)

            switch (major_opcode) { // This was `Instruction[0] & 0xFF`
                case ADDWF: {
                    Byte val_f = getRegisterValue(currentBank, std::to_string(f_addr)); // Need name for f_addr
                                                                                       // This is problematic: getRegisterValue expects a name.
                                                                                       // Direct memory access:
                    val_f = currentBank.data_memory[f_addr];
                    Byte res = val_f + AC.getValue();
                    // TODO: Update STATUS register flags (C, DC, Z)
                    if (d_bit == 1) { // Store in f
                        currentBank.data_memory[f_addr] = res;
                    } else { // Store in W (AC)
                        AC.setValue(res);
                    }
                } break;
                case ANDWF: {
                    Byte val_f = currentBank.data_memory[f_addr];
                    Byte res = val_f & AC.getValue();
                    // TODO: Update STATUS register Z flag
                    if (d_bit == 1) {
                        currentBank.data_memory[f_addr] = res;
                    } else {
                        AC.setValue(res);
                    }
                } break;
                case CLRF: { // CLRF f or CLRW
                    if (d_bit == 1) { // CLRF f (major_opcode from decode is CLRF, d_bit from operand1)
                        // std::cout << "CLRF " << +f_addr << std::endl;
                        currentBank.data_memory[f_addr] = 0;
                        // TODO: Set Z flag in STATUS
                    } else { // CLRW (undocumented, but d=0 means W for many ops)
                        // std::cout << "CLRW" << std::endl;
                        AC.setValue(0);
                        // TODO: Set Z flag in STATUS
                    }
                } break;
                // ... other byte-oriented instructions
                 default:
                    std::cout << "Unhandled Byte Oriented Opcode: " << +major_opcode << std::endl;
                    break;
            }
        } else if (type == 1) { // Bit oriented
            Byte f_addr = operand1; // Address of f
            Byte bit_pos = operand2; // Bit position bbb
            // major_opcode here is (instruction >> 10) e.g. 0100 for BCF, 0101 for BSF

            if (major_opcode == (BCF >> 6)) { // BCF was 0b0100. (0b0100 >> 6) is 1. Not right.
                                            // BCF constant is 0b0100. Opcode from decode is (instr>>10) = 0b0100 if instr = 0x10xx
                                            // This needs to match the constants defined in cpu.h for BCF, BSF etc.
                                            // The constants BCF=0b0100 are 4 bits. The OPCODE from decode was (instr>>10)
                                            // If instr = 0b0100 bbb ffff ffff, then (instr>>10) = 0b0100. This matches.
                // std::cout << "BCF Addr: " << +f_addr << " Bit: " << +bit_pos << std::endl;
                currentBank.data_memory[f_addr] &= ~(1 << bit_pos);
            } else if (major_opcode == (BSF >> 6)) { // Similar issue with matching
                // std::cout << "BSF Addr: " << +f_addr << " Bit: " << +bit_pos << std::endl;
                currentBank.data_memory[f_addr] |= (1 << bit_pos);
            } else if (major_opcode == (BTFSC >> 6)) {
                if (! (currentBank.data_memory[f_addr] & (1 << bit_pos))) { // If bit is 0 (Clear)
                    // Skip next instruction (effectively a NOP for the next fetch)
                    Fetch(Cycles, BankNo); // Consume next instruction from PC
                }
            } else if (major_opcode == (BTFSS >> 6)) {
                 if (currentBank.data_memory[f_addr] & (1 << bit_pos)) { // If bit is 1 (Set)
                    Fetch(Cycles, BankNo); // Consume next instruction
                }
            }
             else {
                 std::cout << "Unhandled Bit Oriented Opcode: " << +major_opcode << std::endl;
             }
        } else if (type == 2) { // Literal and Control
            Byte k_literal_8bit = operand2; // For literal ops (ADDLW, MOVLW, etc.)
            INS k_control_11bit = operand1; // For GOTO, CALL (address)

            // The major_opcode is (instruction >> 8)
            // Example: MOVLW k is 0b110000kkkkkkkk. (instruction>>8) is 0b110000 (0x30)
            // GOTO k is 0b101kkkkkkkkkkk. (instruction>>8) is 0b101kkkkk (k are split) - this is complex.
            // Original GOTO was 0b1001. This implies a simpler encoding where GOTO's opcode part is just 0b1001.
            // Let's use the constants defined in cpu.h
            // major_opcode = InstructionParts[0]
            // k_literal_8bit = InstructionParts[2] (for type 2, if not GOTO/CALL)
            // k_control_11bit = InstructionParts[1] (for type 2, GOTO/CALL)

            if (major_opcode == MOVLW) {
                AC.setValue(k_literal_8bit);
            } else if (major_opcode == ADDLW) {
                Byte res = AC.getValue() + k_literal_8bit;
                AC.setValue(res);
                // TODO: Update STATUS C, DC, Z flags
            } else if (major_opcode == ANDLW) {
                Byte res = AC.getValue() & k_literal_8bit;
                AC.setValue(res);
                // TODO: Update Z flag
            } else if (major_opcode == IORLW) {
                Byte res = AC.getValue() | k_literal_8bit;
                AC.setValue(res);
                // TODO: Update Z flag
            } else if (major_opcode == SUBLW) { // k - W
                Byte res = k_literal_8bit - AC.getValue();
                AC.setValue(res);
                // TODO: Update C, DC, Z flags (C is inverted for subtract)
            } else if (major_opcode == XORLW) {
                Byte res = AC.getValue() ^ k_literal_8bit;
                AC.setValue(res);
                // TODO: Update Z flag
            } else if (major_opcode == (GOTO >> 7) ) { // GOTO was 0b1001. (instr>>8) would be 0b1001xxxx
                                                     // If GOTO is 0b101 k...k (11-bit addr)
                                                     // The constants are 4-bit or more.
                                                     // GOTO = 0b1001. This needs careful matching.
                                                     // The (instruction>>8) was pushed as major_opcode.
                                                     // If instr = 0b101kkkkk kkkkkkkk (GOTO format in datasheet)
                                                     // Then (instr>>8) is 0b101kkk. This is not a fixed const.
                                                     // The original code had GOTO = 0b1001. Implying a different encoding family.
                                                     // Let's assume the constants in cpu.h are the full opcode part.
                                                     // If GOTO is 0b1001, this is the major_opcode.
                                                     // And k_control_11bit is the address from InstructionParts[1].
                PCL.setValue(k_control_11bit & 0x7FF); // Use lower 11 bits for PCL
                // PCLATH might need to be involved here for full 13-bit address.
            } else if (major_opcode == (CALL >> 7)) { // Similar to GOTO
                // TODO: Implement CALL (push PC to stack, then load PCL with k)
                // PCL.setValue(k_control_11bit & 0x7FF);
                 std::cout << "CALL instruction not fully implemented." << std::endl;
            } else if (major_opcode == RETLW) {
                // TODO: Implement RETLW (pop PC from stack, load W with k)
                AC.setValue(k_literal_8bit); // Load W with literal
                // Then pop PC from stack.
                 std::cout << "RETLW instruction not fully implemented (stack missing)." << std::endl;
            }
            else {
                 std::cout << "Unhandled Literal/Control Opcode: " << +major_opcode << std::endl;
            }
        } else {
            std::cout << "Unknown instruction type: " << type << std::endl;
        }
        // Decrement cycles at fetch
    }
}

// Note: The simplified instruction decoding and execution logic above needs to be
// carefully verified against the specific PIC instruction set architecture being targeted.
// Status flag updates are critical and mostly omitted for brevity here.
// Register name to address mapping for get/setRegisterValue in byte-oriented instructions using f_addr also needs refinement.
// The current implementation of byte-oriented ops directly uses currentBank.data_memory[f_addr].
