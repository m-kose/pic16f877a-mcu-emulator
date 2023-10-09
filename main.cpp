#include <iostream>
#include <vector>
#include <cstdint>
#include<bitset>

using Byte = uint8_t;
using INS = uint16_t;
static constexpr Byte max_mem = 128;
const int REG_SIZE = 3;


class Register {
public:
    Byte value;
};


class MemoryBanks {
public:
    //std::vector<Register> registers; // Banks do not have their own registers(PC, AC etc.), so we make this a CPU property instead
    INS program_memory[max_mem];
    Byte data_memory[max_mem];
};


class CPU {

    std::vector<Register> registers;

    Register PCL; // address
    Register PCLATH; // subroutines
    Register AC;
    Register STATUS;


    //// STATUS BITS

    Byte IRP : 1;
    Byte RP1 : 1;
    Byte RP0 : 1;
    Byte TO : 1;
    Byte PD : 1;
    Byte Z : 1;
    Byte DC : 1;
    Byte C : 1;

    //// STATUS BITS


    //// INSTRUCTIONS

    static constexpr Byte ADDWF = 0b00000111; // It loads to AC due to its 8th bit is 0. If its 1, it loads to LSB 7 bytes of address.
    static constexpr Byte ANDWF = 0b00000101; // Same as above(ADDWF)
    static constexpr Byte CLRF = 0b00000001;
    static constexpr Byte COMF = 0b00001001; // Complements: 1010 -> 0101
    static constexpr Byte DECF = 0b00000011; // Decrements: 1010 - 0001
    static constexpr Byte DECFSZ = 0b00001011; // Decrements but skips if zero
    static constexpr Byte INCF = 0b00001010; // Increments: 1010 + 1






    static constexpr Byte BCF = 0b0001;
    //// INSTRUCTIONS

public:
    CPU() {
        initiateRegisters();
        setMemoryBanks();
        Reset();
    }

    int getBankData(int bankNo, int registerNo){
        return registers[registerNo].value;
    }

    int getBankDataSize(int bankNo){
        return sizeof(banks[bankNo].data_memory);
    }

    void getBankDataInfo(int bankNo){
        for(int i = 0; i<max_mem; i++){
            std::cout<<i<<". BYTE: "<<+banks[bankNo].data_memory[i]<<std::endl;
        }
    }

    void editBankData(int bankNo, int regNo, Byte data){
        banks[bankNo].data_memory[regNo] = data;
    }

    Byte getRegister(int regNo){
        return registers[regNo].value;
    }

    void editRegister(int bankNo, int regNo, Byte data){
        registers[regNo].value = data;
    }

    void editProgramMemory(int bankNo, int index, INS data){
        banks[bankNo].program_memory[index] = data;
    }

    std::vector<Byte> decodeInstruction(INS instruction){
        Byte OPCODE; // Instruction OPCODE
        Byte d; // Destination, 0 for AC, 1 for file register
        Byte addr; // Address, if d = 1
        Byte k; // Literal
        Byte b; // Bıts for bit manipulation
        std::vector<Byte> data;
        switch(instruction >> 12){
            case 0:{ // Byte oriented file register operations
                OPCODE = (instruction >> 8);
                addr = (instruction & 0xFF);
                data.push_back(OPCODE);
                data.push_back(addr);
                return data;
            } break;
            case 1:{ // Bit oriented file register operations
                OPCODE = (instruction >> 10);
                addr = (instruction & 0x7F);
                b = ((instruction & 0b0000001110000000) >> 7);
                data.push_back(OPCODE);
                data.push_back(addr);
                data.push_back(b);
            }
            default:{
				std::cout<<"INSTRUCTION NOT DECODED PROPERLY!"<<std::endl;
			} break;
        }
    }

    INS Fetch(int &Cycles, int BankNo){
        INS Data = banks[BankNo].program_memory[registers[0].value]; // Data where PCL points at
        registers[0].value++; // PCL++
        Cycles--;
        return Data;
    }


    void Execute(int &Cycles){
        while(Cycles > 0){
            int BankNo;
            switch((registers[2].value & 0b01100000) >> 5) { // Select the memory bank to store/access data based on the flags of STATUS register
                case 0: {
                    BankNo = 0;
                } break;
                case 1: {
                    BankNo = 1;
                } break;
                case 2: {
                    BankNo = 2;
                }break;
                case 3:{
                    BankNo = 3;
                }break;
                default: {
                    std::cout << "Invalid bank selection. MCU is restarting..." << std::endl;
                    Reset();
                } break;
            }

            std::cout<<"SELECTED BANK: "<<BankNo<<std::endl;

            INS InstructionAddr = Fetch(Cycles, BankNo); // Fetch the instruction

            std::vector<Byte> Instruction = decodeInstruction(InstructionAddr);

            switch(Instruction[0] & 0xFF) {
                case ADDWF: {
                    Byte Value = Fetch(Cycles, BankNo); // Fetch the data
                    if(Instruction[1] >> 7){ // d bit, store in data register if it is 1
                        Byte addr = Instruction[1] & 0x7F; // 0-6 bits of the Instruction ADDWF is an address to store if it's to stored in memory.
                        banks[BankNo].data_memory[addr] = Value + registers[2].value; // Store the stored data + AC in w_addr
                    }
                    else{ // d bit, store in AC if it is 0
                        registers[1].value = Value + registers[1].value; // Store it in AC
                    }
                    /*
                    if(Value + registers[1].value > 0xFF){
                        registers[2].value | 0x01; // Set the Carry C flag of STATUS register if an overflow happens during the addition.
                        carry = registers[1].value - 0xFF;
                    }
                     */ // Probably not needed because the computer that will run this emulator will handle these carry bits anyway. Might look into this more in the future
                }break;
                case ANDWF:{
                    Byte Value = Fetch(Cycles, BankNo);
                    if(Instruction[1] >> 7){ // d bit, store in data register if it is 1
                        Byte addr = Instruction[1] & 0x7F; // 0-6 bits of the Instruction ADDWF is an address to store if it's to stored in memory.
                        banks[BankNo].data_memory[addr] = Value & registers[2].value; // Store the stored data + AC in w_addr
                    }
                    else{ // d bit, store in AC if it is 0
                        registers[2].value = Value & registers[1].value; // Store it in AC
                    }
                }break;
                case CLRF:{ // This will cover both CLRF and CLRW instructions as the only difference between them is the deleted register(d=0 W, d=1 f)
                    Byte addr = Instruction[1] & 0x7F;
                    if(Instruction[1] >> 7){ // d bit is 0, so we clear the f register
                        banks[BankNo].data_memory[addr] = 0;
                    }
                    else{ // d bit is 1, so we clear the AC register
                        registers[2].value = 0;
                    }
                }break;
                case COMF:{
                    Byte Value = Fetch(Cycles, BankNo); // Fetch the data
                    Byte addr = Instruction[1] & 0x7F;
                    if(Instruction[1] >> 7){ // d bit is 0, so we store in the f register
                        banks[BankNo].data_memory[addr] = ~Value;
                    }
                    else{ // d bit is 1, so we store in the AC register
                        registers[2].value = ~Value;
                    }
                }break;
                case DECF:{
                    Byte addr = Instruction[1] & 0x7F;
                    if(Instruction[1] >> 7){ // Store in the file register
                        banks[BankNo].data_memory[addr] = banks[BankNo].data_memory[addr] - 0b00000001;
                    }
                    else{ // Store in the AC
                        registers[2].value = banks[BankNo].data_memory[addr] - 0b00000001;
                    }
                }break;
                case DECFSZ:{
                    Byte addr = Instruction[1] & 0x7F;
                    if(Instruction[1] >> 7){ // Store in the file register
                        if((banks[BankNo].data_memory[addr] - 0b00000001) == 0){
                            break;
                        }
                        banks[BankNo].data_memory[addr] = banks[BankNo].data_memory[addr] - 0b00000001;
                    }
                    else{ // Store in the AC
                        if((banks[BankNo].data_memory[addr] - 0b00000001) == 0){
                            break;
                        }
                        registers[2].value = banks[BankNo].data_memory[addr] - 0b00000001;
                    }
                }break;
                case INCF:{
                    Byte addr = Instruction[1] & 0x7F;
                    if(Instruction[1] >> 7){ // Store in the file register
                        banks[BankNo].data_memory[addr] = banks[BankNo].data_memory[addr] + 0b00000001;
                    }
                    else{ // Store in the AC
                        registers[2].value = banks[BankNo].data_memory[addr] + 0b00000001;
                    }
                }break;
                default:{
                    std::cout<<"INSTRUCTION NOT FETCHED PROPERLY!"<<std::endl;
                }break;
            }
        }
    }
private:
    std::vector<MemoryBanks> banks;

    void initiateRegisters() {
        PCL.value = 0x00;
        PCLATH.value = 0x00;
        AC.value = 0x0F;
        STATUS.value = (1 << 7) | (0 << 6) | (0 << 5) | (0 << 4) | (0 << 3) | (0 << 2) | (0 << 1) | Byte(0);
        registers.push_back(PCL);
        registers.push_back(PCLATH);
        registers.push_back(AC);
        registers.push_back(STATUS);
    }

    void setMemoryBanks() {
        MemoryBanks bank0;

        MemoryBanks bank1;

        MemoryBanks bank2;

        MemoryBanks bank3;

        banks.push_back(bank0);
        banks.push_back(bank1);
        banks.push_back(bank2);
        banks.push_back(bank3);
    }

    void Reset(){

    	// Clear all the information in program memory and general registers
        for(int i = 0; i < 4 ; i++){
            for(int k = 0; k < max_mem; k++){
                banks[i].data_memory[k] = 0;
            }
        }

        for(int i = 0; i < 4 ; i++){
            for(int k = 0; k < max_mem; k++){
                banks[i].program_memory[k] = 0;
            }
        }
        PCL.value = 0x00;
        PCLATH.value = 0x00;
        AC.value = 0x0B;
        IRP = RP1 = RP0 = Z = DC = C = 0;
        TO = PD = 1;
        STATUS.value = (IRP << 7) | (RP1 << 6) | (RP0 << 5) | (TO << 4) | (PD << 3) | (Z << 2) | (DC << 1) | C;
    }
};

int main() {
    CPU PIC16F87;
    int Cycles = 3;
    PIC16F87.editProgramMemory(0, 0, 0b0000011111100000); // instruction(ADDLW)
    PIC16F87.editProgramMemory(0, 1, 0b0000000000001111); // data
    PIC16F87.editProgramMemory(0, 2, 0b0000000111100000); // instruction(CLRF)
    PIC16F87.Execute(Cycles);
    PIC16F87.getBankDataInfo(0);
    std::cout<<"AC REG: "<<PIC16F87.getBankData(0, 2)<<std::endl;
    return 0;
}
