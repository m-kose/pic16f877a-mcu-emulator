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

class Instruction{
public:
    uint16_t INS;
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

    static constexpr INS ADDWF = 0b00000111; // It loads to AC due to its 8th bit is 0. If its 1, it loads to LSB 7 bytes of address.
    static constexpr INS ANDWF = 0b00000101;
    static constexpr INS CLRW = 0b00000001;
    
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

    void editProgramMemory(int bankNo, int index, Byte data){
        banks[bankNo].program_memory[index] = data;
    }

    Byte* decodeInstruction(INS instruction){
        Byte OPCODE; // Instruction OPCODE
        Byte d; // Destination, 0 for AC, 1 for file register
        Byte addr; // Address, if d = 1
        Byte k; // Literal
        Byte b; // Bıts for bit manipulation
        Byte data[5];
        switch(instruction >> 12){
            case 0:{
                std::cout<<"BYTE ORIANTED FILE REGISTER OPERATIONS!"<<std::endl;
                OPCODE = (instruction >> 8);
                addr = (instruction & 0x008F);
                data[0]>>OPCODE;
                data[1]>>addr;
                return data;
            }break;
            default:{
				std::cout<<"INSTRUCTION NOT FETCHED PROPERLY!"<<std::endl;
			} break;
        }
    }

    INS Fetch(int &Cycles, int BankNo){
        Byte Data = banks[BankNo].program_memory[registers[0].value]; // Data where PCL points at
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

            Byte InstructionAddr = Fetch(Cycles, BankNo); // Fetch the instruction

            Byte* Instruction = decodeInstruction(InstructionAddr);

            switch(Instruction[0]) {
                case ADDWF: {
                    Byte Value = Fetch(Cycles, BankNo); // Fetch the data
                    if(ADDWF >> 7){ // d bit, store in data register if it is 1
                        Byte w_addr =  Instruction[1]; // 0-6 bits of the Instruction ADDWF is an address to store if it's to stored in memory.
                        banks[BankNo].data_memory[w_addr] = Value + registers[2].value; // Store the stored data + AC in w_addr
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
                    if(ANDWF >> 7){ // d bit, store in data register if it is 1
                        Byte w_addr =  Instruction[1]; // 0-6 bits of the Instruction ADDWF is an address to store if it's to stored in memory.
                        banks[BankNo].data_memory[w_addr] = Value & registers[2].value; // Store the stored data + AC in w_addr
                    }
                    else{ // d bit, store in AC if it is 0
                        registers[1].value = Value & registers[1].value; // Store it in AC
                    }
                }break;
                default:{
                    std::cout<<"INSTRUCTION NOT FETCHED PROPERLY!"<<std::endl;
                }break;
            }break;

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
                banks[0].data_memory[i] = 0;
            }
        }

        for(int i = 0; i < 4 ; i++){
            for(int k = 0; k < max_mem; k++){
                banks[0].program_memory[i] = 0;
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
    //PIC16F87.editRegister(0, 0, 0b01110111);
    PIC16F87.editProgramMemory(0, 0, 0b0011000011100000);
    PIC16F87.Execute(Cycles);
    PIC16F87.getBankDataInfo(0);
    std::cout<<"AC REG: "<<PIC16F87.getBankData(0, 1)<<std::endl;
    return 0;
}
