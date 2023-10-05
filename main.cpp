#include <iostream>
#include <vector>
#include <cstdint>

using Byte = uint8_t;
static constexpr Byte max_mem = 128;
const int REG_SIZE = 3;

////////

class Register {
public:
    Byte value;
};

class MemoryBanks {
public:
    std::vector<Register> registers;
    Byte data[max_mem];
};

class CPU {
    Register PC;
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

    static constexpr Byte ADDWF = 0b01110111; // It loads to AC due to its 4th bit is 1. If its 0, it loads to LSB 2 bytes of address.
    static constexpr Byte ANDWF = 0b01010111;
    //// INSTRUCTIONS

public:
    CPU() {
        initiateRegisters();
        setMemoryBanks();
        Reset();
    }

    int getBankData(int bankNo, int registerNo){
        return banks[bankNo].registers[registerNo].value;
    }

    int getBankDataSize(int bankNo){
        return sizeof(banks[bankNo].data);
    }

    void getBankDataInfo(int bankNo){
        for(int i = 0; i<max_mem; i++){
            std::cout<<i<<". BYTE: "<<+banks[bankNo].data[i]<<std::endl;
        }
    }

    void editBankData(int bankNo, int regNo, Byte data){
        banks[bankNo].data[regNo] = data;
    }

    Byte getRegister(int regNo){
        return registers[regNo].value;
    }

    void editRegister(int bankNo, int regNo, Byte data){
        banks[bankNo].registers[regNo].value = data;
    }

    Byte FetchInstruction(int &Cycles){
        Byte Data = banks[0].registers[0].value; // PC
        banks[0].registers[0].value++; // PC++
        Cycles--;
        return Data;
    }
    void Execute(int &Cycles){
        while(Cycles > 0){
            Byte Instruction = FetchInstruction(Cycles); // Fetch the instruction
            switch(Instruction & 0xFF) {
                case ADDWF: {
                    Byte Value = FetchInstruction(Cycles); // Fetch the data
                    if(ADDWF & (1 << 4)){
                        Byte w_addr =  Instruction & 0x0F; // First 4 bits(LSB) of the Instruction ADDWF is an address to store if it's to stored in memory.
                        banks[0].data[w_addr] = Value + banks[0].registers[1].value; // Store the stored data + AC in w_addr
                    }
                    else{
                        banks[0].registers[1].value = Value + banks[0].registers[1].value; // Store it in AC
                    }
                    if(Value + banks[0].registers[1].value > 0xFF){
                        banks[0].registers[2].value | 0x01; // Set the Carry C flag of STATUS register if an overflow happens during the addition.
                    }
                }break;
                case ANDWF:{
                    Byte Value = FetchInstruction(Cycles);
                    if(ADDWF & (1 << 4)){
                        Byte w_addr =  Instruction & 0x0F; // First 4 bits(LSB) of the Instruction ANDWF is an address to store if it's to stored in memory.
                        banks[0].data[w_addr] = Value & banks[0].registers[1].value; // Store the stored data + AC in w_addr
                    }
                    else{
                        banks[0].registers[1].value = Value & banks[0].registers[1].value; // Store it in AC
                    }
                }break;
                default:{
                    std::cout<<"INSTRUCTION NOT FETCHED PROPERLY!"<<std::endl;
                }break;

            }break;

        }
    }
private:
    std::vector<Register> registers;
    std::vector<MemoryBanks> banks;

    void initiateRegisters() {
        PC.value = 0x0C;
        AC.value = 0x0F;
        STATUS.value = (1 << 7) | (0 << 6) | (0 << 5) | (0 << 4) | (0 << 3) | (0 << 2) | (0 << 1) | Byte(0);
        registers.push_back(PC);
        registers.push_back(AC);
        registers.push_back(STATUS);
    }

    void setMemoryBanks() {
        MemoryBanks bank1;
        bank1.registers = registers;
        /*
        for(int i = 0; i < max_mem ; i++){
            if(registers.size() > i){
                bank1.data[i] = registers[i].value;
            }
            else{
                bank1.data[i] = 0;
            }
        }*/ // PC AND AC REGISTERS ARE NOT SUPPOSED TO BE IN THE MEMORY BANK.

        for(int i = 0; i < max_mem; i++){
            bank1.data[i] = 0;
        }

        banks.push_back(bank1);
    }


    void Reset(){
        for(int i = 0; i < max_mem ; i++){
            banks[0].data[i] = 0;
        }
        PC.value = 0x00;
        AC.value = 0x0B;
        IRP = RP1 = RP0 = Z = DC = C = 0;
        TO = PD = 1;
        STATUS.value = (IRP << 7) | (RP1 << 6) | (RP0 << 5) | (TO << 4) | (PD << 3) | (Z << 2) | (DC << 1) | C;
    }
};


int main() {
    CPU PIC16F87;
    int Cycles = 3;
    PIC16F87.editRegister(0, 0, 0b01110111);
    PIC16F87.Execute(Cycles);
    PIC16F87.getBankDataInfo(0);
    std::cout<<"AC REG: "<<PIC16F87.getBankData(0, 1)<<std::endl;
    return 0;
}
