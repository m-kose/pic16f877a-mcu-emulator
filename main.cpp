#include <iostream>
#include <vector>
#include <cstdint>

using Byte = uint8_t;
using Opcode = uint16_t;
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

    static constexpr Byte ADDWF = 0xA0; // It loads to AC due to its 9th MSB is 1.



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

    Byte FetchInstruction(int &Cycles){
        Byte Data = banks[0].data[0]; // PC
        banks[0].data[0]++; // PC++
        Cycles--;
        return Data;
    }
    void Execute(int &Cycles){
        while(Cycles > 0){
            Byte Instruction = FetchInstruction(Cycles); // Fetch the instruction
            switch(Instruction) {
                case ADDWF: {
                    Byte Value = FetchInstruction(Cycles); // Fetch the data
                    banks[0].data[1] = Value + banks[0].data[0];
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
        PC.value = 0x0F;
        AC.value = 0x0A;
        STATUS.value = (1 << 7) | (0 << 6) | (0 << 5) | (0 << 4) | (0 << 3) | (0 << 2) | (0 << 1) | Byte(0);
        registers.push_back(PC);
        registers.push_back(AC);
        registers.push_back(STATUS);
    }

    void setMemoryBanks() {
        MemoryBanks bank1;
        bank1.registers = registers;
        for(int i = 0; i < max_mem ; i++){
            if(registers.size() > i){
                bank1.data[i] = registers[i].value;
            }
            else{
                bank1.data[i] = 0;
            }
        }
        banks.push_back(bank1);
    }


    void Reset(){
        for(int i = 0; i < max_mem ; i++){
            if(i < REG_SIZE){
                continue;
            }
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
    /*
    std::cout << +PIC16F87.getBankData(0, 0) << std::endl;
    std::cout << +PIC16F87.getBankDataSize(0) << std::endl;
    PIC16F87.getBankDataInfo(0);
    std::cout<<PIC16F87.getRegister(0)<<std::endl;
    */
    PIC16F87.editBankData(0, 0, 0xA0);
    PIC16F87.Execute(Cycles);
    PIC16F87.getBankDataInfo(0);
    return 0;
}
