#include <iostream>
#include <vector>
#include <cstdint>
#include<bitset>
#include<map>

// WORK ON BANK SELECTION BASED ON THE ADDRESS TOP LIMIT(IF >128BYTE THEN SELECT NEXT BANK)



using Byte = uint8_t;
using INS = uint16_t;
static constexpr Byte max_mem = 128;
const int REG_SIZE = 3;
/*

class Register {
public:
    Byte value;
};
*/


class Register {
public:
    Register() : value(0) {}
    void setValue(uint8_t val) {
        value = val;
    }
    Byte getValue() const {
        return value;
    }

    Byte value;
};



class MemoryBanks {
public:
    //std::vector<Register> registers; // Registers are stored in the data memory but each bank has different set of registers hence the class attribution.
    std::vector<std::map<std::string, Register*>> registers;
    INS program_memory[max_mem];
    Byte data_memory[max_mem];
};




class CPU {

    std::vector<Register> registers;
    //////////////// REGISTER LIST ////////////////

    /*
     Some Special Function Registers are
     mirrored in multiple banks such as TRISD or PORTB.
    ie TRISD in bank x and TRISD in bank y store the exact same data.
    */
    Register AC;
    Register indraddr;
    Register TMR0;
    Register PCL; // address
    Register STATUS;
    Register FSR;
    Register PORTA;
    Register PORTB;
    Register PORTC;
    Register PORTD;
    Register PORTE;
    Register PCLATH; // subroutines
    Register INTCON;
    Register PIR1;
    Register PIR2;
    Register TMR1L;
    Register TM1HL;
    Register T1CON;
    Register TMR2;
    Register T2CON;
    Register SSPBUF;
    Register SSPCON;
    Register CCPR1L;
    Register CCPR1H;
    Register CCP1CON;
    Register RCSTA;
    Register TXREG;
    Register RCREG;
    Register CCPR2L;
    Register CCPR2H;
    Register CCP2CON;
    Register ADRESH;
    Register ADCON0;
    Register OPTION_REG;
    Register TRISA;
    Register TRISB;
    Register TRISC;
    Register TRISD;
    Register TRISE;
    Register PIE1;
    Register PIE2;
    Register PCON;
    Register EMPTYREG; // 0x00, empty bytes for unimplemented memories
    Register PR2;
    Register SSPADD;
    Register SSPSTAT;
    Register TXSTA;
    Register SPBRG;
    Register CMCON;
    Register CVRCON;
    Register ADRESL;
    Register ADCON1;
    Register EEDATA;
    Register EEADR;
    Register EEDATH;
    Register EEADRH;
    Register EECON1;
    Register EECON2;
    //////////////// REGISTER LIST ////////////////


    //// BYTE ORIENTED INSTRUCTIONS ////
    static constexpr Byte ADDWF = 0b00000111; // It loads to AC due to its 8th bit is 0. If its 1, it loads to LSB 7 bytes of address.
    static constexpr Byte ANDWF = 0b00000101; // Same as above(ADDWF)
    static constexpr Byte CLRF = 0b00000001; // Clear the data stored in the f(data memory) register
    static constexpr Byte COMF = 0b00001001; // Complements: 1010 -> 0101
    static constexpr Byte DECF = 0b00000011; // Decrements: 1010 - 0001
    static constexpr Byte DECFSZ = 0b00001011; // Decrements, skip if the result is zero
    static constexpr Byte INCF = 0b00001010; // Increments: 1010 + 1
    static constexpr Byte INCFSZ = 0b00001111; // Increments, skip if the result is zero
    static constexpr Byte IORWF = 0B00000100; // Inclusive OR W with f
    static constexpr Byte MOVF = 0b00001000; // Move f
    static constexpr Byte MOVWF = 0b00000000; // Move W to f
    static constexpr Byte RLF = 0b00001101; // Rotate f left through carry
    static constexpr Byte RRF = 0B00001100; // Rotate f right through carry
    //// BYTE ORIENTED INSTRUCTIONS ////



    //// BIT ORIENTED INSTRUCTIONS ////
    static constexpr Byte BCF = 0b0100;
    static constexpr Byte BSF = 0b0101;
    static constexpr Byte BTFSC = 0b0110;
    static constexpr Byte BTFSS = 0b0111;
    //// BIT ORIENTED INSTRUCTIONS ////


    //// LITERAL AND CONTROL OPS ////
    static constexpr Byte ADDLW = 0b00111110;
    static constexpr Byte ANDLW = 0b00111001;
    static constexpr Byte CALL = 0b1000;
    static constexpr Byte GOTO = 0b1001;
    static constexpr Byte IORLW = 0b00111000;
    static constexpr Byte MOVLW = 0b00110000;
    static constexpr Byte RETLW = 0b00110100;
    static constexpr Byte SUBLW = 0b00111100;
    static constexpr Byte XORLW = 0b00111010;
    //// LITERAL AND CONTROL OPS ////


public:
    CPU() {
        initiateRegisters();
        setMemoryBanks();
        Reset();
    }

    void getBankDataInfo(int bankNo){
        for(int i = 0; i<max_mem; i++){
            std::cout<<i<<". BYTE: "<<+banks[bankNo].data_memory[i]<<std::endl;
        }
    }

    void editProgramMemory(int bankNo, int index, INS data){
        banks[bankNo].program_memory[index] = data;
    }

    void setRegisterValue(MemoryBanks& bank, const std::string& registerName, Byte value) {
        for (std::map<std::string, Register*>& registers : bank.registers) {
            if (registers.find(registerName) != registers.end()) {
                registers[registerName]->setValue(value);
                return; // Exit the loop once the register is found and updated.
            }
        }

        // Handle the case where the register doesn't exist in any bank
        std::cout << "Register " << registerName << " not found in any bank." << std::endl;
        // You can handle the error as needed.
    }


    Byte getRegisterValue(const MemoryBanks& bank, const std::string& registerName) {
        for (const std::map<std::string, Register*>& registers : bank.registers) {
            if (registers.find(registerName) != registers.end()) {
                // Check if the register exists in the current bank's map
                return registers.at(registerName)->getValue();
            }
        }

        // Handle the case where the register doesn't exist in any bank
        std::cout << "Register " << registerName << " not found in any bank." << std::endl;
        // You can return a default value or handle the error as needed.
        return 0;
    }



    std::vector<Byte> decodeInstruction(INS instruction){
        int Type;
        Byte OPCODE; // Instruction OPCODE
        Byte d; // Destination, 0 for AC, 1 for file register
        Byte addr; // Address, if d = 1
        Byte control_k; // Literal for GOTO and CALL instructions
        Byte literal_k; // Literal for literal ops instructions
        Byte b; // Bıts for bit manipulation
        std::vector<Byte> data;
        switch(instruction >> 12){
            case 0:{ // Byte oriented file register operations
                std::cout<<"TYPE 0(BYTE ORIENTED OPS)"<<std::endl;
                Type = 0;
                OPCODE = (instruction >> 8);
                addr = (instruction & 0xFF);
                data.push_back(OPCODE);
                data.push_back(addr);
                data.push_back(Type);
                return data;
            }
            case 4:
            case 5:
            case 6:
            case 7:{ // Bit oriented file register operations
                std::cout<<"TYPE 1(BIT ORIENTED OPS)"<<std::endl;
                Type = 1;
                OPCODE = (instruction >> 10);
                addr = (instruction & 0x7F);
                b = ((instruction & 0b0000001110000000) >> 7);
                data.push_back(OPCODE);
                data.push_back(addr);
                data.push_back(b);
                data.push_back(Type);
                return data;
            }
            case 3:
            case 8:
            case 9 :{ // Literal and control operations
                std::cout<<"TYPE 3(LITERAL AND CONTROL OPS)"<<std::endl;
                Type = 2;
                OPCODE = (instruction >> 8);
                control_k = (instruction & 0b11111111111);
                literal_k = (instruction & 0xFF);
                data.push_back(OPCODE);
                data.push_back(control_k);
                data.push_back(literal_k);
                data.push_back(Type);
                return data;
            }
            /*
            case 9:{
                OPCODE = (instruction >> 12);
                k = (instruction & 0b11111111111);
                data.push_back(OPCODE);
                data.push_back(k);
            }break;
             */
            default:{
				std::cout<<"INSTRUCTION IS NOT DECODED PROPERLY!"<<std::endl;
			} break;
        }
    }

    INS Fetch(int &Cycles, int BankNo){
        INS Data = banks[BankNo].program_memory[getRegisterValue(banks[BankNo], "PCL")]; // Data where PCL points at
        if(Data == 0){
            Cycles = 0;
            return -1;
        }
        Byte PCL = getRegisterValue(banks[BankNo], "PCL");
        PCL++;
        setRegisterValue(banks[BankNo], "PCL", PCL);
        //banks[BankNo].registers[2].value++; // PCL++
        Cycles--;
        return Data;
    }


    /*
     I store registers of each bank independently,
     so in this function we ensure that
     registers data and the registers in the data memory are consistent
     */
    void updateRegistersOnBank(){
        for(int i = 0; i < 4; i++){
            int k = 0;
            for(const auto& entry: banks[i].registers[k]){
                const std::string& registerName = entry.first;
                banks[i].data_memory[k] = entry.second->getValue();
                k++;
            }
        }
    }


    void Execute(int &Cycles){
        int BankNo = 0;
        while(Cycles > 0){
            updateRegistersOnBank();
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
            if(Instruction.back() == 0){ // Byte oriented
                switch(Instruction[0] & 0xFF) {
                    //// Byte oriented instructions ////
                    case ADDWF: {
                        std::cout<<"ADDWF"<<std::endl;
                        Byte addr = Instruction[1] & 0x7F; // 0-6 bits of the Instruction ADDWF is an address to store if it's to stored in memory.
                        if(Instruction[1] >> 7){ // d bit, store in data register if it is 1
                            //banks[BankNo].data_memory[addr] = banks[BankNo].data_memory[addr] + banks[BankNo].registers.back().value; // Store the stored data + AC in w_addr
                            //banks[BankNo].data_memory[addr] = banks[BankNo].data_memory[addr] + banks[BankNo].registers["AC"]->getValue();
                            banks[BankNo].data_memory[addr] = banks[BankNo].data_memory[addr] + getRegisterValue(banks[BankNo], "AC");
                        }
                        else{ // d bit, store in AC if it is 0

                            //banks[BankNo].registers["AC"]->setValue((banks[BankNo].data_memory[addr]) + banks[BankNo].registers["AC"]->getValue()); // Store it in AC
                            Byte value = (banks[BankNo].data_memory[addr]) + getRegisterValue(banks[BankNo], "AC");
                            setRegisterValue(banks[BankNo], "AC", value);
                        }
                        /*
                        if(Value + registers[1].value > 0xFF){
                            registers[2].value | 0x01; // Set the Carry C flag of STATUS register if an overflow happens during the addition.
                            carry = registers[1].value - 0xFF;
                        }
                         */ // Probably not needed because the computer that will run this emulator will handle these carry bits anyway. Might look into this more in the future
                    }break;
                    case ANDWF:{
                        std::cout<<"ANDWF"<<std::endl;
                        Byte addr = Instruction[1] & 0x7F; // 0-6 bits of the Instruction ADDWF is an address to store if it's to stored in memory.
                        if(Instruction[1] >> 7){ // d bit, store in data register if it is 1
                            banks[BankNo].data_memory[addr] = banks[BankNo].data_memory[addr] & getRegisterValue(banks[BankNo], "AC"); // Store the stored data + AC in w_addr
                        }
                        else{ // d bit, store in AC if it is 0
                            //banks[BankNo].registers["AC"]->setValue((banks[BankNo].data_memory[addr]) & banks[BankNo].registers["AC"]->getValue()); // Store it in AC
                            Byte value = (banks[BankNo].data_memory[addr]) + getRegisterValue(banks[BankNo], "AC");
                            setRegisterValue(banks[BankNo], "AC", value);
                        }
                    }break;
                    case CLRF:{ // This will cover both CLRF and CLRW instructions as the only difference between them is the deleted register(d=0 W, d=1 f)
                        Byte addr = Instruction[1] & 0x7F;
                        if(Instruction[1] >> 7){ // d bit is 0, so we clear the f register
                            std::cout<<"CLRF"<<std::endl;
                            banks[BankNo].data_memory[addr] = 0;
                        }
                        else{ // d bit is 1, so we clear the AC register
                            std::cout<<"CLRW"<<std::endl;
                            setRegisterValue(banks[BankNo], "AC", 0);
                            //banks[BankNo].registers["AC"]->setValue(0);
                        }
                    }break;
                    case COMF:{
                        std::cout<<"COMF"<<std::endl;
                        Byte addr = Instruction[1] & 0x7F;
                        if(Instruction[1] >> 7){ // d bit is 0, so we store in the f register
                            banks[BankNo].data_memory[addr] = ~banks[BankNo].data_memory[addr];
                        }
                        else{ // d bit is 1, so we store in the AC register
                            Byte value = ~banks[BankNo].data_memory[addr];
                            setRegisterValue(banks[BankNo], "AC", value);
                            //banks[BankNo].registers["AC"].setValue(~banks[BankNo].data_memory[addr]);
                        }
                    }break;
                    case DECF:{
                        std::cout<<"DECF"<<std::endl;
                        Byte addr = Instruction[1] & 0x7F;
                        if(Instruction[1] >> 7){ // Store in the file register
                            banks[BankNo].data_memory[addr] = banks[BankNo].data_memory[addr] - 0b00000001;
                        }
                        else{ // Store in the AC
                            Byte value = banks[BankNo].data_memory[addr] - 0b00000001;
                            setRegisterValue(banks[BankNo], "AC", value);
                            //banks[BankNo].registers["AC"].setValue(banks[BankNo].data_memory[addr] - 0b00000001);
                        }
                    }break;
                    case DECFSZ:{
                        std::cout<<"DECFSZ"<<std::endl;
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
                            Byte value = banks[BankNo].data_memory[addr] - 0b00000001;
                            setRegisterValue(banks[BankNo], "AC", value);
                            //banks[BankNo].registers.back().value = banks[BankNo].data_memory[addr] - 0b00000001;
                        }
                    }break;
                    case INCF:{
                        std::cout<<"INCF"<<std::endl;
                        Byte addr = Instruction[1] & 0x7F;
                        if(Instruction[1] >> 7){ // Store in the file register
                            banks[BankNo].data_memory[addr] = banks[BankNo].data_memory[addr] + 0b00000001;
                        }
                        else{ // Store in the AC
                            Byte value = banks[BankNo].data_memory[addr] + 0b00000001;
                            setRegisterValue(banks[BankNo], "AC", value);
                            //banks[BankNo].registers.back().value = banks[BankNo].data_memory[addr] + 0b00000001;
                        }
                    }break;
                    case INCFSZ:{
                        std::cout<<"INCFSZ"<<std::endl;
                        Byte addr = Instruction[1] & 0x7F;
                        if(Instruction[1] >> 7){
                            if((banks[BankNo].data_memory[addr] + 0b00000001) == 0){
                                break;
                            }
                            banks[BankNo].data_memory[addr] = banks[BankNo].data_memory[addr] + 0b00000001;
                        }
                        else{
                            if((banks[BankNo].data_memory[addr] + 0b00000001) == 0){
                                break;
                            }
                            Byte value = banks[BankNo].data_memory[addr] + 0b00000001;
                            setRegisterValue(banks[BankNo], "AC", value);
                            //banks[BankNo].registers.back().value = banks[BankNo].data_memory[addr] + 0b00000001;
                        }
                    }break;
                    case IORWF:{
                        std::cout<<"IORWF"<<std::endl;
                        Byte addr = Instruction[1] & 0x7F;
                        if(Instruction[1] >> 7){
                            banks[BankNo].data_memory[addr] = getRegisterValue(banks[BankNo], "AC") | banks[BankNo].data_memory[addr];
                        }
                        else{
                            Byte value = getRegisterValue(banks[BankNo], "AC") | banks[BankNo].data_memory[addr];
                            setRegisterValue(banks[BankNo], "AC", value);
                            //banks[BankNo].registers.back().value = banks[BankNo].registers.back().value | banks[BankNo].data_memory[addr];;
                        }
                    }break;
                    case MOVF:{
                        std::cout<<"MOVF"<<std::endl;
                        Byte addr = Instruction[0] & 0x7F;
                        if(Instruction[1] >> 7){ // f->f, in other words it does nothing. Good for testing the Z register.
                            continue;
                        }
                        else{
                            setRegisterValue(banks[BankNo], "AC", banks[BankNo].data_memory[addr]);
                        }
                    }break;
                    case MOVWF:{
                        std::cout<<"MOVWF"<<std::endl;
                        Byte addr = Instruction[0] & 0x7F;
                        banks[BankNo].data_memory[addr] = getRegisterValue(banks[BankNo], "AC");
                    }break;
                    case RLF:{
                        std::cout<<"RLF"<<std::endl;
                        Byte addr = Instruction[0] & 0x7F;
                        Byte c = (banks[BankNo].data_memory[addr] >> 6); // Get the new C(carry) bit depends on the MSB of data stored in f register
                        if(c == 0){ // Clear the C(carry) bit of STATUS register if the c is 0
                            Byte newStatusReg = getRegisterValue(banks[BankNo], "STATUS") & 0xFE;
                            setRegisterValue(banks[BankNo], "STATUS", newStatusReg);
                        }
                        else if(c == 1){ // Set the C(carry) bit of STATUS register if the c is 1
                            Byte newStatusReg = getRegisterValue(banks[BankNo], "STATUS") | 0x01;
                            setRegisterValue(banks[BankNo], "STATUS", newStatusReg);
                        }
                        Byte newValue = (banks[BankNo].data_memory[addr] << 1); // Rotated to the left(left bitshift)
                        if(Instruction[1] >> 7){ // Store in f register
                            banks[BankNo].data_memory[addr] = newValue;
                        }
                        else{ // Store in AC
                            setRegisterValue(banks[BankNo], "AC", newValue);
                        }
                    }break;
                    case RRF:{
                        std::cout<<"RRF"<<std::endl;
                        Byte addr = Instruction[0] & 0x7F;
                        Byte c = (banks[BankNo].data_memory[addr] << 6); // Get the new C(carry) bit depends on the MSB of data stored in f register
                        c = (c >> 6);
                        if(c == 0){ // Clear the C(carry) bit of STATUS register if the c is 0
                            Byte newStatusReg = getRegisterValue(banks[BankNo], "STATUS") & 0xFE;
                            setRegisterValue(banks[BankNo], "STATUS", newStatusReg);
                        }
                        else if(c == 1){ // Set the C(carry) bit of STATUS register if the c is 1
                            Byte newStatusReg = getRegisterValue(banks[BankNo], "STATUS") | 0x01;
                            setRegisterValue(banks[BankNo], "STATUS", newStatusReg);
                        }
                        Byte newValue = (banks[BankNo].data_memory[addr] >> 1); // Rotated to the left(left bitshift)
                        if(Instruction[1] >> 7){ // Store in f register
                            banks[BankNo].data_memory[addr] = newValue;
                        }
                        else{ // Store in AC
                            setRegisterValue(banks[BankNo], "AC", newValue);
                        }
                    }break;
                        //// Byte oriented instructions ////

                    default:{
                        std::cout<<"BYTE INSTRUCTION NOT FETCHED PROPERLY!"<<std::endl;
                    }break;
                }
            }
            else if(Instruction.back() == 1){ // Bit oriented
                switch(Instruction[0] & 0xFF){
                    case BCF:{ // Clear b bit of f reg
                        std::cout<<"BCF"<<std::endl;
                        Byte addr = Instruction[1];
                        int b = Instruction[2];
                        banks[BankNo].data_memory[addr] &= ~(1 << b);
                    }break;
                    case BSF:{ // Set b bit of f reg
                        std::cout<<"BSF"<<std::endl;
                        Byte addr = Instruction[1];
                        int b = Instruction[2];
                        banks[BankNo].data_memory[addr] |= (1 << b);
                    }break;

                    case BTFSC:{
                        std::cout<<"BTFSC"<<std::endl;
                        Byte addr = Instruction[1];
                        int b = Instruction[2];
                        if(banks[BankNo].data_memory[addr] & ~(1 << b)){
                            Cycles++;
                            Fetch(Cycles, BankNo);
                        }
                        else{
                            // NOP instruction
                        }
                    }break;
                    case BTFSS:{
                        std::cout<<"BTFSS"<<std::endl;
                        Byte addr = Instruction[1];
                        int b = Instruction[2];
                        if(banks[BankNo].data_memory[addr] | (1 << b)){
                            Cycles++;
                            Fetch(Cycles, BankNo);
                        }
                    }
                    default:
                        std::cout<<"UNABLE TO FETCH THE INSTRUCTION"<<std::endl;
                }
            }
            else if(Instruction.back() == 2){
                switch(Instruction[0] & 0xFF){
                    case ADDLW:{
                        std::cout<<"ADDLW"<<std::endl;
                        Byte value = (Instruction[2]) + getRegisterValue(banks[BankNo], "AC");
                        setRegisterValue(banks[BankNo], "AC", value);
                        //banks[BankNo].registers.back().value = Instruction[2] + banks[BankNo].registers.back().value;
                    }break;
                    case ANDLW:{
                        std::cout<<"ANDLW"<<std::endl;
                        Byte value = (Instruction[2]) & getRegisterValue(banks[BankNo], "AC");
                        setRegisterValue(banks[BankNo], "AC", value);
                        //banks[BankNo].registers.back().value = Instruction[2] & banks[BankNo].registers.back().value;
                    }break;
                    case GOTO:{
                        std::cout<<"GOTO"<<std::endl;
                        setRegisterValue(banks[BankNo], "PCL", Instruction[1]);
                    }break;
                    case CALL:{
                        std::cout<<"CALL"<<std::endl;
                        // Implement CALL
                    } break;
                    case IORLW:{
                        std::cout<<"IORLW"<<std::endl;
                        Byte value = Instruction[2] | getRegisterValue(banks[BankNo], "AC");
                        setRegisterValue(banks[BankNo], "AC", value);
                        //banks[BankNo].registers.back().value = Instruction[2] | banks[BankNo].registers.back().value;
                    }break;
                    case MOVLW:{
                        std::cout<<"MOVLW"<<std::endl;
                        setRegisterValue(banks[BankNo], "AC", Instruction[2]);
                        //banks[BankNo].registers.back().value =  Instruction[2];
                    }break;
                    case SUBLW:{
                        std::cout<<"SUBLW"<<std::endl;
                        Byte value = Instruction[2] - getRegisterValue(banks[BankNo], "AC");
                        setRegisterValue(banks[BankNo], "AC", value);
                        //banks[BankNo].registers.back().value = Instruction[2] - banks[BankNo].registers.back().value;
                    }
                    default:
                        std::cout<<"UNABLE TO FETCH THE LITERAL & CONTROL INSTRUCTION!"<<std::endl;
                }
            }
        }
            }
private:
    std::vector<MemoryBanks> banks;

    void initiateRegisters() {
        PCL.value = 0x00;
        PCLATH.value = 0x00;
        AC.value = 0x0F;
        STATUS.value = 0b1000000;
        registers.push_back(PCL);
        registers.push_back(PCLATH);
        registers.push_back(AC);
        registers.push_back(STATUS);
    }

    void setMemoryBanks() {
        //////// BANK 0 ////////
        MemoryBanks bank0;
        std::map<std::string, Register*> bank0Registers;
        bank0Registers["indraddr"] = &indraddr;
        bank0Registers["TMR0"] = &TMR0;
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
        bank0Registers["TM1HL"] = &TM1HL;
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
        bank0Registers["AC"] = &AC;

        bank0.registers.push_back(bank0Registers);
        //////// BANK 0 ////////

        //////// BANK 1 ////////
        MemoryBanks bank1;
        std::map<std::string, Register*> bank1Registers;

        bank1Registers["indraddr"] = &indraddr;
        bank1Registers["PCL"] = &PCL;
        bank1Registers["STATUS"] = &STATUS;
        bank1Registers["FSR"] = &FSR;
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
        bank1Registers["SSPCON"] = &SSPCON;
        bank1Registers["PR2"] = &PR2;
        bank1Registers["SSPADD"] = &SSPADD;
        bank1Registers["SSPSTAT"] = &SSPSTAT;
        bank1Registers["EMPTYREG1"] = &EMPTYREG;
        bank1Registers["EMPTYREG2"] = &EMPTYREG;
        bank1Registers["EMPTYREG3"] = &EMPTYREG;
        bank1Registers["TXSTA"] = &TXSTA;
        bank1Registers["SPBRG"] = &SPBRG;
        bank1Registers["EMPTYREG4"] = &EMPTYREG;
        bank1Registers["EMPTYREG5"] = &EMPTYREG;
        bank1Registers["CMCON"] = &CMCON;
        bank1Registers["CVRCON"] = &CVRCON;
        bank1Registers["ADRESL"] = &ADRESL;
        bank1Registers["ADCON1"] = &ADCON1;
        bank1Registers["AC"] = &AC;

        bank1.registers.push_back(bank1Registers);

        //////// BANK 1 ////////

        //////// BANK 2 ////////
        MemoryBanks bank2;
        std::map<std::string, Register*> bank2Registers;

        bank2Registers["indraddr"] = &indraddr;
        bank2Registers["TMR0"] = &TMR0;
        bank2Registers["PCL"] = &PCL;
        bank2Registers["STATUS"] = &STATUS;
        bank2Registers["FSR"] = &FSR;
        bank2Registers["EMPTYREG1"] = &EMPTYREG;
        bank2Registers["PORTB"] = &PORTB;
        bank2Registers["EMPTYREG2"] = &EMPTYREG;
        bank2Registers["EMPTYREG3"] = &EMPTYREG;
        bank2Registers["PCLATH"] = &PCLATH;
        bank2Registers["INTCON"] = &INTCON;
        bank2Registers["EEDATA"] = &EEDATA;
        bank2Registers["EEADR"] = &EEADR;
        bank2Registers["EEDATH"] = &EEDATH;
        bank2Registers["EEADRH"] = &EEADRH;
        bank2Registers["AC"] = &AC;

        bank2.registers.push_back(bank2Registers);

        //////// BANK 2 ////////

        //////// BANK 3 ////////
        MemoryBanks bank3;
        std::map<std::string, Register*> bank3Registers;

        bank3Registers["indraddr"] = &indraddr;
        bank3Registers["OPTION_REG"] = &OPTION_REG;
        bank3Registers["PCL"] = &PCL;
        bank3Registers["STATUS"] = &STATUS;
        bank3Registers["FSR"] = &FSR;
        bank3Registers["EMPTYREG1"] = &EMPTYREG;
        bank3Registers["TRISB"] = &TRISB;
        bank3Registers["EMPTYREG2"] = &EMPTYREG;
        bank3Registers["EMPTYREG3"] = &EMPTYREG;
        bank3Registers["PCLATH"] = &PCLATH;
        bank3Registers["INTCON"] = &INTCON;
        bank3Registers["EECON1"] = &EECON1;
        bank3Registers["EECON2"] = &EECON2;
        bank3Registers["EMPTYREG4"] = &EMPTYREG; // Reserved memory, should be empty
        bank3Registers["EMPTYREG5"] = &EMPTYREG; // Reserved memory, should be empty
        bank3Registers["AC"] = &AC;

        bank3.registers.push_back(bank3Registers);

        //////// BANK 3 ////////

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
        AC.value = 0x0F;
        STATUS.value = 0b00011000;
    }
};

int main() {
    CPU PIC16F87;
    int Cycles = 3;

    PIC16F87.editProgramMemory(0, 0, 0b0011111000000001); // instruction(ADDLW)
    PIC16F87.editProgramMemory(0, 1, 0b0000000011111111); // instruction(MOVWF)
    PIC16F87.editProgramMemory(0, 2, 0b0000000111100000); // instruction(CLRF)

    PIC16F87.Execute(Cycles);
    //PIC16F87.getBankDataInfo(0);
    return 0;
}
