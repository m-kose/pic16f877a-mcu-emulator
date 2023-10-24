#include <iostream>
#include <vector>
#include <cstdint>
#include<bitset>


// WORK ON BANK SELECTION BASED ON THE ADDRESS TOP LIMIT(IF >128BYTE THEN SELECT NEXT BANK)



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
    std::vector<Register> registers; // Registers are stored in the data memory but each bank has different set of registers hence the class attribution.
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
    Register indraddr = {0x00};
    Register TMR0;
    Register PCL = {0x00}; // address
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
    Register EMPTYREG = {0x00}; // 0x00, empty bytes for unimplemented memories
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

    void editRegisters(int bankNo, int regNo, Byte data){
        banks[bankNo].registers[regNo].value = data;
    }

    void editProgramMemory(int bankNo, int index, INS data){
        banks[bankNo].program_memory[index] = data;
    }

    void getRegData(int bankNo, int regNo){
        std::cout<<"DATA: "<<+banks[bankNo].registers[regNo].value;
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
                if(addr > 64){
                    switch(uint(addr / 64)){
                        case 2:{
                            registers[2].value &= 0b10111111;
                            std::cout<<"BANK 1"<<std::endl;
                        }break;
                        case 3:{
                            registers[2].value &= 0b11011111;
                            std::cout<<"BANK 2"<<std::endl;
                        }break;
                        case 4:{
                            registers[2].value &= 0b11111111;
                            std::cout<<"BANK 3"<<std::endl;
                        }break;
                    }
                }
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
                if(addr > 64){
                    switch(uint(addr / 64)){
                        case 2:{
                            registers[2].value &= 0b10111111;
                            std::cout<<"BANK 1"<<std::endl;
                        }break;
                        case 3:{
                            registers[2].value &= 0b11011111;
                            std::cout<<"BANK 2"<<std::endl;
                        }break;
                        case 4:{
                            registers[2].value &= 0b11111111;
                            std::cout<<"BANK 3"<<std::endl;
                        }break;
                    }
                }
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
        INS Data = banks[BankNo].program_memory[banks[BankNo].registers[2].value]; // Data where PCL points at
        banks[BankNo].registers[2].value++; // PCL++
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
            for(int k = 0; k<banks[i].registers.size(); k++){
                banks[i].data_memory[k] = banks[i].registers[k].value;
            }
        }
    }


    void Execute(int &Cycles){
        updateRegistersOnBank();
        while(Cycles > 0){
            int BankNo = 0;
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
                            banks[BankNo].data_memory[addr] = banks[BankNo].data_memory[addr] + banks[BankNo].registers.back().value; // Store the stored data + AC in w_addr
                        }
                        else{ // d bit, store in AC if it is 0
                            banks[BankNo].registers.back().value = banks[BankNo].data_memory[addr] + banks[BankNo].registers.back().value; // Store it in AC
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
                            banks[BankNo].data_memory[addr] = banks[BankNo].data_memory[addr] & banks[BankNo].registers.back().value; // Store the stored data + AC in w_addr
                        }
                        else{ // d bit, store in AC if it is 0
                            banks[BankNo].registers.back().value = banks[BankNo].data_memory[addr] & banks[BankNo].registers.back().value; // Store it in AC
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
                            banks[BankNo].registers.back().value = 0;
                        }
                    }break;
                    case COMF:{
                        std::cout<<"COMF"<<std::endl;
                        Byte addr = Instruction[1] & 0x7F;
                        if(Instruction[1] >> 7){ // d bit is 0, so we store in the f register
                            banks[BankNo].data_memory[addr] = ~banks[BankNo].data_memory[addr];
                        }
                        else{ // d bit is 1, so we store in the AC register
                            banks[BankNo].registers.back().value = ~banks[BankNo].data_memory[addr];
                        }
                    }break;
                    case DECF:{
                        std::cout<<"DECF"<<std::endl;
                        Byte addr = Instruction[1] & 0x7F;
                        if(Instruction[1] >> 7){ // Store in the file register
                            banks[BankNo].data_memory[addr] = banks[BankNo].data_memory[addr] - 0b00000001;
                        }
                        else{ // Store in the AC
                            banks[BankNo].registers.back().value = banks[BankNo].data_memory[addr] - 0b00000001;
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
                            banks[BankNo].registers.back().value = banks[BankNo].data_memory[addr] - 0b00000001;
                        }
                    }break;
                    case INCF:{
                        std::cout<<"INCF"<<std::endl;
                        Byte addr = Instruction[1] & 0x7F;
                        if(Instruction[1] >> 7){ // Store in the file register
                            banks[BankNo].data_memory[addr] = banks[BankNo].data_memory[addr] + 0b00000001;
                        }
                        else{ // Store in the AC
                            banks[BankNo].registers.back().value = banks[BankNo].data_memory[addr] + 0b00000001;
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
                            banks[BankNo].registers.back().value = banks[BankNo].data_memory[addr] + 0b00000001;
                        }
                    }break;
                    case IORWF:{
                        std::cout<<"IORWF"<<std::endl;
                        Byte addr = Instruction[1] & 0x7F;
                        if(Instruction[1] >> 7){
                            banks[BankNo].data_memory[addr] = banks[BankNo].registers.back().value | banks[BankNo].data_memory[addr];
                        }
                        else{
                            banks[BankNo].registers.back().value = banks[BankNo].registers.back().value | banks[BankNo].data_memory[addr];;
                        }
                    }break;
                    case MOVF:{
                        std::cout<<"MOVF"<<std::endl;
                        Byte addr = Instruction[0] & 0x7F;
                        if(Instruction[1] >> 7){ // f->f, in other words it does nothing. Good for testing the Z register.
                            continue;
                        }
                        else{
                            banks[BankNo].registers.back().value = banks[BankNo].data_memory[addr];
                        }
                    }break;
                    case MOVWF:{
                        std::cout<<"MOVWF"<<std::endl;
                        Byte addr = Instruction[0] & 0x7F;
                        banks[BankNo].data_memory[addr] = banks[BankNo].registers.back().value;
                    }break;
                    case RLF:{
                        // Implement Rotate f register through carry. : 1011 0111 -> 0110 1110 & C = 1
                    }break;
                        //// Byte oriented instructions ////

                        //// Bit oriented instructions ////
                    default:{
                        std::cout<<"BYTE INSTRUCTION NOT FETCHED PROPERLY!"<<std::endl;
                    }break;
                }
            }
            else if(Instruction.back() == 1){ // Bit oriented
                switch(Instruction[0] & 0xFF){
                    case BCF:{
                        std::cout<<"BCF"<<std::endl;
                        Byte addr = Instruction[1];
                        int b = Instruction[2];
                        banks[BankNo].data_memory[addr] &= ~(1 << b);
                    }break;
                    case BSF:{
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
                        banks[BankNo].registers.back().value = Instruction[2];
                    }break;
                    case ANDLW:{
                        std::cout<<"ANDLW"<<std::endl;
                        banks[BankNo].registers.back().value = Instruction[2] & banks[BankNo].registers.back().value;
                    }break;
                    case GOTO:{
                        std::cout<<"GOTO"<<std::endl;
                        banks[BankNo].registers[2].value = Instruction[1];
                    }break;
                    case CALL:{
                        std::cout<<"CALL"<<std::endl;
                        // Implement CALL
                    } break;
                    case IORLW:{
                        std::cout<<"IORLW"<<std::endl;
                        banks[BankNo].registers.back().value = Instruction[2] | banks[BankNo].registers.back().value;
                    }break;
                    case MOVLW:{
                        std::cout<<"MOVLW"<<std::endl;
                        banks[BankNo].registers.back().value =  Instruction[2];
                    }break;
                    case SUBLW:{
                        std::cout<<"SUBLW"<<std::endl;
                        banks[BankNo].registers.back().value = Instruction[2] - banks[BankNo].registers.back().value;
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
        STATUS.value = (1 << 7) | (0 << 6) | (0 << 5) | (0 << 4) | (0 << 3) | (0 << 2) | (0 << 1) | Byte(0);
        registers.push_back(PCL);
        registers.push_back(PCLATH);
        registers.push_back(AC);
        registers.push_back(STATUS);
    }

    void setMemoryBanks() {
        //////// BANK 0 ////////
        MemoryBanks bank0;
        bank0.registers.push_back(indraddr); // @00h
        bank0.registers.push_back(TMR0);
        bank0.registers.push_back(PCL);
        bank0.registers.push_back(STATUS);
        bank0.registers.push_back(FSR);
        bank0.registers.push_back(PORTA);
        bank0.registers.push_back(PORTB);
        bank0.registers.push_back(PORTC);
        bank0.registers.push_back(PORTD);
        bank0.registers.push_back(PORTE);
        bank0.registers.push_back(PCLATH);
        bank0.registers.push_back(INTCON);
        bank0.registers.push_back(PIR1);
        bank0.registers.push_back(PIR2);
        bank0.registers.push_back(TMR1L);
        bank0.registers.push_back(TM1HL);
        bank0.registers.push_back(T1CON);
        bank0.registers.push_back(TMR2);
        bank0.registers.push_back(T2CON);
        bank0.registers.push_back(SSPBUF);
        bank0.registers.push_back(SSPCON);
        bank0.registers.push_back(CCPR1L);
        bank0.registers.push_back(CCPR1H);
        bank0.registers.push_back(CCP1CON);
        bank0.registers.push_back(RCSTA);
        bank0.registers.push_back(TXREG);
        bank0.registers.push_back(RCREG);
        bank0.registers.push_back(CCPR2L);
        bank0.registers.push_back(CCPR2H);
        bank0.registers.push_back(CCP2CON);
        bank0.registers.push_back(ADRESH);
        bank0.registers.push_back(ADCON0);
        bank0.registers.push_back(AC); // 20h
        //////// BANK 0 ////////

        //////// BANK 1 ////////
        MemoryBanks bank1;

        bank1.registers.push_back(indraddr); // 80h
        bank1.registers.push_back(PCL);
        bank1.registers.push_back(STATUS);
        bank1.registers.push_back(FSR);
        bank1.registers.push_back(TRISA);
        bank1.registers.push_back(TRISB);
        bank1.registers.push_back(TRISC);
        bank1.registers.push_back(TRISD);
        bank1.registers.push_back(TRISE);
        bank1.registers.push_back(PCLATH);
        bank1.registers.push_back(INTCON);
        bank1.registers.push_back(PIE1);
        bank1.registers.push_back(PIE2);
        bank1.registers.push_back(PCON);
        bank1.registers.push_back(SSPCON);
        bank1.registers.push_back(PR2);
        bank1.registers.push_back(SSPADD);
        bank1.registers.push_back(SSPSTAT);
        bank1.registers.push_back(EMPTYREG);
        bank1.registers.push_back(EMPTYREG);
        bank1.registers.push_back(EMPTYREG);
        bank1.registers.push_back(TXSTA);
        bank1.registers.push_back(SPBRG);
        bank1.registers.push_back(EMPTYREG);
        bank1.registers.push_back(EMPTYREG);
        bank1.registers.push_back(CMCON);
        bank1.registers.push_back(CVRCON);
        bank1.registers.push_back(ADRESL);
        bank1.registers.push_back(ADCON1);
        bank1.registers.push_back(AC); // A0h
        //////// BANK 1 ////////

        //////// BANK 2 ////////
        MemoryBanks bank2;

        bank2.registers.push_back(indraddr); // 100H
        bank2.registers.push_back(TMR0);
        bank2.registers.push_back(PCL);
        bank2.registers.push_back(STATUS);
        bank2.registers.push_back(FSR);
        bank2.registers.push_back(EMPTYREG);
        bank2.registers.push_back(PORTB);
        bank2.registers.push_back(EMPTYREG);
        bank2.registers.push_back(EMPTYREG);
        bank2.registers.push_back(EMPTYREG);
        bank2.registers.push_back(PCLATH);
        bank2.registers.push_back(INTCON);
        bank2.registers.push_back(EEDATA);
        bank2.registers.push_back(EEADR);
        bank2.registers.push_back(EEDATH);
        bank2.registers.push_back(EEADRH);
        bank2.registers.push_back(AC); // 110H
        //////// BANK 2 ////////

        //////// BANK 3 ////////
        MemoryBanks bank3;

        bank3.registers.push_back(indraddr); // 180h
        bank3.registers.push_back(OPTION_REG);
        bank3.registers.push_back(PCL);
        bank3.registers.push_back(STATUS);
        bank3.registers.push_back(FSR);
        bank3.registers.push_back(EMPTYREG);
        bank3.registers.push_back(TRISB);
        bank3.registers.push_back(EMPTYREG);
        bank3.registers.push_back(EMPTYREG);
        bank3.registers.push_back(EMPTYREG);
        bank3.registers.push_back(PCLATH);
        bank3.registers.push_back(INTCON);
        bank3.registers.push_back(EECON1);
        bank3.registers.push_back(EECON1);
        bank3.registers.push_back(EMPTYREG); // Reserved memory, should be empty
        bank3.registers.push_back(EMPTYREG); // Reserved memory, should be empty
        bank3.registers.push_back(AC); // 190h
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
        AC.value = 0x0B;
        IRP = RP1 = RP0 = Z = DC = C = 0;
        TO = PD = 1;
        STATUS.value = (IRP << 7) | (RP1 << 6) | (RP0 << 5) | (TO << 4) | (PD << 3) | (Z << 2) | (DC << 1) | C;
    }
};

int main() {
    CPU PIC16F87;
    int Cycles = 3;


    PIC16F87.editProgramMemory(0, 0, 0b0011111000000001); // instruction(ADDLW)
    PIC16F87.editProgramMemory(0, 1, 0b0000000011111111); // instruction(MOVWF)
    PIC16F87.editProgramMemory(0, 2, 0b0000000111100000); // instruction(CLRF)


    PIC16F87.Execute(Cycles);
    //PIC16F87.getBankDataInfo(2);
    //PIC16F87.getRegData(0, 32); // AC value
    //std::cout<<"AC REG: "<<PIC16F87.getBankData(0, 2)<<std::endl;
    return 0;
}
