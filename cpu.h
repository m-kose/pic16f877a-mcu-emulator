#ifndef CPU_H
#define CPU_H

#include <vector>
#include <string>
#include <cstdint> // For uint8_t, uint16_t
#include "memory_banks.h" // Defines MemoryBanks, INS, Byte, max_mem
// register.h is included by memory_banks.h

// Using declarations for types defined in other headers, if preferred
// using Byte = uint8_t;
// using INS = uint16_t;

class CPU {
public:
    CPU();
    void getBankDataInfo(int bankNo);
    void editProgramMemory(int bankNo, int index, INS data);
    void setRegisterValue(MemoryBanks& bank, const std::string& registerName, Byte value);
    Byte getRegisterValue(const MemoryBanks& bank, const std::string& registerName);
    std::vector<Byte> decodeInstruction(INS instruction);
    INS Fetch(int &Cycles, int BankNo);
    void updateRegistersOnBank();
    void Execute(int &Cycles);
    void Reset();

private:
    // Register definitions (AC, PCL, STATUS, etc.)
    // These are now members of the CPU class
    Register AC;
    Register indraddr;
    Register TMR0;
    Register PCL;
    Register STATUS;
    Register FSR;
    Register PORTA;
    Register PORTB;
    Register PORTC;
    Register PORTD;
    Register PORTE;
    Register PCLATH;
    Register INTCON;
    Register PIR1;
    Register PIR2;
    Register TMR1L;
    Register TM1HL; // Note: Original code had TM1HL, typo for TMR1H?
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
    Register EMPTYREG;
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

    std::vector<MemoryBanks> banks; // Each CPU has its own set of memory banks
    // std::vector<Register> registers; // This was in the original CPU class, seems to be for special registers like PCL, AC, STATUS. Replaced by individual Register members.

    void initiateRegisters(); // Helper for constructor
    void setMemoryBanks();    // Helper for constructor

public: // Moved OPCODES to public for access from main.cpp
    // Instruction OPCODES (static constexpr Byte ...)
    // Byte oriented instructions
    static constexpr Byte ADDWF = 0b00000111;
    static constexpr Byte ANDWF = 0b00000101;
    static constexpr Byte CLRF = 0b00000001;
    static constexpr Byte COMF = 0b00001001;
    static constexpr Byte DECF = 0b00000011;
    static constexpr Byte DECFSZ = 0b00001011;
    static constexpr Byte INCF = 0b00001010;
    static constexpr Byte INCFSZ = 0b00001111;
    static constexpr Byte IORWF = 0B00000100;
    static constexpr Byte MOVF = 0b00001000;
    static constexpr Byte MOVWF = 0b00000000;
    static constexpr Byte RLF = 0b00001101;
    static constexpr Byte RRF = 0B00001100;

    // Bit oriented instructions
    static constexpr Byte BCF = 0b0100;
    static constexpr Byte BSF = 0b0101;
    static constexpr Byte BTFSC = 0b0110;
    static constexpr Byte BTFSS = 0b0111;

    // Literal and control ops
    static constexpr Byte ADDLW = 0b00111110;
    static constexpr Byte ANDLW = 0b00111001;
    static constexpr Byte CALL = 0b1000;
    static constexpr Byte GOTO = 0b1001;
    static constexpr Byte IORLW = 0b00111000;
    static constexpr Byte MOVLW = 0b00110000;
    static constexpr Byte RETLW = 0b00110100;
    static constexpr Byte SUBLW = 0b00111100;
    static constexpr Byte XORLW = 0b00111010;
};

#endif // CPU_H
