This is an pathetic attempt to build a PIC16F877A Microcontroller emulator. 
Got inspired by this Youtube video(https://youtu.be/qJgsuQoy9bc?list=PLLwK93hM93Z13TRzPx9JqTIn33feefl37), and having lots of free time got me thinking, why should I not build something similiar for an MCU that we used in Microcontrollers class.
This project is nowhere near finished, but it'd make a good template for someone who's trying to build an MCU/CPU. Also, it has a lot of potantiel for upgrading and optimizing.
So, here's what i think should be done next:

1- Making this project more modular. The whole program in one code(700+ lines of code) is a big no no, I built it on top of each stuff I learned, so it all ended up in one code. But this need to be fixed.


2- Implementing more features. Such as interrupts, subroutines, I/O simulations(Reading from PORTB etc.), direct/indirect writing, PCL&PCLATH(as in PIC16F877A MCU) instead of one single Program Counter(PC) and EEPROM. You can even implement the timer modules if you are crazy.


3- Implementing more registers. Most of the features that i listed above, will also require fully implemented registers such as PCL, PCLATH, EECON1, EEADDR etc..


4- A GUI. It'd a nice to have a GUI of a Microcontroller Development Kit, with all the LEDs and stuff.


5- Optimization and optimization.

--------

Datasheet: https://ww1.microchip.com/downloads/aemDocuments/documents/MCU08/ProductDocuments/DataSheets/39582C.pdf


More detailed instruction set: https://ww1.microchip.com/downloads/en/devicedoc/33023a.pdf

--------

-----------------------------------------------------
HOW DOES THE PROGRAM WORK


It is pretty straightforward, you add an instruction to the Program Memory

    CPU PIC16F87;

    PIC16F87.editProgramMemory(0, 0, 0b0011111000000001);
    

as you see here, the instruction is added to the bank 0's(1st param), 0th index(2nd param). When the program is first run, PCL will fetch the instruction at 0th index in Program Memory.
The insruction stored in Program Memory is 16bits long, the OPCODE is 8 bits as seen in the code:

    static constexpr Byte ADDLW = 0b00111110;

and the rest of the bits either represents the address of the content or literal number depending on the instruction type.

So, we do have one instruction in our Program Memory. When we want to execute it, we simply do:

    PIC16F87.Execute(Cycles);

this will start running the MCU and will go on running until it reaches an empty Program Memory content or it runs out of Cycles(Each fetch from Program Memory results in --Cycles).
In the Execute function, we first fetch the instruction:

    INS InstructionAddr = Fetch(Cycles, BankNo);

INS data type here means Instruction and is 16 bits,

    using INS = uint16_t;

In the Fetch function, it will fetch the program memory[PCL] basically, then PCL++ and --Cycles. And then we return the instruction, for decoding.:

    std::vector<Byte> Instruction = decodeInstruction(InstructionAddr);

In the decoding part we simply split the instruction into parts(OPCODE, Address, Literal, d bit) depends on what kind of instruction it is(Byte, Bit or Literal&Control) and return them in a vector.
And after the decoding part, lastly, we execute the instructions based on their OPCODE. Here's what MOVLW execution looks like:

    case MOVLW:{
                std::cout<<"MOVLW"<<std::endl;
                setRegisterValue(banks[BankNo], "AC", Instruction[2]);
    }break;
