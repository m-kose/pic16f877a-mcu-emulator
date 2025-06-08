#ifndef REGISTER_H
#define REGISTER_H

#include <cstdint>

using Byte = uint8_t;

class Register {
public:
    Register();
    void setValue(uint8_t val);
    Byte getValue() const;
    Byte value; // It might be better to make this private and use getValue()
};

#endif // REGISTER_H
