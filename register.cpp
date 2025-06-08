#include "register.h"

Register::Register() : value(0) {}

void Register::setValue(uint8_t val) {
    value = val;
}

Byte Register::getValue() const {
    return value;
}
