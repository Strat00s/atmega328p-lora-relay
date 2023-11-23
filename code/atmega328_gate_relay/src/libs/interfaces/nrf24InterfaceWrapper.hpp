#pragma once
#include <stdio.h>
#include "interfaceWrapper.hpp"
#include "libs/sx127x.hpp"


class nrf24InterfaceWrapper : public interfaceWrapper{
private:
    /* data */
public:
    uint8_t transmitData(uint8_t *data, uint8_t len);
    uint8_t receiveData(uint8_t *data, uint8_t *len);
    uint8_t hasData();
};
