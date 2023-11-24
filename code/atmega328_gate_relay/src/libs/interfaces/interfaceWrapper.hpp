#pragma once
#include <stdio.h>

//interface type

#define IW_TYPE_NONE      0
#define IW_TYPE_MQTT      1
#define IW_TYPE_LORA_137M 2
#define IW_TYPE_LORA_434M 3
#define IW_TYPE_LORA_868M 4
#define IW_TYPE_LORA_2_4G 5
#define IW_TYPE_NRF24     6
#define IW_TYPE_RF_443    7
#define IW_TYPE_ESP_NOW   8
#define IW_TYPE_ERR       9 //Anything equal or above this value is unknown interface


class interfaceWrapper {
private:
    /* data */
    uint8_t i_type;
public:
    interfaceWrapper(uint8_t interface_type) {
        this->i_type = interface_type;
    }
    ~interfaceWrapper() {};

    void setType(uint8_t interface_type) {
        this->i_type = interface_type;
    }
    uint8_t getType() {
        return this->i_type;
    }

    virtual uint8_t transmitData(uint8_t *data, uint8_t len) = 0;
    virtual uint8_t getData(uint8_t *data, uint8_t *len) = 0;
    virtual uint8_t startReception() = 0;
    virtual uint8_t hasData() = 0;
    virtual uint8_t clearIRQ() = 0;
};
