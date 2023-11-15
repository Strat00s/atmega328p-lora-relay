#include "libs/sx127x.hpp"
#include "libs/tinyMesh.hpp"
#include <Arduino.h>
#include <SPI.h>


#define MISO PIN_PB4
#define MOSI PIN_PB3
#define SCK  PIN_PB5
#define CS   PIN_PB2
#define RST  PIN_PB1
#define DIO0 PIN_PB0

#define STATUS_LED PIN_PD2


SX127X lora(CS, RST, DIO0);
TinyMesh tm(TM_TYPE_NODE);


//SX127X callbacks
uint8_t pinRead(uint8_t pin) {
    return digitalRead(pin);
}
void __delay(uint32_t ms) {
    delay(ms);
}
uint32_t __micros() {
    return micros();
}
void SPIBeginTransfer() {
    SPI.beginTransaction({14000000, MSBFIRST, SPI_MODE0});
}
void SPIEndTransfer() {
    SPI.endTransaction();
}
void SPITransfer(uint8_t addr, uint8_t *buffer, size_t length) {
    SPI.transfer(addr);
    SPI.transfer(buffer, length);
}



auto status_timer = millis();
uint16_t pattern_intr[2][2] = {{4900, 100}, {500, 100}};
//uint8_t pattern = 0;
uint8_t interval = 0;

void status(int pattern) {
    if (millis() - status_timer > pattern_intr[pattern][interval]) {
        status_timer = millis();
        interval = !interval;
        digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
    }
}


void setup() {
    Serial.begin(9600);
    SPI.begin();

    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);

    lora.registerMicros(__micros);
    lora.registerDelay(__delay);
    lora.registerPinMode(pinMode, INPUT, OUTPUT);
    lora.registerPinWrite(digitalWrite);
    lora.registerPinRead(pinRead);
    lora.registerSPIBeginTransfer(SPIBeginTransfer);
    lora.registerSPIEndTransfer(SPIEndTransfer);
    lora.registerSPITransfer(SPITransfer);

    lora.reset();

    uint8_t rc = lora.begin(434.0, 0x12, 8, LORA_BANDWIDTH_125kHz, LORA_SPREADING_FACTOR_9, LORA_CODING_RATE_4_7);
    if (rc) {
        Serial.println("lora begin failed");
        while (true) {
            status(1);
        }
    }

    digitalWrite(STATUS_LED, LOW);
}


auto send_timer = millis();
void loop() {
    status(0);

    if (millis() - send_timer > 2000) {
        send_timer = millis();
        packet_t packet;
        auto ret = tm.buildPacket(&packet, 255, TM_MSG_REGISTER);
        if (ret) {
            Serial.print("Failed to build packet: ");
            Serial.print(ret, 2);
            Serial.println();
        }
        else {
            ret = 0;
            ret = lora.transmit(packet.raw, packet.fields.data_length + TM_HEADER_LENGTH);
            if (!(ret & IRQ_FLAG_TX_DONE)) {
                Serial.print("Failed to send data: ");
                Serial.print(ret, 2);
                Serial.println();
            }
        }
    }
}

