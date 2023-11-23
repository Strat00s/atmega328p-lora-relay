#include "libs/sx127x.hpp"
#include "libs/tinyMesh.hpp"
#include <Arduino.h>
#include <SPI.h>
#include "libs/interfaces/lora434InterfaceWrapper.hpp"

#define MISO PIN_PB4
#define MOSI PIN_PB3
#define SCK  PIN_PB5
#define CS   PIN_PB2
#define RST  PIN_PB1
#define DIO0 PIN_PB0

#define STATUS_LED PIN_PD2

/*----(FUNCTION MACROS)----*/
#define IF_X_FALSE(x, msg, cmd) {if (x) {Serial.println(msg); cmd;}}
#define IF_X_TRUE(x, msg, cmd)  {if (x) {Serial.print(msg); Serial.print(x, 2); Serial.print(" ("); Serial.print(x); Serial.println(")"); cmd;}}



TinyMesh tm(TM_TYPE_NODE);
SX127X lora(CS, RST, DIO0);
lora434InterfaceWrapper lora_if(&lora);



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

void failed() {
    while(true) {
        status(1);
    }
}


void printPacket(packet_t packet) {
    Serial.print("Version:             ");
    Serial.println(packet.fields.version);
    Serial.print("Device type:         ");
    Serial.println(packet.fields.node_type);
    Serial.print("Message id:          ");
    Serial.println(tm.getMessageId(packet));
    Serial.print("Source address:      ");
    Serial.println(packet.fields.src_addr);
    Serial.print("Destination address: ");
    Serial.println(packet.fields.dst_addr);
    Serial.print("Port:                ");
    Serial.println(packet.fields.port);
    Serial.print("Message type:        ");
    Serial.println(packet.fields.msg_type);
    Serial.print("Data length:         ");
    Serial.println(packet.fields.data_len);
    Serial.print("Data: ");
    for (int i = 0; i < packet.fields.data_len; i++) {
        Serial.print(packet.fields.data[i]);
        Serial.print(" ");
    }
    Serial.println();
}

int freeRam () {
    extern int __heap_start, *__brkval; 
    int v; 
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}


packet_t packet;


void setup() {
    Serial.begin(9600);
    Serial.println("Start");
    delay(1000);
    SPI.begin();

    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);

    //LORA init
    lora.registerMicros(__micros);
    lora.registerDelay(__delay);
    lora.registerPinMode(pinMode, INPUT, OUTPUT);
    lora.registerPinWrite(digitalWrite);
    lora.registerPinRead(pinRead);
    lora.registerSPIBeginTransfer(SPIBeginTransfer);
    lora.registerSPIEndTransfer(SPIEndTransfer);
    lora.registerSPITransfer(SPITransfer);
    lora.reset();
    Serial.println("after lora");
    uint16_t ret = lora.begin(434.0, 0x12, 8, LORA_BANDWIDTH_125kHz, LORA_SPREADING_FACTOR_9, LORA_CODING_RATE_4_7);
    IF_X_TRUE(ret, "lora begin failed: ", failed());

    Serial.println(freeRam());

    //TM init
    tm.setSeed(46290);
    tm.addPort(16, TM_PORT_IN | TM_PORT_DATA_NONE);

    //try to register
    bool registered = false;
    for (int i = 0; i < 3; i++) {
        Serial.println("Registering: ");

        tm.buildPacket(&packet, 255, TM_MSG_REGISTER);
        printPacket(packet);

        ret = lora_if.transmitData(packet.raw, TM_HEADER_LENGTH);
        IF_X_TRUE(ret, "Failed to transmit data: ", failed());

        ret = tm.savePacket(packet, millis());
        IF_X_TRUE(ret, "Failed to save packet: ", {});

        ret = lora_if.startReception();
        IF_X_TRUE(ret, "Failed to start reception: ", failed());

        auto timer = millis();
        while (millis() - timer < 3000 && !digitalRead(DIO0)) {
            //TODO more logic
            //if packet is a broadcast, ignore it and keep waiting for our response
            //wait for response, not just any packet
        }

        if (!lora_if.hasData()) {
            Serial.println("timeout");
            continue;
        }

        uint8_t len;
        ret = lora_if.getData(packet.raw, &len);
        IF_X_TRUE(ret, "Failed to get data: ", continue);

        printPacket(packet);

        ret = tm.checkHeader(packet);
        IF_X_TRUE(ret, "Incoming packet header bad: ", continue);

        ret = tm.checkPacket(packet);
        IF_X_TRUE(ret, "Incoming packet not answer: ", continue);

        Serial.println("success");

        //address should be the only data
        tm.setAddress(packet.fields.data[0]);
        registered = true;
        break;
    }
    if (!registered)
        failed();

    //port anouncement


    digitalWrite(STATUS_LED, LOW);
}


auto send_timer = millis();
void loop() {
    status(0);

    if (millis() - send_timer > 2000) {
        send_timer = millis();
        auto ret = tm.buildPacket(&packet, 255, TM_MSG_REGISTER);
        if (ret) {
            Serial.print("Failed to build packet: ");
            Serial.println(ret, 2);
        }
        else {
            ret = 0;
            ret = lora.transmit(packet.raw, packet.fields.data_len + TM_HEADER_LENGTH);
            if (!(ret & IRQ_FLAG_TX_DONE)) {
                Serial.print("Failed to send data: ");
                Serial.println(ret, 2);
            }
        }
    }
}

