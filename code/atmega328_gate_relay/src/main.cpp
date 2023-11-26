#include "libs/sx127x.hpp"
#include "libs/tinyMesh.hpp"
#include <Arduino.h>
#include <SPI.h>
#include <avr/wdt.h>
#include "libs/interfaces/lora434InterfaceWrapper.hpp"

#define MISO PIN_PB4
#define MOSI PIN_PB3
#define SCK  PIN_PB5
#define CS   PIN_PB2
#define RST  PIN_PB1
#define DIO0 PIN_PB0

#define STATUS_LED PIN_PD2


#define S_PORT 16 //our single service port

/*----(FUNCTION MACROS)----*/
#define IF_X_FALSE(x, msg, cmd) {if (x) {Serial.println(msg); cmd;}}
#define IF_X_TRUE(x, msg, cmd)  {if (x) {Serial.print(msg); Serial.print(x, 2); Serial.print(" ("); Serial.print(x); Serial.println(")"); cmd;}}

//reset the mcu
#define RESET() {Serial.println("reseting"); wdt_enable(WDTO_15MS); while (true);}


TinyMesh tm(TM_TYPE_NODE);
SX127X lora(CS, RST, DIO0);
lora434InterfaceWrapper lora_if(&lora);

packet_t packet;



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



uint16_t pattern_intr[2][2] = {{4900, 100}, {500, 100}};
//uint8_t pattern = 0;
uint8_t interval = 0;

void status(int pattern) {
    static auto status_timer = millis();
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


void printPacket() {
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


void handleIncomingAnswer() {
    if (packet.fields.msg_type == TM_MSG_OK) {
        Serial.println("OK Answer:");
        printPacket();
        return;
    }

    if (packet.fields.msg_type == TM_MSG_ERR) {
        Serial.println("ERR Answer:");
        printPacket();
        return;
    }

    if (packet.fields.msg_type == TM_MSG_CUSTOM) {
        Serial.println("Custom Answer:");
        printPacket();
        return;
    }
}

uint16_t sendPacketOnInterface() {
    uint16_t ret = 0;

    Serial.println("Sending packet:");
    printPacket();
    Serial.println("");

    uint32_t packet_id = tm.createPacketID(packet);
    ret = lora_if.transmitData(packet.raw, TM_HEADER_LENGTH + packet.fields.data_len);
    IF_X_TRUE(ret, "Failed to transmit data: ", return ret);

    ret = tm.savePacketID(packet_id, millis());
    IF_X_TRUE(ret, "Failed to save packet: ", {});
    
    return ret;
}

void handleIncomingRequest() {
    uint16_t ret = 0;
    if (packet.fields.msg_type == TM_MSG_CUSTOM) {
        IF_X_TRUE(packet.fields.port != S_PORT, "Unknown service: ", return);

        Serial.println("Our service not implemented");

        uint8_t buf = TM_SERVICE_NOT_IMPLEMENTED;
        ret = tm.buildPacket(&packet, packet.fields.src_addr, TM_MSG_ERR, tm.getMessageId(packet) + 1, 0, &buf, 1);
        IF_X_TRUE(ret, "Failed to build packet: ", return);

        sendPacketOnInterface();

        digitalWrite(STATUS_LED, HIGH);
        delay(1000);
        digitalWrite(STATUS_LED, LOW);
        return;
    }

    if (packet.fields.msg_type == TM_MSG_PING) {
        ret = tm.buildPacket(&packet, packet.fields.src_addr, TM_MSG_OK);
        IF_X_TRUE(ret, "Failed to build packet: ", return);

        sendPacketOnInterface();
        return;
    }

    if (packet.fields.msg_type == TM_MSG_RESET) {
        ret = tm.buildPacket(&packet, packet.fields.src_addr, TM_MSG_OK);
        IF_X_TRUE(ret, "Failed to build packet: ", RESET());
        lora_if.transmitData(packet.raw, TM_HEADER_LENGTH);
        RESET();
    }

    Serial.print("Unhandled request type:");
    Serial.println(packet.fields.msg_type);
    uint8_t buf = TM_SERVICE_NOT_IMPLEMENTED;
    ret = tm.buildPacket(&packet, packet.fields.src_addr, TM_MSG_ERR, 0, &buf, 1);
    IF_X_TRUE(ret, "Failed to build packet: ", return);

    sendPacketOnInterface();
}

//TODO save interfaces or forward = broadcast
void forwardPacket() {
    Serial.println("Forwarding packet");
    //uint16_t ret = tm.buildPacket(&packet, packet.fields.dst_addr, packet.fields.msg_type, packet.fields.port, packet.fields.data, packet.fields.data_len);
    //IF_X_TRUE(ret, "Failed to build packet: ", return);
    sendPacketOnInterface();
}


uint16_t getPacketOnInterface() {
    //get data from lora
    uint8_t len;
    uint16_t ret = 0;
    ret = lora_if.getData(packet.raw, &len);
    IF_X_TRUE(ret, "Failed to get data: ", return ret);
    
    //check if packet is valid
    ret = tm.checkHeader(packet);
    IF_X_TRUE(ret, "Incoming packet header bad: ", return ret);
    return ret;
}


/** @brief Synchronouse send and receive function. Exits once either answer is received or timeout occured enough times.
 * 
 * @param tries How many times to try to send and wait for answer
 * @param timeout Timeout (ms) when waiting for answer
 * @param destination Destionation to which to send the packet
 * @param message_type Message type
 * @param port Service port
 * @param buffer Buffer containing data to be sent
 * @param length Length of data
 * @return 
 */
uint16_t requestAwaitAnswer(uint8_t tries, uint32_t timeout, uint8_t destination, uint8_t message_type, uint8_t port, uint8_t *buffer, uint8_t length) {
    uint16_t ret = 0;

    for (int i = 0; i < tries; i++) {
        ret = tm.buildPacket(&packet, destination, message_type, port, buffer, length);
        IF_X_TRUE(ret, "Failed to build packet: ", continue);

        sendPacketOnInterface();

        //start reception on lora
        ret = lora_if.startReception();
        IF_X_TRUE(ret, "Failed to start reception: ", continue);


        //wait for response
        auto timer = millis();
        while (millis() - timer < timeout) {
            //got some data
            if (digitalRead(DIO0)) {
                ret = getPacketOnInterface();
                IF_X_TRUE(ret, "Failed to get valid packet on interface: ", continue);

                ret = tm.checkPacket(packet);
                IF_X_TRUE(ret != TM_IN_ANSWER, "Incoming packet is not an answer: ", continue);
                Serial.println("Got answer:");
                printPacket();
                Serial.println("");
                return ret;
            }

            //clear saved packets in the meantime
            tm.clearSavedPackets(millis());
        }

        Serial.println("timeout");
    }

    return ret;
}


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
    tm.setPort(S_PORT, TM_PORT_IN | TM_PORT_DATA_NONE);
    tm.setAddress(46);

    //1. register
    ret = requestAwaitAnswer(3, TM_TIME_TO_STALE, tm.getGatewayAddress(), TM_MSG_REGISTER, 0, nullptr, 0);
    IF_X_TRUE(ret, "Failed to register: ", failed());
    Serial.println("Registered successfully");

    //save data from registration
    tm.clearSavedPackets(millis());
    //tm.setAddress(packet.fields.data[0]);
    tm.setGatewayAddress(packet.fields.src_addr);

    //2. tell gateway our ports
    uint8_t buf[2] = {S_PORT, TM_PORT_IN | TM_PORT_DATA_NONE};
    ret = requestAwaitAnswer(3, TM_TIME_TO_STALE, tm.getGatewayAddress(), TM_MSG_PORT_ANOUNCEMENT, 0, buf, 2);
    IF_X_TRUE(ret, "Failed to send port anouncement: ", failed());
    

    digitalWrite(STATUS_LED, LOW);
}


auto status_timer = millis();
void loop() {
    uint16_t ret = 0;

    //if (millis() - status_timer > 7000) {
    //    status_timer = millis();
    //    uint8_t msg[] = "ALIVE";
    //    sendPacketOnInterface(tm.getGatewayAddress(), TM_MSG_STATUS, 0, msg, 5);
    //}

    //status(0);

    if (digitalRead(DIO0)) {
        ret = getPacketOnInterface();
        IF_X_TRUE(ret, "get interface failed", return);

        ret = tm.checkPacket(packet);
        IF_X_TRUE(ret == TM_ERR_IN_ANSWER,    "Incoming packet is unrequested answer", return);
        IF_X_TRUE(ret == TM_ERR_IN_TYPE,      "Incoming packet has wrong type", return);
        IF_X_TRUE(ret == TM_ERR_IN_PORT,      "Incoming packet has wrong port", return);
        IF_X_TRUE(ret == TM_ERR_IN_DUPLICATE, "Incoming packet is a duplicate", return);

        //save received packet
        IF_X_TRUE(tm.savePacket(packet, millis()), "Failed to save packet: ", {});

        if (ret == TM_IN_ANSWER) {
            handleIncomingAnswer();
        }
        if (ret == TM_IN_REQUEST) {
            handleIncomingRequest();
        }
        if (ret == TM_IN_BROADCAST) {
            forwardPacket();
            handleIncomingRequest();
        }
        if (ret == TM_IN_FORWARD) {
            forwardPacket();
        }

        lora_if.startReception();
    }

    ret = tm.clearSavedPackets(millis());
    if (ret) {
        Serial.print("Removed stale packet(s): ");
        Serial.println(ret);
    }
    //if (millis() - status_timer > 15000) {
    //    //TODO send status
    //    ret = getPacketOnInterface();
    //    IF_X_TRUE(ret, "Failed to get data: ", {});
    //}
}

