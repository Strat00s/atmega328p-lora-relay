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


#define SERVICE_PORT1 16
#define SERVICE_PORT2 32
#define SERVICE_PORT3 198

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


/** @brief Does not do much right now
 * 
 */
void handleIncomingAnswer() {
    if (packet.fields.msg_type == TM_MSG_OK) {
        Serial.println("OK Answer");
        return;
    }

    if (packet.fields.msg_type == TM_MSG_ERR) {
        Serial.println("ERR Answer");
        return;
    }

    if (packet.fields.msg_type == TM_MSG_CUSTOM) {
        Serial.println("Custom Answer");
        return;
    }
}


/** @brief Send packet on our only interface and save it's ID
 * 
 * @return 0 on success, errors that occured during transmission
 */
uint16_t sendPacketOnInterface(bool save = true) {
    uint16_t ret = 0;

    Serial.println("");
    Serial.println("Sending packet:");
    printPacket();

    uint32_t packet_id = tm.createPacketID(packet);
    ret = lora_if.transmitData(packet.raw, TM_HEADER_LENGTH + packet.fields.data_len);
    IF_X_TRUE(ret, "Failed to transmit data: ", return ret);

    if (save) {
        IF_X_TRUE(tm.savePacketID(packet_id, millis()), "Failed to save packet: ", {});
    }

    return ret;
}


/** @brief Get data from interface and try to build a pakcet
 * 
 * @return 0 on success, lora or tm errors on failure
 */
uint16_t getPacketOnInterface() {
    //get data from lora
    uint8_t len;
    uint16_t ret = 0;
    ret = lora_if.getData(packet.raw, &len);
    IF_X_TRUE(ret, "Failed to get data: ", return ret);
    
    //check if packet is valid
    ret = tm.checkHeader(packet);
    IF_X_TRUE(ret, "Incoming packet header bad: ", return ret);

    Serial.println("Got packet:");
    printPacket();
    Serial.println("");

    return ret;
}


void handleIncomingRequest() {
    uint16_t ret = 0;
    if (packet.fields.msg_type == TM_MSG_CUSTOM) {
        if (packet.fields.port == SERVICE_PORT1) {
            ret = tm.buildPacket(&packet, packet.fields.src_addr, TM_MSG_OK, tm.getMessageId(packet) + 1, 0, nullptr, 0);
            IF_X_TRUE(ret, "Failed to build packet: ", return);

            sendPacketOnInterface();

            //Our "service" is a blink
            digitalWrite(STATUS_LED, HIGH);
            delay(1000);
            digitalWrite(STATUS_LED, LOW);
            return;
        }
        else if (packet.fields.port == SERVICE_PORT2) {
            Serial.println("This service is not implemented");
            uint8_t buf = TM_ERR_SERVICE_UNHANDLED;
            ret = tm.buildPacket(&packet, packet.fields.src_addr, TM_MSG_ERR, tm.getMessageId(packet) + 1, 0, &buf, 1);
            IF_X_TRUE(ret, "Failed to build packet: ", return);
            
            sendPacketOnInterface();
            
            return;
        }

        Serial.print("Unknown service port: ");
        Serial.println(packet.fields.port);
        Serial.println("^^^^^^^^^^^^^^^^^^^^^^^^^");
        Serial.println("This should not happen!!!");

        uint8_t buf = TM_ERR_UNKNOWN_PORT;
        ret = tm.buildPacket(&packet, packet.fields.src_addr, TM_MSG_ERR, tm.getMessageId(packet) + 1, 0, &buf, 1);
        IF_X_TRUE(ret, "Failed to build packet: ", return);
        
        sendPacketOnInterface();
        
        return;
    }

    //PING response
    if (packet.fields.msg_type == TM_MSG_PING) {
        ret = tm.buildPacket(&packet, packet.fields.src_addr, TM_MSG_OK);
        IF_X_TRUE(ret, "Failed to build packet: ", return);

        sendPacketOnInterface();

        return;
    }

    //RESET response and reset handling
    if (packet.fields.msg_type == TM_MSG_RESET) {
        ret = tm.buildPacket(&packet, packet.fields.src_addr, TM_MSG_OK);
        IF_X_TRUE(ret, "Failed to build packet: ", RESET());
        lora_if.transmitData(packet.raw, TM_HEADER_LENGTH);
        RESET();
    }

    //Rest is not implemented as this device does not use anything else
    Serial.print("Unhandled request type:");
    Serial.println(packet.fields.msg_type);
    uint8_t buf = TM_ERR_MSG_UNHANDLED;
    ret = tm.buildPacket(&packet, packet.fields.src_addr, TM_MSG_ERR, 0, &buf, 1);
    IF_X_TRUE(ret, "Failed to build packet: ", return);

    sendPacketOnInterface();
}


/** @brief Blocking send and receive function. Exits once either answer is received or timeout/error occured enough times.
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
                Serial.println("Incoming packet is an answer");
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
    //Status LED init
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);

    //uart init
    Serial.begin(9600);
    Serial.println("Start");
    
    //SPI init
    SPI.begin();

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

    //TinyMesh init
    tm.setSeed(46290);
    tm.addPort(SERVICE_PORT1);
    tm.addPort(SERVICE_PORT2);
    tm.setAddress(46);

    //1. register
    ret = requestAwaitAnswer(3, TM_TIME_TO_STALE, tm.getGatewayAddress(), TM_MSG_REGISTER, 0, nullptr, 0);
    IF_X_TRUE(ret, "Failed to register: ", failed());
    Serial.println("Registered successfully");

    //save data from registration
    tm.clearSavedPackets(millis());
    tm.setGatewayAddress(packet.fields.src_addr);

    //2. tell gateway our ports
    uint8_t buf = SERVICE_PORT1;
    ret = requestAwaitAnswer(3, TM_TIME_TO_STALE, tm.getGatewayAddress(), TM_MSG_PORT_ANOUNCEMENT, 0, &buf, 1);
    IF_X_TRUE(ret, "Failed to send port anouncement: ", failed());
    

    digitalWrite(STATUS_LED, LOW);
}


auto status_timer = millis();
void loop() {
    uint16_t ret = 0;

    status(0);

    //single interface receive signal
    if (digitalRead(DIO0)) {
        ret = getPacketOnInterface();
        if (ret) {
            Serial.println("Get packet on interface failed");
            lora_if.startReception();
            return;
        }
        
        ret = tm.checkPacket(packet);
        if (ret == TM_ERR_IN_DUPLICATE) {
            Serial.println("Incoming packet is a duplicate");
            lora_if.startReception();
            return;
        }

        //save anything that is not duplicate
        IF_X_TRUE(tm.savePacket(packet), "Failed to save packet", {});

        IF_X_TRUE(ret == TM_ERR_IN_ANSWER,    "Incoming packet is unrequested answer", {});
        IF_X_TRUE(ret == TM_ERR_IN_TYPE,      "Incoming packet has wrong type", {});    //This shouldn't ever happen
        
        if (ret == TM_ERR_IN_PORT) {
            Serial.print("Incoming packet has unknown port: ");
            Serial.println(packet.fields.port);
            
            uint8_t buf = TM_ERR_UNKNOWN_PORT;
            ret = tm.buildPacket(&packet, packet.fields.src_addr, TM_MSG_ERR, tm.getMessageId(packet) + 1, 0, &buf, 1);
            if (ret) {
                Serial.print("Failed to build a packet: ");
                Serial.println(ret);
            }
            else
                sendPacketOnInterface();
        }

        if (ret == TM_IN_ANSWER) {
            handleIncomingAnswer();
        }
        if (ret == TM_IN_REQUEST) {
            handleIncomingRequest();
        }
        if (ret == TM_IN_BROADCAST) {
            sendPacketOnInterface(false);
            handleIncomingRequest();
        }
        if (ret == TM_IN_FORWARD) {
            sendPacketOnInterface(false);
        }

        //start receiving data again
        lora_if.startReception();
    }

    ret = tm.clearSavedPackets(millis());
    if (ret) {
        Serial.print("Removed stale packet(s): ");
        Serial.println(ret);
    }
}

