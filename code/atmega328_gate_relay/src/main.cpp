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


/*----(FUNCTION MACROS)----*/
#define IF_X_FALSE(x, msg, cmd) {if (x) {Serial.println(msg); cmd;}}
#define IF_X_TRUE(x, msg, cmd)  {if (x) {Serial.print(msg); Serial.print(x, 2); Serial.print(" ("); Serial.print(x); Serial.println(")"); cmd;}}

//reset the mcu
#define RESET() {Serial.println("reseting"); wdt_enable(WDTO_15MS); while (true);}


TinyMesh tm(TM_NODE_TYPE_NODE);
SX127X lora(CS, RST, DIO0);
lora434InterfaceWrapper lora_if(&lora);

uint16_t pattern_intr[2][2] = {{4900, 100}, {500, 100}};
uint8_t interval = 0;

packet_t packet;

//Timer for scheduling
auto status_timer = millis();
auto alive_timer = millis();


/*----(SX127X CALLBACKS)----*/
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


/*----(HELPERS & OTHER)----*/
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
    Serial.println(tm.getBits(packet.fields.flags, TM_NODE_TYPE_MSB, TM_NODE_TYPE_LSB));
    Serial.print("Message id:          ");
    Serial.println(tm.getMessageId(&packet));
    Serial.print("Repeate count:       ");
    Serial.println(tm.getBits(packet.fields.flags, TM_RPT_CNT_MSB, TM_RPT_CNT_LSB));
    Serial.print("Source address:      ");
    Serial.println(packet.fields.source);
    Serial.print("Destination address: ");
    Serial.println(packet.fields.destination);
    Serial.print("Message type:        ");
    Serial.println(tm.getBits(packet.fields.flags, TM_MSG_TYPE_MSB, TM_MSG_TYPE_LSB));
    Serial.print("Data length:         ");
    Serial.println(packet.fields.data_length);
    Serial.print("Data: ");
    for (int i = 0; i < packet.fields.data_length; i++) {
        Serial.print(packet.fields.data[i]);
        Serial.print(" ");
    }
    Serial.println();
}


/** @brief Handler for incoming packets. Not used since this node does not send anything except for register */
void handleIncomingResponse() {
    uint8_t msg_type = tm.getBits(packet.fields.flags, TM_MSG_TYPE_MSB, TM_MSG_TYPE_LSB);
    if (msg_type == TM_MSG_OK) {
        Serial.println("OK Answer");
        return;
    }

    if (msg_type == TM_MSG_ERR) {
        Serial.println("ERR Answer");
        return;
    }

    if (msg_type == TM_MSG_CUSTOM) {
        Serial.println("Custom Answer");
        return;
    }
}

/** @brief Send packet on our only interface and save it's ID
 * 
 * @return 0 on success, errors that occured during transmission
 */
uint8_t sendPacketOnInterface() {
    uint8_t ret = 0;

    Serial.println("");
    Serial.println("Sending packet:");
    printPacket();

    ret = lora_if.transmitData(packet.raw, TM_HEADER_LENGTH + packet.fields.data_length);
    IF_X_TRUE(ret, "Failed to transmit data: ", return ret);
    return ret;
}

/** @brief Get data from interface and try to build a pakcet
 * 
 * @return 0 on success, lora or tm errors on failure
 */
uint8_t getPacketOnInterface() {
    //get data from lora
    uint8_t len;
    uint8_t ret = 0;
    ret = lora_if.getData(packet.raw, &len);
    IF_X_TRUE(ret, "Failed to get data: ", return ret);
    
    //check if packet is valid
    ret = tm.checkHeader(&packet);
    IF_X_TRUE(ret, "Incoming packet header bad: ", return ret);

    Serial.println("Got packet:");
    printPacket();
    Serial.println("");

    return ret;
}

void handleIncomingRequest(bool repeat = false) {
    uint16_t ret = 0;
    uint8_t msg_Type = tm.getBits(packet.fields.flags, TM_MSG_TYPE_MSB, TM_MSG_TYPE_LSB);
    uint8_t rpt_cnt = tm.getBits(packet.fields.flags, TM_RPT_CNT_MSB, TM_RPT_CNT_LSB);
    if (msg_Type == TM_MSG_CUSTOM) {

        //listen for data starting with T(oggle) and being long exactly 1
        if (packet.fields.data[0] == 'T' && packet.fields.data_length == 1) {
            ret = tm.buildPacket(&packet, packet.fields.source, tm.getMessageId(&packet) + 1, TM_MSG_OK, nullptr, 0, rpt_cnt);
            IF_X_TRUE(ret, "Failed to build packet: ", return);

            sendPacketOnInterface();

            //Handle our service only if not repeat
            if (!repeat) {
                digitalWrite(STATUS_LED, HIGH);
                delay(1000);
                digitalWrite(STATUS_LED, LOW);
            }
            return;
        }
        else {
            Serial.println("Unknown service");
            uint8_t buf = TM_ERR_SERVICE_UNHANDLED;
            ret = tm.buildPacket(&packet, packet.fields.source, tm.getMessageId(&packet) + 1, TM_MSG_ERR, &buf, 1, rpt_cnt);
            IF_X_TRUE(ret, "Failed to build packet: ", return);

            sendPacketOnInterface();

            return;
        }
    }

    //PING response
    if (msg_Type == TM_MSG_PING) {
        ret = tm.buildPacket(&packet, packet.fields.source, tm.getMessageId(&packet) + 1, TM_MSG_OK, nullptr, 0, rpt_cnt);
        IF_X_TRUE(ret, "Failed to build packet: ", return);

        sendPacketOnInterface();

        return;
    }


    //Nothing else is handled
    Serial.print("Unhandled request type:");
    Serial.println(msg_Type);
    uint8_t buf = TM_ERR_MSG_UNHANDLED;
    ret = tm.buildPacket(&packet, packet.fields.source, tm.getMessageId(&packet) + 1, TM_MSG_ERR, &buf, 1, rpt_cnt);
    IF_X_TRUE(ret, "Failed to build packet: ", return);

    sendPacketOnInterface();
}


/** @brief Blocking send and receive function. Exits once either answer is received or timeout/error occured enough times.
 * 
 * @param destination Destionation to which to send the packet
 * @param message_type Message type
 * @param buffer Buffer containing data to be sent
 * @param length Length of data
 * @param timeout Timeout (ms) when waiting for answer
 * @return 
 */
uint8_t requestAwait(uint8_t destination, uint8_t message_type, uint8_t *buffer, uint8_t length, uint32_t timeout = 1000) {
    uint8_t ret = 0;
    uint16_t msg_id = tm.lcg();

    for (int i = 0; i < TM_MAX_REPEAT + 1; i++) {
        ret = tm.buildPacket(&packet, destination, msg_id, message_type, buffer, length, i);
        Serial.print("build packet: ");
        Serial.println(ret);
        IF_X_TRUE(ret, "Failed to build packet: ", continue);

        sendPacketOnInterface();

        //start reception on lora
        ret = lora_if.startReception();
        IF_X_TRUE(ret, "Failed to start reception: ", continue);

        //timeout return value
        ret = 0xFF;

        //wait for response
        auto timer = millis();
        while (millis() - timer < timeout) {
            //got some data
            if (digitalRead(DIO0)) {
                ret = getPacketOnInterface();
                IF_X_TRUE(ret, "failed to get packet: ", continue);

                ret = tm.checkPacket(&packet);
                Serial.println(ret);
                IF_X_TRUE((ret & TM_PACKET_RESPONSE) == 0, "not an answer: ", continue);
                Serial.println("Incoming packet is an answer");
                return 0;
            }
        }
        //tm.clearSentQueue();
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
    lora.registerSPIBeginTransfer(SPIBeginTransfer);
    lora.registerSPIEndTransfer(SPIEndTransfer);
    lora.registerSPITransfer(SPITransfer);
    lora.reset();
    Serial.println("after lora");
    uint16_t ret = lora.begin(434.0, 0x12, 8, LORA_BANDWIDTH_125kHz, LORA_SPREADING_FACTOR_9, LORA_CODING_RATE_4_7);
    IF_X_TRUE(ret, "lora begin failed: ", failed());

    //TinyMesh init
    tm.setSeed(46290);
    tm.setAddress(46);

    //1. register
    ret = requestAwait(tm.getGatewayAddress(), TM_MSG_REGISTER, nullptr, 0);
    IF_X_TRUE(ret, "Failed to register: ", failed());
    Serial.println("Registered successfully");

    //save data from registration
    ret = tm.clearSentQueue();
    Serial.println(ret);
    tm.setGatewayAddress(packet.fields.source);

    digitalWrite(STATUS_LED, LOW);
}


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
        
        ret = tm.checkPacket(&packet);
        if (ret & TM_PACKET_DUPLICATE) {
            Serial.println("Incoming packet is a duplicate");
            lora_if.startReception();
            return;
        }

        //save anything that is not duplicate
        //tm.savePacket(&packet);

        IF_X_TRUE(ret & TM_PACKET_RND_RESPONSE,    "Incoming packet is unrequested answer", return;);

        if (ret & TM_PACKET_REQUEST) {
            handleIncomingRequest(ret & TM_PACKET_REPEAT);
        }
        if (ret & TM_PACKET_RESPONSE) {
            handleIncomingResponse();
        }
        if (ret & TM_PACKET_FORWARD) {
            sendPacketOnInterface();
        }
        
        //start receiving data again
        lora_if.startReception();
    }

    if (millis() - alive_timer > 60000) {
        alive_timer = millis();
        ret = tm.buildPacket(&packet, tm.getGatewayAddress(), tm.lcg(), TM_MSG_PING);
        IF_X_TRUE(ret, "Failed to build packet: ", return;);
        sendPacketOnInterface();
        lora_if.startReception();
    }
}

