/** @file tinyMesh.hpp
 * @author Lukáš Baštýř (l.bastyr@seznam.cz, 492875)
 * @brief TinyMesh is a simple protocol for IoT devices.
 * @version 0.1
 * @date 27-11-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once
#include <stdint.h>

//TODOs are mostly for V1.1 (if that's ever gonna happen)
/*
# Packet structure
-------------HEADER-------------
    VERSION             8b
    DEVICE TYPE         8b
    MESSAGE ID          16b
    SOURCE ADDRESS      8b
    DESTINATION ADDRESS 8b
    PORT NUMBER         8b
    MESSAGE TYPE        8b
    DATA LENGTH         8b
--------------DATA--------------
    DATA...             256 - header


# Rules
All predefined messages have simple flow: single request and a single answer (might not arrive)
    Message ID of an answer (OK, ERR) is equal to message ID of request + 1 
Only custom messages are allowed to have flow of any size (continuous request, response, request, response, ...)
    Message ID is increased by 1 for every new message in this flow.

# Predefined messages packet structure and flow:
    ok
        VERSION             1
        NODE TYPE           x
        MESSAGE ID          n+1
        SOURCE ADDRESS      z
        DESTINATION ADDRESS y
        PORT NUMBER         0
        MESSAGE TYPE        OK
        DATA LENGTH         l
        DATA...             l bytes (depends on answered message)
    err
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n+1
        SOURCE ADDRESS      z
        DESTINATION ADDRESS y
        PORT NUMBER         0
        MESSAGE TYPE        ERR
        DATA LENGTH         1
        DATA...             ERROR_CODE
    Ping
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n
        SOURCE ADDRESS      y
        DESTINATION ADDRESS z
        PORT NUMBER         0
        MESSAGE TYPE        PING
        DATA LENGTH         0
    Register
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n
        SOURCE ADDRESS      0
        DESTINATION ADDRESS 255
        PORT NUMBER         0
        MESSAGE TYPE        REGISTER
        DATA LENGTH         0
    TODO Request
        VERSION
        DEVICE TYPE
        MESSAGE ID
        SOURCE ADDRESS
        DESTINATION ADDRESS
        PORT NUMBER
        MESSAGE TYPE
        DATA LENGTH
        DATA...
    TODO Resend
        VERSION
        DEVICE TYPE
        MESSAGE ID
        SOURCE ADDRESS
        DESTINATION ADDRESS
        PORT NUMBER
        MESSAGE TYPE
        DATA LENGTH
        DATA...
    Port anouncement
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n
        SOURCE ADDRESS      y
        DESTINATION ADDRESS z (gateway)
        PORT NUMBER         0
        MESSAGE TYPE        PORT_ADVERTISEMENT
        DATA LENGTH         1 * l
        DATA...             l PORTs
            PORT      8b
            ...
    TODO Route anouncement
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n
        SOURCE ADDRESS      y
        DESTINATION ADDRESS z (gateway)
        PORT NUMBER         0
        MESSAGE TYPE        ROUTE_ANOUNCEMENT
        DATA LENGTH         2 * l
        DATA...             LISTENER_ADDRESS and PORT pairs
            LISTENER_ADDRESS 8b
            LISTEN_PORT      8b
            ...
    Reset
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n
        SOURCE ADDRESS      y
        DESTINATION ADDRESS z
        PORT NUMBER         0
        MESSAGE TYPE        RESET
        DATA LENGTH         0
    Status
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n
        SOURCE ADDRESS      y
        DESTINATION ADDRESS z
        PORT NUMBER         0
        MESSAGE TYPE        STATUS
        DATA LENGTH         l
        DATA...             string of size l
    TODO Combined
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n
        SOURCE ADDRESS      y
        DESTINATION ADDRESS z
        PORT NUMBER         0
        MESSAGE TYPE        COMBINED
        DATA LENGTH         l
        DATA...             shortened packets
            PORT 8b
            TYPE 8b
            LEN  8b (l)
            DATA 8b * l
    Custom
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n
        SOURCE ADDRESS      y
        DESTINATION ADDRESS z
        PORT NUMBER         p
        MESSAGE TYPE        DATA
        DATA LENGTH         l
        DATA...             CUSTOM (l bytes)
*/


#define TM_VERSION 1

//PACKET PART SIZES
#define TM_HEADER_LENGTH 9
#define TM_DATA_LENGTH   256 - TM_HEADER_LENGTH

//RETURN FLAGS
#define TM_OK                   0b0000000000000000
#define TM_ERR_NULL             0b0000000000000001 //packet is null or given buffer is null
#define TM_ERR_HEADER_LEN       0b0000000000000010 //invalid header length
#define TM_ERR_TRUNCATED        0b0000000000000100 //data truncated during copying (probably buffer length)
#define TM_ERR_VERSION          0b0000000000001000 //invalid version in header
#define TM_ERR_DEVICE_TYPE      0b0000000000100000 //unknown device type in header
#define TM_ERR_SOURCE_ADDR      0b0000000001000000 //invalid source address in header
#define TM_ERR_DATA_LEN         0b0000000010000000 //invalid data length in header
#define TM_ERR_MSG_TYPE         0b0000000100000000 //invalid message type in header
#define TM_ERR_MSG_TYPE_PORT    0b0000001000000000 //invalid message type and port combination
#define TM_ERR_MSG_TYPE_ADDRESS 0b0000010000000000 //invalid message type and address combination
#define TM_ERR_MSG_TYPE_LEN     0b0000100000000000 //invalid message type and length combination

//TM_ERR_MESSAGES 
#define TM_ERR_MSG_UNHANDLED     1
#define TM_ERR_SERVICE_UNHANDLED 2
#define TM_ERR_ADDRESS_LIMIT     3
#define TM_ERR_UNKNOWN_NODE      4
#define TM_ERR_UNKNOWN_PORT      5

//Check packet returns
#define TM_IN_ANSWER        0 //OK, ERR and custom are the only valid responses
#define TM_IN_REQUEST       1 //packet is a request (anything but OK and ERR)
#define TM_IN_BROADCAST     2 //packet is a broadcast -> handle (, answer) and forward
#define TM_IN_FORWARD       3 //incoming packet is not for us and is to be forwarded
#define TM_ERR_IN_ANSWER    4 //packet is an answer, but we did not make any requests
#define TM_ERR_IN_TYPE      5 //incoming packet is a response but is not OK or ERR or custom
#define TM_ERR_IN_PORT      6 //incoming packet is for us but port is not registed
#define TM_ERR_IN_DUPLICATE 7 //incoming packet is probably a diplicate

//check packet
#define TM_ERR_PORT_COUNT        1
#define TM_ERR_PORT_EXISTS       2
#define TM_ERR_DEFAULT_PORT      3
#define TM_ERR_PORT_DOESNT_EXIST 4

//save packet id
#define TM_ERR_SENT_COUNT   1
//#define TM_ERR_DUPLICATE_ID 2

/*----(MESSAGE TYPES)----*/
//response
#define TM_MSG_OK                0 //ok erponse
#define TM_MSG_ERR               1 //error response
//request
#define TM_MSG_PING              2  //ping device
#define TM_MSG_REGISTER          3  //register to the newtwork
#define TM_MSG_PORT_ANOUNCEMENT  4  //anounce what ports a NODE is using
#define TM_MSG_RESET             5  //request a device configuration reset
#define TM_MSG_STATUS            6  //RAW string
#define TM_MSG_CUSTOM            7 //send custom data (to some port)
#define TM_MSG_MAX               8
//#define TM_MSG_ROUTE_ANOUNCEMENT 6  //anounc already known routes //TODO v1.1 
//#define TM_MSG_COMBINED          9  //data contain multiple messages in format |TYPE|LEN|DATA|TYPE... //TODO v1.1

//NODE TYPES
#define TM_TYPE_GATEWAY 0 //device is a gateway
#define TM_TYPE_NODE    1 //device is a normal node
#define TM_TYPE_LP_NODE 2 //device is a low power node
#define TM_TYPE_MAX     3


//DEFAULT CONFIG
#define TM_DEFAULT_ADDRESS   0
#define TM_BROADCAST_ADDRESS 255
#define TM_DEFAULT_PORT      0

#define TM_TIME_TO_STALE     3000 //time in ms for a saved packet to become stale
#define TM_PORT_COUNT        2    //how manny ports to store (minimum 1)
#define TM_SENT_Q_SIZE       10   //this array is of type uint64_t, so it takes a lot of space!


typedef union{
    struct {
        uint8_t version;
        uint8_t node_type;
        uint8_t msg_id_msb;
        uint8_t msg_id_lsb; //cannot be zero, must differ from last sent message
        uint8_t src_addr;
        uint8_t dst_addr;
        uint8_t port;
        uint8_t msg_type;
        uint8_t data_len;
        uint8_t data[TM_DATA_LENGTH];
    } fields;
    uint8_t raw[TM_HEADER_LENGTH + TM_DATA_LENGTH];
} packet_t;


class TinyMesh {
private:
    uint8_t version                       = TM_VERSION;           //supported TinyMesh version
    uint8_t address                       = TM_DEFAULT_ADDRESS;   //this NODE address
    uint8_t gateway                       = TM_BROADCAST_ADDRESS; //gateway address
    uint8_t node_type                     = TM_TYPE_NODE;         //this NODE type
    uint8_t sent_cnt                      = 0;                    //current number of saved sent packets
    uint8_t port_cnt                      = 0;                    //current number of saved ports
    uint64_t sent_packets[TM_SENT_Q_SIZE] = {0};
    //port_cfg_t ports[TM_PORT_COUNT]       = {{TM_DEFAULT_PORT, TM_PORT_INOUT | TM_PORT_DATA_CUSTOM}}; //TODO use it (add, remove, get, set,...)
    uint8_t ports[TM_PORT_COUNT]          = {TM_DEFAULT_PORT}; //TODO use it (add, remove, get, set,...)



    uint16_t lcg(uint16_t seed = 0);
    uint32_t (*millis)() = nullptr;

public:

    /** @brief Create TinyMesh instance.
     * The class containes a simple LCG for pseudo random message ID generation where the seed is used.
     * @param node_type Node type
     * @param seed Starting seed for LCG
     */
    TinyMesh(uint8_t node_type);

    /** @brief Create TinyMesh instance.
     * The class containes a simple LCG for pseudo random message ID generation where the seed is used.
     * @param node_type Node type
     * @param address Node address
     * @param seed Starting seed for LCG
     */
    TinyMesh(uint8_t node_type, uint8_t address);

    /** @brief Create TinyMesh instance.
     * The class containes a simple LCG for pseudo random message ID generation where the seed is used.
     * @param node_type Node type
     * @param version Protocol version to be supported
     * @param address Node address
     * @param seed Starting seed for LCG
     */
    TinyMesh(uint8_t node_type, uint8_t address, uint8_t version);
    ~TinyMesh();


    void setSeed(uint16_t seed = 42069);

    /** @brief Register time keeping function for creating timestamps for packets.
     * It is expected that this function returns time in milliseconds between individual calls of this function.
     * If other time unit is used, edit TM_TIME_TO_STALE macro (or redefine it).
     * 
     * @param millis Function pointer to function returning milliseconds since boot
     */
    void registerMillis(uint32_t (*millis)());

    /** @brief Set protocol version.
     * Max version is TM_VERSION.
     * @param version 
     */
    void setVersion(uint8_t version);

    /** @brief Set node address.
     * Address 255 will be converted to 0
     * 
     * @param address 
     */
    void setAddress(uint8_t address);

    void setGatewayAddress(uint8_t address);
    
    /** @brief Set node type.
     * Unknown node types will be converted to TM_TYPE_NODE.
     * 
     * @param node_type 
     */
    void setDeviceType(uint8_t node_type);

    /** @brief Set message ID for specified packet.
     * 
     * @param packet Packet for which to set message ID
     * @param msg_id New message ID
     */
    void setMessageId(packet_t *packet, uint16_t msg_id);

    uint8_t getVersion();
    uint8_t getAddress();
    uint8_t getGatewayAddress();
    uint8_t getDeviceType();

    /** @brief Construct and return the message ID.
     * 
     * @param packet Packet from which to get message ID
     * @return Message ID
     */
    uint16_t getMessageId(packet_t packet);

    /** @brief Change port type or if port does not exist, create it
     * 
     * @param port Port number
     * @param type Port type
     * @return TM_OK on succesful change or creation, TM_ERR_PORT_COUNT when there is not enough space to create new port.
     */
    uint8_t addPort(uint8_t port);

    /** @brief Get copy of a port
     * 
     * @param port Port to search for
     * @return port_cfg_t - Found port on success, {0, 0} on failure.
     */
    bool hasPort(uint8_t port);
    
    /** @brief Remove port
     * 
     * @param port Port number
     * @return port_cfg_t - Found port on success, {0, 0} on failure.
     */
    uint8_t removePort(uint8_t port);

    /** @brief Build packet from a buffer.
     * 
     * @param packet Packet to which to store header and data
     * @param data Raw packet data
     * @param length Data length
     * @return TM_OK on succes, TM_ERR_... macros on error
     */
    uint16_t buildPacket(packet_t *packet, uint8_t *buffer, uint8_t length);

    /** @brief Build packet from specific data.
     * Message ID is generated using builtin LCG.
     * Runs checkHeader() at the end.
     * 
     * @param packet Packet poiner where to store header and data
     * @param destination Destination node address
     * @param message_type Message type
     * @param port Port to which to send data
     * @param data Data which to send
     * @param length Length of data
     * @return TM_OK on succes, TM_ERR_... macros on error
     */
    uint16_t buildPacket(packet_t *packet, uint8_t destination, uint8_t message_type, uint8_t port = 0, uint8_t *data = nullptr, uint8_t length = 0);
    
    /** @brief Build packet from specific data.
     * Runs checkHeader() at the end.
     * 
     * @param packet Packet poiner where to store header and data
     * @param destination Destination node address
     * @param message_type Message type
     * @param msg_id Message ID
     * @param port Port to which to send data
     * @param data Data which to send
     * @param length Length of data
     * @return TM_OK on succes, TM_ERR_... macros on error
     */
    uint16_t buildPacket(packet_t *packet, uint8_t destination, uint8_t message_type, uint16_t msg_id, uint8_t port = 0, uint8_t *data = nullptr, uint8_t length = 0);

    /** @brief Check if stored packet has valid header and header data.
     * 
     * @return TM_OK on succes, TM_ERR_... macros on error
     */
    uint16_t checkHeader(packet_t packet);

    /** @brief Create packet ID that is mostly internally used to save sent/forwarded packets
     * 
     * @param message_id Packet message ID
     * @param src_addr Packet source address
     * @param dst_addr Packet destination address
     * @param time Time of packet ID creation
     * @return 
     */
    uint32_t createPacketID(uint16_t message_id, uint8_t src_addr, uint8_t dst_addr);
    /** @brief Create packet ID that is mostly internally used to save sent/forwarded packets
     * 
     * @param packet Packet whose ID is to be created 
     * @param time Time of packet ID creation
     * @return 
     */
    uint32_t createPacketID(packet_t packet);

    /** @brief Used before sending data on some interface to later check if incoming packet is an answer to our packet. 
     * Save packet to queue for later checking if incoming packet is an answer to rhis packet.
     * Uses a continuous array of predefined size TM_SAVE_Q_SIZE. If force == true, it will just shift packet IDs if new one is to be added.
     * Same array is also used for forwarded messages to check for duplicit packets (should be called to save those too).
     * 
     * @param packet Packet whose poacket ID is to be created and stored.
     * @param time Current time in ms to which TM_TIME_TO_STALE is added when saving. When 0 registered millis function is used or tts is set to 0.
     * @param force Force save packet even if there is not space left. Will just shift the last packet out of the array.
     * @return TM_OK on success, TM_ERR_NO_SPACE if the underlying array is full.
     */
    uint8_t savePacket(packet_t packet, uint32_t time = 0, bool force = false);

    /** @brief 
     * 
     * @param packet_id 
     * @param time 
     * @param force 
     * @return 
     */
    uint8_t savePacketID(uint32_t packet_id, uint32_t time =0 , bool force = false);

    /** @brief Clear entire sent_packet queue
     * 
     * @param time Current time in ms to compare packet tts with. If 0, registered millis is used or packet is not cleared.
     * @param force Force clear all packets
     */
    uint8_t clearSavedPackets(uint32_t time = 0, bool force = false);

    /** @brief Check if incoming packet is an answer to some of our previously sent packets, or is to be forwarded or is a duplicate.
     * Request or any previous packet must first be saved using savePacket()).
     * 
     * @param packet Packet whose ID is to be checked
     * @return TM_IN_ANSWER or TM_IN_REQUEST if packet is valid for us, TM_ERR_IN... otherwise.
     */
    uint8_t checkPacket(packet_t packet);
};
