#pragma once
#include <stdio.h>

// VER DEV MIM MIL SAR DAR PRT M_T LEN DATA
/*
# Packet structure
------------HEADER------------
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


# Predefined messages packet structure
    ok
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n+1
        SOURCE ADDRESS      z
        DESTINATION ADDRESS y
        PORT NUMBER         0
        MESSAGE TYPE        OK
        DATA LENGTH         0
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
    Device config
        VERSION             1
        DEVICE TYPE         G
        MESSAGE ID          n+1
        SOURCE ADDRESS      GATREWAY_ADDRESS
        DESTINATION ADDRESS 0
        PORT NUMBER         0
        MESSAGE TYPE        CONFIG
        DATA LENGTH         1
        DATA...             DEVICE_ADDRESS
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
    Port advertisement
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n
        SOURCE ADDRESS      y
        DESTINATION ADDRESS z (gateway)
        PORT NUMBER         0
        MESSAGE TYPE        PORT_ADVERTISEMENT
        DATA LENGTH         2 * l
        DATA...             l pairs of PORT and DATA_TYPE
            PORT      8b
            DATA_TYPE 8b
                PORT DIR  0bxx000000
                DATA TYPE 0b00xxxxxx
    Route solicitation
        VERSION             1
        DEVICE TYPE         G
        MESSAGE ID          n
        SOURCE ADDRESS      x
        DESTINATION ADDRESS y
        PORT NUMBER         0
        MESSAGE TYPE        ROUTE_SOLICITATION
        DATA LENGTH         2 + l
        DATA...             LISTENER_ADDRESS, PORT and l extra PORTs
            LISTENER_ADDRESS 8b
            LISTEN_PORT      8b
            EXTRA_PORTS      8b * l
    Route anouncement
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n
        SOURCE ADDRESS      y
        DESTINATION ADDRESS z (gateway)
        PORT NUMBER         0
        MESSAGE TYPE        ROUTE_ANOUNCEMENT
        DATA LENGTH         2 + l
        DATA...             LISTENER_ADDRESS, PORT and l extra PORTs
            LISTENER_ADDRESS 8b
            LISTEN_PORT      8b
            EXTRA_PORTS      8b * l
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
    Combined
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

/*
When sending a new message, message ID must be randomly generated (or at least last used ID cannot be used)
When responding to a message, increase message ID and dela with possible overflows
If node has adrress 0, it must listen on address 255

Any message not using port 0, gateways must rebroadcast
If message with n+1 msg_id is received before broadcasting, scrap the rebroadcast

Everyone must rebroadcast when address does not match


# No gateway
Send registration
No response -> probably no gateway
Address is kept 0
Listen on address 255 for all your ports
    //if port 0 and message id == last sent message id + 1 -> listen to message (probably a new gateway)
Start normal operation (sending data to ports) with address 255 (broadcast)
Wait for possible response, but none is expected

### Gateway is added
Send registration
No response -> probably no gateway
Set own address (to anything but 0 and 255)
If any message is intercepted, answer on port 0 with combined message and id of n+1 (so normal answer)
    ...COMBINED|7+x|ERR,1,NETWORK_CHANGE|CFG,2,DID,KEY|...

# Gateway exists
Send registration
Get address (and key)
listen on all own ports
start normal operation
register ports 
    if ports are output, expect gateway to send you port solicitation when manully routed
anounce already registered ports



# Reserved fields:
    ADDRESS 0 - for devices with no address. Only gateways should answer to these
    ADDRESS 255 - broadcast. All devices should listen on this address, but only gateway should answer
    PORT: 0 - all predefined message types other than DATA

if (addr == 0)
    listen to all addresses


ADDR 0 -> reserved for new nodes
ADDR 255 -> broadcasted to all nodes

### Message types
# Check if device is in network
request:  ping (uint32_t timestamp)
     VER DEV MIM MIL SAR DAR PRT M_T LEN
    | 1 | x |   n   |SID|RID| 0 |PNG| 0 |
    x - depends on device
    n - randomly generated other than 0
    SID - source address/ID
    DID - destination address/ID

response: OK (timestamp back)
     VER DEV MIM MIL SAR DAR PRT M_T LEN
    | 1 | x |  n+1  |DID|SID| 0 | OK| 0 |


# Adding new device
request:  REGISTER
     VER DEV MIM MIL SAR DAR PRT M_T LEN
    | 1 | x |   n   | 0 |255| 0 |REG| 0 |
    x - depends on device
    n - randomly generated other than 0

resposne: DEVICE_CONFIG (device ID, secret key), ERR (8bit err code)
     VER DEV MIM MIL SAR DAR PRT M_T LEN DATA
    | 1 | G |  n+1  |GID| 0 | 0 |CFG| 2 |DID|KEY|
    | 1 | G |  n+1  |GID| 0 | 0 |ERR| 1 |ERR_CODE|
    G - gateway specific fields
    GID - gateway address/ID
    DID - new device address/ID
    KEY - secret key for usage on network
    ERR - error code

response: OK, ERR(8bit err code)
     VER DEV MIM MIL SAR DAR PRT M_T LEN DATA
    | 1 | x |  n+2  |DID|GID| 0 | OK| 0 |
    | 1 | x |  n+2  |DID|GID| 0 |ERR| 1 |ERR|
    x - depends on device
    n - randomly generated other than 0
    SID - source address/ID
    DID - destination address/ID


# Sending data:
request:  send (data length, data)
     VER DEV MIM MIL SAR DAR PRT M_T LEN DATA
    | 1 | x |   n   |SID|DID| y |DTA| z |DATA|
    x - depends on device
    n - randomly generated other than 0
    SID - source address/ID
    DID - destination address/ID
    y - service port/ID
    z - data length

    If DID == 255, node automatically registers this port on gateway and user must manually router it to the desired output node (using mqtt)


response  OK, ERR (8bit err code)
     VER DEV MIM MIL SAR DAR PRT M_T LEN DATA...
    | 1 | x |  n+1  |DID|SID| 0 | OK| 0 |
    | 1 | x |  n+1  |DID|SID| 0 |ERR| 1 |ERR_CODE|


# Requesting data:
request:  data request (16bit data id)
response: send (data length, data)
response: OK, ERR (8bit err code)


# port advertisement:
request: port advertisement (port, data_type pairs)
     VER DEV MIM MIL SAR DAR PRT M_T LEN DATA
    | 1 | x |   n   |SID|255| 0 |PRT| z |PORT|TYPE|...|
    x - depends on device
    n - randomly generated other than 0
    SID - source address/ID
    z - data length in multiples of 2

response: OK, ERR (err_code)
     VER DEV MIM MIL SAR DAR PRT M_T LEN DATA...
    | 1 | G |  n+1  |GID|SID| 0 | OK| 0 |
    | 1 | G |  n+1  |GID|SID| 0 |ERR| 1 |ERR_CODE|


# Device types
0 Gateway
1 Node
2 LP Node //is not expected to response



version          -> protocol version
device type      -> idk
msg_id           -> distinguish between repeated messages
source addr      -> to whom data belong, to whom to answser
destination addr -> to whom to send the data, from whom to expect and aswer
port             -> what "service" are we interested in from destination, which service are we supposed to provide
message type     -> prespecified messages


address + port -> define operations and data destination

*/

#define CALLBACKS 0



#define TM_VERSION 1

//PACKET SIZES
#define TM_HEADER_LENGTH 9
#define TM_DATA_LENGTH   256 - TM_HEADER_LENGTH

//RETURN FLAGS
#define TM_OK                   0b0000000000000000

#define TM_ERR_NULL             0b0000000000000001
#define TM_ERR_PACKET_LEN       0b0000010000000000
#define TM_ERR_TRUNCATED        0b0001000000000000

#define TM_ERR_VERSION          0b0000000000000010
#define TM_ERR_MSG_ID           0b0000000000000100
#define TM_ERR_DEVICE_TYPE      0b0000000000001000
#define TM_ERR_SOURCE_ADDR      0b0000000000010000
#define TM_ERR_DATA_LEN         0b0000000000100000
#define TM_ERR_MSG_TYPE         0b0000000001000000
#define TM_ERR_MSG_TYPE_PORT    0b0000000010000000
#define TM_ERR_MSG_TYPE_ADDRESS 0b0000000100000000
#define TM_ERR_MSG_TYPE_LEN     0b0000001000000000
#define TM_ERR_DATA_NULL        0b0000100000000000

#define TM_ERR_CFG_ADDRESS      0b0001000000000000

//MESSAGE TYPES
#define TM_MSG_OK            0
#define TM_MSG_ERR           1
#define TM_MSG_PING          2  //ping device
#define TM_MSG_REGISTER      3  //register to the newtwork
#define TM_MSG_DEVICE_CONFIG 4  //send device configuration back
#define TM_MSG_PORT_ADVERT   5  //advertise custom port for listening/accepting data on
#define TM_MSG_ROUTE_SOLICIT 6  //when user manually routes ports and addresses, send this to output node
#define TM_MSG_ROUTE_ANOUNC  7  //anounc already known routes
#define TM_MSG_RESET         8  //
#define TM_MSG_STATUS        9  //RAW string
#define TM_MSG_COMBINED      10 //data contain multiple messages in format |TYPE|LEN|DATA|TYPE...
#define TM_MSG_CUSTOM        11 //send custom data (to some port)

//PORT DATA TYPES
#define TM_PORT_DATA_NONE   0
#define TM_PORT_DATA_INT8   1
#define TM_PORT_DATA_INT16  2
#define TM_PORT_DATA_INT32  3
#define TM_PORT_DATA_STR    4
#define TM_PORT_DATA_CUSTOM 5

//PORT DIRECTIONS
#define TM_PORT_IN      0b00000000;
#define TM_PORT_OUT     0b01000000;
#define TM_PORT_INOUT   0b10000000;

//NODE TYPES
#define TM_TYPE_GATEWAY 0
#define TM_TYPE_NODE    1
#define TM_TYPE_LP_NODE 2

//ERROR CODES
#define TM_EC_CFG_ADDRESS 1
#define TM_


#define TM_POS_VERSION  0
#define TM_POS_DEV_TYPE 1
#define TM_POS_MSG_ID_M 2
#define TM_POS_MSG_ID_L 3
#define TM_POS_SRC_ADDR 4
#define TM_POS_DST_ADDR 5
#define TM_POS_PORT     6
#define TM_POS_MSG_TYPE 7
#define TM_POS_DATA_LEN 8


//DEFAULT CONFIG
#define TM_DEFAULT_ADDRESS   0
#define TM_BROADCAST_ADDRESS 255
#define TM_DEFAULT_PORT      0

#define PORT_COUNT      10
#define SEND_QUEUE_SIZE 10


typedef union{
    struct {
        uint8_t version;
        uint8_t device_type;
        uint8_t msg_id_msb;
        uint8_t msg_id_lsb; //cannot be zero, must differ from last sent message
        uint8_t source_addr;
        uint8_t dest_addr;
        uint8_t port;
        uint8_t msg_type;
        uint8_t data_length;
        uint8_t data[TM_DATA_LENGTH];
    } fields;
    uint8_t raw[TM_HEADER_LENGTH + TM_DATA_LENGTH];
} packet_t;

typedef struct {
    uint8_t dest_addr;
    uint8_t port;
    uint8_t msg_type;
    uint8_t length;
    uint8_t data[TM_DATA_LENGTH];
} short_packet_t;

typedef struct {
    uint8_t port;
    uint8_t dir_type;
} port_cfg_t;


class TinyMesh {
private:
    uint8_t version         = TM_VERSION;
    uint8_t address         = 0;
    uint8_t gateway_address = 0;
    uint8_t device_type     = 0;
    uint16_t timeout        = 2000;

    //flags_8b_t flags;
    
    uint16_t lcg(uint16_t seed);


    /*----(CALLBACKS REQUIRED)----*/
#if CALLBACKS == 1
    port_cfg_t port_cfg[PORT_COUNT];
    short_packet_t send_queue[SEND_QUEUE_SIZE];
    bool (*hasData)();
    void (*getData)(uint8_t *buffer, uint8_t length);
    void (*sendData)(uint8_t *buffer, uint8_t length);
    void (*delay)(uint32_t delay_ms);
#endif

public:
    TinyMesh();
    TinyMesh(uint8_t device_type);
    TinyMesh(uint8_t address, uint8_t device_type);
    TinyMesh(uint8_t version, uint8_t address, uint8_t device_type);
    ~TinyMesh();


    void setVersion(uint8_t version);
    void setAddress(uint8_t address);
    void setDeviceType(uint8_t device_type);
    
    uint8_t getVersion();
    uint8_t getAddress();
    uint8_t getDeviceType();


    //uint8_t addPort(port_cfg_t port_configuration);
    //uint8_t addPort(uint8_t port, uint8_t direction, uint8_t data_type);

    /** @brief Build packet from specified data
     * 
     * @param packet Packet poiner where to store header and data
     * @param destination Destination node address
     * @param message_type Message type
     * @param port Port to which to send data
     * @param data Data which to send
     * @param length Length of data
     * @return 
     */
    uint16_t buildPacket(packet_t *packet, uint8_t destination, uint8_t message_type, uint8_t port = 0, uint8_t *data = nullptr, uint8_t length = 0);

    /** @brief 
     * 
     * @param packet Packet to which to store header and data
     * @param data Raw packet data
     * @param length Data length
     * @return 
     */
    uint16_t readPacket(packet_t *packet, uint8_t *buffer, uint8_t length);

    /** @brief Try and answer the packet and overwrite it as answer
     * 
     * @param packet Input packet (in correct format) to be read and rewriten as output packet
     * @return 
     */
    uint16_t buildAnswerHeader(packet_t *packet);

    /** @brief Check if stored packet has valid header and header data.
     * Sets flags when field contain invalid values
     * 
     * @return TM_OK on succes, TM_ERR_PACKET_CHECK_FAILED on error
     */
    uint16_t checkPacket(packet_t *packet);


    uint16_t getMessageId(packet_t packet);


    /*----(CALLBACKS REQUIRED)----*/
#if CALLBACKS == 1
    void registerHasData(bool (*func)());
    void registerGetData(void (*func)(uint8_t, uint8_t));
    void registerSendData(void (*func)(uint8_t));
    void registerDelay(void (*func)(uint32_t));

    /** @brief Send pin to destination and wait for response.
     * Works only if all callbacks are setup properly.
     *
     * @param destination Destination node address
     * @param result Ping result (in ms)
     * @param timeout Timeout in ms to wait for a response
     * @return request status
     */
    uint8_t sendPing(uint8_t destination, uint16_t *result, uint16_t timeout);
    
    /** @brief Send pin to destination and wait for response (using global timeout)
     * Works only if all callbacks are setup properly.
     * 
     * @param destination Destination node address
     * @param result Ping result
     * @return request status
     */
    uint8_t sendPing(uint8_t destination, uint16_t *result);

    /** @brief Try and register to network
     * Works only if all callbacks are setup properly.
     * 
     * @return request status
     */
    uint8_t sendRegister();
    
    /** @brief
     * Works only if all callbacks are setup properly.
     * 
     * @param destination Destination node address
     * @param device_address New node address 
     * @return request status
     */
    uint8_t sendConfiguration(uint8_t destination, uint8_t device_address);
    
    /** @brief
     * Works only if all callbacks are setup properly.
     * 
     * @param destination Destination node address
     * @param data Data to send
     * @param length Length of data
     * @return request status
     */
    uint8_t sendData(uint8_t destination, uint8_t *data, uint8_t length);
    
    /** @brief
     * Works only if all callbacks are setup properly.
     * 
     * @param ports Array of ports
     * @param length Number of ports
     * @return request status
     */
    uint8_t sendPortAdvertisement(/*gateway addr*/uint8_t *ports, uint8_t length);
    
    /** @brief
     * Works only if all callbacks are setup properly.
     * 
     * @param destination Destination node address
     * @param routers Array of routes
     * @param length Number of routes
     * @return request status
     */
    uint8_t sendRouteSolicitation(uint8_t destination, uint8_t *routes, uint8_t length);
    
    /** @brief
     * Works only if all callbacks are setup properly.
     * 
     * @param routers Array of routes
     * @param length Number of routes
     * @return request status
     */
    uint8_t sendRouteAnouncement(/*gateway addr*/uint8_t *routes, uint8_t length);
    
    /** @brief
     * Works only if all callbacks are setup properly.
     * 
     * @param destination Destination node address
     * @return request status
     */
    uint8_t sendCombined(uint8_t destination/*TODO*/);
    
    /** @brief
     * Works only if all callbacks are setup properly.
     * 
     * @param destination Destination node address
     * @return request status
     */
    uint8_t sendReset(uint8_t destination);
    
    /** @brief
     * Works only if all callbacks are setup properly.
     * 
     * @param destination Destination node address
     * @param status Status string
     * @param str_len Length of status string
     * @return request status
     */
    uint8_t sendStatus(uint8_t destination, char *status, uint8_t str_len);
    
    /** @brief
     * Works only if all callbacks are setup properly.
     * 
     * @param destination Destination node address
     * @return request status
     */
    uint8_t sendOk(uint8_t destination);
    
    /** @brief Send error to destination
     * Works only if all callbacks are setup properly.
     * 
     * @param destination Destination node address
     * @param error_code Error code to send
     * @return request status
     */
    uint8_t sendError(uint8_t destination, uint8_t error_code);

    /** @brief 
     * Works only if all callbacks are setup properly.
     *
     * @param destination Destination node address
     * @param port Port where to send data
     * @param message_type Type of message to send
     * @param data Data to send
     * @param length Length of data
     * @return 
     */
    uint8_t sendPacket(uint8_t destination, uint8_t port, uint8_t message_type, uint8_t *data = nullptr, uint8_t length = 0);

    /** @brief If all callbacks are used, calling this will do all the work.
     * 1. Check if there are any new data.
     * 2. Get and parse the new data.
     * 3. Answer if possible.
     * 
     * @param packet pointer to packet_t structure for storing received packet
     * @return SP_OK on success, SP_ERR_... on error
     */
    uint8_t loop(packet_t *packet);

#endif
};
