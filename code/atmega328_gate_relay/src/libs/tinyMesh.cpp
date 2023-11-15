#include "tinyMesh.hpp"
#include <string.h>

TinyMesh::TinyMesh() {

}

TinyMesh::TinyMesh(uint8_t device_type) {
    setDeviceType(device_type);
}

TinyMesh::TinyMesh(uint8_t address, uint8_t device_type) : TinyMesh(device_type) {
    setAddress(address);
}

TinyMesh::TinyMesh(uint8_t version, uint8_t address, uint8_t device_type) : TinyMesh(address, device_type) {
    setVersion(version);
}

TinyMesh::~TinyMesh() {

}


void TinyMesh::setVersion(uint8_t version) {
    if (version > TM_VERSION)
        this->version = TM_VERSION;
    else
        this->version = version;
}

void TinyMesh::setAddress(uint8_t address) {
    if (address == 255)
        this->address = 0;
    else
        this->address = address;
}

void TinyMesh::setDeviceType(uint8_t device_type) {
    if (device_type != TM_TYPE_GATEWAY && device_type != TM_TYPE_NODE && device_type != TM_TYPE_LP_NODE)
        this->device_type = TM_TYPE_NODE;
    else
        this->device_type = device_type;
}
    
uint8_t TinyMesh::getVersion() {
    return this->version;
}

uint8_t TinyMesh::getAddress() {
    return this->address;
}

uint8_t TinyMesh::getDeviceType() {
    return this->address;
}


uint16_t TinyMesh::buildPacket(packet_t *packet, uint8_t destination, uint8_t message_type, uint8_t port, uint8_t *buffer, uint8_t length) {
    uint16_t ret = TM_OK;
    
    if (packet == nullptr)
        return TM_ERR_NULL;

    //build packet
    packet->fields.version     = this->version;
    packet->fields.device_type = this->device_type;
    //TODO generate message id
    packet->fields.source_addr = this->address;
    packet->fields.dest_addr   = destination;
    packet->fields.port        = port;
    packet->fields.msg_type    = message_type;
    packet->fields.data_length = length;

    //check if packet is valid
    ret |= checkPacket(packet);

    //no data, but length is given -> don't copy anything
    if (length != 0 && buffer == nullptr)
        ret |= TM_ERR_DATA_NULL;
    //data are given
    else if (buffer != nullptr) {
        //valid length -> copy all data
        if (!(ret & TM_ERR_DATA_LEN))
            memcpy(packet->fields.data, buffer, length);
        //length > max -> truncate
        else {
            memcpy(packet->fields.data, buffer, TM_DATA_LENGTH);
            ret |= TM_ERR_TRUNCATED;
        }
    }

    return ret;
}

uint16_t TinyMesh::readPacket(packet_t *packet, uint8_t *buffer, uint8_t length) {
    uint16_t ret = TM_OK;

    if (packet == nullptr || buffer == nullptr)
        return TM_ERR_NULL;

    //at least header size
    if (length < TM_HEADER_LENGTH)
        return TM_ERR_PACKET_LEN;

    //copy fields
    memcpy(packet->raw, buffer, TM_HEADER_LENGTH);

    //check packet
    ret |= checkPacket(packet);

    //data length <= max data length
    if (!(ret & TM_ERR_DATA_LEN)) {
        //input buffer length more than packet data length
        if (length - TM_HEADER_LENGTH > packet->fields.data_length) {
            ret |= TM_ERR_TRUNCATED;
            memcpy(packet->fields.data, buffer, packet->fields.data_length);
        }
        //input buffer length less than packet data length
        if (length - TM_HEADER_LENGTH < packet->fields.data_length) {
            ret |= TM_ERR_DATA_LEN;
            memcpy(packet->fields.data, buffer, length - TM_HEADER_LENGTH);
        }
        else
            memcpy(packet->fields.data, buffer, packet->fields.data_length);
    }
    //data length > max data length -> truncate
    else {
        ret |= TM_ERR_TRUNCATED;
        memcpy(packet->fields.data, buffer, TM_DATA_LENGTH);
    }

    return ret;
}

uint16_t TinyMesh::buildAnswerHeader(packet_t *packet) {
    uint16_t ret = TM_OK;

    packet->fields.version = TM_VERSION;
    packet->fields.device_type = this->device_type;
    
    uint16_t msg_id = ((uint16_t)(packet->fields.msg_id_msb) << 8 | (uint16_t)(packet->fields.msg_id_lsb)) + 1;
    packet->fields.msg_id_msb = (uint8_t)(msg_id >> 8);
    packet->fields.msg_id_lsb = (uint8_t)msg_id;

    uint8_t new_source_address = this->address;

    packet->fields.port = 0;
    packet->fields.data_length = 0;
    
    //packet->fields.port = TM_DEFAULT_PORT;
    switch (packet->fields.msg_type) {
    case TM_MSG_REGISTER:
        packet->fields.msg_type = TM_MSG_DEVICE_CONFIG;
        break;
    
    case TM_MSG_DEVICE_CONFIG:
        if (packet->fields.data[0] == 0 || packet->fields.data[0] == 255) {
            ret |= TM_ERR_CFG_ADDRESS;
            packet->fields.msg_type    = TM_MSG_ERR;
            packet->fields.data_length = 1;
            packet->fields.data[0]     = TM_EC_CFG_ADDRESS;
            break;
        }
        new_source_address = packet->fields.data[0];
        [[fallthrough]]
    case TM_MSG_PING:
    case TM_MSG_PORT_ADVERT:
    case TM_MSG_ROUTE_SOLICIT:
    case TM_MSG_ROUTE_ANOUNC:
    case TM_MSG_RESET:
    case TM_MSG_STATUS:
    case TM_MSG_COMBINED:
    case TM_MSG_CUSTOM: packet->fields.msg_type = TM_MSG_OK; break;
    default: break;
    }

    packet->fields.dest_addr   = packet->fields.source_addr;
    packet->fields.source_addr = new_source_address;

    return ret;
}


//TODO test
uint16_t TinyMesh::checkPacket(packet_t *packet) {
    uint16_t ret = TM_OK;

    //unsuported version
    if (packet->fields.version > TM_VERSION)
        ret |= TM_ERR_VERSION;

    //unknown device type
    if (packet->fields.device_type > TM_TYPE_LP_NODE)
        ret |= TM_ERR_DEVICE_TYPE;

    //invalid message id
    if (packet->fields.msg_id_lsb == 0 && packet->fields.msg_id_msb == 0)
        ret |= TM_ERR_MSG_ID;

    //invalid source address
    if (packet->fields.source_addr == 255)
        ret |= TM_ERR_SOURCE_ADDR;

    //unknown message type
    if (packet->fields.msg_type > TM_MSG_CUSTOM)
        ret |= TM_ERR_MSG_TYPE;

    //custom and port 0, predefined and port other than 0
    if ((packet->fields.msg_type == TM_MSG_CUSTOM && packet->fields.port == 0) ||
        (packet->fields.msg_type <= TM_MSG_COMBINED && packet->fields.port != 0))
        ret |= TM_ERR_MSG_TYPE_PORT;

    //data too long
    if (packet->fields.data_length > TM_DATA_LENGTH)
        ret |= TM_ERR_DATA_LEN;

    //message type, address and data length invalid combinations
    switch (packet->fields.msg_type) {
        case TM_MSG_REGISTER:
            if (packet->fields.source_addr != 0 || packet->fields.dest_addr != 255)
                ret |= TM_ERR_MSG_TYPE_ADDRESS;
            [[fallthrough]];
        case TM_MSG_OK:
        case TM_MSG_PING:
        case TM_MSG_RESET:
            if (packet->fields.data_length != 0)
                ret |= TM_ERR_MSG_TYPE_LEN;
            break;

        case TM_MSG_DEVICE_CONFIG:
            if (packet->fields.dest_addr != 0)
                ret |= TM_ERR_MSG_TYPE_ADDRESS;
            [[fallthrough]];
        case TM_MSG_ERR:
            if (packet->fields.data_length != 1)
                ret |= TM_ERR_MSG_TYPE_LEN;
            break;

        case TM_MSG_PORT_ADVERT:
            if (packet->fields.data_length < 2 || packet->fields.data_length % 2 != 0)
                ret |= TM_ERR_MSG_TYPE_LEN;
            if (packet->fields.dest_addr != this->gateway_address)
                ret |= TM_ERR_MSG_TYPE_ADDRESS;
            break;

        case TM_MSG_ROUTE_ANOUNC:
            if (packet->fields.dest_addr != this->gateway_address)
                ret |= TM_ERR_MSG_TYPE_ADDRESS;
            [[fallthrough]];
        case TM_MSG_ROUTE_SOLICIT:
            if (packet->fields.data_length < 2)
                ret |= TM_ERR_MSG_TYPE_LEN;
            break;
    default: break;
    }

    return ret;
}

uint16_t TinyMesh::getMessageId(packet_t packet) {
    return (((uint16_t)packet.fields.msg_id_msb) << 8) | ((uint16_t)packet.fields.msg_id_lsb);
}


uint16_t lcg(uint16_t seed) {
    return 42;
}

