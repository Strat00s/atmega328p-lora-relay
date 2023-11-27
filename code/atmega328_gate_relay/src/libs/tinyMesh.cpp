/** @file tinyMesh.cpp
 * @author Lukáš Baštýř (l.bastyr@seznam.cz, 492875)
 * @brief TinyMesh is a simple protocol for IoT devices.
 * @version 0.1
 * @date 27-11-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "tinyMesh.hpp"
#include <string.h>


TinyMesh::TinyMesh(uint8_t node_type) {
    setDeviceType(node_type);
}

TinyMesh::TinyMesh(uint8_t node_type, uint8_t address) : TinyMesh(node_type) {
    setAddress(address);
}

TinyMesh::TinyMesh(uint8_t node_type, uint8_t address, uint8_t version) : TinyMesh(address, node_type) {
    setVersion(version);
}

TinyMesh::~TinyMesh() {

}


void TinyMesh::setSeed(uint16_t seed) {
    lcg(seed);
}

void TinyMesh::registerMillis(uint32_t (*millis)()) {
    this->millis = millis;
}


void TinyMesh::setVersion(uint8_t version) {
    if (version > TM_VERSION)
        this->version = TM_VERSION;
    else
        this->version = version;
}

void TinyMesh::setAddress(uint8_t address) {
    if (address == 255)
        this->address = TM_DEFAULT_ADDRESS;
    else
        this->address = address;
}

void TinyMesh::setGatewayAddress(uint8_t address) {
    this->gateway = address;
}

void TinyMesh::setDeviceType(uint8_t node_type) {
    if (node_type > TM_TYPE_LP_NODE)
        this->node_type = TM_TYPE_NODE;
    else
        this->node_type = node_type;
}

void TinyMesh::setMessageId(packet_t *packet, uint16_t msg_id) {
    packet->fields.msg_id_msb = msg_id >> 8;
    packet->fields.msg_id_lsb = msg_id;
}

uint8_t TinyMesh::getVersion() {
    return this->version;
}

uint8_t TinyMesh::getAddress() {
    return this->address;
}

uint8_t TinyMesh::getGatewayAddress() {
    return this->gateway;
}

uint8_t TinyMesh::getDeviceType() {
    return this->address;
}

uint16_t TinyMesh::getMessageId(packet_t packet) {
    return ((uint16_t)packet.fields.msg_id_msb) << 8 | packet.fields.msg_id_lsb;
}


uint8_t TinyMesh::addPort(uint8_t port) {
    //check if port exists
    for (size_t i = 0; i < TM_PORT_COUNT; i++) {
        if (this->ports[i] == port) {
            return TM_ERR_PORT_EXISTS;
        }
    }

    //port does not exist -> create it
    for (size_t i = 1; i < TM_PORT_COUNT; i++) {
        if (this->ports[i] == 0) {
            this->ports[i] = port;
            return TM_OK;
        }
    }

    //not enough space to create new port
    return TM_ERR_PORT_COUNT;
}

bool TinyMesh::hasPort(uint8_t port) {
    for (size_t i = 0; i < TM_PORT_COUNT; i++) {
        if (this->ports[i] == port)
            return true;
    }
    return false;
}

uint8_t TinyMesh::removePort(uint8_t port) {
    if (port == 0)
        return TM_ERR_DEFAULT_PORT;

    for (size_t i = 1; i < TM_PORT_COUNT; i++) {
        if (this->ports[i] == port) {
            this->ports[i] = 0;
            return TM_OK;
        }
    }
    return TM_ERR_PORT_DOESNT_EXIST;
}


uint16_t TinyMesh::buildPacket(packet_t *packet, uint8_t *buffer, uint8_t length) {
    uint16_t ret = TM_OK;

    if (packet == nullptr || buffer == nullptr)
        return TM_ERR_NULL;

    //at least header size
    if (length < TM_HEADER_LENGTH)
        return TM_ERR_HEADER_LEN;

    //copy fields
    memcpy(packet->raw, buffer, TM_HEADER_LENGTH);

    //check packet
    ret |= checkHeader(*packet);

    //data length <= max data length
    if (!(ret & TM_ERR_DATA_LEN)) {
        //input buffer length more than packet data length
        if (length - TM_HEADER_LENGTH > packet->fields.data_len) {
            ret |= TM_ERR_TRUNCATED;
            memcpy(packet->fields.data, buffer + TM_HEADER_LENGTH, packet->fields.data_len);
        }
        //input buffer length less than packet data length
        if (length - TM_HEADER_LENGTH < packet->fields.data_len) {
            ret |= TM_ERR_DATA_LEN;
            memcpy(packet->fields.data, buffer + TM_HEADER_LENGTH, length - TM_HEADER_LENGTH);
        }
        else
            memcpy(packet->fields.data, buffer + TM_HEADER_LENGTH, packet->fields.data_len);
    }
    //data length > max data length -> truncate
    else {
        ret |= TM_ERR_TRUNCATED;
        memcpy(packet->fields.data, buffer + TM_HEADER_LENGTH, TM_DATA_LENGTH);
    }

    return ret;
}

uint16_t TinyMesh::buildPacket(packet_t *packet, uint8_t destination, uint8_t message_type, uint8_t port, uint8_t *buffer, uint8_t length) {
    uint8_t ret = buildPacket(packet, destination, message_type, lcg(), port, buffer, length);
    return ret;
}

uint16_t TinyMesh::buildPacket(packet_t *packet, uint8_t destination, uint8_t message_type, uint16_t msg_id, uint8_t port, uint8_t *buffer, uint8_t length) {
    uint16_t ret = TM_OK;

    if (packet == nullptr)
        return TM_ERR_NULL;

    //build packet
    packet->fields.version   = this->version;
    packet->fields.node_type = this->node_type;
    setMessageId(packet, msg_id);
    packet->fields.src_addr  = this->address;
    packet->fields.dst_addr  = destination;
    packet->fields.port      = port;
    packet->fields.msg_type  = message_type;
    packet->fields.data_len  = length;

    //check if header is valid
    ret |= checkHeader(*packet);

    //no data, but length is given -> don't copy anything
    if (length != 0 && buffer == nullptr)
        ret |= TM_ERR_NULL;
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


uint16_t TinyMesh::checkHeader(packet_t packet) {
    uint16_t ret = TM_OK;

    //unsuported version
    if (packet.fields.version > TM_VERSION)
        ret |= TM_ERR_VERSION;

    //unknown device type
    if (packet.fields.node_type >= TM_TYPE_MAX)
        ret |= TM_ERR_DEVICE_TYPE;

    //invalid message id
    //if ()
    //    ret |= TM_ERR_MSG_ID;

    //invalid source address
    if (packet.fields.src_addr == TM_BROADCAST_ADDRESS)
        ret |= TM_ERR_SOURCE_ADDR;

    //unknown message type
    if (packet.fields.msg_type >= TM_MSG_MAX)
        ret |= TM_ERR_MSG_TYPE;

    //custom and port 0, predefined and port other than 0
    if ((packet.fields.msg_type == TM_MSG_CUSTOM && packet.fields.port == TM_DEFAULT_PORT) ||
        (packet.fields.msg_type <  TM_MSG_CUSTOM && packet.fields.port != TM_DEFAULT_PORT))
        ret |= TM_ERR_MSG_TYPE_PORT;

    //data too long
    if (packet.fields.data_len > TM_DATA_LENGTH)
        ret |= TM_ERR_DATA_LEN;

    //message type, address and data length invalid combinations
    switch (packet.fields.msg_type) {
        case TM_MSG_REGISTER:
        case TM_MSG_PING:
        case TM_MSG_RESET:
            if (packet.fields.data_len != 0)
                ret |= TM_ERR_MSG_TYPE_LEN;
            break;

        case TM_MSG_ERR:
            if (packet.fields.data_len != 1)
                ret |= TM_ERR_MSG_TYPE_LEN;
            break;
        case TM_MSG_PORT_ANOUNCEMENT:
            if (packet.fields.data_len == 0)
                ret |= TM_ERR_MSG_TYPE_LEN;
            break;
        //TODO v1.1
        //case TM_MSG_ROUTE_ANOUNCEMENT:
        //    if (packet.fields.data_len % 2 != 0)
        //        ret |= TM_ERR_MSG_TYPE_LEN;
        //    break;
        default: break;
    }

    return ret;
}

uint8_t TinyMesh::savePacketID(uint32_t packet_id, uint32_t time, bool force) {
    if (millis != nullptr && !time)
        time = millis();
    time += TM_TIME_TO_STALE;

    if (sent_cnt >= TM_SENT_Q_SIZE) {
        //if full and force, just shift everything
        if (force) {
            for (int i = TM_SENT_Q_SIZE - 1; i > 0; i--) {
                sent_packets[i] = sent_packets[i - 1];
            }
            sent_packets[0] = (uint64_t)time << 32 | packet_id;
        }
        return TM_ERR_SENT_COUNT;
    }

    //find first empty space and save packet ID
    for (int i = 0; i < TM_SENT_Q_SIZE; i++) {
        if (sent_packets[i] == 0) {
            sent_packets[i] = ((uint64_t)time) << 32 | packet_id;
            break;
        }
    }
    sent_cnt++;

    return TM_OK;
}

uint8_t TinyMesh::savePacket(packet_t packet, uint32_t time, bool force) {
    uint32_t packet_id = createPacketID(packet);
    return savePacketID(packet_id, time, force);
}

uint8_t TinyMesh::clearSavedPackets(uint32_t time,  bool force) {
    //millis not registered, time not given, not forced -> nothing to do
    if (millis == nullptr && !time && !force)
        return 0;
    
    uint8_t removed = 0;
    for (int i = 0; i < TM_SENT_Q_SIZE; i++) {
        if (sent_packets[i] == 0)
            continue;
        //remove all on force
        if (force) {
            sent_packets[i] = 0;
            sent_cnt = 0;
            removed++;
            continue;
        }
        //remove stale packet IDs
        if ((millis != nullptr && sent_packets[i] >> 32 <= millis()) || (time && sent_packets[i] >> 32 <= time)) {
            sent_packets[i] = 0;
            sent_cnt--;
            removed++;
        }
    }
    return removed;
}

uint8_t TinyMesh::checkPacket(packet_t packet) {
    //if packet is in sent queue, it is a duplicate packet
    uint32_t packet_id = createPacketID(packet);
    for (int i = 0; i < TM_SENT_Q_SIZE; i++) {
        if (packet_id == (uint32_t)sent_packets[i])
            return TM_ERR_IN_DUPLICATE;
    }

    //packet is a broadcast
    if (packet.fields.dst_addr == TM_BROADCAST_ADDRESS) {
        if (packet.fields.port != 0)
            return TM_ERR_IN_PORT;
        return TM_IN_BROADCAST;
    }

    //user must save this packet manually
    if (packet.fields.dst_addr != this->address) {
        return TM_IN_FORWARD;
    }

    //check if packet is a response to our previous request
    packet_id = createPacketID(getMessageId(packet) - 1, packet.fields.dst_addr, packet.fields.src_addr);
    for (int i = 0; i < TM_SENT_Q_SIZE; i++) {
        //answer to request
        if (packet_id == (uint32_t)sent_packets[i] || packet_id >> 8 == ((uint32_t)sent_packets[i]) >> 8) {
            //check for valid port
            if (!this->hasPort(packet.fields.port))
                return TM_ERR_IN_PORT;
            
            //packet is a valid answer to our request
            if (packet.fields.msg_type == TM_MSG_OK || packet.fields.msg_type == TM_MSG_ERR || packet.fields.msg_type == TM_MSG_CUSTOM)
                return TM_IN_ANSWER;

            return TM_ERR_IN_TYPE;
        }
    }

    //random answer to us
    if (packet.fields.msg_type == TM_MSG_OK || packet.fields.msg_type == TM_MSG_ERR)
        return TM_ERR_IN_ANSWER;

    //packet is for us but is not a response -> new packet
    //user must save this packet manually
    if (!this->hasPort(packet.fields.port))
        return TM_ERR_IN_PORT;
    return TM_IN_REQUEST;
}


uint32_t TinyMesh::createPacketID(uint16_t message_id, uint8_t src_addr, uint8_t dst_addr) {
    return ((uint32_t)message_id) << 16 | ((uint16_t)src_addr) << 8 | dst_addr;
}

uint32_t TinyMesh::createPacketID(packet_t packet) {
    return ((uint32_t)getMessageId(packet)) << 16 | ((uint16_t)packet.fields.src_addr) << 8 | packet.fields.dst_addr;
}

uint16_t TinyMesh::lcg(uint16_t seed) {
    static uint16_t x = seed;
    uint16_t a = 25214903917 & 0xFFFF;
    uint16_t c = 11;
    x = (a * x + c) % 0xFFFF;
    return x;
}

