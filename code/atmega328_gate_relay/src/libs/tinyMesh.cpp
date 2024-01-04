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
    setNodeType(node_type);
}

TinyMesh::TinyMesh(uint8_t node_type, uint8_t address) : TinyMesh(node_type) {
    setAddress(address);
}

TinyMesh::~TinyMesh() {

}


void TinyMesh::setSeed(uint16_t seed) {
    lcg(seed);
}

void TinyMesh::registerMillis(unsigned long (*millis)()) {
    this->millis = millis;
}


void TinyMesh::setVersion(uint8_t version) {
    if (version > TM_VERSION)
        this->version = TM_VERSION;
    else
        this->version = version;
}

void TinyMesh::setAddress(uint8_t address) {
    if (address == TM_BROADCAST_ADDRESS)
        this->address = TM_DEFAULT_ADDRESS;
    else
        this->address = address;
}

void TinyMesh::setGatewayAddress(uint8_t address) {
    this->gateway = address;
}

void TinyMesh::setNodeType(uint8_t node_type) {
    if (node_type > TM_NODE_TYPE_OTHER)
        this->node_type = TM_NODE_TYPE_OTHER;
    else
        this->node_type = node_type;
}

void TinyMesh::setMessageId(packet_t *packet, uint16_t message_id) {
    packet->fields.msg_id_msb = message_id >> 8;
    packet->fields.msg_id_lsb = message_id;
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

uint8_t TinyMesh::getNodeType() {
    return this->node_type;
}

uint16_t TinyMesh::getMessageId(packet_t *packet) {
    return ((uint16_t)packet->fields.msg_id_msb) << 8 | packet->fields.msg_id_lsb;
}


uint8_t TinyMesh::buildPacket(packet_t *packet, uint8_t destination, uint16_t message_id, uint8_t message_type, uint8_t *data, uint8_t length) {
    uint16_t ret = TM_OK;

    if (packet == nullptr)
        return TM_ERR_NULL;

    //build packet
    setMessageId(packet, message_id);
    packet->fields.version                   = this->version;
    packet->fields.source                    = this->address;
    packet->fields.destination               = destination;
    packet->fields.data_length               = length;
    packet->fields.flags.fields.node_type    = this->node_type;
    packet->fields.flags.fields.message_type = message_type;

    //check if header is valid
    ret |= checkHeader(packet);

    //return now if there are no data to copy
    if (!length)
        return ret;

    //data are null, but some are to be copied -> don't copy anything
    if (data == nullptr && length != 0)
        return ret |= TM_ERR_DATA_NULL;

    if (length > TM_DATA_LENGTH) {
        memcpy(packet->fields.data, data, TM_DATA_LENGTH);
        ret |= TM_ERR_DATA_LENGTH;
    }
    else
        memcpy(packet->fields.data, data, length);

    return ret;
}

uint8_t TinyMesh::buildPacket(packet_t *packet, uint8_t *buffer, uint8_t length) {
    uint8_t ret = TM_OK;

    if (packet == nullptr || buffer == nullptr)
        return TM_ERR_NULL;

    //build packet
    //setMessageId(packet, message_id);

    if (length < TM_HEADER_LENGTH)
        return TM_ERR_BUF_LEN;

    //copy header
    memcpy(packet->raw, buffer, TM_HEADER_LENGTH);

    //check if header is valid
    ret |= checkHeader(packet);


    uint8_t len2copy = 0;
    if (ret & TM_ERR_DATA_LENGTH)
        len2copy = TM_DATA_LENGTH;
    else
        len2copy = packet->fields.data_length;

    //packet is shorter than wanted data
    if (length  - TM_HEADER_LENGTH < len2copy) {
        memcpy(packet->fields.data, buffer + TM_HEADER_LENGTH, length  - TM_HEADER_LENGTH);
        ret |= TM_ERR_BUF_LEN;
    }
    else
        memcpy(packet->fields.data, buffer + TM_HEADER_LENGTH, len2copy);

    return ret;
}

uint8_t TinyMesh::checkHeader(packet_t *packet) {
    uint8_t ret = TM_OK;

    //unsuported version
    if (packet->fields.version > TM_VERSION)
        ret |= TM_ERR_VERSION;

    if (packet->fields.source == TM_BROADCAST_ADDRESS)
        ret |= TM_ERR_ADDRESS;

    //data too long
    if (packet->fields.data_length > TM_DATA_LENGTH)
        ret |= TM_ERR_DATA_LENGTH;

    switch (packet->fields.flags.fields.message_type) {
        case TM_MSG_OK: break;
        case TM_MSG_ERR:
            if (packet->fields.data_length != 1)
                ret |= TM_ERR_MSG_TYPE_LEN;
            break;
        case TM_MSG_REGISTER:
        case TM_MSG_PING:
        case TM_MSG_RESET:
            if (packet->fields.data_length != 0)
                ret |= TM_ERR_MSG_TYPE_LEN;
            break;
        case TM_MSG_STATUS: break;
        case TM_MSG_CUSTOM: break;
        default:
            ret |= TM_ERR_MSG_TYPE;
            break;
    }

    return ret;
}


uint8_t TinyMesh::checkPacket(packet_t *packet) {
    //if packet is in sent queue, it is a duplicate packet
    packet_id_t packet_id = createPacketID(packet);
    for (auto spid : this->sent_queue) {
        if (spid.rid == 0)
            continue;
        if (spid.rid == packet_id.rid)
            return TM_PACKET_DUPLICATE;
    }

    //not for us nor broadcast
    if (packet_id.fields.destination != this->address && packet_id.fields.destination != TM_BROADCAST_ADDRESS)
        return TM_PACKET_FORWARD;

    //go through saved packets and find if this is a response
    packet_id = createPacketID((uint32_t)packet->fields.destination, (uint32_t)packet->fields.source, getMessageId(packet) - 1);
    for (auto spid : this->sent_queue) {
        if (spid.rid == 0)
            continue;

        //packet is a response to something from us
        //if request was a brodcast, don't care about response source
        packet_id_t tmp = packet_id;
        tmp.fields.destination = 0xFF;
        if (spid.rid == packet_id.rid || spid.rid == tmp.rid)
            return TM_PACKET_RESPONSE;
    }

    //packet is not a response to anything from us, but still is a response -> random response
    if (packet_id.fields.destination == this->address && (packet->fields.flags.fields.message_type == TM_MSG_OK || packet->fields.flags.fields.message_type == TM_MSG_ERR))
        return TM_PACKET_RND_RESPONSE;
    
    //packet is a brodcast request
    if (packet_id.fields.destination == TM_BROADCAST_ADDRESS)
        return TM_PACKET_FORWARD | TM_PACKET_REQUEST;

    //anything else is just a normal request
    return TM_PACKET_REQUEST;
}

void TinyMesh::savePacketID(packet_id_t packet_id) {
    //don't add duplicite packet ids (remove later)
    for (auto spid : this->sent_queue) {
        if (spid.rid == 0)
            continue;
        if (spid.rid == packet_id.rid)
            return;
    }

    //shift queue and add new packet_id
    memmove(this->sent_queue + 1, this->sent_queue, sizeof(this->sent_queue) - sizeof(this->sent_queue[0]));
    this->sent_queue[0] = packet_id;
    if (millis != nullptr)
        last_msg_time = millis();
}

void TinyMesh::savePacket(packet_t *packet) {
    savePacketID(createPacketID(packet));
}

uint8_t TinyMesh::clearSentQueue(bool force) {
    if (force || (millis != nullptr && this->last_msg_time + TM_CLEAR_TIME < millis())) {
        this->last_msg_time = millis == nullptr? 0 : millis();
        memset(this->sent_queue, 0, sizeof(this->sent_queue));
        return 1;
    }
    return 0;
}


packet_id_t TinyMesh::createPacketID(packet_t *packet) {
    packet_id_t packet_id;
    packet_id.fields.source = packet->fields.source;
    packet_id.fields.destination = packet->fields.destination;
    packet_id.fields.msg_id_msb = packet->fields.msg_id_msb;
    packet_id.fields.msg_id_lsb = packet->fields.msg_id_lsb;
    return packet_id;
}
packet_id_t TinyMesh::createPacketID(uint8_t source, uint8_t destination, uint16_t message_id) {
    packet_id_t packet_id;
    packet_id.fields.source = source;
    packet_id.fields.destination = destination;
    packet_id.fields.msg_id_msb = message_id >> 8;
    packet_id.fields.msg_id_lsb = message_id;
    return packet_id;
}



uint16_t TinyMesh::lcg(uint16_t seed) {
    static uint16_t x = seed;
    uint16_t a = 25214903917 & 0xFFFF;
    uint16_t c = 11;
    x = (a * x + c) % 0xFFFF;
    return x;
}

