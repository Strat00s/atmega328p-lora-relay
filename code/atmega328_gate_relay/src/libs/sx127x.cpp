/**
 * @file SX127X.cpp
 * @author Lukáš Baštýř (l.bastyr@seznam.cz)
 * @brief SX127X library
 * @version 0.2
 * @date 30-08-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "sx127x.hpp"


//private
void SX127X::setValidFrequency(float frequency) {
    uint32_t f_rf = frequency * (((uint32_t)1 << 19) / 32);
    setRegister(REG_FRF_MSB, f_rf >> 16);
    setRegister(REG_FRF_MID, f_rf >> 8);
    setRegister(REG_FRF_LSB, f_rf);
}

//public
SX127X::SX127X(uint8_t cs, uint8_t rst, uint8_t dio0) {
    this->cs   = cs;
    this->rst  = rst;
    this->dio0 = dio0;
}

SX127X::SX127X(uint8_t cs, uint8_t rst, uint8_t dio0, uint8_t dio1) : SX127X(cs, rst, dio0) {
    this->dio1 = dio1;
}

SX127X::~SX127X() {

}


//callback functions
void SX127X::registerPinMode(void (*func)(uint8_t, uint8_t), uint8_t input, uint8_t output) {
    this->pinMode                   = func;
    this->input                     = input;
    this->output                    = output;
    this->flags.single.has_pin_mode = true;
}

void SX127X::registerPinWrite(void (*func)(uint8_t, uint8_t), uint8_t high, uint8_t low) {
    this->pinWrite                   = func;
    this->high                       = high;
    this->low                        = low;
    this->flags.single.has_pin_write = true;
}

void SX127X::registerPinRead(uint8_t (*func)(uint8_t)) {
    this->pinRead = func;
    this->flags.single.has_pin_read  = true;
}

void SX127X::registerDelay(void (*func)(uint32_t)) {
    this->delay                  = func;
    this->flags.single.has_delay = true;
}

void SX127X::registerMicros(uint32_t (*micros)()) {
    this->micros = micros;
    this->flags.single.has_micros = true;
}

void SX127X::registerSPIBeginTransfer(void (*func)()) {
    this->SPIBeginTransfer              = func;
    this->flags.single.has_spi_start_tr = true;
}

void SX127X::registerSPIEndTransfer(void (*func)()) {
    this->SPIEndTransfer              = func;
    this->flags.single.has_spi_end_tr = true;
}

void SX127X::registerSPITransfer(void (*func)(uint8_t, uint8_t *, size_t)) {
    this->SPITransfer               = func;
    this->flags.single.has_transfer = true;
}


uint8_t SX127X::begin(float frequency, uint8_t sync_word, uint16_t preamble_len, uint8_t bandwidth, uint8_t spreading_factor, uint8_t coding_rate) {
    //check that all required callbacks were set
    if (!this->flags.single.has_pin_write ||
        !this->flags.single.has_pin_read  ||
        !this->flags.single.has_transfer  ||
        !this->flags.single.has_micros    ||
        !this->flags.single.has_delay)
        return ERR_MISSING_CALLBACK;

    //toggle pin modes if it was setup
    if (this->flags.single.has_pin_mode) {
        this->pinMode(this->cs, this->output);
        this->pinMode(this->rst, this->output);
        this->pinMode(this->dio0, this->input);
        if (this->dio1 != 0)
            this->pinMode(this->dio1, this->input);
    }

    //set pins to their default state
    this->pinWrite(this->cs, this->high);
    this->pinWrite(this->rst, this->high);

    //reset the chip on start
    reset();

    //check chip version
    this->chip_version = getVersion();
    switch (this->chip_version) {
        case SX1272_CHIP_VERSION:
        case SX1276_CHIP_VERSION:
        case RFM95_CHIP_VERSION: break;
        //case RFM69_CHIP_VERSION: break;
        default: return ERR_INVALID_CHIP_VERSION;
    }

    //go to standby to change and set default settings
    setMode(SX127X_OP_MODE_STANDBY);

    //set lora mode
    setModemMode(SX127X_MODEM_MODE_LORA);

    //set LoRa sync word
    setSyncWord(sync_word);

    //set preamble length
    setPreambleLength(preamble_len);

    //turn off frequency hopping
    setFrequencyHopping(0);

    //set default rx timeout
    setRxTimeout(100);

    //set frequency
    setFrequency(frequency);

    //125khz bandwidth
    setBandwidth(bandwidth);

    //set spreading factor to 7
    setSpreadingFactor(spreading_factor);

    //set error coding rate to 4/5
    setCodingRate(coding_rate);

    //set automatic gain
    setGain(SX127X_LNA_GAIN_AUTOMATIC);

    //set default current limit
    setCurrentLimit(100);

    //set some power output
    setPower(13);

    //enable crc
    setCRC(true);

    return 0;
}


void SX127X::reset() {
    this->pinWrite(this->rst, this->low);
    this->delay(1);
    this->pinWrite(this->rst, this->high);
    this->delay(10);    //5ms just don't seem to be reliable with ESP
}

uint8_t SX127X::getVersion() {
    return readRegister(REG_VERSION);
}

void SX127X::setMode(uint8_t mode) {
    //skip going into standby (most used mode) when already in standby
    if (mode == SX127X_OP_MODE_STANDBY && this->in_standby)
        return;

    setRegister(REG_OP_MODE, mode, 0, 2);

    //save if in standby
    this->in_standby = mode == SX127X_OP_MODE_STANDBY;
}

void SX127X::setModemMode(uint8_t modem) {
    setMode(SX127X_OP_MODE_SLEEP);

    setRegister(REG_OP_MODE, modem, 7, 7);

    setMode(SX127X_OP_MODE_STANDBY);
}

uint8_t SX127X::getModemMode() {
    return readRegister(REG_OP_MODE, 7, 7);
}


void SX127X::setSyncWord(uint8_t sync_word) {
    writeRegister(REG_SYNC_WORD, sync_word);
}

uint8_t SX127X::setPreambleLength(uint16_t preamble_len) {
    if (preamble_len < 6)
        return ERR_INVALID_PREAMBLE_LEN;
    writeRegister(REG_PREAMBLE_MSB, (uint8_t)((preamble_len >> 8) & 0xFF));
    writeRegister(REG_PREAMBLE_LSB, (uint8_t)(preamble_len & 0xFF));
    return 0;
}

uint8_t SX127X::setFrequency(float frequency) {
    if (this->chip_version == SX1272_CHIP_VERSION && (frequency < 860 || frequency > 1020))
        return ERR_INNVALID_FREQUENCY;
    if (this->chip_version == SX1276_CHIP_VERSION && (frequency < 137 || frequency > 1020))
        return ERR_INNVALID_FREQUENCY;
    //if (this->chip_version == RFM69_CHIP_VERSION)
    if (this->chip_version == RFM95_CHIP_VERSION && (frequency < 433 || frequency > 915))
        return ERR_INNVALID_FREQUENCY;

    //go to standby first
    setMode(SX127X_OP_MODE_STANDBY);

    //calculate and save new frequency
    setValidFrequency(frequency);

    //save the frequency
    this->frequency = frequency;

    return 0;
}

uint8_t SX127X::setBandwidth(uint8_t bandwidth) {
    if (getModemMode() != SX127X_MODEM_MODE_LORA)
        return ERR_INVALID_MODEM_MODE;

    switch (bandwidth) {
        case LORA_BANDWIDTH_7_8kHz:   this->bw =   7.8;  break;
        case LORA_BANDWIDTH_10_4kHz:  this->bw =  10.4;  break;
        case LORA_BANDWIDTH_15_6kHz:  this->bw =  15.6;  break;
        case LORA_BANDWIDTH_20_8kHz:  this->bw =  20.8;  break;
        case LORA_BANDWIDTH_31_25kHz: this->bw =  31.25; break;
        case LORA_BANDWIDTH_41_7kHz:  this->bw =  41.7;  break;
        case LORA_BANDWIDTH_62_5kHz:  this->bw =  62.5;  break;
        case LORA_BANDWIDTH_125kHz:   this->bw = 125.0;  break;
        case LORA_BANDWIDTH_250kHz:   this->bw = 250.0;  break;
        case LORA_BANDWIDTH_500kHz:   this->bw = 500.0;  break;
        default: return ERR_INVALID_BANDWIDTH;
    }

    setRegister(REG_MODEM_CONFIG_1, bandwidth, 4, 7);

    //check and enable/disable LDRO
    setLowDataRateOptimalization();
    return 0;
}

uint8_t SX127X::setSpreadingFactor(uint8_t spreading_factor) {
    if (getModemMode() != SX127X_MODEM_MODE_LORA)
        return ERR_INVALID_MODEM_MODE;

    switch (spreading_factor) {
        case LORA_SPREADING_FACTOR_6:
        case LORA_SPREADING_FACTOR_7:
        case LORA_SPREADING_FACTOR_8:
        case LORA_SPREADING_FACTOR_9:
        case LORA_SPREADING_FACTOR_10:
        case LORA_SPREADING_FACTOR_11:
        case LORA_SPREADING_FACTOR_12: break;
        default: return ERR_INVALID_SPREADING_FACTOR;
    }

    setRegister(REG_MODEM_CONFIG_2, spreading_factor, 4, 7);
    //special configuration for spreading factor 6
    if (spreading_factor == LORA_SPREADING_FACTOR_6) {
        setImplicitHeader();
        //setRegister(REG_MODEM_CONFIG_1, LORA_IMPLICIT_HEADER, 0, 0);
        setRegister(REG_DETECT_OPTIMIZE, LORA_DETECTION_OPTIMIZE_SF6, 0, 2);
        setRegister(REG_DETECTION_THRESHOLD, LORA_DETECTION_THRESHOLD_SF6);
    }
    //default everything when sf != 6
    else {
        setExplicitHeader();
        //setRegister(REG_MODEM_CONFIG_1, LORA_EXPLICIT_HEADER, 0, 0);
        setRegister(REG_DETECT_OPTIMIZE, LORA_DETECTION_OPTIMIZE_SF7_12, 0, 2);
        setRegister(REG_DETECTION_THRESHOLD, LORA_DETECTION_THRESHOLD_SF7_12);
    }

    this->sf = spreading_factor >> 4;

    //check and enable/disable LDRO
    setLowDataRateOptimalization();
    return 0;
}

uint8_t SX127X::setImplicitHeader() {
    if (getModemMode() != SX127X_MODEM_MODE_LORA)
        return ERR_INVALID_MODEM_MODE;

    setRegister(REG_MODEM_CONFIG_1, LORA_IMPLICIT_HEADER, 0, 0);

    return 0;
}

uint8_t SX127X::setExplicitHeader() {
    if (getModemMode() != SX127X_MODEM_MODE_LORA)
        return ERR_INVALID_MODEM_MODE;

    setRegister(REG_MODEM_CONFIG_1, LORA_EXPLICIT_HEADER, 0, 0);

    return 0;
}

void SX127X::setPayloadLength(uint8_t length) {
    writeRegister(REG_PAYLOAD_LENGTH, length);
}

uint8_t SX127X::setCodingRate(uint8_t coding_rate) {
    if (getModemMode() != SX127X_MODEM_MODE_LORA)
        return ERR_INVALID_MODEM_MODE;

    switch(coding_rate) {
        case LORA_CODING_RATE_4_5:
        case LORA_CODING_RATE_4_6:
        case LORA_CODING_RATE_4_7:
        case LORA_CODING_RATE_4_8: break;
        default: return ERR_INVALID_CODING_RATE;
    }

    //this->cr = (coding_rate + 0b00001000) >> 1;
    setRegister(REG_MODEM_CONFIG_1, coding_rate, 1, 3);

    return 0;
}

uint8_t SX127X::setGain(uint8_t gain) {
    if ((gain >> 5) > 6)
        return ERR_INVALID_GAIN;

    setMode(SX127X_OP_MODE_STANDBY);

    if (getModemMode() != SX127X_MODEM_MODE_LORA)
        return ERR_INVALID_MODEM_MODE;

    //set automatic gain control
    if (gain == 0)
        setRegister(REG_MODEM_CONFIG_3, LORA_AGC_AUTO_ON, 2, 2);
    else {
        setRegister(REG_MODEM_CONFIG_3, LORA_AGC_AUTO_OFF, 2, 2);
        setRegister(REG_LNA, gain, 5, 7);
    }

    return 0;
}

uint8_t SX127X::setCRC(bool enable) {
    if (getModemMode() != SX127X_MODEM_MODE_LORA)
        return ERR_INVALID_MODEM_MODE;
    
    if(enable)
        setRegister(REG_MODEM_CONFIG_2, LORA_RX_PAYLOAD_CRC_ON, 2, 2);
    else
        setRegister(REG_MODEM_CONFIG_2, LORA_RX_PAYLOAD_CRC_OFF, 2, 2);
    
    return 0;
}

void SX127X::setLowDataRateOptimalization(bool force) {
    if (getModemMode() != SX127X_MODEM_MODE_LORA)
        return;

    float ts = float(uint16_t(1) << this->sf) / this->bw;
    if(ts >= 16.0 || force)
        setRegister(REG_MODEM_CONFIG_3, LORA_LOW_DATA_RATE_OPT_ON, 3, 3);
    else
        setRegister(REG_MODEM_CONFIG_3, LORA_LOW_DATA_RATE_OPT_OFF, 3, 3);
}

void SX127X::setFrequencyHopping(uint8_t period) {
    writeRegister(REG_HOP_PERIOD, HOP_PERIOD_OFF);
}

uint8_t SX127X::setRxTimeout(uint16_t symbol_cnt) {
    uint8_t status = 0;
    if (symbol_cnt < 4 || symbol_cnt > 1023) {
        status = WARN_INVALID_TIMEOUT_SYMBOL_CNT;
        symbol_cnt = 100;
    }
    
    writeRegister(REG_SYMB_TIMEOUT_LSB, symbol_cnt);
    setRegister(REG_MODEM_CONFIG_2, symbol_cnt >> 8, 0, 1);

    this->symbol_cnt = symbol_cnt;

    return status;
}


uint8_t SX127X::setCurrentLimit(uint8_t max_current) {
    if ((max_current < 45 || max_current > 240) && max_current != 0)
        return ERR_INVALID_CURRENT_LIMIT;

    setMode(SX127X_OP_MODE_STANDBY);

    //disable OCP
    if (max_current == 0)
        setRegister(REG_OCP, SX127X_OCP_OFF, 5, 5);
    if (max_current <= 120)
        setRegister(REG_OCP, ((max_current - 45) / 5) | SX127X_OCP_ON, 0, 5);
    else if (max_current <= 240)
        setRegister(REG_OCP, ((max_current + 30) / 10) | SX127X_OCP_ON, 0, 5);

    return 0;
}

uint8_t SX127X::setPower(int8_t power, bool pa_boost) {
    if (pa_boost && power != 20 && (power < 2 || power > 17))
        return ERR_INVALID_POWER;
    if (!pa_boost && (power < -4 || power > 15))
        return ERR_INVALID_POWER;

    uint8_t pa_config = 0;
    setMode(SX127X_OP_MODE_STANDBY);

    //using PA_BOOST pin
    if (pa_boost) {
        //calculate correct register value for specified power
        if (power == 20)
            pa_config = SX127X_PA_SELECT_BOOST | 0b01110000 | 0b00001111;
        else 
            pa_config = SX127X_PA_SELECT_BOOST | 0b01110000 | (power - 2);
        writeRegister(REG_PA_CONFIG, pa_config);
        setRegister(REG_PA_DAC, SX127X_PA_BOOST_ON, 0, 2);
    }
    //using RFO pin
    else {
        //calculate correct register value for specified power.
        //Is shifted by -0.2dBm for -4dBm (-4.2dBm)
        if (power == -4)
            pa_config = SX127X_PA_SELECT_RFO | 0b00000000 | (power + 4);
        //get rid of the 0.2 offset
        else if (power < 0)
            pa_config = SX127X_PA_SELECT_RFO | 0b00100000 | (power + 3);
        else
            pa_config = SX127X_PA_SELECT_RFO | 0b01110000 | power;
        writeRegister(REG_PA_CONFIG, pa_config);
        setRegister(REG_PA_DAC, SX127X_PA_BOOST_OFF, 0, 2);
    }

    return 0;
}


void SX127X::errataFix(bool receive) {
    if (getModemMode() != SX127X_MODEM_MODE_LORA)
        return;

    //sx1272/3 errata
    //Receiver Spurious Reception fix
    if (this->chip_version == SX1272_CHIP_VERSION)
        setRegister(0x31, 0b10000000, 7, 7);

    //sx1276/7/8 errata
    if (this->chip_version == SX1278_CHIP_VERSION) {
        //Sensitivity Optimization with a 500 kHz Bandwidth fix
        uint8_t bandwidth = readRegister(REG_MODEM_CONFIG_1, 4, 7);
        if (bandwidth == LORA_BANDWIDTH_500kHz) {
            if (this->frequency >= 862.0 && this->frequency <= 1020.0) {
                writeRegister(0x36, 0x02);
                writeRegister(0x3A, 0x64);
            }
            if (this->frequency >= 410.0 && this->frequency <= 525.0) {
                writeRegister(0x36, 0x02);
                writeRegister(0x3A, 0x7F);
            }
        }
        //reset the register back
        else
            writeRegister(0x36, 0x03);
        
        //Receiver Spurious Reception of a LoRa Signal fix
        uint8_t reg0x31_7 = 0;
        uint8_t reg0x2f = 0x44;
        uint8_t reg0x30 = 0x00;
        float new_frequency = this->frequency;
        switch (bandwidth) {
            case LORA_BANDWIDTH_7_8kHz:
                reg0x2f = 0x48;
                new_frequency += 0.00781;
                break;
            case LORA_BANDWIDTH_10_4kHz:
                new_frequency += 0.01042;
                break;
            case LORA_BANDWIDTH_15_6kHz:
                new_frequency += 0.01562;
                break;
            case LORA_BANDWIDTH_20_8kHz:
                new_frequency += 0.02083;
                break;
            case LORA_BANDWIDTH_31_25kHz:
                new_frequency += 0.03125;
                break;
            case LORA_BANDWIDTH_41_7kHz:
                new_frequency += 0.04167;
                break;
            case LORA_BANDWIDTH_62_5kHz:
                reg0x2f = 0x40;
                break;
            case LORA_BANDWIDTH_125kHz:
                reg0x2f = 0x40;
                break;
            case LORA_BANDWIDTH_250kHz:
                reg0x2f = 0x40;
                break;
            case LORA_BANDWIDTH_500kHz:
                reg0x31_7 = 0b10000000;
                reg0x2f = readRegister(0x2F);
                reg0x30 = readRegister(0x30);
                break;
            default: return;
        }

        setMode(SX127X_OP_MODE_STANDBY);

        //modify frequency only when receiving
        if (receive)
            setValidFrequency(new_frequency);
        else
            setValidFrequency(this->frequency);

        //modify the registers
        setRegister(0x31, reg0x31_7, 7, 7);
        writeRegister(0x2F, reg0x2f);
        writeRegister(0x30, reg0x30);
    }

}


uint8_t SX127X::transmit(uint8_t *data, uint8_t length, uint8_t soft) {
    setMode(SX127X_OP_MODE_STANDBY);

    //set IO mapping for dio0 to be end of transmission
    setRegister(REG_DIO_MAPPING_1, DIO0_LORA_TX_DONE, 6, 7);

    //set packet length
    setRegister(REG_PAYLOAD_LENGTH, length);

    //set FIFO pointers (all 256 bytes used for TX)
    setRegister(REG_FIFO_TX_BASE_ADDR, 0);
    setRegister(REG_FIFO_ADDR_PTR, 0);

    //write data to FIFO
    writeRegistersBurst(REG_FIFO, data, length);

    //apply errata fixes
    errataFix(false);

    //clear interrupt flags
    clearIrqFlags();

    //start transmission
    setMode(SX127X_OP_MODE_TX);

    //if dio0 is not working properly, check the register instead of waiting for the pin to go high
    if (soft)
        while(!(readRegister(REG_IRQ_FLAGS) & IRQ_FLAG_TX_DONE));
    //wait for transmission to end
    else
        while(!this->pinRead(this->dio0));

    //go back to standby
    setMode(SX127X_OP_MODE_STANDBY);

    //read and clear interrupts
    uint8_t reg = getIrqFlags();
    clearIrqFlags();
    return reg;
}


uint8_t SX127X::receiveBlocking(uint8_t *data, uint8_t length) {
    receiveStart(length);

    uint8_t status = 0;
    //calcualte timeout (us)
    uint32_t timeout = (symbol_cnt * float(uint16_t(1) << this->sf) / this->bw) * 1000;
    uint32_t start = this->micros(); 

    //wait for successful reception or exit on timeout
    while (!this->pinRead(this->dio0)) {
        //has dio1
        if (this->dio1 != 0) {
            //timeout on dio1
            if (this->pinRead(this->dio1)) {
                status = ERR_RX_TIMEOUT;
                break;
            }
        }
        //no dio1
        else {
            //timeout from timer
            if (this->micros() - start > timeout) {
                status = ERR_RX_TIMEOUT;
                break;
            }
        }
    }

    status = receiveEnd();

    //read data on success
    if (!status)
        readData(data, length);

    return status;
}

void SX127X::receiveStart(uint8_t length) {
    setMode(SX127X_OP_MODE_STANDBY);

    //set IO mapping
    setRegister(REG_DIO_MAPPING_1, DIO0_LORA_RX_DONE | DIO1_LORA_RX_TIMEOUT, 4, 7);

    //when using SF6, payload length must be know in advance
    if (this->sf == 6)
        setRegister(REG_PAYLOAD_LENGTH, length);

    //set FIFO pointers (all 256 bytes used for RX)
    setRegister(REG_FIFO_RX_BASE_ADDR, 0);  //where to start storing new data
    setRegister(REG_FIFO_ADDR_PTR, 0);      //from where to read on reception

    //apply errata fixes
    errataFix(true);

    //clear interrupt flags
    clearIrqFlags();

    //start receiving
    setMode(SX127X_OP_MODE_RXSINGLE);
}

uint8_t SX127X::receiveEnd() {
    //go back to standby
    setMode(SX127X_OP_MODE_STANDBY);

    uint8_t status = checkPayloadIntegrity();
    
    //clear IRQ flags
    writeRegister(REG_IRQ_FLAGS, 0xFF);

    return status;
}

void SX127X::receiveContinuous(uint8_t length) {
    setMode(SX127X_OP_MODE_STANDBY);

    //set IO mapping
    setRegister(REG_DIO_MAPPING_1, DIO0_LORA_RX_DONE, 6, 7);

    //when using SF6, payload length must be know in advance
    if (this->sf == 6)
        setRegister(REG_PAYLOAD_LENGTH, length);

    //set FIFO pointers (all 256 bytes used for RX)
    setRegister(REG_FIFO_RX_BASE_ADDR, 0);  //where to start storing new data
    setRegister(REG_FIFO_ADDR_PTR, 0);      //from where to read on reception

    //apply errata fixes
    errataFix(true);

    //clear interrupt flags
    clearIrqFlags();

    //start receiving
    setMode(SX127X_OP_MODE_RXCONTINUOUS);
}


uint8_t SX127X::checkPayloadIntegrity() {
    uint8_t irq_flags = getIrqFlags();

    //check valid header and CRC
    if (!(irq_flags & IRQ_FLAG_VALID_HEADER))
        return ERR_INVALID_HEADER;
    if (getIrqFlags() & IRQ_FLAG_PAYLOAD_CRC_ERROR)
        return ERR_CRC_MISMATCH;
    return 0;
}

uint8_t SX127X::getPayloadLength() {
    //when using SF6, payload length is known in advance
    if (this->sf == 6)
        return readRegister(REG_PAYLOAD_LENGTH);
    
    return readRegister(REG_RX_NB_BYTES);
}

void SX127X::readData(uint8_t *data, uint8_t length) {
    uint8_t payload_length = getPayloadLength();

    if (length > payload_length || length == 0)
        length = payload_length;

    readRegistersBurst(REG_FIFO, data, length);
}


uint8_t SX127X::getIrqFlags() {
    return readRegister(REG_IRQ_FLAGS);
}

void SX127X::clearIrqFlags() {
    writeRegister(REG_IRQ_FLAGS, 0xFF);
}


void SX127X::SPIMakeTransaction(uint8_t addr, uint8_t *data, size_t length) {
    if (this->flags.single.has_spi_start_tr)
        this->SPIBeginTransfer();

    this->pinWrite(this->cs, this->low);
    this->SPITransfer(addr, data, length);
    this->pinWrite(this->cs, this->high);

    if (this->flags.single.has_spi_end_tr)
        this->SPIEndTransfer();
}

uint8_t SX127X::readRegister(uint8_t addr, uint8_t mask_lsb, uint8_t mask_msb) {
    addr &= SX127X_READ_MASK;
    uint8_t reg = 0;
    SPIMakeTransaction(addr, &reg);

    uint8_t keep_mask = ~(0xFF >> (8 - mask_lsb) | 0xFF << (mask_msb + 1));
    return reg & keep_mask;
}

void SX127X::readRegistersBurst(uint8_t addr, uint8_t *data, size_t length) {
    addr &= SX127X_READ_MASK;
    SPIMakeTransaction(addr, data, length);
}

void SX127X::writeRegister(uint8_t addr, uint8_t data) {
    addr |= SX127X_WRITE_MASK;
    SPIMakeTransaction(addr, &data);
}

void SX127X::writeRegistersBurst(uint8_t addr, uint8_t *data, size_t length) {
    addr |= SX127X_WRITE_MASK;
    SPIMakeTransaction(addr, data, length);
}

void SX127X::setRegister(uint8_t addr, uint8_t data, uint8_t mask_lsb, uint8_t mask_msb) {
    uint8_t reg = readRegister(addr);
    
    //calculate register clear mask
    uint8_t clr_mask = (0xFF >> (8 - mask_lsb) | 0xFF << (mask_msb + 1));
    data = (reg & clr_mask) | (data & ~clr_mask);

    writeRegister(addr, data);
}
