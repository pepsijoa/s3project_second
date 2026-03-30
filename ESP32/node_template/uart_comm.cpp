#include "uart_comm.h"
#include <DW1000NgConstants.hpp>
#include <SPI.h>
#include <cstring>

// ── 초기화 ────────────────────────────────────────────────────
void UartComm::begin() {
    proto_parser_reset(&_parser);
    // HardwareSerial(1) = UART1
    // begin(baud, config, rxPin, txPin)
    _serial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    SPI.begin(UWB_PIN_SCK, UWB_PIN_MISO, UWB_PIN_MOSI, UWB_PIN_SS);
    DW1000Ng::initializeNoInterrupt(UWB_PIN_SS, UWB_PIN_RST);

    device_configuration_t uwbConfig = {
        false,
        true,
        true,
        true,
        false,
        SFDMode::STANDARD_SFD,
        Channel::CHANNEL_5,
        DataRate::RATE_6800KBPS,
        PulseFrequency::FREQ_16MHZ,
        PreambleLength::LEN_128,
        PreambleCode::CODE_3
    };

    DW1000Ng::applyConfiguration(uwbConfig);
    DW1000Ng::setNetworkId(10);
    DW1000Ng::setDeviceAddress((uint16_t)NODE_ID);
    DW1000Ng::setAntennaDelay(16436);
    DW1000Ng::startReceive();

    Serial.printf("[Node%d] UWB init complete (SS=%d RST=%d IRQ=%d)\n",
                  NODE_ID, UWB_PIN_SS, UWB_PIN_RST, UWB_PIN_IRQ);
}

// ── 수신 처리 (loop() 에서 호출) ─────────────────────────────
// CLAUDE.md §4: loop() 수신 패턴
//   while (available()) → read() → FSM feed()
//   delay() 호출 없이 즉시 반환
void UartComm::process() {
    while (_serial.available()) {
        uint8_t b = (uint8_t)_serial.read();

        uint8_t payload[PROTO_MAX_PAYLOAD];
        uint8_t plen = 0;

        if (proto_parser_feed(&_parser, b, payload, &plen)) {
            handle_packet(payload, plen);
        }
    }

    process_uwb();
}

// ── 수신 패킷 처리 ───────────────────────────────────────────
void UartComm::handle_packet(const uint8_t *payload, uint8_t len) {
    if (len < 1) return;

    if (payload[0] == CMD_PING && len >= 3) {
        // 빅 엔디안 역직렬화 (CLAUDE.md §5: 비트 시프트 연산 필수)
        const uint16_t seq = ((uint16_t)payload[1] << 8)
                           |  (uint16_t)payload[2];

        Serial.printf("[Node%d] PING recv: SEQ=%u\n", NODE_ID, seq);

        // PONG 응답: [CMD_PONG][SEQ_HI][SEQ_LO][NODE_ID]
        uint8_t pong[4];
        pong[0] = CMD_PONG;
        pong[1] = (seq >> 8) & 0xFF;   // 빅 엔디안 직렬화 (CLAUDE.md §5)
        pong[2] =  seq       & 0xFF;
        pong[3] = (uint8_t)NODE_ID;

        send(pong, sizeof(pong));
        Serial.printf("[Node%d] PONG sent: SEQ=%u\n", NODE_ID, seq);

    } else if (payload[0] == CMD_RANGE_REQUEST && len >= 3) {
        const uint16_t seq = ((uint16_t)payload[1] << 8)
                           |  (uint16_t)payload[2];
        start_ranging(seq);

    } else if (payload[0] == CMD_CTRL_FWD && len >= 4) {
        // payload: [CMD_CTRL_FWD][topic_len][topic...][payload_len_hi][payload_len_lo][payload...]
        const uint8_t topic_len = payload[1];
        if (len < 2u + topic_len + 2u) {
            Serial.printf("[Node%d] CTRL_FWD 길이 오류\n", NODE_ID);
            return;
        }
        const uint8_t* topic = payload + 2;
        const uint16_t plen = ((uint16_t)payload[2 + topic_len] << 8)
                            |  (uint16_t)payload[3 + topic_len];
        if (len < 2u + topic_len + 2u + plen) {
            Serial.printf("[Node%d] CTRL_FWD 페이로드 길이 오류\n", NODE_ID);
            return;
        }
        const uint8_t* pdata = payload + 2 + topic_len + 2;

        if (_rangeState != RangeState::IDLE) {
            Serial.printf("[Node%d] CTRL_FWD 무시 - 레인징 진행 중\n", NODE_ID);
            return;
        }
        send_ctrl_fwd(topic, topic_len, pdata, (uint8_t)plen);

    } else {
        Serial.printf("[Node%d] 알 수 없는 CMD=0x%02X len=%u\n",
                      NODE_ID, payload[0], len);
    }
}

void UartComm::start_ranging(uint16_t seq) {
    if (_rangeState != RangeState::IDLE) {
        Serial.printf("[Node%d] range busy, request ignored (SEQ=%u)\n", NODE_ID, seq);
        return;
    }

    _activeSeq = seq;
    _rangeState = RangeState::WAIT_POLL_ACK;
    _rangeDeadlineMs = millis() + 300;
    send_poll();
    Serial.printf("[Node%d] range start (SEQ=%u)\n", NODE_ID, _activeSeq);
}

void UartComm::send_poll() {
    memset(_uwbData, 0, sizeof(_uwbData));
    _uwbData[0] = UWB_POLL;
    DW1000Ng::setTransmitData(_uwbData, UWB_FRAME_LEN);
    DW1000Ng::startTransmit();
}

void UartComm::send_range() {
    memset(_uwbData, 0, sizeof(_uwbData));
    _uwbData[0] = UWB_RANGE;

    byte futureTimeBytes[LENGTH_TIMESTAMP];
    uint64_t timeRangeSent = DW1000Ng::getSystemTimestamp();
    timeRangeSent += DW1000NgTime::microsecondsToUWBTime(3000);
    DW1000NgUtils::writeValueToBytes(futureTimeBytes, timeRangeSent, LENGTH_TIMESTAMP);
    DW1000Ng::setDelayedTRX(futureTimeBytes);
    timeRangeSent += DW1000Ng::getTxAntennaDelay();

    DW1000NgUtils::writeValueToBytes(_uwbData + 1, _timePollSent, LENGTH_TIMESTAMP);
    DW1000NgUtils::writeValueToBytes(_uwbData + 6, _timePollAckReceived, LENGTH_TIMESTAMP);
    DW1000NgUtils::writeValueToBytes(_uwbData + 11, timeRangeSent, LENGTH_TIMESTAMP);

    DW1000Ng::setTransmitData(_uwbData, UWB_FRAME_LEN);
    DW1000Ng::startTransmit(TransmitMode::DELAYED);
}

void UartComm::process_uwb() {
    // ── 전송 완료 처리 (레인징 중만) ─────────────────
    if (_rangeState != RangeState::IDLE && DW1000Ng::isTransmitDone()) {
        DW1000Ng::clearTransmitStatus();
        DW1000Ng::startReceive();
    }

    // ── 레인징 타임아웃 ───────────────────────────
    if (_rangeState != RangeState::IDLE &&
        (int32_t)(millis() - _rangeDeadlineMs) > 0) {
        fail_ranging("timeout");
        return;
    }

    // ── 수신 완료 확인 ───────────────────────────────
    if (!DW1000Ng::isReceiveDone()) {
        return;
    }

    const int dataLen = DW1000Ng::getReceivedDataLength();
    if (dataLen != UWB_FRAME_LEN) {
        DW1000Ng::clearReceiveStatus();
        DW1000Ng::startReceive();
        if (_rangeState != RangeState::IDLE) {
            fail_ranging("invalid frame len");
        }
        return;
    }

    byte frame[UWB_FRAME_LEN];
    DW1000Ng::getReceivedData(frame, UWB_FRAME_LEN);
    const byte msgId = frame[0];

    // ── IDLE: motor 알림 프레임 처리 ───────────────────
    if (_rangeState == RangeState::IDLE) {
        DW1000Ng::clearReceiveStatus();
        DW1000Ng::startReceive();
        if (msgId == UWB_MODE_NOTIFY) {
            const uint8_t carId = frame[1];
            const uint8_t mode  = frame[2];
            uint8_t np[3];
            np[0] = CMD_MODE_NOTIFY;
            np[1] = (uint8_t)NODE_ID;
            np[2] = mode;
            send(np, sizeof(np));
            Serial.printf("[Node%d] MODE_NOTIFY: car=%u mode=%u\n",
                          NODE_ID, carId, mode);
        }
        return;
    }

    // ── WAIT_POLL_ACK ──────────────────────────────────
    if (_rangeState == RangeState::WAIT_POLL_ACK) {
        if (msgId != UWB_POLL_ACK) {
            DW1000Ng::clearReceiveStatus();
            DW1000Ng::startReceive();
            fail_ranging("expected POLL_ACK");
            return;
        }
        // getReceiveTimestamp() 는 clearReceiveStatus() 전에 호출
        _timePollSent        = DW1000Ng::getTransmitTimestamp();
        _timePollAckReceived = DW1000Ng::getReceiveTimestamp();
        _rangeState          = RangeState::WAIT_RANGE_REPORT;
        _rangeDeadlineMs     = millis() + 300;
        DW1000Ng::clearReceiveStatus();
        send_range();
        return;
    }

    // ── WAIT_RANGE_REPORT ──────────────────────────────
    if (_rangeState == RangeState::WAIT_RANGE_REPORT) {
        DW1000Ng::clearReceiveStatus();
        DW1000Ng::startReceive();

        if (msgId == UWB_RANGE_FAILED) {
            fail_ranging("anchor reported failure");
            return;
        }
        if (msgId != UWB_RANGE_REPORT) {
            fail_ranging("expected RANGE_REPORT");
            return;
        }

        float rawDistance = 0.0f;
        memcpy(&rawDistance, frame + 1, sizeof(float));
        const float distanceMeters = rawDistance / DISTANCE_OF_RADIO_INV;
        const uint16_t distanceCm  = meters_to_cm(distanceMeters);
        const int16_t rssiCentiDbm = (int16_t)(DW1000Ng::getReceivePower() * 100.0f);
        const uint8_t carId        = frame[5];

        report_range(carId, distanceCm, rssiCentiDbm);
        _rangeState = RangeState::IDLE;
        Serial.printf("[Node%d] range done: car=%u, %u cm, RSSI=%d cdbm\n",
                      NODE_ID, carId, distanceCm, (int)rssiCentiDbm);
    }
}

void UartComm::report_range(uint8_t car_id, uint16_t distance_cm, int16_t rssi_centi_dbm) {
    uint8_t payload[7];
    payload[0] = CMD_RANGE_REPORT;
    payload[1] = (uint8_t)NODE_ID;
    payload[2] = car_id;
    payload[3] = (distance_cm >> 8) & 0xFF;
    payload[4] = distance_cm & 0xFF;
    payload[5] = (rssi_centi_dbm >> 8) & 0xFF;
    payload[6] = rssi_centi_dbm & 0xFF;
    send(payload, sizeof(payload));
}

void UartComm::fail_ranging(const char *reason) {
    report_range(0x00, 0xFFFF, (int16_t)0x8000);
    _rangeState = RangeState::IDLE;
    DW1000Ng::forceTRxOff();
    DW1000Ng::startReceive();
    Serial.printf("[Node%d] range failed: %s\n", NODE_ID, reason);
}

uint16_t UartComm::meters_to_cm(float meters) {
    if (meters <= 0.0f) {
        return 0;
    }
    float cm = meters * 100.0f;
    if (cm > 65534.0f) {
        cm = 65534.0f;
    }
    return (uint16_t)(cm + 0.5f);
}

// ── CRC-16 CCITT (poly=0x1021, init=0xFFFF) ─────────────────
// motor.ino / sender.ino 의 calculateCRC16() 와 동일한 알고리즘
uint16_t UartComm::calc_crc16(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else              crc <<= 1;
        }
    }
    return crc;
}

// ── CMD_CTRL_FWD: Proto-B UWB 프레임 빌드 후 차량으로 전송 ───
// Proto-B 프레임 형식 (motor.ino / sender.ino 호환):
//   [0x7E][PUBLISH=0x01][msgId_HI][msgId_LO][QoS=0x00]
//   [topicLen][topic...][payloadLen_HI][payloadLen_LO][payload...]
//   [CRC_HI][CRC_LO][0x7F]
// QoS=0 (AT_MOST_ONCE): motor ACK 전송 없음 → 수신 버퍼 오염 방지
void UartComm::send_ctrl_fwd(const uint8_t* topic, uint8_t topic_len,
                              const uint8_t* payload, uint8_t payload_len) {
    uint8_t buf[128];
    int pos = 0;

    buf[pos++] = 0x7E;                         // START_BYTE
    buf[pos++] = 0x01;                         // PUBLISH
    buf[pos++] = (_ctrlMsgId >> 8) & 0xFF;     // msgId HI  (빅 엔디안 직렬화)
    buf[pos++] = _ctrlMsgId & 0xFF;            // msgId LO
    _ctrlMsgId++;
    buf[pos++] = 0x00;                         // QoS AT_MOST_ONCE
    buf[pos++] = topic_len;
    for (int i = 0; i < topic_len;   i++) buf[pos++] = topic[i];
    buf[pos++] = (payload_len >> 8) & 0xFF;    // payloadLen HI
    buf[pos++] = payload_len & 0xFF;           // payloadLen LO
    for (int i = 0; i < payload_len; i++) buf[pos++] = payload[i];

    // CRC-16: buf[1] 부터 현재 pos-1 까지 (START_BYTE 제외)
    uint16_t crc = calc_crc16(buf + 1, (uint16_t)(pos - 1));
    buf[pos++] = (crc >> 8) & 0xFF;
    buf[pos++] = crc & 0xFF;
    buf[pos++] = 0x7F;                         // END_BYTE

    DW1000Ng::setTransmitData(buf, pos);
    DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);

    // 전송 완료 대기 (millis 기반 타임아웃, delay() 없음)
    const uint32_t deadline = millis() + 100;
    while (!DW1000Ng::isTransmitDone()) {
        if ((int32_t)(millis() - deadline) > 0) {
            Serial.printf("[Node%d] CTRL_FWD 전송 타임아웃\n", NODE_ID);
            break;
        }
    }
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::startReceive();

    Serial.printf("[Node%d] CTRL_FWD 전송: '%.*s' = '%.*s'\n",
                  NODE_ID,
                  (int)topic_len,   (const char*)topic,
                  (int)payload_len, (const char*)payload);
}

// ── 송신 ─────────────────────────────────────────────────────
void UartComm::send(const uint8_t *payload, uint8_t len) {
    uint8_t frame[PROTO_MAX_FRAME];
    const size_t frame_len = proto_build_frame(payload, len,
                                               frame, sizeof(frame));
    if (frame_len == 0) {
        Serial.println("[uart] 프레임 빌드 실패");
        return;
    }
    _serial.write(frame, frame_len);
}
