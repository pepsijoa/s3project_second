#include "uart_comm.h"

// ── 초기화 ────────────────────────────────────────────────────
void UartComm::begin() {
    proto_parser_reset(&_parser);
    // HardwareSerial(1) = UART1
    // begin(baud, config, rxPin, txPin)
    _serial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
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

    } else {
        Serial.printf("[Node%d] 알 수 없는 CMD=0x%02X len=%u\n",
                      NODE_ID, payload[0], len);
    }
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
