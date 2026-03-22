#pragma once

#include <HardwareSerial.h>
#include "protocol.h"

// ── 노드별 설정 — 업로드 전에 반드시 노드에 맞게 수정 ──────────
// Node1: NODE_ID=1 / Node2: NODE_ID=2 / Node3: NODE_ID=3
#define NODE_ID      1
#define UART_TX_PIN  17     // 라즈베리파이 RX 에 연결된 GPIO
#define UART_RX_PIN  16     // 라즈베리파이 TX 에 연결된 GPIO
#define UART_BAUD    115200
// ──────────────────────────────────────────────────────────────

// ── UartComm 클래스 ───────────────────────────────────────────
// HardwareSerial(1) → UART1 (NOT Serial/UART0, CLAUDE.md §4)
// loop() 에서 process() 를 매 틱마다 호출
class UartComm {
public:
    // setup() 에서 1회 호출
    void begin();

    // loop() 에서 매 틱마다 호출 — 수신 바이트를 FSM 파서에 투입
    // delay() 없이 즉시 반환 (CLAUDE.md §4)
    void process();

    // 페이로드를 프레임으로 감싸 UART1 로 송신
    void send(const uint8_t *payload, uint8_t len);

private:
    // UART0(Serial)는 USB 디버그 전용 — 반드시 UART1 사용
    HardwareSerial _serial{1};
    ProtoParser    _parser;

    void handle_packet(const uint8_t *payload, uint8_t len);
};
