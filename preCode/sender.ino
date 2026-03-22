#include <DW1000Ng.hpp>
#include <SPI.h>

const uint8_t PIN_SCK = 18;
const uint8_t PIN_MISO = 19;
const uint8_t PIN_MOSI = 23;
const uint8_t PIN_SS = 5;
const uint8_t PIN_RST = 22;
const uint8_t PIN_IRQ = 4;

// 프로토콜 상수
const uint8_t START_BYTE = 0x7E;
const uint8_t END_BYTE = 0x7F;
enum MessageType { PUBLISH = 0x01 };
enum QoS { AT_LEAST_ONCE = 1 };

uint16_t msgId = 1;
uint8_t txBuffer[128]; // 전송 버퍼

uint8_t msgSendrepeat = 3;
device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    true,
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,     // 2번 대신 기본 5번 사용
    DataRate::RATE_6800KBPS, // 110KBPS -> 6.8MBPS (고속 모드가 설정 오류가 적음)
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_128, // 고속 모드에 맞는 짧은 프리앰블
    PreambleCode::CODE_3
};

// CRC 계산 함수 (Receiver와 동일)
uint16_t calculateCRC16(uint8_t* data, int len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc = crc << 1;
        }
    }
    return crc;
}

// 패킷 생성 및 전송 함수
void sendMqttOverUwb(String topic, String payload) {
    int pos = 0;
    int payloadLen = payload.length();
    
    // 1. 헤더 작성
    txBuffer[pos++] = START_BYTE;
    txBuffer[pos++] = PUBLISH;
    txBuffer[pos++] = (msgId >> 8) & 0xFF;
    txBuffer[pos++] = msgId & 0xFF;
    txBuffer[pos++] = AT_LEAST_ONCE;
    
    // 2. 토픽 작성
    txBuffer[pos++] = (uint8_t)topic.length();
    for (int i = 0; i < topic.length(); i++) {
        txBuffer[pos++] = (uint8_t)topic[i];
    }
    
    // 3. 페이로드 작성
    txBuffer[pos++] = (payloadLen >> 8) & 0xFF;
    txBuffer[pos++] = payloadLen & 0xFF;
    for (int i = 0; i < payloadLen; i++) {
        txBuffer[pos++] = (uint8_t)payload[i];
    }
    
    // 4. CRC 계산 및 추가
    uint16_t crc = calculateCRC16(txBuffer + 1, pos - 1);
    txBuffer[pos++] = (crc >> 8) & 0xFF;
    txBuffer[pos++] = crc & 0xFF;
    
    // 5. 풋터 추가
    txBuffer[pos++] = END_BYTE;
    
    // 6. 전송
    DW1000Ng::setTransmitData(txBuffer, pos);
    DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);
    
    while(!DW1000Ng::isTransmitDone()) {
        yield();
    }
    DW1000Ng::clearTransmitStatus();
    msgId++;
    Serial.println("Sent packet: " + topic + " -> " + payload);
}

void setup() {
    Serial.begin(9600);
    Serial.println(F("### DW1000Ng-arduino-sender-test ###"));

    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS);
    DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
    
    DW1000Ng::initializeNoInterrupt(PIN_SS);
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
    DW1000Ng::setDeviceAddress(5);
    DW1000Ng::setNetworkId(10);
    DW1000Ng::setAntennaDelay(16436);
    
    // // DWM 확인
    char msg[128];
    DW1000Ng::getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: "); Serial.println(msg);
    DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: "); Serial.println(msg);
    DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: "); Serial.println(msg);
    DW1000Ng::getPrintableDeviceMode(msg);
    Serial.print("Device mode: "); Serial.println(msg);
}

void loop() {
    if (Serial.available() > 0) {
        // 엔터키(\n)가 입력될 때까지 문자열 읽기
        String input = Serial.readStringUntil('\n');
        input.trim(); // 앞뒤 공백 및 개행문자(\r\n) 제거

        if (input.length() > 0) {
            // 공백을 기준으로 토픽과 페이로드 분리
            int spaceIndex = input.indexOf(' ');

            if (spaceIndex > 0) {
                String topic = input.substring(0, spaceIndex);
                String payload = input.substring(spaceIndex + 1);
                
                // 전송 함수 호출
                for(int i =0; i<msgSendrepeat; i++){
                  sendMqttOverUwb(topic, payload);
                  delay(500);
                }
                
            } else {
                // 공백이 없는 경우 (페이로드가 없는 경우 등) 처리
                Serial.println(F("Error: Format should be 'TOPIC PAYLOAD' (separated by space)"));
            }
        }
    }
}