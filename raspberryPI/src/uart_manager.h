#pragma once

#include <cstdint>
#include <string>
#include <functional>
#include "protocol.h"

// ── UART 포트 내부 상태 ──────────────────────────────────────────────────────
struct UartPort {
    int         fd      = -1;
    std::string path;
    int         node_id = 0;
    proto::Parser parser;
};

// ── UartManager ───────────────────────────────────────────────────────────────
// 단일 epoll 루프로 최대 3개의 UART 포트를 다중화.
// std::thread / pthread 사용 없음. (CLAUDE.md §3 준수)
class UartManager {
public:
    // 수신 완료 콜백: (node_id, payload 포인터, payload 길이)
    using RxCallback = std::function<void(int, const uint8_t*, uint8_t)>;

    ~UartManager();

    // 포트 열기: 내부적으로 termios Raw 모드 + epoll 등록 수행
    bool open_port(int node_id, const std::string& device_path, int baud);

    void set_rx_callback(RxCallback cb);

    // 지정 노드에 프레임 전송 (build_frame 후 write)
    bool send(int node_id, const uint8_t* payload, uint8_t len);

    // epoll fd 노출 — main 에서 timerfd 등 추가 fd 를 같은 인스턴스에 등록할 수 있도록 허용
    int epoll_fd() const { return epoll_fd_; }

    // 수신된 fd 를 처리 (main 의 epoll 루프에서 호출)
    // 반환값: 해당 fd 가 UART 포트이면 true, 아니면 false
    bool dispatch(int fd);

private:
    bool configure_termios(int fd, int baud);

    int       epoll_fd_   = -1;
    UartPort  ports_[3];
    int       port_count_ = 0;
    RxCallback rx_cb_;
};
