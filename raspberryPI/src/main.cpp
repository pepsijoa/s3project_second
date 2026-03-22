// main.cpp — Raspberry Pi UART 통신 테스트 진입점
//
// 단일 스레드 epoll 루프 구조:
//   epoll_wait → [timerfd 이벤트] 2초마다 PING × 3
//              → [UART 수신 이벤트] PONG 파싱 후 콘솔 출력
//
// CLAUDE.md §3 준수:
//   - std::thread / pthread 없음
//   - select() / poll() 없음
//   - /dev/ttyAMA* 하드코딩 없음 (udev 심볼릭 링크 또는 argv 경유)

#include "uart_manager.h"
#include "protocol.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <csignal>
#include <cerrno>

#include <unistd.h>
#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <time.h>

// ── 전역 종료 플래그 ──────────────────────────────────────────────────────────
static volatile sig_atomic_t g_running = 1;

static void on_signal(int) {
    g_running = 0;
}

// ── 수신 콜백 ─────────────────────────────────────────────────────────────────
static void on_pong(int node_id, const uint8_t* payload, uint8_t len) {
    if (len < 1) return;

    if (payload[0] == proto::CMD_PONG && len >= 4) {
        // 빅 엔디안 역직렬화 (CLAUDE.md §5: 비트 시프트 연산 필수)
        const uint16_t seq = (static_cast<uint16_t>(payload[1]) << 8)
                           |  static_cast<uint16_t>(payload[2]);
        const uint8_t  nid = payload[3];
        printf("[RX] PONG ← Node%d  |  NodeID=%u  SEQ=%u\n", node_id, nid, seq);
    } else {
        printf("[RX] Node%d: 알 수 없는 cmd=0x%02X len=%u\n", node_id, payload[0], len);
    }
}

// ── PING 전송 ─────────────────────────────────────────────────────────────────
static void send_ping_all(UartManager& mgr, const int* active_nodes, int active_count,
                          uint16_t* seq) {
    for (int index = 0; index < active_count; index++) {
        const int node = active_nodes[index];
        uint8_t payload[3];
        payload[0] = proto::CMD_PING;
        // 빅 엔디안 직렬화 (CLAUDE.md §5)
        payload[1] = (*seq >> 8) & 0xFF;
        payload[2] =  *seq       & 0xFF;

        if (mgr.send(node, payload, 3)) {
            printf("[TX] PING → Node%d  SEQ=%u\n", node, *seq);
        }
        (*seq)++;
    }
}

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    // udev 심볼릭 링크 경로 (argv 또는 기본값)
    // 절대로 /dev/ttyAMA* 를 직접 사용하지 않음 (CLAUDE.md §3)
    const char* default_dev[3] = {
        "/dev/ttyESP_Node1",
        "/dev/ttyESP_Node2",
        "/dev/ttyESP_Node3",
    };
    const int active_count = (argc > 1) ? (argc - 1) : 3;
    if (active_count < 1 || active_count > 3) {
        fprintf(stderr, "사용법: %s [dev1] [dev2] [dev3]\n", argv[0]);
        return 1;
    }

    const char* dev[3] = {
        (argc > 1) ? argv[1] : default_dev[0],
        (argc > 2) ? argv[2] : default_dev[1],
        (argc > 3) ? argv[3] : default_dev[2],
    };
    const int active_nodes[3] = {1, 2, 3};
    const int baud = 115200;

    UartManager mgr;
    mgr.set_rx_callback(on_pong);

    for (int i = 0; i < active_count; i++) {
        if (!mgr.open_port(i + 1, dev[i], baud)) {
            return 1;
        }
    }

    // ── timerfd 생성 (2초 주기 PING) ─────────────────────────────────────────
    int tfd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
    if (tfd < 0) {
        perror("timerfd_create");
        return 1;
    }
    struct itimerspec ts{};
    ts.it_value.tv_sec    = 1;  // 1초 후 첫 발화
    ts.it_interval.tv_sec = 2;  // 이후 2초마다
    if (timerfd_settime(tfd, 0, &ts, nullptr) < 0) {
        perror("timerfd_settime");
        return 1;
    }

    // timerfd 를 UartManager 의 epoll 인스턴스에 추가
    struct epoll_event ev{};
    ev.events  = EPOLLIN;
    ev.data.fd = tfd;
    if (epoll_ctl(mgr.epoll_fd(), EPOLL_CTL_ADD, tfd, &ev) < 0) {
        perror("epoll_ctl timerfd");
        return 1;
    }

    printf("=== UART PING 테스트 시작 (Ctrl+C 로 종료) ===\n");

    // ── 단일 스레드 epoll 이벤트 루프 ────────────────────────────────────────
    uint16_t seq = 0;
    struct epoll_event events[8];

    while (g_running) {
        const int nfds = epoll_wait(mgr.epoll_fd(), events, 8, 500);
        if (nfds < 0) {
            if (errno == EINTR) continue;
            perror("epoll_wait");
            break;
        }

        for (int i = 0; i < nfds; i++) {
            const int fd = events[i].data.fd;

            if (fd == tfd) {
                // timerfd 소비 (읽지 않으면 계속 EPOLLIN 발생)
                uint64_t expirations;
                if (read(tfd, &expirations, sizeof(expirations)) > 0) {
                    send_ping_all(mgr, active_nodes, active_count, &seq);
                }
            } else {
                // UART 수신 처리
                if (events[i].events & EPOLLERR) {
                    fprintf(stderr, "[epoll] EPOLLERR fd=%d\n", fd);
                }
                if (events[i].events & EPOLLIN) {
                    mgr.dispatch(fd);
                }
            }
        }
    }

    close(tfd);
    printf("\n=== 종료 ===\n");
    return 0;
}
