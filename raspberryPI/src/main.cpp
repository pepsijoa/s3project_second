// main.cpp — Raspberry Pi UART 통신 대화형 테스트
//
// 단일 스레드 epoll 루프 구조:
//   epoll_wait → [stdin 이벤트] 사용자 입력에 따라 PING / RANGE 전송
//              → [UART 수신 이벤트] PONG / RANGE_REPORT 파싱 후 콘솔 출력
//
// 사용법:
//   PING  — 모든 활성 노드에 PING 전송 (PONG 응답 확인)
//   RANGE — 모든 활성 노드에 RANGE_REQUEST 전송 (거리 측정)
//   quit  — 프로그램 종료
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
#include <cmath>
#include <array>

#include <unistd.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <time.h>

// ── 전역 종료 플래그 ──────────────────────────────────────────────────────────
static volatile sig_atomic_t g_running = 1;

static void on_signal(int) {
    g_running = 0;
}

struct NodeRange {
    bool valid = false;
    uint16_t distance_cm = 0;
    int16_t rssi_centi_dbm = 0;
    uint64_t updated_ms = 0;
};

static NodeRange g_ranges[4];
static uint64_t g_now_ms = 0;

static bool compute_position(const std::array<double, 3>& dist_m,
                             double* out_x, double* out_y) {
    // 고정 앵커 좌표(미터): Node1(0,0), Node2(2,0), Node3(1,1.732)
    const double x1 = 0.0, y1 = 0.0;
    const double x2 = 2.0, y2 = 0.0;
    const double x3 = 1.0, y3 = 1.732;

    const double d1 = dist_m[0];
    const double d2 = dist_m[1];
    const double d3 = dist_m[2];

    const double A = 2.0 * (x2 - x1);
    const double B = 2.0 * (y2 - y1);
    const double C = (d1 * d1 - d2 * d2) - (x1 * x1 - x2 * x2) - (y1 * y1 - y2 * y2);

    const double D = 2.0 * (x3 - x1);
    const double E = 2.0 * (y3 - y1);
    const double F = (d1 * d1 - d3 * d3) - (x1 * x1 - x3 * x3) - (y1 * y1 - y3 * y3);

    const double det = (A * E) - (B * D);
    if (std::fabs(det) < 1e-9) {
        return false;
    }

    *out_x = (C * E - B * F) / det;
    *out_y = (A * F - C * D) / det;
    return true;
}

static void try_print_position() {
    const uint64_t freshness_ms = 1500;
    for (int node = 1; node <= 3; node++) {
        if (!g_ranges[node].valid) return;
        if (g_now_ms - g_ranges[node].updated_ms > freshness_ms) return;
    }

    std::array<double, 3> dist_m = {
        g_ranges[1].distance_cm / 100.0,
        g_ranges[2].distance_cm / 100.0,
        g_ranges[3].distance_cm / 100.0,
    };

    double x = 0.0;
    double y = 0.0;
    if (!compute_position(dist_m, &x, &y)) {
        fprintf(stderr, "[POS] trilateration failed: singular matrix\n");
        return;
    }

    printf("[POS] car=(%.3f, %.3f)m  d=[%.2f, %.2f, %.2f]m\n",
           x, y, dist_m[0], dist_m[1], dist_m[2]);
}

// ── 수신 콜백 ─────────────────────────────────────────────────────────────────
static void on_pong(int node_id, const uint8_t* payload, uint8_t len) {
    if (len < 1) return;

    struct timespec ts_now{};
    clock_gettime(CLOCK_MONOTONIC, &ts_now);
    g_now_ms = static_cast<uint64_t>(ts_now.tv_sec) * 1000ULL
             + static_cast<uint64_t>(ts_now.tv_nsec / 1000000ULL);

    if (payload[0] == proto::CMD_PONG && len >= 4) {
        // 빅 엔디안 역직렬화 (CLAUDE.md §5: 비트 시프트 연산 필수)
        const uint16_t seq = (static_cast<uint16_t>(payload[1]) << 8)
                           |  static_cast<uint16_t>(payload[2]);
        const uint8_t  nid = payload[3];
        printf("[RX] PONG ← Node%d  |  NodeID=%u  SEQ=%u\n", node_id, nid, seq);
    } else if (payload[0] == proto::CMD_RANGE_REPORT && len >= 7) {
        const uint8_t reported_node = payload[1];
        const uint8_t car_id = payload[2];
        const uint16_t distance_cm = (static_cast<uint16_t>(payload[3]) << 8)
                                   |  static_cast<uint16_t>(payload[4]);
        const int16_t rssi_centi_dbm = static_cast<int16_t>(
            (static_cast<uint16_t>(payload[5]) << 8) | static_cast<uint16_t>(payload[6])
        );

        if (reported_node < 1 || reported_node > 3) {
            printf("[RX] RANGE_REPORT invalid node=%u\n", reported_node);
            return;
        }

        if (distance_cm == 0xFFFFu) {
            printf("[RX] RANGE_REPORT Node%u car=%u failed\n", reported_node, car_id);
            g_ranges[reported_node].valid = false;
            return;
        }

        g_ranges[reported_node].valid = true;
        g_ranges[reported_node].distance_cm = distance_cm;
        g_ranges[reported_node].rssi_centi_dbm = rssi_centi_dbm;
        g_ranges[reported_node].updated_ms = g_now_ms;

        printf("[RX] RANGE_REPORT Node%u  car=%u  dist=%u cm  rssi=%.2f dBm\n",
               reported_node, car_id, distance_cm, rssi_centi_dbm / 100.0);
        try_print_position();
    } else if (payload[0] == proto::CMD_MODE_NOTIFY && len >= 3) {
        const uint8_t mode = payload[2];
        const char* mode_str = (mode == 1) ? "MQTT_CONTROL (HTTP BLOCKED)"
                                           : "HTTP_CONTROL (HTTP ENABLED)";
        printf("[RX] MODE_NOTIFY ← Node%d  →  차량 FSM: %s\n", node_id, mode_str);
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

static void send_range_request_all(UartManager& mgr,
                                   const int* active_nodes,
                                   int active_count,
                                   uint16_t* seq) {
    for (int index = 0; index < active_count; index++) {
        const int node = active_nodes[index];
        uint8_t payload[3];
        payload[0] = proto::CMD_RANGE_REQUEST;
        payload[1] = (*seq >> 8) & 0xFF;
        payload[2] = *seq & 0xFF;

        if (mgr.send(node, payload, 3)) {
            printf("[TX] RANGE_REQUEST → Node%d  SEQ=%u\n", node, *seq);
        }
        (*seq)++;
    }
}

// CMD_CTRL_FWD 전송: RPi → 노드(UART) → motor(UWB Proto-B 릴레이)
// payload 형식: [CMD_CTRL_FWD][topic_len][topic...][payload_len_hi][payload_len_lo][payload...]
static void send_ctrl_fwd_all(UartManager& mgr, const int* active_nodes, int active_count,
                               const char* topic, const char* payload_str) {
    const size_t topic_len   = strlen(topic);
    const size_t payload_len = strlen(payload_str);

    if (topic_len > 63 || payload_len > 63) {
        printf("[!] CMD_CTRL_FWD 내용이 너무 깁니다 (topic≤63, payload≤63)\n");
        return;
    }

    // 페이로드 빌드: [CMD][topic_len][topic...][payloadLenHi][payloadLenLo][payload...]
    uint8_t buf[132];
    size_t pos = 0;
    buf[pos++] = proto::CMD_CTRL_FWD;
    buf[pos++] = (uint8_t)topic_len;
    memcpy(buf + pos, topic, topic_len);       pos += topic_len;
    buf[pos++] = (payload_len >> 8) & 0xFF;
    buf[pos++] = payload_len & 0xFF;
    memcpy(buf + pos, payload_str, payload_len); pos += payload_len;

    for (int index = 0; index < active_count; index++) {
        const int node = active_nodes[index];
        if (mgr.send(node, buf, (uint8_t)pos)) {
            printf("[TX] CMD_CTRL_FWD → Node%d  topic='%s'  payload='%s'\n",
                   node, topic, payload_str);
        }
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

    // ── stdin 을 non-blocking + epoll 에 등록 ────────────────────────────────
    const int stdin_fd = STDIN_FILENO;
    {
        int flags = fcntl(stdin_fd, F_GETFL, 0);
        fcntl(stdin_fd, F_SETFL, flags | O_NONBLOCK);
    }
    struct epoll_event ev{};
    ev.events  = EPOLLIN;
    ev.data.fd = stdin_fd;
    if (epoll_ctl(mgr.epoll_fd(), EPOLL_CTL_ADD, stdin_fd, &ev) < 0) {
        perror("epoll_ctl stdin");
        return 1;
    }

    printf("=== UART 대화형 테스트 (%d개 노드) ===\n", active_count);
    printf("명령어: PING | RANGE | CMD <topic> <payload> | quit\n");
    printf("  예) CMD command/controlled 1\n");
    printf("      CMD command/motor 50.0\n");
    printf("      CMD command/led 1\n");
    printf("> ");
    fflush(stdout);

    // ── 단일 스레드 epoll 이벤트 루프 ────────────────────────────────────────
    uint16_t ping_seq = 0;
    uint16_t range_seq = 0;
    char stdin_buf[256];
    int stdin_buf_pos = 0;
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

            if (fd == stdin_fd) {
                // stdin 에서 한 줄 읽기
                char ch;
                while (read(stdin_fd, &ch, 1) == 1) {
                    if (ch == '\n') {
                        stdin_buf[stdin_buf_pos] = '\0';

                        // 타임스탬프 갱신
                        struct timespec ts_now{};
                        clock_gettime(CLOCK_MONOTONIC, &ts_now);
                        g_now_ms = static_cast<uint64_t>(ts_now.tv_sec) * 1000ULL
                                 + static_cast<uint64_t>(ts_now.tv_nsec / 1000000ULL);

                        // 명령 파싱 (대소문자 무관)
                        if (strcasecmp(stdin_buf, "PING") == 0) {
                            send_ping_all(mgr, active_nodes, active_count, &ping_seq);
                        } else if (strcasecmp(stdin_buf, "RANGE") == 0) {
                            send_range_request_all(mgr, active_nodes, active_count, &range_seq);
                        } else if (strcasecmp(stdin_buf, "quit") == 0 ||
                                   strcasecmp(stdin_buf, "exit") == 0) {
                            g_running = 0;
                        } else if (strncasecmp(stdin_buf, "CMD ", 4) == 0) {
                            // CMD <topic> <payload>
                            char* rest = stdin_buf + 4;
                            char* sp   = strchr(rest, ' ');
                            if (!sp) {
                                printf("[!] 형식: CMD <topic> <payload>  예) CMD command/controlled 1\n");
                            } else {
                                *sp = '\0';
                                send_ctrl_fwd_all(mgr, active_nodes, active_count,
                                                  rest, sp + 1);
                            }
                        } else if (stdin_buf_pos > 0) {
                            printf("[?] 알 수 없는 명령: '%s'  (PING / RANGE / CMD <topic> <payload> / quit)\n", stdin_buf);
                        }

                        stdin_buf_pos = 0;
                        if (g_running) {
                            printf("> ");
                            fflush(stdout);
                        }
                    } else if (stdin_buf_pos < (int)sizeof(stdin_buf) - 1) {
                        stdin_buf[stdin_buf_pos++] = ch;
                    }
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

    printf("\n=== 종료 ===\n");
    return 0;
}
