#include "uart_manager.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/epoll.h>
#include <cerrno>
#include <cstring>
#include <cstdio>

// ── 디버그 헥스 덤프 ──────────────────────────────────────────────────────────
static void dump_rx_bytes(int node_id, const uint8_t* data, size_t len) {
    printf("[RX-RAW] Node%d %zu bytes:", node_id, len);
    for (size_t i = 0; i < len; i++) {
        printf(" %02X", data[i]);
    }
    printf("\n");
}

// ── baud rate 변환 ────────────────────────────────────────────────────────────
static speed_t to_speed(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        default:
            fprintf(stderr, "[uart] 미지원 baud %d, 115200 사용\n", baud);
            return B115200;
    }
}

// ── termios Raw 모드 설정 ─────────────────────────────────────────────────────
// CLAUDE.md §3: cfmakeraw() 필수, VMIN=0, VTIME=0, 지정 플래그 소거
bool UartManager::configure_termios(int fd, int baud) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        return false;
    }

    cfmakeraw(&tty);                        // ICANON, ECHO, IXON, ISIG 등 일괄 소거
    cfsetispeed(&tty, to_speed(baud));
    cfsetospeed(&tty, to_speed(baud));

    // 비동기 비블로킹 읽기 강제
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    // 명시적 플래그 소거 (cfmakeraw 누락 방어)
    tty.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO | ECHOE | ISIG | IEXTEN));
    tty.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | ICRNL | INLCR));

    // 8N1, 수신 활성화
    tty.c_cflag &= static_cast<tcflag_t>(~(CSIZE | PARENB | CSTOPB));
    tty.c_cflag |= CS8 | CREAD | CLOCAL;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return false;
    }
    return true;
}

// ── 소멸자 ────────────────────────────────────────────────────────────────────
UartManager::~UartManager() {
    for (int i = 0; i < port_count_; i++) {
        if (ports_[i].fd >= 0) {
            close(ports_[i].fd);
        }
    }
    if (epoll_fd_ >= 0) {
        close(epoll_fd_);
    }
}

// ── 포트 열기 ─────────────────────────────────────────────────────────────────
// CLAUDE.md §3: O_RDWR|O_NOCTTY|O_NONBLOCK 필수
bool UartManager::open_port(int node_id, const std::string& device_path, int baud) {
    if (port_count_ >= 3) {
        fprintf(stderr, "[uart] 최대 3개 포트까지만 지원됩니다\n");
        return false;
    }

    int fd = open(device_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        perror(("open: " + device_path).c_str());
        return false;
    }

    if (!configure_termios(fd, baud)) {
        close(fd);
        return false;
    }

    // 포트 오픈 직후 남아 있는 커널 RX/TX 버퍼 찌꺼기를 제거한다.
    if (tcflush(fd, TCIOFLUSH) != 0) {
        perror("tcflush");
    }

    // epoll 인스턴스 생성 (첫 포트 등록 시)
    if (epoll_fd_ < 0) {
        epoll_fd_ = epoll_create1(EPOLL_CLOEXEC);
        if (epoll_fd_ < 0) {
            perror("epoll_create1");
            close(fd);
            return false;
        }
    }

    // CLAUDE.md §3: EPOLLIN|EPOLLERR, EPOLLET 사용 금지 (레벨 트리거 기본값 유지)
    struct epoll_event ev{};
    ev.events  = EPOLLIN | EPOLLERR;
    ev.data.fd = fd;
    if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd, &ev) < 0) {
        perror("epoll_ctl ADD");
        close(fd);
        return false;
    }

    ports_[port_count_] = UartPort{ fd, device_path, node_id, proto::Parser{} };
    port_count_++;

    printf("[uart] Node%d 열림: %s @ %d baud\n", node_id, device_path.c_str(), baud);
    return true;
}

// ── 콜백 설정 ─────────────────────────────────────────────────────────────────
void UartManager::set_rx_callback(RxCallback cb) {
    rx_cb_ = std::move(cb);
}

// ── 송신 ──────────────────────────────────────────────────────────────────────
bool UartManager::send(int node_id, const uint8_t* payload, uint8_t len) {
    for (int i = 0; i < port_count_; i++) {
        if (ports_[i].node_id != node_id) continue;

        auto frame = proto::build_frame(payload, len);
        const ssize_t written = write(ports_[i].fd, frame.data(), frame.size());
        if (written < 0) {
            perror("[uart] write");
            return false;
        }
        if (static_cast<size_t>(written) != frame.size()) {
            fprintf(stderr, "[uart] 부분 쓰기: %zd / %zu bytes\n",
                    written, frame.size());
        }
        return true;
    }
    fprintf(stderr, "[uart] Node%d 를 찾을 수 없음\n", node_id);
    return false;
}

// ── fd 디스패치 (main 의 epoll 루프에서 호출) ────────────────────────────────
bool UartManager::dispatch(int fd) {
    for (int i = 0; i < port_count_; i++) {
        if (ports_[i].fd != fd) continue;

        uint8_t buf[256];
        ssize_t n = read(fd, buf, sizeof(buf));
        if (n <= 0) {
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("[uart] read");
            }
            return true;
        }

        dump_rx_bytes(ports_[i].node_id, buf, static_cast<size_t>(n));

        // FSM 파서에 바이트 단위로 투입
        for (ssize_t j = 0; j < n; j++) {
            uint8_t payload[proto::MAX_PAYLOAD];
            uint8_t plen = 0;
            if (ports_[i].parser.feed(buf[j], payload, &plen)) {
                if (rx_cb_) {
                    rx_cb_(ports_[i].node_id, payload, plen);
                }
            }
        }
        return true;
    }
    return false; // UART 포트가 아닌 fd
}
