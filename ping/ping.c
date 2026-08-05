/*
 * ping.c - A ping (ICMP Echo) program written in C
 *
 * Platform implementations:
 *   Windows    : uses IcmpSendEcho from the IP Helper API
 *                (Microsoft's recommended way; no admin rights required)
 *   Linux/macOS: uses a SOCK_RAW socket + ICMP protocol (requires root/sudo)
 *
 * Build:
 *   Linux/macOS:  gcc -o ping ping.c
 *   Windows:      gcc -o ping.exe ping.c -lws2_32 -liphlpapi
 *
 * Usage:
 *   ping [-c count] [-i interval_ms] [-t timeout_ms] <hostname-or-IP>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef _WIN32
#include <winsock2.h>   /* must come before windows.h */
#include <windows.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>   /* must be included before icmpapi.h */
#include <icmpapi.h>
#include <process.h>    /* _getpid */
#define SLEEP_MS(ms) Sleep(ms)
#ifdef _MSC_VER
#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "iphlpapi.lib")
#endif
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/ip_icmp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <sys/time.h>
#define INVALID_SOCKET (-1)
#define SOCKET int
#define CLOSE_SOCKET(fd) close(fd)
#define SLEEP_MS(ms) usleep((ms) * 1000)
#endif

/* On 64-bit Windows, IcmpSendEcho fills the reply in a 32-bit layout,
 * so the reply must be read as ICMP_ECHO_REPLY32 or fields misalign (MSDN). */
#ifdef _WIN64
typedef ICMP_ECHO_REPLY32 echo_reply_t;
#define ECHO_REPLY_ADDR(r)  ((r)->Address)
#define ECHO_REPLY_STATUS(r) ((r)->Status)
#define ECHO_REPLY_RTT(r)   ((r)->RoundTripTime)
#else
typedef ICMP_ECHO_REPLY echo_reply_t;
#define ECHO_REPLY_ADDR(r)  ((r)->Address)
#define ECHO_REPLY_STATUS(r) ((r)->Status)
#define ECHO_REPLY_RTT(r)   ((r)->RoundTripTime)
#endif

#define ICMP_ECHO_REPLY    0
#define ICMP_ECHO_REQUEST  8

#define ICMP_HEADER_LEN    8     /* ICMP header length (no data) */
#define ICMP_DATA_LEN      48    /* data length (timestamp + padding) */
#define PACKET_LEN         (ICMP_HEADER_LEN + ICMP_DATA_LEN)

#define DEFAULT_COUNT      4
#define DEFAULT_INTERVAL   1000  /* ms */
#define DEFAULT_TIMEOUT    2000  /* ms */
#define RECV_BUF_SIZE      512

#ifdef _WIN32
/* ---- Windows: IcmpSendEcho context ---- */
typedef struct {
    HANDLE handle;    /* handle returned by IcmpCreateFile() */
    DWORD  dst_ip;    /* destination IPv4 address (network byte order) */
} ping_ctx_t;
#else
/* ---- Unix: raw socket context ---- */
typedef struct {
    SOCKET fd;
    struct sockaddr_in dst;
    unsigned short pid;
} ping_ctx_t;
#endif

/* Monotonic clock in ms (Linux path; Windows RTT comes from IcmpSendEcho) */
static unsigned long long now_ms(void)
{
#ifdef _WIN32
    return (unsigned long long)GetTickCount64();
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (unsigned long long)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
#endif
}

#ifndef _WIN32
/* ICMP header structure (used by the Unix raw socket path) */
typedef struct {
    unsigned char  type;
    unsigned char  code;
    unsigned short checksum;
    unsigned short id;
    unsigned short seq;
} icmp_header_t;

/* Compute the ICMP checksum (RFC 1071), used by the Unix raw socket path */
static unsigned short icmp_checksum(void *data, int len)
{
    unsigned short *p = (unsigned short *)data;
    unsigned long sum = 0;

    while (len > 1) {
        sum += *p++;
        len -= 2;
    }
    if (len == 1)
        sum += *(unsigned char *)p;
    while (sum >> 16)
        sum = (sum & 0xffff) + (sum >> 16);
    return (unsigned short)~sum;
}

/* Extract the ICMP message from a receive buffer: skip the IPv4 header if any */
static icmp_header_t *extract_icmp(unsigned char *buf, int len, int *icmp_len)
{
    if (len >= 20 && (buf[0] >> 4) == 4) {
        int ip_hlen = (buf[0] & 0x0f) * 4;
        if (ip_hlen >= 20 && len > ip_hlen) {
            *icmp_len = len - ip_hlen;
            return (icmp_header_t *)(buf + ip_hlen);
        }
    }
    *icmp_len = len;
    return (icmp_header_t *)buf;
}
#endif /* !_WIN32 */

/* ---- Platform implementation: init ---- */
static int ping_init(ping_ctx_t *ctx, struct in_addr addr)
{
#ifdef _WIN32
    ctx->dst_ip = addr.s_addr;
    ctx->handle = IcmpCreateFile();
    if (ctx->handle == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Error: IcmpCreateFile failed (error code=%lu)\n", GetLastError());
        return -1;
    }
    return 0;
#else
    ctx->fd = socket(AF_INET, SOCK_RAW, IPPROTO_ICMP);
    if (ctx->fd == INVALID_SOCKET) {
        fprintf(stderr, "Error: failed to create raw socket (errno=%d)\n", errno);
        fprintf(stderr, "  Please run this program as root (sudo)\n");
        return -1;
    }
    memset(&ctx->dst, 0, sizeof(ctx->dst));
    ctx->dst.sin_family = AF_INET;
    ctx->dst.sin_addr = addr;
    ctx->pid = (unsigned short)(getpid() & 0xffff);
    return 0;
#endif
}

/* ---- Platform implementation: close ---- */
static void ping_close(ping_ctx_t *ctx)
{
#ifdef _WIN32
    if (ctx->handle != INVALID_HANDLE_VALUE)
        IcmpCloseHandle(ctx->handle);
#else
    CLOSE_SOCKET(ctx->fd);
#endif
}

/* ---- Platform implementation: send one echo and wait for the reply ----
 * Returns 1 on success and fills rtt(ms), source IP, TTL (ttl=-1 if unavailable);
 * returns 0 on timeout/failure. */
static int ping_echo(ping_ctx_t *ctx, int seq, int timeout_ms,
                     unsigned long long *rtt_ms, char *from_ip, size_t ip_len, int *ttl)
{
#ifdef _WIN32
    (void)seq;   /* IcmpSendEcho manages the sequence internally */
    /* Request data: timestamp + padding */
    unsigned char send_data[ICMP_DATA_LEN];
    unsigned long long ts = now_ms();
    memcpy(send_data, &ts, sizeof(ts));
    memset(send_data + sizeof(ts), 'a', sizeof(send_data) - sizeof(ts));

    /* ReplySize must be at least one ICMP_ECHO_REPLY + data length + 8 (ICMP error msg) */
    DWORD reply_size = (DWORD)(sizeof(echo_reply_t) + sizeof(send_data) + 8);
    unsigned char *reply = (unsigned char *)malloc(reply_size);
    if (!reply)
        return 0;

    DWORD n = IcmpSendEcho(ctx->handle, ctx->dst_ip, send_data,
                           (WORD)sizeof(send_data), NULL, reply,
                           reply_size, (DWORD)timeout_ms);
    if (n > 0) {
        echo_reply_t *r = (echo_reply_t *)reply;
        if (ECHO_REPLY_STATUS(r) == IP_SUCCESS) {
            struct in_addr a;
            a.s_addr = ECHO_REPLY_ADDR(r);
            snprintf(from_ip, ip_len, "%s", inet_ntoa(a));
            *rtt_ms = ECHO_REPLY_RTT(r);
            *ttl = -1;   /* IcmpSendEcho does not return the TTL */
            free(reply);
            return 1;
        }
    }
    free(reply);
    return 0;
#else
    /* Build the ICMP echo request: header + data (timestamp + padding) */
    unsigned char packet[PACKET_LEN];
    icmp_header_t icmp;
    unsigned long long ts = now_ms();

    memset(packet, 0, sizeof(packet));
    icmp.type = ICMP_ECHO_REQUEST;
    icmp.code = 0;
    icmp.checksum = 0;
    icmp.id = htons(ctx->pid);
    icmp.seq = htons((unsigned short)seq);
    memcpy(packet, &icmp, sizeof(icmp));

    memcpy(packet + ICMP_HEADER_LEN, &ts, sizeof(ts));
    memset(packet + ICMP_HEADER_LEN + sizeof(ts), 'a',
           ICMP_DATA_LEN - sizeof(ts));

    icmp.checksum = htons(icmp_checksum(packet, sizeof(packet)));
    memcpy(packet, &icmp, sizeof(icmp));

    if (sendto(ctx->fd, (const char *)packet, sizeof(packet), 0,
               (struct sockaddr *)&ctx->dst, sizeof(ctx->dst)) == -1) {
        fprintf(stderr, "Error: sendto failed (errno=%d)\n", errno);
        return 0;
    }

    /* Wait for the reply */
    fd_set fds;
    struct timeval tv;
    struct sockaddr_in from;
    socklen_t from_len = sizeof(from);
    unsigned char rbuf[RECV_BUF_SIZE];

    FD_ZERO(&fds);
    FD_SET(ctx->fd, &fds);
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    if (select((int)ctx->fd + 1, &fds, NULL, NULL, &tv) > 0 && FD_ISSET(ctx->fd, &fds)) {
        int n = (int)recvfrom(ctx->fd, (char *)rbuf, sizeof(rbuf), 0,
                              (struct sockaddr *)&from, &from_len);
        if (n > 0) {
            int icmp_len;
            icmp_header_t *r = extract_icmp(rbuf, n, &icmp_len);
            if (r->type == ICMP_ECHO_REPLY && ntohs(r->id) == ctx->pid) {
                *rtt_ms = now_ms() - ts;
                snprintf(from_ip, ip_len, "%s", inet_ntoa(from.sin_addr));
                *ttl = (rbuf[0] >> 4) == 4 ? rbuf[8] : -1;
                return 1;
            }
        }
    }
    return 0;
#endif
}

static void print_usage(const char *prog)
{
    printf("Usage: %s [options] <hostname-or-IP-address>\n", prog);
    printf("Options:\n");
    printf("  -c count      number of pings to send (default %d)\n", DEFAULT_COUNT);
    printf("  -i interval_ms  delay between pings in ms (default %d)\n", DEFAULT_INTERVAL);
    printf("  -t timeout_ms timeout for each reply in ms (default %d)\n", DEFAULT_TIMEOUT);
    printf("  -h            show this help\n");
}

int main(int argc, char *argv[])
{
    int count = DEFAULT_COUNT, interval = DEFAULT_INTERVAL, timeout = DEFAULT_TIMEOUT;
    const char *target = NULL;
    int i;

    /* ---------- Parse command-line arguments ---------- */
    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) {
            count = atoi(argv[++i]);
            if (count <= 0) { fprintf(stderr, "Error: -c count must be a positive integer\n"); return 1; }
        } else if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            interval = atoi(argv[++i]);
            if (interval < 0) { fprintf(stderr, "Error: -i interval cannot be negative\n"); return 1; }
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            timeout = atoi(argv[++i]);
            if (timeout <= 0) { fprintf(stderr, "Error: -t timeout must be a positive integer\n"); return 1; }
        } else if (strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        } else {
            target = argv[i];
        }
    }
    if (!target) {
        print_usage(argv[0]);
        return 1;
    }

#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        fprintf(stderr, "Error: WSAStartup failed\n");
        return 1;
    }
#endif

    /* ---------- Resolve the target host ---------- */
    struct hostent *hp = gethostbyname(target);
    if (!hp || hp->h_addrtype != AF_INET) {
        fprintf(stderr, "Error: cannot resolve host \"%s\"\n", target);
#ifdef _WIN32
        WSACleanup();
#endif
        return 1;
    }
    struct in_addr addr;
    memcpy(&addr, hp->h_addr, hp->h_length);

    /* ---------- Initialize (platform specific) ---------- */
    ping_ctx_t ctx;
    if (ping_init(&ctx, addr) != 0) {
#ifdef _WIN32
        WSACleanup();
#endif
        return 1;
    }

    printf("Pinging %s [%s] with %d packets, interval %d ms, timeout %d ms\n\n",
           target, inet_ntoa(addr), count, interval, timeout);

    /* ---------- Main loop: send / receive ---------- */
    int sent = 0, received = 0;
    double rtt_min = 1e9, rtt_max = 0.0, rtt_sum = 0.0;

    for (i = 1; i <= count; i++) {
        unsigned long long rtt = 0;
        char from_ip[64] = "";
        int ttl = -1;

        sent++;
        if (ping_echo(&ctx, i, timeout, &rtt, from_ip, sizeof(from_ip), &ttl)) {
            received++;
            double rt = (double)rtt;
            if (rt < rtt_min) rtt_min = rt;
            if (rt > rtt_max) rtt_max = rt;
            rtt_sum += rt;
            if (ttl >= 0)
                printf("Reply from %s: bytes=%d time=%.1f ms TTL=%d\n",
                       from_ip, ICMP_DATA_LEN, rt, ttl);
            else
                printf("Reply from %s: bytes=%d time=%.1f ms\n",
                       from_ip, ICMP_DATA_LEN, rt);
        } else {
            printf("Request timed out (seq=%d)\n", i);
        }

        if (i < count)
            SLEEP_MS(interval);
    }

    /* ---------- Statistics ---------- */
    printf("\n--- %s ping statistics ---\n", target);
    printf("%d packets transmitted, %d received, %.0f%% packet loss",
           sent, received, sent ? 100.0 * (sent - received) / sent : 0.0);
    if (received > 0) {
        printf(", round-trip min/avg/max = %.1f/%.1f/%.1f ms\n",
               rtt_min, rtt_sum / received, rtt_max);
    } else {
        printf("\n");
    }

    ping_close(&ctx);
#ifdef _WIN32
    WSACleanup();
#endif
    return received > 0 ? 0 : 1;
}
