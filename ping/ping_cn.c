/*
 * ping.c — 用 C 语言实现的 ping (ICMP Echo) 程序
 *
 * 平台实现：
 *   Windows  : 使用 IP Helper API 的 IcmpSendEcho（官方推荐方式，无需管理员权限）
 *   Linux/macOS : 使用 SOCK_RAW 原始套接字 + ICMP 协议（需要 root/sudo）
 *
 * 编译：
 *   Linux/macOS:  gcc -o ping ping.c
 *   Windows:      gcc -o ping.exe ping.c -lws2_32 -liphlpapi
 *
 * 用法：
 *   ping [-c 次数] [-i 间隔毫秒] [-t 超时毫秒] <主机名或IP>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef _WIN32
#include <winsock2.h>   /* 必须先于 windows.h */
#include <windows.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>   /* 必须在 icmpapi.h 之前包含 */
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

/* 64 位 Windows 下，IcmpSendEcho 返回的回复按 32 位布局填充，
 * 必须用 ICMP_ECHO_REPLY32 读取，否则字段错位（MSDN 要求） */
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

#define ICMP_HEADER_LEN    8     /* ICMP 头部长度（不含数据） */
#define ICMP_DATA_LEN      48    /* 数据区长度（时间戳 + 填充） */
#define PACKET_LEN         (ICMP_HEADER_LEN + ICMP_DATA_LEN)

#define DEFAULT_COUNT      4
#define DEFAULT_INTERVAL   1000  /* 毫秒 */
#define DEFAULT_TIMEOUT    2000  /* 毫秒 */
#define RECV_BUF_SIZE      512

#ifdef _WIN32
/* ---- Windows：IcmpSendEcho 上下文 ---- */
typedef struct {
    HANDLE handle;    /* IcmpCreateFile() 返回的句柄 */
    DWORD  dst_ip;    /* 目标 IPv4 地址（网络字节序） */
} ping_ctx_t;
#else
/* ---- Unix：raw socket 上下文 ---- */
typedef struct {
    SOCKET fd;
    struct sockaddr_in dst;
    unsigned short pid;
} ping_ctx_t;
#endif

/* 单调时钟毫秒（Linux 用；Windows 的 RTT 由 IcmpSendEcho 直接给出） */
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
/* ICMP 头部结构（Unix raw socket 路径使用） */
typedef struct {
    unsigned char  type;
    unsigned char  code;
    unsigned short checksum;
    unsigned short id;
    unsigned short seq;
} icmp_header_t;

/* 计算 ICMP 校验和（RFC 1071），Unix raw socket 路径使用 */
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

/* 从接收缓冲提取 ICMP 报文：跳过 IPv4 头（若存在） */
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

/* ---- 平台实现：初始化 ---- */
static int ping_init(ping_ctx_t *ctx, struct in_addr addr)
{
#ifdef _WIN32
    ctx->dst_ip = addr.s_addr;
    ctx->handle = IcmpCreateFile();
    if (ctx->handle == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "错误: IcmpCreateFile 失败 (错误码=%lu)\n", GetLastError());
        return -1;
    }
    return 0;
#else
    ctx->fd = socket(AF_INET, SOCK_RAW, IPPROTO_ICMP);
    if (ctx->fd == INVALID_SOCKET) {
        fprintf(stderr, "错误: 创建原始套接字失败 (errno=%d)\n", errno);
        fprintf(stderr, "  请以 root 身份运行本程序 (sudo)\n");
        return -1;
    }
    memset(&ctx->dst, 0, sizeof(ctx->dst));
    ctx->dst.sin_family = AF_INET;
    ctx->dst.sin_addr = addr;
    ctx->pid = (unsigned short)(getpid() & 0xffff);
    return 0;
#endif
}

/* ---- 平台实现：关闭 ---- */
static void ping_close(ping_ctx_t *ctx)
{
#ifdef _WIN32
    if (ctx->handle != INVALID_HANDLE_VALUE)
        IcmpCloseHandle(ctx->handle);
#else
    CLOSE_SOCKET(ctx->fd);
#endif
}

/* ---- 平台实现：发送一次 echo 并等待回复 ----
 * 成功返回 1，并输出 rtt(ms)、来源 IP、TTL（不可得时 ttl=-1）；
 * 超时/失败返回 0。 */
static int ping_echo(ping_ctx_t *ctx, int seq, int timeout_ms,
                     unsigned long long *rtt_ms, char *from_ip, size_t ip_len, int *ttl)
{
#ifdef _WIN32
    (void)seq;   /* IcmpSendEcho 内部管理序号 */
    /* 请求数据：时间戳 + 填充 */
    unsigned char send_data[ICMP_DATA_LEN];
    unsigned long long ts = now_ms();
    memcpy(send_data, &ts, sizeof(ts));
    memset(send_data + sizeof(ts), 'a', sizeof(send_data) - sizeof(ts));

    /* ReplySize 需至少为 一个 ICMP_ECHO_REPLY + 数据长度 + 8（ICMP 错误报文） */
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
            *ttl = -1;   /* IcmpSendEcho 不返回 TTL */
            free(reply);
            return 1;
        }
    }
    free(reply);
    return 0;
#else
    /* 构造 ICMP 回显请求：头部 + 数据（时间戳 + 填充） */
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
        fprintf(stderr, "错误: sendto 失败 (errno=%d)\n", errno);
        return 0;
    }

    /* 等待回复 */
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
    printf("用法: %s [选项] <目标主机名或IP地址>\n", prog);
    printf("选项:\n");
    printf("  -c 次数      发送的 ping 包数量（默认 %d）\n", DEFAULT_COUNT);
    printf("  -i 间隔毫秒  两次发送的间隔（默认 %d）\n", DEFAULT_INTERVAL);
    printf("  -t 超时毫秒  等待回复的超时时间（默认 %d）\n", DEFAULT_TIMEOUT);
    printf("  -h           显示本帮助\n");
}

int main(int argc, char *argv[])
{
    int count = DEFAULT_COUNT, interval = DEFAULT_INTERVAL, timeout = DEFAULT_TIMEOUT;
    const char *target = NULL;
    int i;

    /* ---------- 解析命令行参数 ---------- */
    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) {
            count = atoi(argv[++i]);
            if (count <= 0) { fprintf(stderr, "错误: -c 次数必须为正整数\n"); return 1; }
        } else if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            interval = atoi(argv[++i]);
            if (interval < 0) { fprintf(stderr, "错误: -i 间隔不能为负\n"); return 1; }
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            timeout = atoi(argv[++i]);
            if (timeout <= 0) { fprintf(stderr, "错误: -t 超时必须为正整数\n"); return 1; }
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
        fprintf(stderr, "错误: WSAStartup 失败\n");
        return 1;
    }
#endif

    /* ---------- 解析目标主机 ---------- */
    struct hostent *hp = gethostbyname(target);
    if (!hp || hp->h_addrtype != AF_INET) {
        fprintf(stderr, "错误: 无法解析主机 \"%s\"\n", target);
#ifdef _WIN32
        WSACleanup();
#endif
        return 1;
    }
    struct in_addr addr;
    memcpy(&addr, hp->h_addr, hp->h_length);

    /* ---------- 初始化（平台相关） ---------- */
    ping_ctx_t ctx;
    if (ping_init(&ctx, addr) != 0) {
#ifdef _WIN32
        WSACleanup();
#endif
        return 1;
    }

    printf("正在 ping %s [%s] 发送 %d 个包，间隔 %d ms，超时 %d ms\n\n",
           target, inet_ntoa(addr), count, interval, timeout);

    /* ---------- 主循环：发送 / 接收 ---------- */
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
                printf("来自 %s 的回复: 字节=%d 时间=%.1f ms TTL=%d\n",
                       from_ip, ICMP_DATA_LEN, rt, ttl);
            else
                printf("来自 %s 的回复: 字节=%d 时间=%.1f ms\n",
                       from_ip, ICMP_DATA_LEN, rt);
        } else {
            printf("请求超时 (seq=%d)\n", i);
        }

        if (i < count)
            SLEEP_MS(interval);
    }

    /* ---------- 统计 ---------- */
    printf("\n--- %s ping 统计 ---\n", target);
    printf("%d 个包已发送，%d 个已接收，%.0f%% 丢失",
           sent, received, sent ? 100.0 * (sent - received) / sent : 0.0);
    if (received > 0) {
        printf("，往返时间 min/avg/max = %.1f/%.1f/%.1f ms\n",
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
