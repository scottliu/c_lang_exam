/*
 * mvcs_door_monitor.c
 *
 * 通过串口监听 MVCS <-> 门操作员系统线路通讯:
 *   - 打开串口 (RS485 总线, 默认 38.4 kbps / 8N1), 只读监听
 *   - 字节流喂入 mvcs_rx_t 状态机解析 HDLC 帧
 *   - 按 A1 设备类型区分 CDR(13H, MVCS->门) / CD(83H, 门->MVCS), 解码字段并打印
 *
 * 三种输入方式:
 *   mvcs_door_monitor <串口设备> [-b 波特率] [-v] [-o 日志] [--no-log]
 *   mvcs_door_monitor --file <字节流文件> [-v]    # 离线分析抓包文件
 *   mvcs_door_monitor --selftest                  # 内置帧回放自测
 *
 * 日志: 每次运行自动把监听结果写入日志文件 (默认 mvcs_monitor_YYYYMMDD_HHMMSS.log,
 * 可用 -o <路径> 指定, --no-log 关闭), 同时实时输出到控制台。
 *
 * 串口参数: 协议规定 38.4 kbps (MVCS_BAUDRATE); 数据位/校验/停止位文档未明确,
 * 默认 8N1, 可通过 -b 调整波特率。Windows 下端口名如 COM3 / COM10, 自动加 \\.\ 前缀。
 *
 * 编译:
 *   Windows: clang -Wall -Wextra -O2 -o mvcs_door_monitor.exe mvcs_door_monitor.c mvcs_door.c
 *   Linux : clang -Wall -Wextra -O2 -o mvcs_door_monitor  mvcs_door_monitor.c mvcs_door.c
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mvcs_door.h"

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#else
#  include <errno.h>
#  include <fcntl.h>
#  include <sys/time.h>
#  include <termios.h>
#  include <time.h>       /* struct tm / localtime_r */
#  include <unistd.h>
#endif

static int g_verbose = 0;
static FILE *g_log = NULL;   /* 日志文件 (NULL = 未启用), 所有输出双路写入 */

/* ======================== 日志输出 ========================
 * out(): 与 printf 同签名, 输出到控制台并同时写入日志文件 */
static int out(const char *fmt, ...)
{
    va_list ap;
    int     n;

    va_start(ap, fmt);
    n = vprintf(fmt, ap);
    va_end(ap);

    if (g_log) {
        va_start(ap, fmt);
        vfprintf(g_log, fmt, ap);
        fflush(g_log);          /* 逐行落盘, 便于中断时日志完整 */
        va_end(ap);
    }
    return n;
}

/* 生成默认日志文件名 mvcs_monitor_YYYYMMDD_HHMMSS.log */
static void make_default_log_name(char *buf, size_t cap)
{
#ifdef _WIN32
    SYSTEMTIME st;
    GetLocalTime(&st);
    snprintf(buf, cap, "mvcs_monitor_%04u%02u%02u_%02u%02u%02u.log",
             st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
#else
    struct timeval tv;
    struct tm      t;
    gettimeofday(&tv, NULL);
    localtime_r(&tv.tv_sec, &t);
    snprintf(buf, cap, "mvcs_monitor_%04d%02d%02d_%02d%02d%02d.log",
             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
             t.tm_hour, t.tm_min, t.tm_sec);
#endif
}

/* 打开日志文件 (path 为 NULL 时使用默认名), 成功返回 0 */
static int log_open(const char *path)
{
    char name[64];

    if (path)
        snprintf(name, sizeof(name), "%s", path);
    else
        make_default_log_name(name, sizeof(name));

    g_log = fopen(name, "a");   /* 追加模式: 不覆盖已有日志 */
    if (!g_log) {
        fprintf(stderr, "警告: 无法打开日志文件 %s, 仅输出到控制台\n", name);
        return -1;
    }
    out("日志文件: %s\n", name);
    return 0;
}

static void log_close(void)
{
    if (g_log) {
        fflush(g_log);
        fclose(g_log);
        g_log = NULL;
    }
}

/* ======================== 时间戳 ======================== */
static void print_timestamp(void)
{
#ifdef _WIN32
    SYSTEMTIME st;
    GetLocalTime(&st);
    out("[%02u:%02u:%02u.%03u] ", st.wHour, st.wMinute,
           st.wSecond, st.wMilliseconds);
#else
    struct timeval tv;
    struct tm      t;
    gettimeofday(&tv, NULL);
    localtime_r(&tv.tv_sec, &t);
    out("[%02d:%02d:%02d.%03ld] ", t.tm_hour, t.tm_min,
           t.tm_sec, (long)(tv.tv_usec / 1000));
#endif
}

/* ======================== 帧解码打印 ======================== */
static void print_hex(const char *tag, const uint8_t *p, size_t n)
{
    size_t i;

    out("  %s: ", tag);
    for (i = 0; i < n; i++)
        out("%02X ", p[i]);
    out("\n");
}

static void door_no_str(uint8_t no, char *buf, size_t cap)
{
    switch (no) {
    case MVCS_DOOR_NO_1:       snprintf(buf, cap, "1号门");   break;
    case MVCS_DOOR_NO_2:       snprintf(buf, cap, "2号门");   break;
    case MVCS_DOOR_NO_3:       snprintf(buf, cap, "3号门");   break;
    case MVCS_DOOR_NO_4:       snprintf(buf, cap, "4号门");   break;
    case MVCS_DOOR_NO_ALL:     snprintf(buf, cap, "全部门");  break;
    default:                   snprintf(buf, cap, "门号%02XH", no); break;
    }
}

/* BCD 字节 -> 十进制 (0x26 -> 26) */
static unsigned bcd2dec(uint8_t b)
{
    return (unsigned)(((b >> 4) & 0x0F) * 10 + (b & 0x0F));
}

static void print_cdr(const mvcs_cdr_fields_t *f)
{
    out("  CDR: 发车测试[开始请求=%d 执行请求=%d] 项目=0x%02X%s"
           " 时间校验=%d\n",
           f->dt_start_req, f->dt_exec_req, f->dt_item_no,
           f->dt_item_no == MVCS_DT_ITEM_DOOR_OPEN_CLOSE_TEST
               ? "(门开关测试)" : "",
           f->time_valid);
    if (f->time_valid)
        out("  时间: 20%02u-%02u-%02u %02u:%02u:%02u (BCD)\n",
               bcd2dec(f->year), bcd2dec(f->month), bcd2dec(f->date),
               bcd2dec(f->hour), bcd2dec(f->minute), bcd2dec(f->second));
    if (f->door_open_cmd) {
        char list[32] = "";
        size_t o = 0;
        int    i;

        for (i = 0; i < 4; i++) {
            if (f->door_open_cmd & (1u << i)) {
                int n = snprintf(list + o, sizeof(list) - o, "%s%d号门",
                                 o ? "," : "", i + 1);
                if (n > 0)
                    o += (size_t)n;
            }
        }
        out("  开门命令: %s\n", list);
    }
}

static void print_faults(const uint8_t *t23, const uint8_t *t24)
{
    static const struct { uint8_t idx, bit; const char *name; } f23[] = {
        { 23, 0, "IPM故障" },  { 23, 1, "电流传感器故障" },
        { 23, 2, "编码器故障" },{ 23, 3, "EEPROM故障" },
        { 23, 4, "DLS故障" },  { 23, 5, "自诊断错误" },
        { 23, 7, "初始操作未完成" },
        { 24, 0, "过压" },     { 24, 3, "门轻故障" },
        { 24, 4, "门重故障" }, { 24, 5, "关门超时" },
        { 24, 6, "开门超时" }, { 24, 7, "过热" },
    };
    size_t i;
    int    any = 0;

    out("  故障: ");
    for (i = 0; i < sizeof(f23) / sizeof(f23[0]); i++) {
        const uint8_t *t = f23[i].idx == 23 ? t23 : t24;
        if (*t & (1u << f23[i].bit)) {
            out("%s%s", any ? "," : "", f23[i].name);
            any = 1;
        }
    }
    out("%s\n", any ? "" : "无");
}

static void print_cd(const mvcs_cd_fields_t *f)
{
    size_t i;

    out("  发车测试: 条件应答=%d 执行中=%d 结束=%d 项目=0x%02X%s\n",
           f->dt_conditions, f->dt_executing, f->dt_ends, f->dt_item_no,
           f->dt_item_no == MVCS_DT_ITEM_DOOR_OPEN_CLOSE_TEST
               ? "(门开关测试)" : "");
    if (f->dt_open_ok || f->dt_open_ng || f->dt_close_ok || f->dt_close_ng)
        out("  判定: 开门[OK=%d NG=%d] 关门[OK=%d NG=%d]\n",
               f->dt_open_ok, f->dt_open_ng,
               f->dt_close_ok, f->dt_close_ng);
    if (f->dt_time_ms || f->dt_current_ma || f->dt_open_pos_mm)
        out("  测量: 时间=%u(0.1s)=%.1fs 电流=%u(0.05A)=%.2fA "
               "门开位置=%u(5mm)=%u mm\n",
               f->dt_time_ms, f->dt_time_ms / 10.0,
               f->dt_current_ma, f->dt_current_ma * 0.05,
               f->dt_open_pos_mm, (unsigned)f->dt_open_pos_mm * 5u);

    out("  门状态: ");
    {
        const struct { int v; const char *n; } s[] = {
            { f->door_fully_open,  "全开"  },
            { f->door_fully_closed, "全关" },
            { f->door_opening,     "开门中" },
            { f->door_closing,     "关门中" },
            { f->door_close_with_obstruction, "带障碍关门" },
            { f->door_in_initial_operation,   "初始操作" },
        };
        int any = 0;
        for (i = 0; i < sizeof(s) / sizeof(s[0]); i++)
            if (s[i].v) {
                out("%s%s", any ? "," : "", s[i].n);
                any = 1;
            }
        out("%s\n", any ? "" : "无");
    }

    if (f->crew_switch_manual_unlocking || f->emergency_handle_manual_unlocking ||
        f->obstruction_buzzer || f->obstruction_recycle_timeout)
        out("  车厢信号: %s%s%s%s\n",
               f->crew_switch_manual_unlocking ? "乘务员手动解锁 " : "",
               f->emergency_handle_manual_unlocking ? "紧急手柄手动解锁 " : "",
               f->obstruction_buzzer ? "障碍蜂鸣器 " : "",
               f->obstruction_recycle_timeout ? "障碍回收超时 " : "");

    if (f->motor_driving_state || f->overload || f->dcs_status ||
        f->dis_status || f->dls_status)
        out("  内部信息: %s%s%s%s%s\n",
               f->motor_driving_state ? "电机驱动 " : "",
               f->overload ? "过载 " : "",
               f->dcs_status ? "DCS " : "",
               f->dis_status ? "DIS " : "",
               f->dls_status ? "DLS " : "");

    out("  重复关闭计数=%u 版本=0x%04X 校验和=0x%04X\n",
           f->reclose_count, f->program_version, f->program_checksum);
}

static void print_frame(const mvcs_rx_t *rx)
{
    size_t info_len = rx->frame_len - (2 + MVCS_CONTROL_LEN + MVCS_CRC_LEN);
    const uint8_t *info = rx->body + 3;
    char no[16];

    print_timestamp();
    door_no_str(rx->rx_addr2, no, sizeof(no));

    if (rx->rx_addr1 == MVCS_ADDR_CDR) {
        out("MVCS -> %s (CDR) Ctrl=%02X 信息%zuB\n",
               no, rx->rx_control, info_len);
        if (info_len >= MVCS_CDR_INFO_LEN) {
            mvcs_cdr_fields_t f;
            mvcs_cdr_unpack(info, &f);
            print_cdr(&f);
        }
    } else if (rx->rx_addr1 == MVCS_ADDR_CD) {
        out("%s -> MVCS (CD) Ctrl=%02X 信息%zuB\n",
               no, rx->rx_control, info_len);
        if (info_len >= MVCS_CD_INFO_LEN) {
            mvcs_cd_fields_t f;
            uint8_t t23[1], t24[1];

            mvcs_cd_unpack(info, &f);
            print_cd(&f);
            /* 故障位在 TEXT[23]/TEXT[24] */
            t23[0] = info[23];
            t24[0] = info[24];
            print_faults(t23, t24);
        }
    } else {
        out("未知设备类型 %02XH, %s, Ctrl=%02X 信息%zuB\n",
               rx->rx_addr1, no, rx->rx_control, info_len);
    }

    if (g_verbose)
        print_hex("原始帧体", rx->body, rx->frame_len);
}

/* ======================== 字节流处理 ======================== */
static void feed_byte(mvcs_rx_t *rx, uint8_t b)
{
    int ev = mvcs_rx_feed(rx, b);

    switch (ev) {
    case MVCS_RX_EVENT_FRAME:
        print_frame(rx);
        break;
    case MVCS_RX_EVENT_CRC_ERR:
        print_timestamp();
        out("!! 帧 CRC 校验失败, 已丢弃 (协议规定不回复)\n");
        break;
    case MVCS_RX_EVENT_ABORT:
        print_timestamp();
        out("!! 检测到 abort (连续 7 个 1), 已复位\n");
        break;
    default:
        break;
    }
}

/* ======================== 串口 (Windows) ======================== */
#ifdef _WIN32
static HANDLE g_serial = INVALID_HANDLE_VALUE;

static int serial_open(const char *dev, unsigned baud)
{
    char    path[64];
    DCB     dcb;
    COMMTIMEOUTS to;

    if (strncmp(dev, "\\\\.\\", 4) == 0)
        snprintf(path, sizeof(path), "%s", dev);
    else
        snprintf(path, sizeof(path), "\\\\.\\%s", dev);

    g_serial = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                           OPEN_EXISTING, 0, NULL);
    if (g_serial == INVALID_HANDLE_VALUE)
        return -1;

    if (!GetCommState(g_serial, &dcb))
        return -1;
    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary  = TRUE;
    dcb.fParity  = FALSE;
    if (!SetCommState(g_serial, &dcb))
        return -1;

    /* 100 ms 轮询超时, 便于 Ctrl+C 退出 */
    to.ReadIntervalTimeout        = MAXDWORD;
    to.ReadTotalTimeoutMultiplier = 0;
    to.ReadTotalTimeoutConstant   = 100;
    to.WriteTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant  = 0;
    SetCommTimeouts(g_serial, &to);
    PurgeComm(g_serial, PURGE_RXCLEAR);
    return 0;
}

static int serial_read_byte(uint8_t *b)
{
    DWORD n = 0;

    if (!ReadFile(g_serial, b, 1, &n, NULL))
        return GetLastError() == ERROR_OPERATION_ABORTED ? 0 : -1;
    return n == 1 ? 1 : 0;
}

static void serial_close(void)
{
    if (g_serial != INVALID_HANDLE_VALUE) {
        CloseHandle(g_serial);
        g_serial = INVALID_HANDLE_VALUE;
    }
}

/* ======================== 串口 (POSIX) ======================== */
#else
static int g_serial = -1;

static speed_t baud_to_speed(unsigned baud)
{
    switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    default:     return B38400;
    }
}

static int serial_open(const char *dev, unsigned baud)
{
    struct termios tio;

    /* O_NDELAY: 打开时不阻塞于载波检测; 打开后立即恢复阻塞读 */
    g_serial = open(dev, O_RDONLY | O_NOCTTY | O_NDELAY);
    if (g_serial < 0)
        return -1;
    if (fcntl(g_serial, F_SETFL, 0) != 0)   /* 清除 O_NDELAY */
        return -1;

    if (tcgetattr(g_serial, &tio) != 0)
        return -1;
    cfmakeraw(&tio);
    cfsetispeed(&tio, baud_to_speed(baud));
    cfsetospeed(&tio, baud_to_speed(baud));
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cc[VMIN]  = 1;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(g_serial, TCSANOW, &tio) != 0)
        return -1;
    return 0;
}

static int serial_read_byte(uint8_t *b)
{
    ssize_t n;

    for (;;) {
        n = read(g_serial, b, 1);
        if (n == 1)
            return 1;
        if (n < 0 && errno == EINTR)
            continue;
        return n == 0 ? 0 : -1;
    }
}

static void serial_close(void)
{
    if (g_serial >= 0) {
        close(g_serial);
        g_serial = -1;
    }
}
#endif

/* ======================== 运行模式 ======================== */
static int run_serial(const char *dev, unsigned baud)
{
    mvcs_rx_t rx;
    uint8_t   b;
    int       r;

    if (serial_open(dev, baud) != 0) {
        fprintf(stderr, "无法打开串口 %s (请检查端口名与权限)\n", dev);
        return 1;
    }
    out("监听 %s @ %u bps 8N1 ... (Ctrl+C 退出)\n", dev, baud);

    mvcs_rx_init(&rx);
    for (;;) {
        r = serial_read_byte(&b);
        if (r == 1)
            feed_byte(&rx, b);
        else if (r < 0) {
            fprintf(stderr, "串口读取错误, 退出\n");
            break;
        }
    }
    serial_close();
    return 0;
}

static int run_file(const char *path)
{
    FILE    *fp = fopen(path, "rb");
    mvcs_rx_t rx;
    uint8_t   b;
    size_t    total = 0;

    if (!fp) {
        fprintf(stderr, "无法打开文件 %s\n", path);
        return 1;
    }
    mvcs_rx_init(&rx);
    while (fread(&b, 1, 1, fp) == 1) {
        feed_byte(&rx, b);
        total++;
    }
    fclose(fp);
    out("--- 回放完成: 共 %zu 字节 ---\n", total);
    return 0;
}

static int run_selftest(void)
{
    mvcs_cdr_fields_t req;
    mvcs_cd_fields_t  st;
    uint8_t cdr_text[MVCS_CDR_INFO_LEN], cd_text[MVCS_CD_INFO_LEN];
    uint8_t frame[MVCS_FRAME_MAX_LEN];
    mvcs_rx_t rx;
    size_t   flen, i;

    out("== 自测: 回放 1 帧 CDR + 1 帧 CD ==\n\n");

    /* 帧 1: MVCS 向 1 号门下发开门命令 */
    memset(&req, 0, sizeof(req));
    req.time_valid = 1;
    req.year = 0x26; req.month = 0x08; req.date = 0x07;
    req.hour = 0x22; req.minute = 0x46; req.second = 0x00;
    req.door_open_cmd = MVCS_DOOR_NO_1 | MVCS_DOOR_NO_3;
    mvcs_cdr_pack(&req, cdr_text);
    flen = mvcs_frame_build(MVCS_ADDR_CDR, MVCS_DOOR_NO_1, 0x00,
                            cdr_text, MVCS_CDR_INFO_LEN,
                            frame, sizeof(frame));
    mvcs_rx_init(&rx);
    for (i = 0; i < flen; i++)
        feed_byte(&rx, frame[i]);
    out("\n");

    /* 帧 2: 3 号门回复门状态 (开门中 + 一项故障) */
    memset(&st, 0, sizeof(st));
    st.door_opening = 1;
    st.motor_driving_state = 1;
    st.dcs_status = 1;
    st.ipm_failure = 1;
    st.reclose_count = 2;
    st.program_version = 0x0103;
    st.program_checksum = 0xABCD;
    mvcs_cd_pack(&st, cd_text);
    flen = mvcs_frame_build(MVCS_ADDR_CD, MVCS_DOOR_NO_3, 0x00,
                            cd_text, MVCS_CD_INFO_LEN,
                            frame, sizeof(frame));
    for (i = 0; i < flen; i++)
        feed_byte(&rx, frame[i]);
    out("\n--- 自测完成 ---\n");
    return 0;
}

static void usage(const char *prog)
{
    out("用法:\n");
    out("  %s <串口设备> [-b 波特率] [-v] [-o 日志] [--no-log]  监听串口 (默认 38400 bps 8N1)\n", prog);
    out("  %s --file <字节流文件> [-v] [-o 日志] [--no-log]     离线回放抓包字节流\n", prog);
    out("  %s --selftest [-o 日志] [--no-log]                   内置帧回放自测\n", prog);
    out("  %s -h                                                帮助\n", prog);
    out("选项:\n");
    out("  -o <路径>   日志文件路径 (默认 mvcs_monitor_时间戳.log, 追加写入)\n");
    out("  --no-log    关闭日志, 仅输出到控制台\n");
    out("  -b <波特率> 串口波特率 (默认 38400)\n");
    out("  -v          每帧额外打印原始字节\n");
    out("示例: %s COM3   /   %s /dev/ttyUSB0\n", prog, prog);
}

int main(int argc, char *argv[])
{
    unsigned baud = MVCS_BAUDRATE;
    const char *dev = NULL;
    const char *file = NULL;
    const char *log_path = NULL;
    int no_log = 0;
    int selftest = 0;
    int rc;
    int i;

#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);   /* 控制台 UTF-8 输出 */
#endif

    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else if (strcmp(argv[i], "--selftest") == 0) {
            selftest = 1;
        } else if (strcmp(argv[i], "--file") == 0) {
            if (i + 1 < argc)
                file = argv[++i];
        } else if (strcmp(argv[i], "-b") == 0) {
            if (i + 1 < argc)
                baud = (unsigned)strtoul(argv[++i], NULL, 10);
        } else if (strcmp(argv[i], "-o") == 0) {
            if (i + 1 < argc)
                log_path = argv[++i];
        } else if (strcmp(argv[i], "--no-log") == 0) {
            no_log = 1;
        } else if (strcmp(argv[i], "-v") == 0) {
            g_verbose = 1;
        } else if (argv[i][0] == '-') {
            fprintf(stderr, "未知选项: %s\n", argv[i]);
            usage(argv[0]);
            return 1;
        } else if (!dev) {
            dev = argv[i];
        }
    }

    /* 默认每次运行都记录日志 (可用 --no-log 关闭) */
    if (!no_log && (dev || file || selftest))
        log_open(log_path);

    if (selftest)
        rc = run_selftest();
    else if (file)
        rc = run_file(file);
    else if (dev)
        rc = run_serial(dev, baud);
    else {
        usage(argv[0]);
        rc = 1;
    }

    log_close();
    return rc;
}
