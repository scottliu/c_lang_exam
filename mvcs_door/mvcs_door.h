/*
 * mvcs_door.h
 *
 * MVCS (Mitsubishi Vehicle Control System) <-> Door Operator System 通信协议实现
 *
 * 依据文档: WDR502215E h 第 4.6 / 4.7 章节
 *   - 4.7.1  门操作员系统站号(VCS No.)设置
 *   - 4.7.2  MVCS 通信规格 (物理层 / 数据链路层 / 收发规则 / CDR / CD 内容)
 *   - 4.7.3  发车测试功能
 *
 * 协议要点:
 *   - RS485 三线半双工, NRZi 编码, 38.4 kbps, 轮询周期 20 ms
 *   - HDLC 兼容帧: Opening PAD(4B) + Flag(7EH) + Address1/Upper(1B) + Address2/Lower(1B)
 *                  + Control(1B) + Information(CDR 20B / CD 36B) + CRC(2B) + Flag(7EH)
 *   - 地址: 设备类型 13H=CDR(MVCS->门), 83H=CD(门->MVCS), FFH=全选;
 *           门号 01H~04H (1 号门位于 1st side/右侧, 2 号门位于 2nd side/左侧)
 *   - CRC: CCITT-1 多项式 x16+x12+x5+1 (0x1021)
 */
#ifndef MVCS_DOOR_H
#define MVCS_DOOR_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 物理层 (4.7.2.1) ======================== */
#define MVCS_BAUDRATE              38400UL   /* 传输速度 38.4 kbps ±0.1% */
#define MVCS_POLL_CYCLE_MS         20        /* 轮询周期 20 ms */
#define MVCS_MAX_STATIONS          4         /* 连接站数: 4 */
#define MVCS_TRANSFER_BYTES        80        /* 传输能力: 80 bytes / 20 ms */
#define MVCS_GROUP_1ST_SIDE        0x01U     /* 1st side(右): 01H,03H */
#define MVCS_GROUP_2ND_SIDE        0x02U     /* 2nd side(左): 02H,04H */

/* 半双工发送使能控制时序 (4.7.2.3-5) */
#define MVCS_TX_ENABLE_LEAD_BITS   1         /* opening PAD 开始前至少 1 bit 使能 */
#define MVCS_TX_ENABLE_TRAIL_BITS  6         /* closing flag 结束后 6 bits 内释放 */

/* ======================== 数据链路层 (4.7.2.2) ======================== */
#define MVCS_OPENING_PAD_LEN       4         /* Opening PAD: 4 字节 */
#define MVCS_FLAG                  0x7EU     /* Opening/Closing flag: 0111 1110B */
#define MVCS_CONTROL_LEN           1         /* 控制命令 1 字节 */

/* 设备类型 (Address1/Upper) */
#define MVCS_ADDR_CDR              0x13U     /* CDR: MVCS -> 门 */
#define MVCS_ADDR_CD               0x83U     /* CD:  门 -> MVCS */
#define MVCS_ADDR_ALL              0xFFU     /* 全部设备类型选择 */

/* 门号 (Address2/Lower) */
#define MVCS_DOOR_NO_1             0x01U
#define MVCS_DOOR_NO_2             0x02U
#define MVCS_DOOR_NO_3             0x03U
#define MVCS_DOOR_NO_4             0x04U
#define MVCS_DOOR_NO_ALL           0xFFU

/* 信息字段长度 */
#define MVCS_CDR_INFO_LEN          20        /* CDR: 接收数据 20 字节 */
#define MVCS_CD_INFO_LEN           36        /* CD:  发送数据 36 字节 */

/* CRC: CCITT-1, 多项式 x16+x12+x5+1 */
#define MVCS_CRC_POLY              0x1021U
#define MVCS_CRC_INIT              0xFFFFU
#define MVCS_CRC_LEN               2

/* 帧最大长度: PAD4 + Flag1 + [A1+A2+Ctrl+Info36+CRC2 = 41 B (bit-stuffing 后 <= 约 50 B)]
 *                + Flag1 = 4+1+50+1 = 56 B, 满足 80 bytes/20 ms 传输能力 */
#define MVCS_FRAME_MAX_LEN         64

/* ======================== CDR 字段 (4.7.2.4) ======================== */
/* TEXT No.2: 发车测试执行项目编号 */
#define MVCS_DT_ITEM_DOOR_OPEN_CLOSE_TEST  0x01U   /* 01H = 门开关测试 */

/* TEXT No.14: 开门命令位 (bit0..bit3 = 门1..门4) */
#define MVCS_DOOR_OPEN_CMD_MASK    0x0FU

typedef struct {
    uint8_t dt_start_req;   /* 发车测试开始请求      (TEXT[1] bit7) */
    uint8_t dt_exec_req;    /* 发车测试执行请求      (TEXT[1] bit0) */
    uint8_t dt_item_no;     /* 发车测试执行项目编号  (TEXT[2], BCD, 01H=门开关测试) */
    uint8_t time_valid;     /* 时间校验              (TEXT[6] bit0) */
    uint8_t year;           /* 年  BCD (TEXT[7]) */
    uint8_t month;          /* 月  BCD (TEXT[8]) */
    uint8_t date;           /* 日  BCD (TEXT[9]) */
    uint8_t hour;           /* 时  BCD (TEXT[10]) */
    uint8_t minute;         /* 分  BCD (TEXT[11]) */
    uint8_t second;         /* 秒  BCD (TEXT[12]) */
    uint8_t door_open_cmd;  /* 开门命令位图, bit0..3 = 门1..4 (TEXT[14]) */
} mvcs_cdr_fields_t;

/* ======================== CD 字段 (4.7.2.5) ======================== */
typedef struct {
    uint8_t dt_conditions;  /* 发车测试条件应答      (TEXT[1] bit0) */
    uint8_t dt_executing;   /* 发车测试执行中        (TEXT[1] bit6) */
    uint8_t dt_ends;        /* 发车测试结束          (TEXT[1] bit7) */
    uint8_t dt_item_no;     /* 执行中测试项目编号    (TEXT[2], BCD, 01H) */

    /* TEXT[3]: 发车测试判定 (bit0/bit1=开门操作 NG/OK, bit2/bit3=关门操作 NG/OK) */
    uint8_t dt_open_ng;
    uint8_t dt_open_ok;
    uint8_t dt_close_ng;
    uint8_t dt_close_ok;

    /* TEXT[4]: 测试项目判定 (bit0 开门时间 / bit1 开门电流 / bit2 开门位置 /
     *           bit3 关门时间 / bit4 关门电流, 1=OK 0=NG) */
    uint8_t jdg_open_time;
    uint8_t jdg_open_current;
    uint8_t jdg_open_position;
    uint8_t jdg_close_time;
    uint8_t jdg_close_current;

    uint8_t  dt_time_ms;        /* 开门/关门时间测量值   (TEXT[5], 0.1 s/bit) */
    uint8_t  dt_current_ma;     /* 平均电流测量值         (TEXT[6], 0.05 A/bit) */
    uint8_t  dt_open_pos_mm;    /* 门开位置测量值         (TEXT[7], 5 mm/bit, 全关=0) */
    uint16_t door_open_dim_mm;  /* 门开启尺寸             (TEXT[10..11], 1 mm/bit) */
    uint32_t door_open_close_count; /* 门开关计数          (TEXT[12..14], 1 count/bit, 低 24 位) */

    /* TEXT[19]: 门状态 (bit0 全开 / bit1 全关 / bit2 开门中 / bit3 关门中 /
     *           bit4 带障碍关门 / bit5 初始操作) */
    uint8_t door_fully_open;
    uint8_t door_fully_closed;
    uint8_t door_opening;
    uint8_t door_closing;
    uint8_t door_close_with_obstruction;
    uint8_t door_in_initial_operation;

    /* TEXT[20]: 来自车厢的信号 (bit0 乘务员开关手动解锁 / bit1 紧急手柄手动解锁 /
     *           bit5 障碍蜂鸣器 / bit6 障碍回收超时) */
    uint8_t crew_switch_manual_unlocking;
    uint8_t emergency_handle_manual_unlocking;
    uint8_t obstruction_buzzer;
    uint8_t obstruction_recycle_timeout;

    /* TEXT[22]: 内部信息 (bit0 电机驱动状态 / bit1 过载 / bit2 DCS 状态 /
     *           bit3 DIS 状态 / bit4 DLS 状态) */
    uint8_t motor_driving_state;
    uint8_t overload;
    uint8_t dcs_status;
    uint8_t dis_status;
    uint8_t dls_status;

    /* TEXT[23]: 故障信息 1 (bit0 IPM 故障 / bit1 电流传感器故障 / bit2 编码器故障 /
     *           bit3 EEPROM 故障 / bit4 DLS 故障 / bit5 自诊断错误 / bit7 初始操作未完成) */
    uint8_t ipm_failure;
    uint8_t current_sensor_fault;
    uint8_t encoder_fault;
    uint8_t eeprom_memory_fault;
    uint8_t dls_fault;
    uint8_t self_diagnosis_fault;
    uint8_t initial_operation_uncompleted;

    /* TEXT[24]: 故障信息 2 (bit0 过压 / bit3 门轻故障 / bit4 门重故障 /
     *           bit5 关门超时 / bit6 开门超时 / bit7 过热) */
    uint8_t overvoltage;
    uint8_t door_light_failure;
    uint8_t door_heavy_failure;
    uint8_t door_close_timeout;
    uint8_t door_open_timeout;
    uint8_t overheat;

    uint8_t  reclose_count;     /* 重复关闭计数   (TEXT[30], 参数值) */
    uint16_t program_version;   /* 程序数据版本   (TEXT[32..33]) */
    uint16_t program_checksum;  /* 程序数据校验和 (TEXT[34..35]) */
} mvcs_cd_fields_t;

/* ======================== API ======================== */
/* CRC-CCITT (多项式 0x1021, 初值 0xFFFF, 不反转), 用于帧内 A1..Information 的校验 */
uint16_t mvcs_crc_ccitt(const uint8_t *data, size_t len);

/* CDR / CD 字段编解码 */
void mvcs_cdr_pack(const mvcs_cdr_fields_t *f, uint8_t text[MVCS_CDR_INFO_LEN]);
void mvcs_cdr_unpack(const uint8_t text[MVCS_CDR_INFO_LEN], mvcs_cdr_fields_t *f);
void mvcs_cd_pack(const mvcs_cd_fields_t *f, uint8_t text[MVCS_CD_INFO_LEN]);
void mvcs_cd_unpack(const uint8_t text[MVCS_CD_INFO_LEN], mvcs_cd_fields_t *f);

/*
 * 构建 HDLC 帧 (含 Opening PAD + bit-stuffing):
 *   out 缓冲至少 MVCS_FRAME_MAX_LEN 字节, 返回帧总长; 失败返回 0
 *   addr1: 设备类型 (MVCS_ADDR_CDR / MVCS_ADDR_CD), addr2: 门号 (01H~04H / FFH)
 */
size_t mvcs_frame_build(uint8_t addr1, uint8_t addr2, uint8_t control,
                        const uint8_t *info, size_t info_len,
                        uint8_t *out, size_t out_cap);

/* 逐字节接收状态机 (半双工轮询, 需在 MVCS_TX_ENABLE_TRAIL_BITS 内释放发送使能)
 *
 * 用法: mvcs_rx_init(&rx); 然后对每个收到的字节调用 mvcs_rx_feed(&rx, byte)。
 * 返回 MVCS_RX_EVENT_FRAME 时, 本帧 CRC 校验已通过, 帧体解析结果:
 *   rx.body[0] = A1 (设备类型), rx.body[1] = A2 (门号), rx.body[2] = Control,
 *   rx.body[3 .. 3+info_len-1] = Information, 其后 2 字节为帧内 CRC;
 *   info_len = rx.len - (2 + MVCS_CONTROL_LEN + MVCS_CRC_LEN)。
 * 返回 CRC_ERR / ABORT 时本帧已丢弃 (协议规定接收错误不回复, 4.7.2.3-3)。 */
typedef struct {
    int      in_frame;        /* 是否已在帧内 (收到 opening flag) */
    int      stuffing;        /* 当前接收字节已收集位数 0..7 (去填充/组字节用) */
    int      ones_run;        /* 连续 1 计数 (去填充用) */
    size_t   len;             /* 已收集的帧体字节数 (不含 flag) */
    uint8_t  body[MVCS_FRAME_MAX_LEN];
    uint8_t  rx_addr1;        /* 解析出的地址 (供上层过滤) */
    uint8_t  rx_addr2;
    uint8_t  rx_control;
} mvcs_rx_t;

#define MVCS_RX_EVENT_NONE       0   /* 无完整事件 */
#define MVCS_RX_EVENT_FRAME      1   /* 收到完整且 CRC 校验通过的一帧 */
#define MVCS_RX_EVENT_CRC_ERR    2   /* 收到帧但 CRC 校验失败 (丢弃, 不回复) */
#define MVCS_RX_EVENT_ABORT      3   /* 收到连续 7 个以上 1 (HDLC abort) */

void mvcs_rx_init(mvcs_rx_t *rx);
int  mvcs_rx_feed(mvcs_rx_t *rx, uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif /* MVCS_DOOR_H */
