/*
 * mvcs_door_demo.c
 *
 * MVCS <-> Door Operator System 协议演示与自测:
 *   1. CRC-CCITT 已知向量校验 (CRC-CCITT-FALSE("123456789") = 0x29B1)
 *   2. CDR (MVCS -> 门) 编解码 + HDLC 帧构建/接收往返
 *   3. CD  (门 -> MVCS) 编解码 + HDLC 帧构建/接收往返
 *   4. 模拟 20 ms 轮询周期: MVCS 下发开门命令, 门回复状态
 *   5. 错误注入: 翻转一 bit, 验证接收端 CRC 错误处理 (不回复)
 *
 * 编译: gcc -Wall -Wextra -o mvcs_door_demo mvcs_door_demo.c mvcs_door.c
 */
#include <stdio.h>
#include <string.h>

#include "mvcs_door.h"

static int failures = 0;

#define CHECK(cond, msg)                                                    \
    do {                                                                    \
        if (cond) {                                                         \
            printf("[PASS] %s\n", msg);                                     \
        } else {                                                            \
            printf("[FAIL] %s\n", msg);                                     \
            failures++;                                                     \
        }                                                                   \
    } while (0)

static void hex_dump(const char *tag, const uint8_t *p, size_t n)
{
    size_t i;

    printf("  %-28s", tag);
    for (i = 0; i < n; i++)
        printf("%02X ", p[i]);
    printf("\n");
}

/* 把整帧逐字节喂入接收状态机, 返回最后一次事件 */
static int feed_all(mvcs_rx_t *rx, const uint8_t *frame, size_t len)
{
    size_t i;
    int    ev = MVCS_RX_EVENT_NONE;

    for (i = 0; i < len; i++)
        ev = mvcs_rx_feed(rx, frame[i]);
    return ev;
}

/* 模拟 1.5 Mbps 时钟下的一个发送位周期 (仅演示时序注释用) */
#define BIT_TIME_US  (1000000UL / MVCS_BAUDRATE)

static void test_crc(void)
{
    uint16_t crc = mvcs_crc_ccitt((const uint8_t *)"123456789", 9);

    printf("== 1. CRC-CCITT 已知向量 ==\n");
    printf("  crc(\"123456789\") = 0x%04X (期望 0x29B1)\n", crc);
    CHECK(crc == 0x29B1u, "CRC-CCITT 向量校验");
}

static void test_cdr_roundtrip(void)
{
    mvcs_cdr_fields_t f, f2;
    uint8_t text[MVCS_CDR_INFO_LEN];
    uint8_t frame[MVCS_FRAME_MAX_LEN];
    mvcs_rx_t rx;
    size_t    flen;

    printf("\n== 2. CDR (MVCS -> 门) 往返 ==\n");

    memset(&f, 0, sizeof(f));
    f.dt_start_req  = 1;              /* 发车测试开始请求 */
    f.dt_exec_req   = 0;
    f.dt_item_no    = MVCS_DT_ITEM_DOOR_OPEN_CLOSE_TEST; /* 01H 门开关测试 */
    f.time_valid    = 1;              /* 时间有效 */
    f.year  = 0x26;                   /* 2026 年 (BCD) */
    f.month = 0x08;
    f.date  = 0x07;
    f.hour  = 0x22;
    f.minute = 0x44;
    f.second = 0x48;
    f.door_open_cmd = MVCS_DOOR_NO_1 | MVCS_DOOR_NO_3; /* 打开 1 号、3 号门 */

    mvcs_cdr_pack(&f, text);
    hex_dump("CDR TEXT (20B):", text, MVCS_CDR_INFO_LEN);

    /* MVCS 下发: 地址 13H(CDR) + 01H(1 号门) */
    flen = mvcs_frame_build(MVCS_ADDR_CDR, MVCS_DOOR_NO_1, 0x00,
                            text, MVCS_CDR_INFO_LEN, frame, sizeof(frame));
    CHECK(flen > 0 && flen <= MVCS_FRAME_MAX_LEN, "CDR 帧构建成功");
    hex_dump("HDLC 帧:", frame, flen);

    mvcs_rx_init(&rx);
    CHECK(feed_all(&rx, frame, flen) == MVCS_RX_EVENT_FRAME, "CDR 帧接收/CRC 通过");
    CHECK(rx.rx_addr1 == MVCS_ADDR_CDR && rx.rx_addr2 == MVCS_DOOR_NO_1,
          "CDR 帧地址解析正确");

    mvcs_cdr_unpack(rx.body + 3, &f2);
    CHECK(f2.dt_start_req == f.dt_start_req &&
          f2.dt_item_no == f.dt_item_no &&
          f2.time_valid == f.time_valid &&
          f2.year == f.year && f2.month == f.month && f2.date == f.date &&
          f2.hour == f.hour && f2.minute == f.minute && f2.second == f.second,
          "CDR 字段解出一致");
    CHECK(f2.door_open_cmd == f.door_open_cmd, "开门命令位图一致");
}

static void test_cd_roundtrip(void)
{
    mvcs_cd_fields_t f, f2;
    uint8_t text[MVCS_CD_INFO_LEN];
    uint8_t frame[MVCS_FRAME_MAX_LEN];
    mvcs_rx_t rx;
    size_t    flen;

    printf("\n== 3. CD (门 -> MVCS) 往返 ==\n");

    memset(&f, 0, sizeof(f));
    f.dt_conditions = 1;              /* 应答发车测试条件 */
    f.dt_executing  = 1;              /* 测试执行中 */
    f.dt_item_no    = MVCS_DT_ITEM_DOOR_OPEN_CLOSE_TEST;
    f.dt_open_ok    = 1;              /* 开门操作判定 OK */
    f.jdg_open_time     = 1;
    f.jdg_open_current = 1;
    f.jdg_open_position = 1;
    f.dt_time_ms        = 38;         /* 3.8 s (0.1 s/bit) */
    f.dt_current_ma     = 20;         /* 1.0 A  (0.05 A/bit) */
    f.dt_open_pos_mm    = 200;        /* 1000 mm (5 mm/bit) */
    f.door_open_dim_mm  = 1030;
    f.door_open_close_count = 12345;
    f.door_fully_open   = 1;          /* 门全开 */
    f.motor_driving_state = 1;
    f.overload          = 0;
    f.dcs_status        = 1;
    f.reclose_count     = 3;
    f.program_version   = 0x0102;
    f.program_checksum  = 0x5A5A;

    mvcs_cd_pack(&f, text);
    hex_dump("CD TEXT (36B):", text, MVCS_CD_INFO_LEN);

    /* 门回复: 地址 83H(CD) + 01H(1 号门) */
    flen = mvcs_frame_build(MVCS_ADDR_CD, MVCS_DOOR_NO_1, 0x00,
                            text, MVCS_CD_INFO_LEN, frame, sizeof(frame));
    CHECK(flen > 0, "CD 帧构建成功");
    hex_dump("HDLC 帧:", frame, flen);

    mvcs_rx_init(&rx);
    CHECK(feed_all(&rx, frame, flen) == MVCS_RX_EVENT_FRAME, "CD 帧接收/CRC 通过");
    CHECK(rx.rx_addr1 == MVCS_ADDR_CD && rx.rx_addr2 == MVCS_DOOR_NO_1,
          "CD 帧地址解析正确");

    mvcs_cd_unpack(rx.body + 3, &f2);
    CHECK(f2.dt_conditions == f.dt_conditions &&
          f2.dt_executing == f.dt_executing &&
          f2.dt_item_no == f.dt_item_no &&
          f2.dt_open_ok == f.dt_open_ok &&
          f2.jdg_open_time == f.jdg_open_time &&
          f2.jdg_open_position == f.jdg_open_position,
          "CD 发车测试字段一致");
    CHECK(f2.dt_time_ms == f.dt_time_ms &&
          f2.dt_current_ma == f.dt_current_ma &&
          f2.dt_open_pos_mm == f.dt_open_pos_mm &&
          f2.door_open_dim_mm == f.door_open_dim_mm &&
          f2.door_open_close_count == f.door_open_close_count,
          "CD 测量值字段一致");
    CHECK(f2.door_fully_open == 1 && f2.motor_driving_state == 1 &&
          f2.dcs_status == 1 && f2.reclose_count == 3 &&
          f2.program_version == 0x0102 && f2.program_checksum == 0x5A5A,
          "CD 门状态/内部信息/版本字段一致");
}

static void test_poll_cycle(void)
{
    mvcs_cdr_fields_t req;
    mvcs_cd_fields_t  st;
    uint8_t cdr_text[MVCS_CDR_INFO_LEN], cd_text[MVCS_CD_INFO_LEN];
    uint8_t cdr_frame[MVCS_FRAME_MAX_LEN], cd_frame[MVCS_FRAME_MAX_LEN];
    mvcs_rx_t door_rx, mvcs_rx;
    size_t cdr_len, cd_len;
    int    ev;

    printf("\n== 4. 模拟 20 ms 轮询周期 (MVCS -> 门1 -> MVCS) ==\n");

    /* MVCS 下发 CDR: 打开 2 号门, 时间有效 */
    memset(&req, 0, sizeof(req));
    req.time_valid = 1;
    req.year = 0x26; req.month = 0x08; req.date = 0x07;
    req.hour = 0x22; req.minute = 0x45; req.second = 0x00;
    req.door_open_cmd = MVCS_DOOR_NO_2;
    mvcs_cdr_pack(&req, cdr_text);
    cdr_len = mvcs_frame_build(MVCS_ADDR_CDR, MVCS_DOOR_NO_1, 0x00,
                               cdr_text, MVCS_CDR_INFO_LEN,
                               cdr_frame, sizeof(cdr_frame));

    /* 门侧接收 (发送使能在 PAD 前 >= 1 bit 使能, 帧尾后 <= 6 bit 释放) */
    mvcs_rx_init(&door_rx);
    ev = feed_all(&door_rx, cdr_frame, cdr_len);
    CHECK(ev == MVCS_RX_EVENT_FRAME &&
          door_rx.rx_addr2 == MVCS_DOOR_NO_1, "门1 收到 MVCS 开门命令");

    /* 只有地址指定的门回复 (4.7.2.3-2); 门1 在 20 ms 内回复 CD */
    memset(&st, 0, sizeof(st));
    st.door_opening = 1;              /* 门正在打开 */
    st.motor_driving_state = 1;
    mvcs_cd_pack(&st, cd_text);
    cd_len = mvcs_frame_build(MVCS_ADDR_CD, MVCS_DOOR_NO_1, 0x00,
                              cd_text, MVCS_CD_INFO_LEN,
                              cd_frame, sizeof(cd_frame));

    /* MVCS 侧接收 */
    mvcs_rx_init(&mvcs_rx);
    ev = feed_all(&mvcs_rx, cd_frame, cd_len);
    CHECK(ev == MVCS_RX_EVENT_FRAME &&
          mvcs_rx.rx_addr1 == MVCS_ADDR_CD &&
          mvcs_rx.rx_addr2 == MVCS_DOOR_NO_1, "MVCS 收到门1 状态回复");

    printf("  (轮询周期 %d ms, 传输能力 %d bytes/20 ms, 帧长 %d B)\n",
           MVCS_POLL_CYCLE_MS, MVCS_TRANSFER_BYTES, (int)cd_len);
    printf("  半双工发送使能: PAD 前 %d bit 使能, closing flag 后 %d bit 内释放\n",
           MVCS_TX_ENABLE_LEAD_BITS, MVCS_TX_ENABLE_TRAIL_BITS);
}

static void test_crc_error(void)
{
    uint8_t text[MVCS_CD_INFO_LEN];
    uint8_t frame[MVCS_FRAME_MAX_LEN];
    mvcs_rx_t rx;
    size_t flen, i;

    printf("\n== 5. 错误注入: 数据区翻转 1 bit ==\n");
    memset(text, 0xA5, sizeof(text));
    flen = mvcs_frame_build(MVCS_ADDR_CD, MVCS_DOOR_NO_2, 0x00,
                            text, MVCS_CD_INFO_LEN, frame, sizeof(frame));

    /* 在信息字段中翻转一个 bit (帧偏移: PAD4 + Flag1 + A1 + A2 + Ctrl = 8) */
    frame[8] ^= 0x01;

    mvcs_rx_init(&rx);
    CHECK(feed_all(&rx, frame, flen) == MVCS_RX_EVENT_CRC_ERR,
          "CRC 校验失败被检出 (接收错误时不回复, 4.7.2.3-3)");
    CHECK(rx.len == 0, "错误帧已被丢弃");

    /* 双帧连续接收: 错误帧后紧跟正确帧仍能正常接收 */
    for (i = 0; i < 3; i++)
        mvcs_rx_feed(&rx, 0x00);      /* 帧间空闲/填充 */
    flen = mvcs_frame_build(MVCS_ADDR_CD, MVCS_DOOR_NO_2, 0x00,
                            text, MVCS_CD_INFO_LEN, frame, sizeof(frame));
    CHECK(feed_all(&rx, frame, flen) == MVCS_RX_EVENT_FRAME,
          "错误帧后正确帧仍可接收");
}

int main(void)
{
    printf("MVCS <-> Door Operator System 协议演示 (WDR502215E h 4.7)\n");

    test_crc();
    test_cdr_roundtrip();
    test_cd_roundtrip();
    test_poll_cycle();
    test_crc_error();

    printf("\n%s (%d 失败)\n", failures ? "结果: 有失败" : "结果: 全部通过", failures);
    return failures ? 1 : 0;
}
