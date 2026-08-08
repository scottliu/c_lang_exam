/*
 * mvcs_door.c
 *
 * MVCS <-> Door Operator System 通信协议实现 (见 mvcs_door.h 说明)
 *
 * 依据 WDR502215E h 4.7.2:
 *   - 帧: Opening PAD(4B, 用于 DPLL 时钟同步) + Flag(7EH) + A1 + A2 + Control
 *         + Information + CRC(CCITT-1, 2B) + Flag(7EH)
 *   - 位序: HDLC 标准, 每字节 LSB 先发; bit-stuffing 在连续 5 个 1 后插入 0
 *   - CRC 计算范围: A1 + A2 + Control + Information (不含 flag/PAD)
 *
 * 注: 文档仅给出多项式 x16+x12+x5+1, 未明确初值/取反/字节序约定。
 *     本实现采用 HDLC 常用约定: 初值 0xFFFF, 不取反, CRC 高字节先发,
 *     收发两端一致即可互通; 若与既有系统对接需按对方实现调整 MVCS_CRC_INIT。
 */
#include "mvcs_door.h"

/* ======================== CRC-CCITT (x16+x12+x5+1, 0x1021) ======================== */
uint16_t mvcs_crc_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = MVCS_CRC_INIT;
    size_t   i;

    for (i = 0; i < len; i++) {
        uint8_t b = data[i];
        int     bit;

        /* 不反转约定: 字节按 MSB-first 逐位移入 */
        for (bit = 7; bit >= 0; bit--) {
            uint8_t msb = (uint8_t)(crc >> 15) & 1u;
            crc <<= 1;
            if (msb ^ ((b >> bit) & 1u))
                crc ^= MVCS_CRC_POLY;
        }
    }
    return crc;
}

/* ======================== CDR 编解码 (4.7.2.4, 20 字节) ======================== */
void mvcs_cdr_pack(const mvcs_cdr_fields_t *f, uint8_t text[MVCS_CDR_INFO_LEN])
{
    uint8_t i;

    for (i = 0; i < MVCS_CDR_INFO_LEN; i++)
        text[i] = 0;

    /* TEXT[1]: bit0 发车测试执行请求, bit7 发车测试开始请求 */
    if (f->dt_exec_req)
        text[1] |= 0x01u;
    if (f->dt_start_req)
        text[1] |= 0x80u;

    /* TEXT[2]: 发车测试执行项目编号 (BCD, 01H = 门开关测试) */
    text[2] = f->dt_item_no;

    /* TEXT[6] bit0: 时间校验 */
    if (f->time_valid)
        text[6] |= 0x01u;

    /* TEXT[7..12]: 年/月/日/时/分/秒 (BCD 码透传, 不做十进制转换, 由上层按
     * 协议表 4-7 的 "TEXT No.7~12 Time data BCD code" 语义使用) */
    text[7]  = f->year;
    text[8]  = f->month;
    text[9]  = f->date;
    text[10] = f->hour;
    text[11] = f->minute;
    text[12] = f->second;

    /* TEXT[14]: 开门命令位图 bit0..3 = 门1..4 */
    text[14] = f->door_open_cmd & MVCS_DOOR_OPEN_CMD_MASK;
}

void mvcs_cdr_unpack(const uint8_t text[MVCS_CDR_INFO_LEN], mvcs_cdr_fields_t *f)
{
    f->dt_start_req = (text[1] & 0x80u) ? 1 : 0;
    f->dt_exec_req  = (text[1] & 0x01u) ? 1 : 0;
    f->dt_item_no   = text[2];
    f->time_valid   = (text[6] & 0x01u) ? 1 : 0;
    f->year         = text[7];
    f->month        = text[8];
    f->date         = text[9];
    f->hour         = text[10];
    f->minute       = text[11];
    f->second       = text[12];
    f->door_open_cmd = text[14] & MVCS_DOOR_OPEN_CMD_MASK;
}

/* ======================== CD 编解码 (4.7.2.5, 36 字节) ======================== */
static void cd_set_bit(uint8_t text[MVCS_CD_INFO_LEN], uint8_t idx, uint8_t bit,
                       uint8_t val)
{
    if (val)
        text[idx] |= (uint8_t)(1u << bit);
}

void mvcs_cd_pack(const mvcs_cd_fields_t *f, uint8_t text[MVCS_CD_INFO_LEN])
{
    uint8_t i;

    for (i = 0; i < MVCS_CD_INFO_LEN; i++)
        text[i] = 0;

    /* TEXT[1]: bit0 发车测试条件应答, bit6 执行中, bit7 结束 */
    cd_set_bit(text, 1, 0, f->dt_conditions);
    cd_set_bit(text, 1, 6, f->dt_executing);
    cd_set_bit(text, 1, 7, f->dt_ends);

    /* TEXT[2]: 执行中测试项目编号 (BCD, 01H) */
    text[2] = f->dt_item_no;

    /* TEXT[3]: bit0 开门 NG, bit1 开门 OK, bit2 关门 NG, bit3 关门 OK */
    cd_set_bit(text, 3, 0, f->dt_open_ng);
    cd_set_bit(text, 3, 1, f->dt_open_ok);
    cd_set_bit(text, 3, 2, f->dt_close_ng);
    cd_set_bit(text, 3, 3, f->dt_close_ok);

    /* TEXT[4]: bit0 开门时间 / bit1 开门电流 / bit2 开门位置 /
     *          bit3 关门时间 / bit4 关门电流 判定 (1=OK 0=NG) */
    cd_set_bit(text, 4, 0, f->jdg_open_time);
    cd_set_bit(text, 4, 1, f->jdg_open_current);
    cd_set_bit(text, 4, 2, f->jdg_open_position);
    cd_set_bit(text, 4, 3, f->jdg_close_time);
    cd_set_bit(text, 4, 4, f->jdg_close_current);

    /* TEXT[5] 时间测量值 0.1 s/bit, TEXT[6] 平均电流 0.05 A/bit,
     * TEXT[7] 门开位置 5 mm/bit (全关 = 0) */
    text[5]  = f->dt_time_ms;
    text[6]  = f->dt_current_ma;
    text[7]  = f->dt_open_pos_mm;

    /* TEXT[10..11]: 门开启尺寸 1 mm/bit (16 位) */
    text[10] = (uint8_t)(f->door_open_dim_mm & 0xFFu);
    text[11] = (uint8_t)((f->door_open_dim_mm >> 8) & 0xFFu);

    /* TEXT[12..14]: 门开关计数 1 count/bit (低 24 位) */
    text[12] = (uint8_t)(f->door_open_close_count & 0xFFu);
    text[13] = (uint8_t)((f->door_open_close_count >> 8) & 0xFFu);
    text[14] = (uint8_t)((f->door_open_close_count >> 16) & 0xFFu);

    /* TEXT[19]: 门状态 */
    cd_set_bit(text, 19, 0, f->door_fully_open);
    cd_set_bit(text, 19, 1, f->door_fully_closed);
    cd_set_bit(text, 19, 2, f->door_opening);
    cd_set_bit(text, 19, 3, f->door_closing);
    cd_set_bit(text, 19, 4, f->door_close_with_obstruction);
    cd_set_bit(text, 19, 5, f->door_in_initial_operation);

    /* TEXT[20]: 来自车厢的信号 */
    cd_set_bit(text, 20, 0, f->crew_switch_manual_unlocking);
    cd_set_bit(text, 20, 1, f->emergency_handle_manual_unlocking);
    cd_set_bit(text, 20, 5, f->obstruction_buzzer);
    cd_set_bit(text, 20, 6, f->obstruction_recycle_timeout);

    /* TEXT[22]: 内部信息 (bit0 电机驱动 / bit1 过载 / bit2 DCS / bit3 DIS / bit4 DLS) */
    cd_set_bit(text, 22, 0, f->motor_driving_state);
    cd_set_bit(text, 22, 1, f->overload);
    cd_set_bit(text, 22, 2, f->dcs_status);
    cd_set_bit(text, 22, 3, f->dis_status);
    cd_set_bit(text, 22, 4, f->dls_status);

    /* TEXT[23]: 故障信息 1 */
    cd_set_bit(text, 23, 0, f->ipm_failure);
    cd_set_bit(text, 23, 1, f->current_sensor_fault);
    cd_set_bit(text, 23, 2, f->encoder_fault);
    cd_set_bit(text, 23, 3, f->eeprom_memory_fault);
    cd_set_bit(text, 23, 4, f->dls_fault);
    cd_set_bit(text, 23, 5, f->self_diagnosis_fault);
    cd_set_bit(text, 23, 7, f->initial_operation_uncompleted);

    /* TEXT[24]: 故障信息 2 */
    cd_set_bit(text, 24, 0, f->overvoltage);
    cd_set_bit(text, 24, 3, f->door_light_failure);
    cd_set_bit(text, 24, 4, f->door_heavy_failure);
    cd_set_bit(text, 24, 5, f->door_close_timeout);
    cd_set_bit(text, 24, 6, f->door_open_timeout);
    cd_set_bit(text, 24, 7, f->overheat);

    /* TEXT[30]: 重复关闭计数 */
    text[30] = f->reclose_count;

    /* TEXT[32..33] 程序版本, TEXT[34..35] 程序校验和 */
    text[32] = (uint8_t)(f->program_version & 0xFFu);
    text[33] = (uint8_t)((f->program_version >> 8) & 0xFFu);
    text[34] = (uint8_t)(f->program_checksum & 0xFFu);
    text[35] = (uint8_t)((f->program_checksum >> 8) & 0xFFu);
}

void mvcs_cd_unpack(const uint8_t text[MVCS_CD_INFO_LEN], mvcs_cd_fields_t *f)
{
    f->dt_conditions = (text[1] & 0x01u) ? 1 : 0;
    f->dt_executing  = (text[1] & 0x40u) ? 1 : 0;
    f->dt_ends       = (text[1] & 0x80u) ? 1 : 0;
    f->dt_item_no    = text[2];

    f->dt_open_ng    = (text[3] & 0x01u) ? 1 : 0;
    f->dt_open_ok    = (text[3] & 0x02u) ? 1 : 0;
    f->dt_close_ng   = (text[3] & 0x04u) ? 1 : 0;
    f->dt_close_ok   = (text[3] & 0x08u) ? 1 : 0;

    f->jdg_open_time     = (text[4] & 0x01u) ? 1 : 0;
    f->jdg_open_current  = (text[4] & 0x02u) ? 1 : 0;
    f->jdg_open_position = (text[4] & 0x04u) ? 1 : 0;
    f->jdg_close_time    = (text[4] & 0x08u) ? 1 : 0;
    f->jdg_close_current = (text[4] & 0x10u) ? 1 : 0;

    f->dt_time_ms        = text[5];
    f->dt_current_ma     = text[6];
    f->dt_open_pos_mm    = text[7];
    f->door_open_dim_mm  = (uint16_t)(text[10] | ((uint16_t)text[11] << 8));
    f->door_open_close_count =
        (uint32_t)text[12] | ((uint32_t)text[13] << 8) | ((uint32_t)text[14] << 16);

    f->door_fully_open    = (text[19] & 0x01u) ? 1 : 0;
    f->door_fully_closed  = (text[19] & 0x02u) ? 1 : 0;
    f->door_opening       = (text[19] & 0x04u) ? 1 : 0;
    f->door_closing       = (text[19] & 0x08u) ? 1 : 0;
    f->door_close_with_obstruction = (text[19] & 0x10u) ? 1 : 0;
    f->door_in_initial_operation   = (text[19] & 0x20u) ? 1 : 0;

    f->crew_switch_manual_unlocking       = (text[20] & 0x01u) ? 1 : 0;
    f->emergency_handle_manual_unlocking  = (text[20] & 0x02u) ? 1 : 0;
    f->obstruction_buzzer                 = (text[20] & 0x20u) ? 1 : 0;
    f->obstruction_recycle_timeout        = (text[20] & 0x40u) ? 1 : 0;

    f->motor_driving_state = (text[22] & 0x01u) ? 1 : 0;
    f->overload            = (text[22] & 0x02u) ? 1 : 0;
    f->dcs_status          = (text[22] & 0x04u) ? 1 : 0;
    f->dis_status          = (text[22] & 0x08u) ? 1 : 0;
    f->dls_status          = (text[22] & 0x10u) ? 1 : 0;

    f->ipm_failure                    = (text[23] & 0x01u) ? 1 : 0;
    f->current_sensor_fault           = (text[23] & 0x02u) ? 1 : 0;
    f->encoder_fault                  = (text[23] & 0x04u) ? 1 : 0;
    f->eeprom_memory_fault            = (text[23] & 0x08u) ? 1 : 0;
    f->dls_fault                      = (text[23] & 0x10u) ? 1 : 0;
    f->self_diagnosis_fault           = (text[23] & 0x20u) ? 1 : 0;
    f->initial_operation_uncompleted  = (text[23] & 0x80u) ? 1 : 0;

    f->overvoltage          = (text[24] & 0x01u) ? 1 : 0;
    f->door_light_failure   = (text[24] & 0x08u) ? 1 : 0;
    f->door_heavy_failure   = (text[24] & 0x10u) ? 1 : 0;
    f->door_close_timeout   = (text[24] & 0x20u) ? 1 : 0;
    f->door_open_timeout    = (text[24] & 0x40u) ? 1 : 0;
    f->overheat             = (text[24] & 0x80u) ? 1 : 0;

    f->reclose_count   = text[30];
    f->program_version = (uint16_t)(text[32] | ((uint16_t)text[33] << 8));
    f->program_checksum = (uint16_t)(text[34] | ((uint16_t)text[35] << 8));
}

/* ======================== HDLC 帧构建 (4.7.2.2) ========================
 * 输出: [PAD x4][Flag 7EH][stuffed body][Flag 7EH]
 *   body = A1 + A2 + Control + Info + CRC_hi + CRC_lo
 * 位序: 每字节 LSB 先; 连续 5 个 1 后插入 0 (bit-stuffing)
 */
size_t mvcs_frame_build(uint8_t addr1, uint8_t addr2, uint8_t control,
                        const uint8_t *info, size_t info_len,
                        uint8_t *out, size_t out_cap)
{
    uint8_t  body[8 + MVCS_CD_INFO_LEN];
    uint16_t crc;
    size_t   body_len = 2 + MVCS_CONTROL_LEN + info_len;
    size_t   oi = 0;       /* 输出字节索引 */
    unsigned obit = 0;     /* 当前输出字节已写入位数 (0..7) */
    uint8_t  obyte = 0;    /* 当前输出字节 (LSB first) */
    size_t   i, bit;
    int      ones = 0;

#define MVCS_EMIT_BIT(b)                                      \
    do {                                                      \
        if (oi >= out_cap)                                    \
            return 0;               /* 输出溢出保护 */       \
        if (b)                                                \
            obyte |= (uint8_t)(1u << obit);                   \
        else                                                  \
            obyte &= (uint8_t)~(1u << obit);                  \
        if (++obit == 8) {                                    \
            out[oi++] = obyte;                                \
            obyte = 0;                                        \
            obit = 0;                                         \
        }                                                     \
    } while (0)

    if (info_len > MVCS_CD_INFO_LEN || out_cap < MVCS_FRAME_MAX_LEN)
        return 0;

    /* 帧体: A1 + A2 + Control + Info */
    body[0] = addr1;
    body[1] = addr2;
    body[2] = control;
    if (info_len)
        for (i = 0; i < info_len; i++)
            body[3 + i] = info[i];

    /* CRC 计算范围: 帧体 (不含 flag/PAD), CRC 高字节先发 */
    crc = mvcs_crc_ccitt(body, body_len);
    body[body_len]     = (uint8_t)(crc >> 8);
    body[body_len + 1] = (uint8_t)(crc & 0xFFu);
    body_len += MVCS_CRC_LEN;

    /* Opening PAD (4 字节, 供接收端 DPLL 时钟同步; 值可按现场要求配置) */
    for (i = 0; i < MVCS_OPENING_PAD_LEN; i++)
        out[oi++] = 0x00u;

    /* Opening flag */
    out[oi++] = MVCS_FLAG;

    /* 帧体 + bit-stuffing (每字节 LSB 先发, 连续 5 个 1 后插入 0) */
    for (i = 0; i < body_len; i++) {
        for (bit = 0; bit < 8; bit++) {
            uint8_t v = (uint8_t)((body[i] >> bit) & 1u);
            MVCS_EMIT_BIT(v);
            if (v) {
                if (++ones == 5) {      /* 插入填充 0 */
                    MVCS_EMIT_BIT(0);
                    ones = 0;
                }
            } else {
                ones = 0;
            }
        }
    }

    /* 收尾: 写出最后一个未满字节 */
    if (obit > 0)
        out[oi++] = obyte;

    /* Closing flag */
    out[oi++] = MVCS_FLAG;
    return oi;

#undef MVCS_EMIT_BIT
}

/* ======================== 逐字节接收状态机 (4.7.2.3) ========================
 *
 * flag (7EH) 边界处理: 每个位先移入 8 位历史窗口 hist, 仅当 hist != 0x7E 时
 * 才把 8 步前的位延迟提交, 从而保证 flag 的 8 个位不会混入帧体数据;
 * skip 跳过 flag 检测后仍残留在 hist 中的 flag 位。
 * bit-unstuffing 基于提交流计数 (dc_ones): 5 个 1 后的 0 为填充位, 丢弃。
 */
void mvcs_rx_init(mvcs_rx_t *rx)
{
    rx->in_frame  = 0;
    rx->stuffing  = 0;
    rx->ones_run  = 0;
    rx->dc_ones   = 0;
    rx->hist      = 0;
    rx->skip      = 0;
    rx->len       = 0;
    rx->frame_len = 0;
    rx->rx_addr1  = 0;
    rx->rx_addr2  = 0;
    rx->rx_control = 0;
}

int mvcs_rx_feed(mvcs_rx_t *rx, uint8_t byte)
{
    int bit;

    for (bit = 0; bit < 8; bit++) {
        uint8_t v = (uint8_t)((byte >> bit) & 1u);

        rx->hist = (uint8_t)((rx->hist << 1) | v);

        /* 1) flag 检测: 数据经 bit-stuffing 后不可能出现 01111110 */
        if (rx->hist == MVCS_FLAG) {
            if (rx->in_frame && rx->len > 0) {
                /* 帧结束 */
                int ev = MVCS_RX_EVENT_FRAME;

                /* 帧体最小 = A1 + A2 + Control + CRC(2B) = 5 字节 */
                if (rx->len < 2 + MVCS_CONTROL_LEN + MVCS_CRC_LEN) {
                    ev = MVCS_RX_EVENT_CRC_ERR;
                } else {
                    size_t info_len =
                        rx->len - (2 + MVCS_CONTROL_LEN + MVCS_CRC_LEN);

                    if (info_len > MVCS_CD_INFO_LEN) {
                        ev = MVCS_RX_EVENT_CRC_ERR;
                    } else {
                        rx->rx_addr1   = rx->body[0];
                        rx->rx_addr2   = rx->body[1];
                        rx->rx_control = rx->body[2];

                        /* CRC 校验: 对 A1+A2+Control+Info 重算, 与帧内附加的 CRC 比较 */
                        {
                            uint16_t crc = mvcs_crc_ccitt(rx->body,
                                                          rx->len - MVCS_CRC_LEN);
                            uint16_t crc_rx =
                                (uint16_t)((uint16_t)rx->body[rx->len - 2] << 8)
                                | rx->body[rx->len - 1];
                            if (crc != crc_rx)
                                ev = MVCS_RX_EVENT_CRC_ERR;
                        }
                    }
                }

                rx->frame_len = rx->len;    /* 保存帧体长度供上层读取 */
                rx->in_frame = 0;
                rx->len      = 0;
                rx->stuffing = 0;
                rx->dc_ones  = 0;
                rx->ones_run = 0;
                rx->skip     = 7;           /* 跳过 hist 中的 flag 残留位 */
                return ev;
            }
            /* 帧开始 (或连续 flag): 重新进入帧接收状态 */
            rx->in_frame = 1;
            rx->len      = 0;
            rx->stuffing = 0;
            rx->dc_ones  = 0;
            rx->ones_run = 0;
            rx->skip     = 7;
            continue;
        }

        /* 2) abort 检测: 原始位流连续 7 个 1 (flag 只有 6 个 1, 不会误报) */
        if (v)
            rx->ones_run++;
        else
            rx->ones_run = 0;
        if (rx->ones_run >= 7) {
            rx->in_frame = 0;
            rx->len      = 0;
            rx->stuffing = 0;
            rx->dc_ones  = 0;
            rx->ones_run = 0;
            rx->skip     = 0;
            return MVCS_RX_EVENT_ABORT;
        }

        /* 3) 跳过 flag 检测后残留在 hist 中的 flag 位 */
        if (rx->skip > 0) {
            rx->skip--;
            continue;
        }

        /* 4) 延迟提交 8 步前的位 */
        {
            uint8_t b = (uint8_t)((rx->hist >> 7) & 1u);

            if (!rx->in_frame)
                continue;               /* 帧外 (PAD/空闲) 丢弃 */

            /* bit-unstuffing: 去填充后连续 5 个 1 后的 0 是填充位 */
            if (rx->dc_ones == 5 && b == 0) {
                rx->dc_ones = 0;
                continue;
            }
            if (b)
                rx->dc_ones++;
            else
                rx->dc_ones = 0;

            /* 组字节 (LSB first): 每 8 个数据位组成一字节 */
            if (rx->len >= sizeof(rx->body)) {  /* 溢出: 放弃本帧 */
                rx->in_frame = 0;
                rx->len      = 0;
                rx->stuffing = 0;
                rx->dc_ones  = 0;
                rx->ones_run = 0;
                rx->skip     = 0;
                return MVCS_RX_EVENT_ABORT;
            }
            if (rx->stuffing == 0)
                rx->body[rx->len] = 0;      /* 新字节开始 */
            rx->body[rx->len] = (uint8_t)((rx->body[rx->len] >> 1) | (b << 7));
            rx->stuffing = (rx->stuffing + 1) & 7;  /* 位计数 0..7 */
            if (rx->stuffing == 0)
                rx->len++;                  /* 当前字节收集满 */
        }
    }
    return MVCS_RX_EVENT_NONE;
}
