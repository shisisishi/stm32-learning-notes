/**
 * protocol.c —— 二进制帧协议的打包与状态机解析实现
 *
 * 全部是纯 C，不引用任何头文件之外的东西，PC 上可以单独编译测试：
 *   gcc -O2 -std=c99 -Wall -Wextra -o test_protocol test_protocol.c protocol.c
 */
#include "protocol.h"

/* ================================================================== */
/* CRC-8/ATM，多项式 0x07（x^8 + x^2 + x + 1）                        */
/* ================================================================== */

uint8_t Protocol_Crc8(const uint8_t *data, size_t len)
{
    /* CRC 的初值就是 0x00，所以"从头算"等于"从 0x00 接着算" */
    return Protocol_Crc8Continue(0x00u, data, len);
}

uint8_t Protocol_Crc8Continue(uint8_t crc, const uint8_t *data, size_t len)
{
    size_t  i;
    uint8_t bit;

    if (data == NULL) {
        return crc;      /* 没有数据就原样返回，不会把已有结果清零 */
    }

    for (i = 0u; i < len; i++) {
        crc ^= data[i];                 /* 当前字节异或进 CRC 寄存器 */
        for (bit = 0u; bit < 8u; bit++) {
            if ((crc & 0x80u) != 0u) {
                crc = (uint8_t)((crc << 1) ^ 0x07u);   /* 最高位是 1：左移后异或多项式 */
            } else {
                crc = (uint8_t)(crc << 1);             /* 最高位是 0：只左移 */
            }
        }
    }
    return crc;
}

/* ================================================================== */
/* 打包                                                               */
/* ================================================================== */

size_t Protocol_Pack(uint8_t *out, size_t out_size,
                     uint8_t cmd, uint8_t seq,
                     const uint8_t *payload, uint8_t payload_len)
{
    size_t  total;
    size_t  i;

    if (out == NULL) {
        return 0u;
    }
    if (payload_len > PROTO_MAX_PAYLOAD) {
        return 0u;                       /* 载荷超限，宁可不发也不发半帧 */
    }
    if ((payload_len > 0u) && (payload == NULL)) {
        return 0u;
    }

    total = (size_t)PROTO_OVERHEAD + (size_t)payload_len;
    if (out_size < total) {
        return 0u;                       /* 输出缓冲区放不下 */
    }

    /* --- 定长头 --- */
    out[0] = PROTO_SOF1;                 /* 偏移 0 */
    out[1] = PROTO_SOF2;                 /* 偏移 1 */
    out[2] = payload_len;                /* 偏移 2：长度 = 载荷字节数 */
    out[3] = seq;                        /* 偏移 3 */
    out[4] = cmd;                        /* 偏移 4 */

    /* --- 载荷 --- */
    for (i = 0u; i < (size_t)payload_len; i++) {
        out[5u + i] = payload[i];        /* 偏移 5 开始 */
    }

    /*
     * --- 校验 ---
     * 关键：校验范围是 [0, 5+payload_len)，也就是把帧头、长度、序号、命令、
     * 载荷全都算进去。我第一次写的时候只算了载荷，结果长度字段被干扰时
     * 校验照样通过，接收端就按错误的长度去切帧，后面全乱。
     */
    out[5u + (size_t)payload_len] = Protocol_Crc8(out, 5u + (size_t)payload_len);

    return total;
}

/* ================================================================== */
/* 解析器                                                             */
/* ================================================================== */

void Protocol_Init(ProtoParser *p, ProtoFrameCallback cb)
{
    if (p == NULL) {
        return;
    }
    p->crc_error_count = 0u;
    p->frame_count     = 0u;
    p->on_frame        = cb;
    Protocol_Reset(p);
}

void Protocol_Reset(ProtoParser *p)
{
    if (p == NULL) {
        return;
    }
    p->state = PROTO_ST_WAIT_SOF1;
    p->index = 0u;
    p->need  = 0u;
    p->len   = 0u;
    p->seq   = 0u;
    p->cmd   = 0u;
}

const char *Protocol_StateName(ProtoState st)
{
    switch (st) {
    case PROTO_ST_WAIT_SOF1: return "WAIT_SOF1";
    case PROTO_ST_WAIT_SOF2: return "WAIT_SOF2";
    case PROTO_ST_READ_LEN:  return "READ_LEN";
    case PROTO_ST_READ_BODY: return "READ_BODY";
    case PROTO_ST_VERIFY:    return "VERIFY";
    default:                 return "UNKNOWN";
    }
}

ProtoStatus Protocol_ParserFeed(ProtoParser *p, uint8_t byte)
{
    if (p == NULL) {
        return PROTO_ERR_NULL;
    }

    switch (p->state) {

    /* ---------- 状态 1 / 2：找帧头 ---------- */
    case PROTO_ST_WAIT_SOF1:
    case PROTO_ST_WAIT_SOF2:
        if (byte == PROTO_SOF1) {
            /*
             * 收到 0xA5 就认为"可能是帧头"，但要记住"可能"这两个字。
             * 真正确认帧头是在长度和 CRC 都过了之后 —— 这就是为什么
             * 载荷里出现 0xA5 0x5A 也不会把整帧解析坏，最多多走一遍状态机。
             */
            p->state = PROTO_ST_WAIT_SOF2;
            return PROTO_OK;
        }
        if ((p->state == PROTO_ST_WAIT_SOF2) && (byte == PROTO_SOF2)) {
            /* 两个帧头字节都齐了，下一个字节就是长度字段 */
            p->state = PROTO_ST_READ_LEN;
            return PROTO_OK;
        }
        /*
         * 既不是帧头第 1 字节、也不是在等第 2 字节时该来的 0x5A：
         * 丢掉这个字节，回到最初继续找。
         *
         * 说明：WAIT_SOF1 和 WAIT_SOF2 共用一个 case 标号，是为了让
         * "继续找帧头"这段逻辑只写一份。代价是这里三次 return 必须写全 ——
         * 只要漏掉一个，状态机就会永远停在 WAIT_SOF2 再也出不去，
         * 表现是"一帧都收不出来"（我第一次写就漏了 READ_LEN 那个分支）。
         */
        p->state = PROTO_ST_WAIT_SOF1;
        return PROTO_OK;

    /* ---------- 状态 3：读长度 ---------- */
    case PROTO_ST_READ_LEN:
        if (byte > (uint8_t)PROTO_MAX_PAYLOAD) {
            /*
             * 长度不合理，说明这个 0xA5 0x5A 是载荷里的巧合。
             * 关键细节：如果刚收到的这个字节本身就是 0xA5，它有可能才是真帧头的
             * 第一个字节（载荷里出现 ... 0xA5 0x5A 0xA5 0x5A ... 这种情况），
             * 所以要回到 WAIT_SOF2 而不是 WAIT_SOF1，避免漏掉一帧。
             */
            p->state = (byte == PROTO_SOF1) ? PROTO_ST_WAIT_SOF2 : PROTO_ST_WAIT_SOF1;
            return PROTO_ERR_TOO_LONG;
        }
        p->len   = byte;
        p->index = 0u;
        p->need  = (uint8_t)(byte + 2u);   /* 序号(1) + 命令(1) + 载荷(N) */
        /* 载荷长度为 0 时 need == 0，直接跳到校验状态 */
        p->state = (p->need == 0u) ? PROTO_ST_VERIFY : PROTO_ST_READ_BODY;
        return PROTO_OK;

    /* ---------- 状态 4：收序号 + 命令 + 载荷 ---------- */
    case PROTO_ST_READ_BODY:
        p->buf[p->index] = byte;
        p->index++;
        if (p->index >= p->need) {
            p->seq   = p->buf[0];
            p->cmd   = p->buf[1];
            p->state = PROTO_ST_VERIFY;
        }
        return PROTO_OK;

    /* ---------- 状态 5：核对校验 ---------- */
    case PROTO_ST_VERIFY: {
        uint8_t head[3];
        uint8_t calc;

        /*
         * 校验范围要和发送端完全一致：[帧头(2) + 长度(1) + 序号(1) + 命令(1) + 载荷(N)]。
         * 这 5+N 个字节在解析器里不连续（帧头是常量、长度单独存、其余在 buf 里），
         * 所以分两段算：先算帧头 3 个字节，再用中间结果继续算 buf 里的 2+N 个字节。
         */
        head[0] = PROTO_SOF1;
        head[1] = PROTO_SOF2;
        head[2] = p->len;

        calc = Protocol_Crc8(&head[0], 3u);
        calc = Protocol_Crc8Continue(calc, &p->buf[0], (size_t)p->need);

        if (calc == byte) {
            p->frame_count++;
            if (p->on_frame != NULL) {
                /* 载荷从 buf[2] 开始：buf[0] 是序号，buf[1] 是命令 */
                p->on_frame(p->cmd, p->seq, &p->buf[2], p->len);
            }
            p->state = PROTO_ST_WAIT_SOF1;
            return PROTO_FRAME_READY;
        }

        /*
         * 校验失败：整帧丢弃。
         * 也可以只丢第一个字节然后重找帧头，但那样在持续干扰下可能反复重入，
         * 直接回到 WAIT_SOF1 更省事，代价是丢掉紧跟着的那帧开头（下一字节会被当载荷）。
         */
        p->crc_error_count++;
        p->state = PROTO_ST_WAIT_SOF1;
        return PROTO_ERR_CRC;
    }

    default:
        /* 理论上到不了这里（state 在别处都赋了合法值），保底回找帧头状态 */
        p->state = PROTO_ST_WAIT_SOF1;
        return PROTO_ERR_NULL;
    }
}

uint32_t Protocol_ParserFeedBuffer(ProtoParser *p, const uint8_t *data, size_t len)
{
    uint32_t frames = 0u;
    size_t   i;

    if ((p == NULL) || (data == NULL)) {
        return 0u;
    }
    for (i = 0u; i < len; i++) {
        if (Protocol_ParserFeed(p, data[i]) == PROTO_FRAME_READY) {
            frames++;
        }
    }
    return frames;
}
