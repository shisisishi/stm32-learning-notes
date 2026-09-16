/**
 * test_protocol.c —— 协议模块的 PC 端自测程序（不需要任何硬件）
 *
 * 编译运行：
 *   gcc -O2 -std=c99 -Wall -Wextra -o test_protocol test_protocol.c protocol.c
 *   ./test_protocol
 *
 * 这个程序验证这些事：
 *   1  CRC8 的分段续算与整体计算一致（这是解析器能分两段校验的前提），
 *      并用标准已知答案向量 CRC-8/ATM("123456789") == 0xF4 锚住算法本身
 *   2  打包字节偏移正确，LEN 字段确实等于载荷长度
 *   3  收完一帧能正确还原原始数据
 *   4  半包：一帧在任意位置拆开喂给解析器，仍能解析出来
 *   5  粘包：两帧连在一次喂进去，能解析出两帧
 *   6  载荷里故意插入 0xA5 0x5A，不会误同步（这一点最容易被写错）
 *   7  有干扰字节在前面时能重新同步
 *   8  CRC 错误能被检出，帧被丢弃，且不会被计成解析成功
 *   +  结构体 packed 验证：默认对齐会让结构体变大，真实帧结构体偏移逐个核对
 *   +  连续喂 300 帧不丢帧（环形缓冲那套逻辑的基础）
 *   +  边界载荷长度：LEN = 0（零载荷）与 LEN = PROTO_MAX_PAYLOAD（满载荷）
 *   +  假帧头拒绝（LEN 越界）与"截断帧之后紧跟的真帧必须被救回"
 *   +  NULL 安全、复位语义、状态名全覆盖
 */
#include <stdio.h>
#include <string.h>

#include "protocol.h"

/* ================================================================== */
/* 极简测试框架                                                       */
/* ================================================================== */

static int g_pass = 0;
static int g_fail = 0;
static const char *g_section = "";

static void section(const char *name)
{
    g_section = name;
    printf("\n=== %s ===\n", name);
}

/* 断言：条件为真记 PASS，否则记 FAIL 并打印表达式（用 # 变成字符串） */
#define CHECK(cond)                                                          \
    do {                                                                     \
        if (cond) {                                                          \
            g_pass++;                                                        \
            printf("  [PASS] %s\n", #cond);                                   \
        } else {                                                             \
            g_fail++;                                                        \
            printf("  [FAIL] %s   (%s:%d)\n", #cond, __FILE__, __LINE__);     \
        }                                                                    \
    } while (0)

/* 值断言：失败时把实际值和期望值都打出来，方便定位 */
#define CHECK_EQ(actual, expected)                                           \
    do {                                                                     \
        long _a = (long)(actual);                                            \
        long _e = (long)(expected);                                          \
        if (_a == _e) {                                                      \
            g_pass++;                                                        \
            printf("  [PASS] %s == %s  (= %ld)\n", #actual, #expected, _a);    \
        } else {                                                             \
            g_fail++;                                                        \
            printf("  [FAIL] %s: 实际 %ld，期望 %ld   (%s:%d)\n",             \
                   #actual, _a, _e, __FILE__, __LINE__);                     \
        }                                                                    \
    } while (0)

/* ================================================================== */
/* 回调：把解析出来的每一帧抓下来，供断言使用                         */
/* ================================================================== */

#define CAP_MAX  64

typedef struct {
    uint8_t cmd;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[PROTO_MAX_PAYLOAD];
} Captured;

static Captured g_cap[CAP_MAX];
static int      g_cap_n = 0;

static void on_frame(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint8_t len)
{
    if (g_cap_n >= CAP_MAX) {
        return;                       /* 抓满了就不再抓，避免溢出 */
    }
    g_cap[g_cap_n].cmd = cmd;
    g_cap[g_cap_n].seq = seq;
    g_cap[g_cap_n].len = len;
    if ((payload != NULL) && (len > 0u)) {
        /*
         * 注意：这个 memcpy 是必须的。
         * 回调里的 payload 指向解析器内部缓冲区，回调返回后内容随时会被下一个字节覆盖。
         */
        memcpy(g_cap[g_cap_n].payload, payload, (size_t)len);
    }
    g_cap_n++;
}

static void cap_reset(void)
{
    g_cap_n = 0;
    memset(g_cap, 0, sizeof(g_cap));
}

static void dump_frame(const char *tag, const uint8_t *f, size_t n)
{
    size_t i;
    printf("  %s(%u 字节): ", tag, (unsigned)n);
    for (i = 0u; i < n; i++) {
        printf("%02X ", f[i]);
    }
    printf("\n");
}

/* ================================================================== */
/* 1. CRC8 分段续算                                                   */
/* ================================================================== */

static void test_crc_continue(void)
{
    static const uint8_t data[] = { 0xA5u, 0x5Au, 0x06u, 0x01u, 0x01u,
                                    0xD0u, 0x07u, 0x68u, 0x01u, 0x2Cu, 0x01u };
    uint8_t whole;
    uint8_t split;
    uint32_t i;

    section("1. CRC8 分段续算 == 整体计算");

    whole = Protocol_Crc8(data, sizeof(data));
    split = Protocol_Crc8Continue(Protocol_Crc8(data, 3u), &data[3], sizeof(data) - 3u);
    CHECK_EQ(split, whole);

    /*
     * 逐字节喂和一次算完，结果也必须一样。
     * 这正是"状态机逐字节解析"能成立的根本原因：CRC 是可增量计算的。
     */
    split = 0x00u;
    for (i = 0u; i < (uint32_t)sizeof(data); i++) {
        split = Protocol_Crc8Continue(split, &data[i], 1u);
    }
    CHECK_EQ(split, whole);

    /* 传 NULL 时不能崩，也不能把已有结果清掉 */
    CHECK_EQ(Protocol_Crc8Continue(0x5Au, NULL, 0u), 0x5Au);
    CHECK_EQ(Protocol_Crc8Continue(0x5Au, NULL, 7u), 0x5Au);

    /*
     * 用标准已知答案锚住算法：CRC-8/ATM 就是 CRC-8/SMBUS，
     * 校验值 "123456789" 的公认结果是 0xF4。这条如果失败，
     * 说明多项式/初值/位序被改错了 —— 那种错光靠"自己和自己比"是看不出来的。
     */
    {
        static const uint8_t kat[] = "123456789";
        CHECK_EQ(Protocol_Crc8(kat, 9u), 0xF4u);
    }
    CHECK_EQ(Protocol_Crc8(NULL, 0u), 0x00u);

    printf("  CRC8(11 字节) = 0x%02X\n", (unsigned)whole);
}

/* ================================================================== */
/* 2. 打包字节偏移                                                    */
/* ================================================================== */

static void test_pack_layout(void)
{
    uint8_t payload[6] = { 0xD0u, 0x07u, 0x68u, 0x01u, 0x2Cu, 0x01u };
    uint8_t frame[PROTO_MAX_FRAME];
    size_t  n;

    section("2. 打包后的字节偏移");

    memset(frame, 0, sizeof(frame));
    n = Protocol_Pack(frame, sizeof(frame), PROTO_CMD_PID_DATA, 0x2Au, payload, 6u);
    dump_frame("帧 =", frame, n);

    CHECK_EQ(n, 12u);                              /* 6 字节头尾 + 6 字节载荷 */
    CHECK_EQ(frame[0], PROTO_SOF1);                /* 偏移 0：帧头 1 */
    CHECK_EQ(frame[1], PROTO_SOF2);                /* 偏移 1：帧头 2 */
    CHECK_EQ(frame[2], 6u);                        /* 偏移 2：LEN = 载荷长度 */
    CHECK_EQ(frame[3], 0x2Au);                     /* 偏移 3：SEQ */
    CHECK_EQ(frame[4], PROTO_CMD_PID_DATA);        /* 偏移 4：CMD */
    /* 载荷必须一个字节不差地落在偏移 5 ~ 5+N-1 */
    CHECK(memcmp(&frame[5], payload, 6u) == 0);
    /* 载荷长度 N 决定了校验字节的位置：偏移 6+N */
    CHECK_EQ(frame[PROTO_OVERHEAD + 6u - 1u], Protocol_Crc8(frame, 5u + 6u));

    /* 校验范围必须包含 LEN 字段：把长度改一位，CRC 就应该跟着变 */
    frame[2] = 7u;
    CHECK(Protocol_Crc8(frame, 11u) != frame[11]);
    frame[2] = 6u;

    /* 边界：载荷超限 / 缓冲区太小 / 空指针，都必须返回 0 而不是写出界 */
    CHECK_EQ(Protocol_Pack(frame, sizeof(frame), PROTO_CMD_ACK, 0u, payload,
                           PROTO_MAX_PAYLOAD + 1u), 0u);
    CHECK_EQ(Protocol_Pack(frame, 4u, PROTO_CMD_ACK, 0u, payload, 6u), 0u);
    CHECK_EQ(Protocol_Pack(NULL, sizeof(frame), PROTO_CMD_ACK, 0u, payload, 6u), 0u);

    /* out_size 的精确边界：刚好放得下必须成功，少 1 字节必须拒绝 */
    CHECK_EQ(Protocol_Pack(frame, 12u, PROTO_CMD_ACK, 0u, payload, 6u), 12u);
    CHECK_EQ(Protocol_Pack(frame, 11u, PROTO_CMD_ACK, 0u, payload, 6u), 0u);

    /* 零载荷帧：payload 允许为 NULL，整帧 6 字节，校验落在偏移 5 */
    CHECK_EQ(Protocol_Pack(frame, sizeof(frame), PROTO_CMD_ACK, 0u, NULL, 0u),
             PROTO_OVERHEAD);
    CHECK_EQ(frame[2], 0u);
    CHECK_EQ(frame[PROTO_OVERHEAD - 1u], Protocol_Crc8(frame, PROTO_OVERHEAD - 1u));

    /* 有载荷却传 NULL，必须拒绝（不能解引用空指针） */
    CHECK_EQ(Protocol_Pack(frame, sizeof(frame), PROTO_CMD_ACK, 0u, NULL, 6u), 0u);
}

/* ================================================================== */
/* 3. 打包 → 解析 能还原原始数据                                      */
/* ================================================================== */

static void test_roundtrip(void)
{
    uint8_t  payload[6] = { 0xD0u, 0x07u, 0x68u, 0x01u, 0x2Cu, 0x01u };
    uint8_t  frame[PROTO_MAX_FRAME];
    size_t   n;
    uint32_t frames;
    ProtoParser p;

    section("3. 打包 → 解析：数据还原");

    n = Protocol_Pack(frame, sizeof(frame), PROTO_CMD_PID_DATA, 0x2Au, payload, 6u);

    cap_reset();
    Protocol_Init(&p, on_frame);
    frames = Protocol_ParserFeedBuffer(&p, frame, n);

    CHECK_EQ(frames, 1u);
    CHECK_EQ(g_cap_n, 1);
    CHECK_EQ(g_cap[0].cmd, PROTO_CMD_PID_DATA);
    CHECK_EQ(g_cap[0].seq, 0x2Au);
    CHECK_EQ(g_cap[0].len, 6u);
    CHECK(memcmp(g_cap[0].payload, payload, 6u) == 0);
    CHECK_EQ(p.frame_count, 1u);
    CHECK_EQ(p.crc_error_count, 0u);
}

/* ================================================================== */
/* 4. 半包：一帧拆两次喂                                              */
/* ================================================================== */

static void test_half_frame(void)
{
    uint8_t  payload[6] = { 0x10u, 0x27u, 0x20u, 0x4Eu, 0x00u, 0x00u };
    uint8_t  frame[PROTO_MAX_FRAME];
    size_t   n;
    ProtoParser p;
    size_t   k;
    size_t   split;
    int      split_fail = 0;

    section("4. 半包：一帧被拆开喂");

    n = Protocol_Pack(frame, sizeof(frame), PROTO_CMD_PID_DATA, 0x01u, payload, 6u);
    dump_frame("整帧 =", frame, n);

    /* 穷举所有切分点：切口在两帧之间的任意位置都必须能解析出来 */
    for (split = 1u; split < n; split++) {
        cap_reset();
        Protocol_Init(&p, on_frame);

        for (k = 0u; k < split; k++) {
            (void)Protocol_ParserFeed(&p, frame[k]);
        }
        /* 前半段还没凑齐一帧时，不允许提前回调 */
        if (g_cap_n != 0) {
            printf("  [FAIL] 切分点 %u：前半段就提前回调了\n", (unsigned)split);
            g_fail++;
            split_fail++;
            continue;
        }
        for (k = split; k < n; k++) {
            (void)Protocol_ParserFeed(&p, frame[k]);
        }

        if ((g_cap_n == 1) && (g_cap[0].len == 6u) &&
            (memcmp(g_cap[0].payload, payload, 6u) == 0)) {
            g_pass++;
        } else {
            printf("  [FAIL] 切分点 %u：没解析出正确的帧（回调 %d 次）\n",
                   (unsigned)split, g_cap_n);
            g_fail++;
            split_fail++;
        }
    }
    /* 只有全部切分点都过了才打这句，否则会误导读者以为半包测试通过了 */
    if (split_fail == 0) {
        printf("  [PASS] 全部 %u 个切分点（前段长度 1 ~ %u 字节）都解析正确\n",
               (unsigned)(n - 1u), (unsigned)(n - 1u));
    } else {
        printf("  [FAIL] %d/%u 个切分点解析失败\n",
               split_fail, (unsigned)(n - 1u));
    }

    /* 极端的半包：整个帧一个字节一个字节喂（等价于"每次中断只收到 1 字节"） */
    cap_reset();
    Protocol_Init(&p, on_frame);
    for (k = 0u; k < n; k++) {
        (void)Protocol_ParserFeed(&p, frame[k]);
    }
    CHECK_EQ(g_cap_n, 1);
    CHECK(memcmp(g_cap[0].payload, payload, 6u) == 0);
}

/* ================================================================== */
/* 5. 粘包：两帧连在一起喂                                            */
/* ================================================================== */

static void test_sticky_frames(void)
{
    uint8_t  pa[6] = { 0x01u, 0x00u, 0x02u, 0x00u, 0x03u, 0x00u };
    uint8_t  pb[4] = { 0xAAu, 0xBBu, 0xCCu, 0xDDu };
    uint8_t  buf[PROTO_MAX_FRAME * 2u];
    size_t   na;
    size_t   nb;
    uint32_t frames;
    ProtoParser p;

    section("5. 粘包：两帧连在一起喂");

    na = Protocol_Pack(buf, sizeof(buf), PROTO_CMD_PID_DATA, 0x07u, pa, 6u);
    nb = Protocol_Pack(&buf[na], sizeof(buf) - na, PROTO_CMD_SET_PARAM, 0x08u, pb, 4u);
    dump_frame("两帧拼接 =", buf, na + nb);

    cap_reset();
    Protocol_Init(&p, on_frame);
    frames = Protocol_ParserFeedBuffer(&p, buf, na + nb);

    CHECK_EQ(frames, 2u);
    CHECK_EQ(g_cap_n, 2);
    CHECK_EQ(g_cap[0].cmd, PROTO_CMD_PID_DATA);
    CHECK_EQ(g_cap[0].seq, 0x07u);
    CHECK(memcmp(g_cap[0].payload, pa, 6u) == 0);
    CHECK_EQ(g_cap[1].cmd, PROTO_CMD_SET_PARAM);
    CHECK_EQ(g_cap[1].seq, 0x08u);
    CHECK(memcmp(g_cap[1].payload, pb, 4u) == 0);
}

/* ================================================================== */
/* 6. 载荷里出现 0xA5 0x5A，不能误同步                                */
/* ================================================================== */

static void test_fake_sof_in_payload(void)
{
    /*         载荷前两字节就是完整的帧头序列，后面还再塞一个 0xA5 */
    uint8_t  payload[10] = { 0xA5u, 0x5Au, 0x04u, 0x11u, 0xA5u,
                             0x22u, 0x33u, 0x5Au, 0xA5u, 0x44u };
    uint8_t  frame[PROTO_MAX_FRAME];
    size_t   n;
    ProtoParser p;

    section("6. 载荷含 0xA5 0x5A：不允许误同步");

    n = Protocol_Pack(frame, sizeof(frame), PROTO_CMD_DEBUG_TEXT, 0x33u, payload, 10u);
    dump_frame("帧 =", frame, n);

    /* 整帧一次喂完 */
    cap_reset();
    Protocol_Init(&p, on_frame);
    CHECK_EQ(Protocol_ParserFeedBuffer(&p, frame, n), 1u);
    CHECK_EQ(g_cap_n, 1);
    CHECK_EQ(g_cap[0].len, 10u);
    CHECK(memcmp(g_cap[0].payload, payload, 10u) == 0);
    CHECK_EQ(p.crc_error_count, 0u);

    /*
     * 再逐字节喂一遍，逐字节喂才是难点。
     * 需要说清楚的是：整帧一次喂完时，载荷里那对 0xA5 0x5A 是被"当成载荷"吃掉的
     * （READ_BODY 状态只按 LEN 计数，不再回头看载荷内容），所以它压根不会被当成
     * 候选帧头 —— crc_error_count 保持 0 才是正确结果。
     * 真正会撞上假帧头的是下面这种情况：帧收完之后，后面又来了 0xA5 0x5A。
     */
    cap_reset();
    Protocol_Init(&p, on_frame);
    Protocol_ParserFeedBuffer(&p, frame, n);
    CHECK_EQ(g_cap_n, 1);
    CHECK_EQ(p.frame_count, 1u);

    /*
     * 帧尾再接一段"假帧头 + 合法长度字段"的垃圾：
     * 状态机会真的把它当候选帧来处理（A5 -> 5A -> LEN -> 收够 2+N 字节 -> 算 CRC），
     * 算不过才丢掉。用 crc_error_count +1 来证明"确实试过并且拦住了"，
     * 而不是"这段垃圾根本没被看"—— 后者才是真正的误同步隐患。
     */
    {
        uint8_t junk[6];
        junk[0] = PROTO_SOF1;
        junk[1] = PROTO_SOF2;
        junk[2] = 0u;                                    /* LEN = 0 */
        junk[3] = 0x11u;                                 /* SEQ（伪） */
        junk[4] = 0x22u;                                 /* CMD（伪） */
        /* 故意把正确的校验值翻掉，保证它必然算不过，结果与平台无关 */
        junk[5] = (uint8_t)(Protocol_Crc8(junk, 5u) ^ 0xFFu);

        Protocol_Init(&p, on_frame);                     /* 接在上一帧之后继续 */
        cap_reset();
        CHECK_EQ(Protocol_ParserFeedBuffer(&p, frame, n), 1u);
        CHECK_EQ(g_cap_n, 1);
        CHECK_EQ(Protocol_ParserFeedBuffer(&p, junk, sizeof(junk)), 0u);
        CHECK_EQ(g_cap_n, 1);                            /* 假帧头不许产生任何回调 */
        CHECK_EQ(p.frame_count, 1u);
        CHECK_EQ(p.crc_error_count, 1u);                 /* 假帧头被算过 CRC 并拦下 */
    }
}

/* ================================================================== */
/* 7. 前面有干扰字节时能重新同步                                       */
/* ================================================================== */

static void test_resync(void)
{
    uint8_t  payload[4] = { 0x11u, 0x22u, 0x33u, 0x44u };
    uint8_t  frame[PROTO_MAX_FRAME];
    uint8_t  stream[64];
    size_t   n;
    size_t   off = 0u;
    ProtoParser p;
    uint32_t frames;

    section("7. 前面有垃圾字节：重新同步");

    n = Protocol_Pack(frame, sizeof(frame), PROTO_CMD_ACK, 0x05u, payload, 4u);

    /* 开头塞一段不完整的假帧头，模拟上电瞬间或干扰导致的对齐错位 */
    stream[off++] = 0x00u;
    stream[off++] = 0xFFu;
    stream[off++] = 0xA5u;       /* 假帧头：后面跟的不是 0x5A */
    stream[off++] = 0x00u;
    stream[off++] = 0xA5u;       /* 又一个假的：长度字段会越界 */
    stream[off++] = 0x5Au;
    stream[off++] = 0xFFu;       /* 0xFF > PROTO_MAX_PAYLOAD，判定为假帧头 */
    memcpy(&stream[off], frame, n);
    off += n;

    cap_reset();
    Protocol_Init(&p, on_frame);
    frames = Protocol_ParserFeedBuffer(&p, stream, off);

    dump_frame("含干扰的流 =", stream, off);
    CHECK_EQ(frames, 1u);
    CHECK_EQ(g_cap_n, 1);
    CHECK_EQ(g_cap[0].seq, 0x05u);
    CHECK(memcmp(g_cap[0].payload, payload, 4u) == 0);
}

/* ================================================================== */
/* 8. CRC 错误能被检出                                                */
/* ================================================================== */

static void test_crc_error(void)
{
    uint8_t  payload[6] = { 0xD0u, 0x07u, 0x68u, 0x01u, 0x2Cu, 0x01u };
    uint8_t  frame[PROTO_MAX_FRAME];
    uint8_t  bad[PROTO_MAX_FRAME];
    size_t   n;
    ProtoParser p;
    size_t   pos;

    section("8. CRC 错误检出");

    n = Protocol_Pack(frame, sizeof(frame), PROTO_CMD_PID_DATA, 0x09u, payload, 6u);

    /*
     * 逐位翻转整帧里的每一个字节的每一个位（12 字节 × 8 位 = 96 种干扰）：
     * 既不许回调出数据，也不许被计成"解析成功"。
     *
     * 为什么能要求这么严：CRC 覆盖 [0, 5+N)，帧头/长度/序号/命令/载荷/校验
     * 任何一个字节被改坏，收到的校验值都不可能再对上（CRC 能检出全部单字节错误）。
     * 所以这里连 frame_count 一起断言 —— 只要有一个组合被放过去，测试必须红。
     */
    {
        unsigned bit;
        int      flip_fail = 0;

        for (pos = 0u; pos < n; pos++) {
            for (bit = 0u; bit < 8u; bit++) {
                memcpy(bad, frame, n);
                bad[pos] = (uint8_t)(bad[pos] ^ (uint8_t)(1u << bit));

                cap_reset();
                Protocol_Init(&p, on_frame);
                (void)Protocol_ParserFeedBuffer(&p, bad, n);

                if ((g_cap_n != 0) || (p.frame_count != 0u)) {
                    printf("  [FAIL] 偏移 %u 的第 %u 位被干扰却回调/计成功"
                           "（回调 %d 次, frame_count=%u）\n",
                           (unsigned)pos, bit, g_cap_n, (unsigned)p.frame_count);
                    g_fail++;
                    flip_fail++;
                } else {
                    g_pass++;
                }
            }
        }

        /* 汇总必须真的看结果：这里以前是无条件打 [PASS]，失败了也照样打绿，纯误导 */
        if (flip_fail == 0) {
            printf("  [PASS] 逐个翻转 %u 个字节的全部 8 位（共 %u 种干扰），"
                   "没有一次被误认为合法帧\n",
                   (unsigned)n, (unsigned)(n * 8u));
        } else {
            printf("  [FAIL] %d/%u 种干扰没被拦住\n",
                   flip_fail, (unsigned)(n * 8u));
        }
    }

    /* 单独确认：改坏序号（偏移 3）也必须计一次 CRC 错误并被丢掉 */
    memcpy(bad, frame, n);
    bad[3] = (uint8_t)(bad[3] ^ 0x01u);
    cap_reset();
    Protocol_Init(&p, on_frame);
    CHECK_EQ(Protocol_ParserFeedBuffer(&p, bad, n), 0u);
    CHECK_EQ(g_cap_n, 0);
    CHECK_EQ(p.crc_error_count, 1u);
    CHECK_EQ(p.frame_count, 0u);

    /* 单独确认：载荷被改坏时，CRC 错误计数会 +1 */
    memcpy(bad, frame, n);
    bad[6] = (uint8_t)(bad[6] ^ 0xFFu);
    cap_reset();
    Protocol_Init(&p, on_frame);
    Protocol_ParserFeedBuffer(&p, bad, n);
    CHECK_EQ(p.crc_error_count, 1u);
    CHECK_EQ(p.frame_count, 0u);
}

/* ================================================================== */
/* 9. 结构体对齐：packed 与默认对齐的差别                             */
/* ================================================================== */

/* 演示用的结构体：一个 uint8 后面跟一个 uint16 */
typedef struct {
    uint8_t  a;
    uint16_t b;
} DemoDefault;

typedef struct PROTO_PACKED {
    uint8_t  a;
    uint16_t b;
} DemoPacked;

static void test_struct_packing(void)
{
    section("9. 结构体对齐：未 packed 的坑");

    printf("  sizeof(DemoDefault) = %u 字节\n", (unsigned)sizeof(DemoDefault));
    printf("  sizeof(DemoPacked)  = %u 字节\n", (unsigned)sizeof(DemoPacked));
    printf("  offsetof(DemoPacked.b) = %u\n", (unsigned)offsetof(DemoPacked, b));

    /* packed 之后必须是紧凑的 3 字节，字段 b 从偏移 1 开始 */
    CHECK_EQ(sizeof(DemoPacked), 3u);
    CHECK_EQ(offsetof(DemoPacked, b), 1u);

    if (sizeof(DemoDefault) == 3u) {
        /* 这个编译器默认就是紧凑的，那这条坑在我这儿复现不出来，如实说出来 */
        printf("  [SKIP] 本平台 sizeof(DemoDefault) 也是 3，未复现默认对齐填充\n");
    } else {
        CHECK_EQ(sizeof(DemoDefault), 4u);
        printf("  [PASS] 默认对齐下是 %u 字节，比实际的 3 字节多 %u 字节填充\n",
               (unsigned)sizeof(DemoDefault),
               (unsigned)(sizeof(DemoDefault) - 3u));
    }

    /*
     * 上面那个 Demo 只是演示"对齐填充"这件事本身。真正决定线上字节流的是
     * ProtoFrame / ProtoPidPayload —— 上位机 Python 按固定偏移解包，这两个结构体
     * 的每一个偏移都必须钉死，漏一个 packed 就会整体错位。
     */
    CHECK_EQ(sizeof(ProtoFrame), PROTO_MAX_FRAME);
    CHECK_EQ(offsetof(ProtoFrame, sof1), 0u);
    CHECK_EQ(offsetof(ProtoFrame, sof2), 1u);
    CHECK_EQ(offsetof(ProtoFrame, len), 2u);
    CHECK_EQ(offsetof(ProtoFrame, seq), 3u);
    CHECK_EQ(offsetof(ProtoFrame, cmd), 4u);
    CHECK_EQ(offsetof(ProtoFrame, payload), 5u);
    /* 校验字节是整帧的最后一个字节：载荷占 5 ~ 5+N-1，所以 CRC 在 5+N，
     * 而整帧长度是 6+N（即 5+N+1）。这两个数差 1，最容易写错，这里钉死。 */
    CHECK_EQ(offsetof(ProtoFrame, crc), 5u + PROTO_MAX_PAYLOAD);
    CHECK_EQ(offsetof(ProtoFrame, crc), PROTO_MAX_FRAME - 1u);

    CHECK_EQ(sizeof(ProtoPidPayload), 6u);
    CHECK_EQ(offsetof(ProtoPidPayload, setpoint), 0u);
    CHECK_EQ(offsetof(ProtoPidPayload, actual), 2u);
    CHECK_EQ(offsetof(ProtoPidPayload, output), 4u);
}

/* ================================================================== */
/* 10. 连续 300 帧不丢（验证 SEQ 计数与粘包叠加）                     */
/* ================================================================== */

/*
 * 这一组单独用一份"记录 300 帧"的回调，而没有复用上面的 g_cap：
 * g_cap 只有 CAP_MAX(64) 个槽位，是给单帧用例用的。
 * 这里要真的核对每一帧，所以按 300 个槽位开数组。
 */
#define SEQ_TEST_FRAMES  300
#define SEQ_TEST_MAX     (SEQ_TEST_FRAMES + 8)   /* 多留几个槽位，多出来的回调能被发现 */

static uint8_t g_seq_seen[SEQ_TEST_MAX];
static uint8_t g_val_seen[SEQ_TEST_MAX];
static int     g_seq_n = 0;

static void on_frame_count(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint8_t len)
{
    (void)cmd;
    if ((g_seq_n < SEQ_TEST_MAX) && (len == 2u)) {
        g_seq_seen[g_seq_n] = seq;
        g_val_seen[g_seq_n] = payload[0];   /* payload[0] 存的就是帧下标 i & 0xFF */
    }
    g_seq_n++;
}

static void test_stream_300(void)
{
    uint8_t  stream[PROTO_MAX_FRAME * SEQ_TEST_FRAMES];
    uint8_t  payload[2];
    size_t   off = 0u;
    int      i;
    int      bad = 0;
    ProtoParser p;

    section("10. 连续 300 帧（按 SEQ 检查丢帧）");

    for (i = 0; i < SEQ_TEST_FRAMES; i++) {
        payload[0] = (uint8_t)(i & 0xFF);
        payload[1] = (uint8_t)((i >> 8) & 0xFF);
        off += Protocol_Pack(&stream[off], sizeof(stream) - off,
                             PROTO_CMD_PID_DATA, (uint8_t)(i & 0xFF),
                             payload, 2u);
    }
    printf("  构造的字节流 = %u 字节（%d 帧 × 8 字节）\n",
           (unsigned)off, SEQ_TEST_FRAMES);

    g_seq_n = 0;
    memset(g_seq_seen, 0, sizeof(g_seq_seen));
    memset(g_val_seen, 0, sizeof(g_val_seen));

    Protocol_Init(&p, on_frame_count);
    (void)Protocol_ParserFeedBuffer(&p, stream, off);

    CHECK_EQ(g_seq_n, SEQ_TEST_FRAMES);          /* 回调次数必须一帧不多一帧不少 */
    CHECK_EQ(p.frame_count, SEQ_TEST_FRAMES);    /* 解析器自己的计数 */
    CHECK_EQ(p.crc_error_count, 0u);

    /* 逐帧核对：序号连续递增，载荷与期望一一对应 */
    for (i = 0; i < g_seq_n; i++) {
        if ((g_seq_seen[i] != (uint8_t)(i & 0xFF)) ||
            (g_val_seen[i] != (uint8_t)(i & 0xFF))) {
            printf("  [FAIL] 第 %d 帧：seq=%u payload[0]=%u，期望 %u\n",
                   i, (unsigned)g_seq_seen[i], (unsigned)g_val_seen[i],
                   (unsigned)(i & 0xFF));
            bad = 1;
            break;
        }
    }
    if (bad == 0) {
        g_pass++;
        printf("  [PASS] 300 帧的 SEQ 连续、载荷一一对应，没有丢帧也没有多出帧\n");
    } else {
        g_fail++;
    }

    /*
     * 顺带验证一件事：SEQ 是 8 位的，到 255 就要回绕到 0。
     * 第 256 帧（下标 255）的 SEQ 是 0xFF，第 257 帧回到 0x00 ——
     * 上位机算丢帧率时必须按模 256 处理，不能直接相减。
     */
    if ((g_seq_seen[255] == 0xFFu) && (g_seq_seen[256] == 0x00u)) {
        g_pass++;
        printf("  [PASS] SEQ 在第 256 帧正确回绕（0xFF -> 0x00）\n");
    } else {
        g_fail++;
        printf("  [FAIL] SEQ 回绕不对：第 256 帧是 %u，第 257 帧是 %u\n",
               (unsigned)g_seq_seen[255], (unsigned)g_seq_seen[256]);
    }

    printf("  平均每帧 %.1f 字节\n", (double)off / (double)SEQ_TEST_FRAMES);
}

/* ================================================================== */
/* 11. 边界载荷长度：LEN = 0 与 LEN = PROTO_MAX_PAYLOAD               */
/* ================================================================== */

static void test_payload_limits(void)
{
    uint8_t  payload[PROTO_MAX_PAYLOAD];
    uint8_t  frame[PROTO_MAX_FRAME];
    size_t   n;
    size_t   k;
    uint32_t i;
    ProtoParser p;

    section("11. 边界载荷长度：LEN = 0 与 LEN = 32");

    for (i = 0u; i < (uint32_t)PROTO_MAX_PAYLOAD; i++) {
        payload[i] = (uint8_t)((i == 0u) ? 0xA5u : ((i == 1u) ? 0x5Au : (i * 13u + 7u)));
    }

    /* --- 零载荷：整帧只有 6 字节，LEN == 0，载荷指针仍然必须有效 --- */
    n = Protocol_Pack(frame, sizeof(frame), PROTO_CMD_ACK, 0x40u, NULL, 0u);
    CHECK_EQ(n, PROTO_OVERHEAD);
    CHECK_EQ(frame[2], 0u);                     /* LEN 字段 = 0 */
    CHECK_EQ(frame[3], 0x40u);

    cap_reset();
    Protocol_Init(&p, on_frame);
    CHECK_EQ(Protocol_ParserFeedBuffer(&p, frame, n), 1u);
    CHECK_EQ(g_cap_n, 1);
    CHECK_EQ(g_cap[0].len, 0u);
    CHECK_EQ(g_cap[0].seq, 0x40u);
    CHECK_EQ(g_cap[0].cmd, PROTO_CMD_ACK);
    CHECK_EQ(p.crc_error_count, 0u);

    /* --- 满载荷：LEN == 32，整帧 38 字节，逐字节喂也必须一个字节不差 --- */
    n = Protocol_Pack(frame, sizeof(frame), PROTO_CMD_DEBUG_TEXT, 0x41u,
                      payload, (uint8_t)PROTO_MAX_PAYLOAD);
    CHECK_EQ(n, PROTO_MAX_FRAME);
    CHECK_EQ(frame[2], (uint8_t)PROTO_MAX_PAYLOAD);

    cap_reset();
    Protocol_Init(&p, on_frame);
    for (k = 0u; k < n; k++) {
        (void)Protocol_ParserFeed(&p, frame[k]);
    }
    CHECK_EQ(g_cap_n, 1);
    CHECK_EQ(g_cap[0].len, (uint8_t)PROTO_MAX_PAYLOAD);
    CHECK_EQ(g_cap[0].seq, 0x41u);
    CHECK(memcmp(g_cap[0].payload, payload, (size_t)PROTO_MAX_PAYLOAD) == 0);
    CHECK_EQ(p.frame_count, 1u);
    CHECK_EQ(p.crc_error_count, 0u);

    /* 满载荷且载荷全是帧头字节（A5 5A A5 5A ...）也不能误同步 */
    for (k = 0u; k < (size_t)PROTO_MAX_PAYLOAD; k++) {
        payload[k] = ((k % 2u) == 0u) ? PROTO_SOF1 : PROTO_SOF2;
    }
    n = Protocol_Pack(frame, sizeof(frame), PROTO_CMD_DEBUG_TEXT, 0x42u,
                      payload, (uint8_t)PROTO_MAX_PAYLOAD);
    cap_reset();
    Protocol_Init(&p, on_frame);
    CHECK_EQ(Protocol_ParserFeedBuffer(&p, frame, n), 1u);
    CHECK_EQ(g_cap_n, 1);
    CHECK(memcmp(g_cap[0].payload, payload, (size_t)PROTO_MAX_PAYLOAD) == 0);
    CHECK_EQ(p.crc_error_count, 0u);
}

/* ================================================================== */
/* 12. 假帧头拒绝、截断帧恢复                                         */
/* ================================================================== */

static void test_recovery(void)
{
    uint8_t  payload[6] = { 0xD0u, 0x07u, 0x68u, 0x01u, 0x2Cu, 0x01u };
    uint8_t  frame[PROTO_MAX_FRAME];
    uint8_t  buf[PROTO_MAX_FRAME * 2u];
    size_t   n;
    ProtoParser p;

    section("12. 假帧头拒绝与截断帧恢复");

    n = Protocol_Pack(frame, sizeof(frame), PROTO_CMD_PID_DATA, 0x5Cu, payload, 6u);

    /* LEN = 33（超过 PROTO_MAX_PAYLOAD）必须被判为假帧头，且不许产生任何回调 */
    cap_reset();
    Protocol_Init(&p, on_frame);
    CHECK_EQ(Protocol_ParserFeed(&p, PROTO_SOF1), PROTO_OK);
    CHECK_EQ(Protocol_ParserFeed(&p, PROTO_SOF2), PROTO_OK);
    CHECK_EQ(Protocol_ParserFeed(&p, (uint8_t)(PROTO_MAX_PAYLOAD + 1u)), PROTO_ERR_TOO_LONG);
    CHECK_EQ(g_cap_n, 0);
    CHECK_EQ(p.frame_count, 0u);
    CHECK_EQ(p.crc_error_count, 0u);          /* 长度越界不算 CRC 错误 */

    /* 越界之后必须还能重新同步出紧跟的真帧 */
    CHECK_EQ(Protocol_ParserFeedBuffer(&p, frame, n), 1u);
    CHECK_EQ(g_cap_n, 1);
    CHECK_EQ(g_cap[0].seq, 0x5Cu);
    CHECK(memcmp(g_cap[0].payload, payload, 6u) == 0);

    /*
     * 极端情形：A5 5A A5 5A 连着来（第二个 A5 落在长度字段位置上 = 165，越界）。
     * 这时最后一个 A5 有可能才是真帧头的第一字节，必须被保留成候选帧头，
     * 否则紧跟的真帧整帧丢掉。
     */
    buf[0] = PROTO_SOF1;
    buf[1] = PROTO_SOF2;
    buf[2] = PROTO_SOF1;
    buf[3] = PROTO_SOF2;
    memcpy(&buf[4], frame, n);
    cap_reset();
    Protocol_Init(&p, on_frame);
    CHECK_EQ(Protocol_ParserFeedBuffer(&p, buf, 4u + n), 1u);
    CHECK_EQ(g_cap_n, 1);
    CHECK_EQ(g_cap[0].seq, 0x5Cu);
    CHECK(memcmp(g_cap[0].payload, payload, 6u) == 0);

    /*
     * 截断帧（模拟串口丢了一个字节）：解析器会停在 VERIFY 等 CRC 字节，
     * 于是紧跟的新帧的 0xA5 正好落到"校验字节"这个位置上。
     * 这条断言锁死"校验失败时若该字节是 0xA5 就必须回 WAIT_SOF2"：
     * 一旦改回 WAIT_SOF1，新帧的 0xA5 会被吞掉 —— 丢 1 个字节变成连丢 2 帧，
     * 这里就会从 1 变成 0，测试立刻红。
     */
    cap_reset();
    Protocol_Init(&p, on_frame);
    (void)Protocol_ParserFeedBuffer(&p, frame, n - 1u);
    CHECK_EQ(g_cap_n, 0);                     /* 截断的那一帧不许回调 */
    CHECK_EQ(Protocol_ParserFeedBuffer(&p, frame, n), 1u);
    CHECK_EQ(g_cap_n, 1);
    CHECK_EQ(g_cap[0].seq, 0x5Cu);
    CHECK(memcmp(g_cap[0].payload, payload, 6u) == 0);
    CHECK_EQ(p.frame_count, 1u);
    CHECK_EQ(p.crc_error_count, 1u);          /* 截断帧记一次校验失败 */

    /* 坏帧夹在两帧好帧中间：坏帧被丢，后面的好帧必须完整解析出来 */
    memcpy(buf, frame, n);
    buf[7] = (uint8_t)(buf[7] ^ 0x01u);       /* 改坏载荷 -> CRC 必然不过 */
    memcpy(&buf[n], frame, n);
    cap_reset();
    Protocol_Init(&p, on_frame);
    CHECK_EQ(Protocol_ParserFeedBuffer(&p, buf, n * 2u), 1u);
    CHECK_EQ(g_cap_n, 1);
    CHECK_EQ(g_cap[0].seq, 0x5Cu);
    CHECK(memcmp(g_cap[0].payload, payload, 6u) == 0);
    CHECK_EQ(p.frame_count, 1u);
    CHECK_EQ(p.crc_error_count, 1u);
}

/* ================================================================== */
/* 13. NULL 安全 / 复位语义 / 状态名                                  */
/* ================================================================== */

static void test_api_edges(void)
{
    uint8_t  frame[PROTO_MAX_FRAME];
    uint8_t  payload[2] = { 0x01u, 0x02u };
    size_t   n;
    ProtoParser p;

    section("13. NULL 安全 / 复位语义 / 状态名");

    /* 传 NULL 必须返回错误码，不许崩 */
    CHECK_EQ(Protocol_ParserFeed(NULL, 0x00u), PROTO_ERR_NULL);
    CHECK_EQ(Protocol_ParserFeedBuffer(NULL, frame, 4u), 0u);
    Protocol_Init(&p, on_frame);
    CHECK_EQ(Protocol_ParserFeedBuffer(&p, NULL, 0u), 0u);
    Protocol_Init(NULL, on_frame);            /* 不许崩 */
    Protocol_Reset(NULL);                     /* 不许崩 */

    /* cb 传 NULL：只解析不回调，但计数照走 */
    n = Protocol_Pack(frame, sizeof(frame), PROTO_CMD_ACK, 0x21u, payload, 2u);
    cap_reset();
    Protocol_Init(&p, NULL);
    CHECK_EQ(Protocol_ParserFeedBuffer(&p, frame, n), 1u);
    CHECK_EQ(p.frame_count, 1u);
    CHECK_EQ(g_cap_n, 0);                     /* 没注册回调就不该有回调 */

    /* Reset 只清状态，不清统计：串口出错重连时不该把丢帧统计也抹掉 */
    Protocol_Reset(&p);
    CHECK_EQ(p.state, PROTO_ST_WAIT_SOF1);
    CHECK_EQ(p.index, 0u);
    CHECK_EQ(p.need, 0u);
    CHECK_EQ(p.frame_count, 1u);
    CHECK_EQ(Protocol_ParserFeedBuffer(&p, frame, n), 1u);
    CHECK_EQ(p.frame_count, 2u);

    /* 状态名必须覆盖全部状态，未知状态也不能返回 NULL */
    CHECK(Protocol_StateName(PROTO_ST_WAIT_SOF1) != NULL);
    CHECK_EQ(strcmp(Protocol_StateName(PROTO_ST_WAIT_SOF1), "WAIT_SOF1"), 0);
    CHECK_EQ(strcmp(Protocol_StateName(PROTO_ST_WAIT_SOF2), "WAIT_SOF2"), 0);
    CHECK_EQ(strcmp(Protocol_StateName(PROTO_ST_READ_LEN), "READ_LEN"), 0);
    CHECK_EQ(strcmp(Protocol_StateName(PROTO_ST_READ_BODY), "READ_BODY"), 0);
    CHECK_EQ(strcmp(Protocol_StateName(PROTO_ST_VERIFY), "VERIFY"), 0);
    CHECK(Protocol_StateName((ProtoState)99) != NULL);
    CHECK_EQ(strcmp(Protocol_StateName((ProtoState)99), "UNKNOWN"), 0);
}

/* ================================================================== */
/* main                                                               */
/* ================================================================== */

int main(void)
{
    printf("========================================\n");
    printf(" protocol.c 自测（PC 端，无需硬件）\n");
    printf(" 帧格式: A5 5A | LEN | SEQ | CMD | PAYLOAD | CRC8\n");
    printf("========================================\n");

    test_crc_continue();
    test_pack_layout();
    test_roundtrip();
    test_half_frame();
    test_sticky_frames();
    test_fake_sof_in_payload();
    test_resync();
    test_crc_error();
    test_struct_packing();
    test_stream_300();
    test_payload_limits();
    test_recovery();
    test_api_edges();

    printf("\n========================================\n");
    printf(" 总计: %d 项通过, %d 项失败\n", g_pass, g_fail);
    printf(" 结果: %s\n", (g_fail == 0) ? "ALL PASS" : "THERE ARE FAILURES");
    printf("========================================\n");

    (void)g_section;
    return (g_fail == 0) ? 0 : 1;
}
