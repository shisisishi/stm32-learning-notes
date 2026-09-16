/**
 * protocol.h —— 机器人上下位机二进制帧协议（纯 C，与硬件无关）
 *
 * 设计目标：
 *   1. 不依赖任何 STM32 / HAL 头文件，PC 上能单独编译、单独测试；
 *   2. 定长头 + 变长载荷，接收端能靠"长度字段"知道一帧在哪结束；
 *   3. 校验范围覆盖帧头与长度字段，长度被干扰时不会误判成合法帧；
 *   4. 逐字节喂给状态机解析，天然处理"半包"和"粘包"。
 *
 * 帧格式（小端，共 6 + N 字节）：
 *
 *   偏移  0    1    2     3     4     5 .. 5+N-1   6+N
 *        +----+----+-----+-----+-----+-----------+------+
 *        | A5 | 5A | LEN | SEQ | CMD |  PAYLOAD  | CRC8 |
 *        +----+----+-----+-----+-----+-----------+------+
 *          SOF(2B)  1B    1B    1B      N 字节       1B
 *
 *   LEN    = 载荷字节数 N（不含帧头，也不含 CRC）
 *   SEQ    = 帧序号，0~255 循环，用来在上位机上算丢帧率
 *   CMD    = 命令字，区分这一帧的数据是什么（设定值 / 实际值 / 调试文本……）
 *   CRC8   = 从偏移 0 到偏移 5+N-1 的 CRC-8/ATM，多项式 0x07，初值 0x00
 */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* 常量                                                               */
/* ------------------------------------------------------------------ */

/* 帧头（SOF，Start Of Frame）。故意选两个字节：单字节帧头太容易在载荷里撞上。 */
#define PROTO_SOF1              0xA5u
#define PROTO_SOF2              0x5Au

/* 最大载荷。上位机一次要 4 路数据（目标值/实际值/输出/电流），每路 2 字节，
 * 留到 32 字节已经够用；解析器内部缓冲区按这个值静态分配，不动态申请内存。 */
#define PROTO_MAX_PAYLOAD       32u

/* 定长部分：2 字节帧头 + 1 字节长度 + 1 字节序号 + 1 字节命令 + 1 字节校验 */
#define PROTO_OVERHEAD          6u

/* 一帧的最大总长度（含帧头与校验） */
#define PROTO_MAX_FRAME         (PROTO_OVERHEAD + PROTO_MAX_PAYLOAD)

/* 命令字。只列我现在用到的，以后加新数据类型就在这里加一行。 */
#define PROTO_CMD_PID_DATA      0x01u   /* PID 运行数据：目标/实际/输出，各 int16 */
#define PROTO_CMD_DEBUG_TEXT    0x02u   /* 调试文本，载荷是一串 ASCII */
#define PROTO_CMD_SET_PARAM     0x03u   /* 上位机下发参数：参数号 + float 值 */
#define PROTO_CMD_ACK           0x04u   /* 从机应答 */

/* 解析结果码 */
typedef enum {
    PROTO_OK            =  0,   /* 这一字节没凑出一帧，继续喂 */
    PROTO_FRAME_READY   =  1,   /* 收满一帧且校验通过，已经回调出去了 */
    PROTO_ERR_CRC       = -1,   /* 校验失败，整帧丢弃并回到找帧头状态 */
    PROTO_ERR_TOO_LONG  = -2,   /* 长度字段超出 PROTO_MAX_PAYLOAD，判定为假帧头 */
    PROTO_ERR_NULL      = -3    /* 传进来的指针是 NULL */
} ProtoStatus;

/* ------------------------------------------------------------------ */
/* 结构体（注意：结构体只用来"在内存里组织数据"，不直接整块往串口发） */
/* ------------------------------------------------------------------ */

/*
 * 帧头结构体。我特意把它和"字节偏移"分开写：
 * 上位机 Python 那边是按固定偏移解包的，不受 C 编译器对齐规则影响。
 *
 * 如果没有下面那个 packed 属性，编译器会对这个结构体做对齐填充：
 *   struct { uint8_t a; uint32_t b; } 默认占 8 字节而不是 5 字节，
 * 于是 memcpy(&frame, buf, sizeof(frame)) 发出去的就是错位的数据。
 */
#if defined(__GNUC__) || defined(__clang__)
    #define PROTO_PACKED __attribute__((packed))
#elif defined(_MSC_VER)
    #pragma pack(push, 1)
    #define PROTO_PACKED
    #define PROTO_NEED_POP_PACK
#else
    #define PROTO_PACKED
#endif

typedef struct PROTO_PACKED {
    uint8_t sof1;       /* 偏移 0：0xA5 */
    uint8_t sof2;       /* 偏移 1：0x5A */
    uint8_t len;        /* 偏移 2：载荷长度 N */
    uint8_t seq;        /* 偏移 3：帧序号 */
    uint8_t cmd;        /* 偏移 4：命令字 */
    uint8_t payload[PROTO_MAX_PAYLOAD];  /* 偏移 5：载荷 */
    uint8_t crc;        /* 偏移 6+N：校验，只在校验通过后才有意义 */
} ProtoFrame;

#ifdef PROTO_NEED_POP_PACK
    #pragma pack(pop)
#undef PROTO_NEED_POP_PACK
#endif

/* PID 数据载荷：3 个 int16，小端，共 6 字节 */
typedef struct PROTO_PACKED {
    int16_t setpoint;   /* 目标值，例如 转速设定 2000 rpm */
    int16_t actual;     /* 实际值，例如 编码器实测转速 */
    int16_t output;     /* PID 输出，例如 电流值 */
} ProtoPidPayload;

/* ------------------------------------------------------------------ */
/* 解析器状态机                                                       */
/* ------------------------------------------------------------------ */

typedef enum {
    PROTO_ST_WAIT_SOF1 = 0,  /* 找第一个帧头字节 0xA5 */
    PROTO_ST_WAIT_SOF2,      /* 收到 0xA5，等第二个帧头字节 0x5A */
    PROTO_ST_READ_LEN,       /* 读长度字段 */
    PROTO_ST_READ_BODY,      /* 收序号 + 命令 + 载荷（共 2+N 字节） */
    PROTO_ST_VERIFY          /* 读并核对 CRC */
} ProtoState;

/*
 * 一帧解析完成时被调用的回调。
 *   cmd     —— 命令字
 *   seq     —— 帧序号
 *   payload —— 指向解析器内部缓冲区的载荷，回调返回后内容就可能被覆盖，
 *              需要留存就自己拷走
 *   len     —— 载荷长度
 */
typedef void (*ProtoFrameCallback)(uint8_t cmd, uint8_t seq,
                                   const uint8_t *payload, uint8_t len);

typedef struct {
    ProtoState state;                    /* 当前状态 */
    uint8_t    buf[PROTO_MAX_PAYLOAD + 2u]; /* 存 序号 + 命令 + 载荷 */
    uint8_t    index;                    /* buf 里已存了多少字节 */
    uint8_t    need;                     /* 这一帧 buf 里总共要存多少字节 */
    uint8_t    len;                      /* 本帧载荷长度 */
    uint8_t    seq;                      /* 本帧序号 */
    uint8_t    cmd;                      /* 本帧命令字 */
    uint32_t   crc_error_count;          /* 校验失败次数，掉线时看这个 */
    uint32_t   frame_count;              /* 解析成功的帧数 */
    ProtoFrameCallback on_frame;         /* 回调，可为 NULL */
} ProtoParser;

/* ------------------------------------------------------------------ */
/* 接口                                                               */
/* ------------------------------------------------------------------ */

/**
 * CRC-8/ATM：多项式 0x07，初值 0x00，输入不反转，输出不反转，无异或输出。
 *   逐位法：每个字节循环 8 次。8 字节数据 = 64 次循环，
 *   在 72 MHz 的 F103 上不到 1 µs，1 kHz 调用完全没压力。
 */
uint8_t Protocol_Crc8(const uint8_t *data, size_t len);

/**
 * 接着算：用 crc 作为"已经算过前面若干字节"的中间结果，继续算 data。
 * 存在的理由：解析器想校验的那一段字节在内存里不连续
 * （帧头 2 字节是常量，长度字段单独存，后面 2+N 字节在 buf 里），
 * 没法一次 memcpy 成一个连续数组。有了这个函数就能分两段拼出同一个结果。
 *
 * 注意 Protocol_Crc8(p, n) 等价于 Protocol_Crc8Continue(0x00, p, n)，
 * 因为 CRC 的初值就是 0x00。
 */
uint8_t Protocol_Crc8Continue(uint8_t crc, const uint8_t *data, size_t len);

/**
 * 打包。把 cmd / payload 组装成一帧，写进 out。
 *   seq —— 帧序号，调用方自己维护（每发一帧 +1）
 * 返回整帧长度（6 + payload_len）；参数非法返回 0。
 */
size_t Protocol_Pack(uint8_t *out, size_t out_size,
                     uint8_t cmd, uint8_t seq,
                     const uint8_t *payload, uint8_t payload_len);

/* 初始化解析器。cb 为 NULL 时只解析不回调。 */
void Protocol_Init(ProtoParser *p, ProtoFrameCallback cb);

/* 复位解析器：清状态和缓冲，但保留统计计数。串口出错重连时调用。 */
void Protocol_Reset(ProtoParser *p);

/**
 * 状态机核心：喂一个字节进去。
 * 返回 PROTO_FRAME_READY 表示刚刚凑出一整帧（回调已经调用过）。
 * 半包、粘包都靠"逐字节"这件事本身解决，不需要调用方做任何额外处理。
 */
ProtoStatus Protocol_ParserFeed(ProtoParser *p, uint8_t byte);

/* 一次性喂一整块数据，返回成功解析出的帧数。 */
uint32_t Protocol_ParserFeedBuffer(ProtoParser *p, const uint8_t *data, size_t len);

/* 把状态名转成字符串，调试串口打印用 */
const char *Protocol_StateName(ProtoState st);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H */
