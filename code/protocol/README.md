# code/protocol —— 二进制帧协议 + PC 端自测

这一份代码是[第 10 章 · 通信协议设计与数据可视化](../../docs/10-通信协议.md)的配套实现。

它**不依赖任何 STM32 / HAL 头文件**，所以在 PC 上就能单独编译、单独跑测试。
同一份 `protocol.c` 也可以直接拖进 Keil 工程里用（只用到了 `stdint.h` / `stddef.h` / `string.h`）。

## 文件

| 文件 | 作用 |
|---|---|
| `protocol.h` | 帧格式定义、命令字、结构体、接口声明 |
| `protocol.c` | CRC8、打包 `Protocol_Pack()`、状态机解析 `Protocol_ParserFeed()` |
| `test_protocol.c` | PC 端自测程序（10 组用例） |

## 帧格式

小端（little-endian）。定长头 5 字节 + 变长载荷 + 1 字节校验：

```text
偏移  0    1    2     3     4     5 .. 5+N-1   6+N
     +----+----+-----+-----+-----+-----------+------+
     | A5 | 5A | LEN | SEQ | CMD |  PAYLOAD  | CRC8 |
     +----+----+-----+-----+-----+-----------+------+
       SOF(2B)  1B    1B    1B      N 字节       1B
```

| 偏移 | 字段 | 长度 | 说明 |
|---|---|---|---|
| 0 | SOF1 | 1 字节 | 固定 `0xA5`，找同步点 |
| 1 | SOF2 | 1 字节 | 固定 `0x5A`，和 SOF1 一起确认帧头 |
| 2 | LEN | 1 字节 | 载荷字节数 N（不含帧头，不含 CRC），0 ≤ N ≤ 32 |
| 3 | SEQ | 1 字节 | 帧序号 0~255 循环，上位机用它算丢帧率 |
| 4 | CMD | 1 字节 | 命令字，区分数据类型 |
| 5 ~ 5+N-1 | PAYLOAD | N 字节 | 实际数据 |
| 6+N | CRC8 | 1 字节 | 对偏移 0 ~ 5+N-1 算的 CRC-8/ATM（多项式 `0x07`，初值 `0x00`） |

一帧总长度 = `6 + N`，N = 6 时是 12 字节。

命令字目前定义了 4 个（`protocol.h` 里加一行就能扩）：

| 值 | 名字 | 含义 |
|---|---|---|
| `0x01` | `PROTO_CMD_PID_DATA` | PID 运行数据：目标值 / 实际值 / 输出，各 `int16` |
| `0x02` | `PROTO_CMD_DEBUG_TEXT` | 调试文本，载荷是 ASCII |
| `0x03` | `PROTO_CMD_SET_PARAM` | 上位机下发参数 |
| `0x04` | `PROTO_CMD_ACK` | 从机应答 |

## 编译与运行

Windows / Linux / macOS 都一样，需要 gcc（MinGW-w64、MSYS2、WSL 里任意一个都行）：

```bash
cd code/protocol
gcc -O2 -std=c99 -Wall -Wextra -o test_protocol test_protocol.c protocol.c
./test_protocol          # Windows 下是 test_protocol.exe
```

要求是**零警告**编译通过。如果 `-Wall -Wextra` 报出警告，那是代码的问题，不是编译器的脾气。

### 预期输出

下面是我在本机跑出来的真实输出（末尾几行）：

```text
=== 8. CRC 错误检出 ===
  [PASS] 逐个翻转 12 个字节，没有一次回调出错误数据
  [PASS] p.crc_error_count == 1u  (= 1)
  [PASS] p.frame_count == 0u  (= 0)

=== 9. 结构体对齐：未 packed 的坑 ===
  sizeof(DemoDefault) = 4 字节
  sizeof(DemoPacked)  = 3 字节
  offsetof(DemoPacked.b) = 1
  [PASS] sizeof(DemoPacked) == 3u  (= 3)
  [PASS] offsetof(DemoPacked, b) == 1u  (= 1)
  [PASS] sizeof(DemoDefault) == 4u  (= 4)
  [PASS] 默认对齐下是 4 字节，比实际的 3 字节多 1 字节填充

=== 10. 连续 300 帧（按 SEQ 检查丢帧） ===
  构造的字节流 = 2400 字节（300 帧 × 8 字节）
  [PASS] g_seq_n == SEQ_TEST_FRAMES  (= 300)
  [PASS] p.frame_count == SEQ_TEST_FRAMES  (= 300)
  [PASS] p.crc_error_count == 0u  (= 0)
  [PASS] 300 帧的 SEQ 连续、载荷一一对应，没有丢帧也没有多出帧
  [PASS] SEQ 在第 256 帧正确回绕（0xFF -> 0x00）
  平均每帧 8.0 字节

========================================
 总计: 77 项通过, 0 项失败
 结果: ALL PASS
========================================
```

（"多少项通过"会随断言条数变化，**只要有 1 项失败就是没通过**，程序返回非 0。）

> 说明一下我的编译环境：这台机器上装的是 zig 0.14.1（`zig cc` 用的就是 clang，
> 命令行参数和 gcc 兼容），不是 MinGW 的 gcc。`-O2 -std=c99 -Wall -Wextra`
> 零警告、77 项断言全通过都是在这个编译器上实测的。用 MinGW gcc 编译时
> 可能有极个别的警告差异，**如果出现警告请以警告为准去改代码**。

## 测试覆盖了什么

| 组 | 验证内容 | 对应真实场景 |
|---|---|---|
| 1 | CRC8 分段续算 == 整体计算 | 解析器要分两段算 CRC（帧头在常量里、载荷在缓冲区里） |
| 2 | 打包后的字节偏移、LEN 字段、CRC 位置 | 上下位机对齐"第几个字节是什么" |
| 3 | 打包 → 解析能还原原始数据 | 最基本的往返正确性 |
| 4 | **半包**：一帧拆成任意两段、甚至一个字节一个字节喂 | 串口中断一次只收到半个帧 |
| 5 | **粘包**：两帧连在一起一次喂进去 | 主循环一次从环形缓冲取出两帧 |
| 6 | 载荷里故意塞 `0xA5 0x5A` 不误同步 | 载荷数据碰巧和帧头一样 |
| 7 | 前面有垃圾字节时能重新同步 | 上电瞬间、抗干扰导致的错位 |
| 8 | 逐个翻转帧里每个字节，CRC 都要拦住 | 传输误码 |
| 9 | `packed` 与默认对齐的 `sizeof` 差别 | 结构体直接 `memcpy` 发送的坑 |
| 10 | 连续 300 帧按 SEQ 检查不丢帧 | 1 kHz 连续数据流 |

## （可选）Python 上位机

`test_protocol` 只验证协议逻辑，不做绘图。想看曲线，用下面这段 Python 读串口。
二进制协议的好处是**每秒能画 1000 个点**——一帧 12 字节，115200 8N1 下
`115200 / 10 = 11520 字节/秒`，理论上够发 960 帧/秒；而 `printf("%d,%d\n", ...)`
一次要十几到二十几个字节，最多也就 500 行/秒左右，还没算上位机解析字符串的开销。

```python
# upper_monitor.py —— 需要 pyserial：pip install pyserial
# 可选 matplotlib 或 pyqtgraph：pip install pyserial matplotlib
import struct
import serial

SOF = b"\xA5\x5A"

def crc8(data: bytes) -> int:
    """CRC-8/ATM，多项式 0x07，初值 0x00。必须和 protocol.c 完全一致。"""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc

def frames(port: serial.Serial):
    """状态机解帧：和 protocol.c 里的五个状态一一对应。"""
    buf = bytearray()
    while True:
        buf += port.read(port.in_waiting or 1)
        while True:
            if len(buf) < 6:
                break
            start = buf.find(SOF)
            if start < 0:
                buf.clear()
                break
            if start > 0:
                del buf[:start]              # 丢掉帧头前面的垃圾
            if len(buf) < 3:
                break
            length = buf[2]
            total = 6 + length
            if len(buf) < total:
                break                        # 半包：等下一批字节
            frame, rest = bytes(buf[:total]), buf[total:]
            buf = bytearray(rest)
            if crc8(frame[:-1]) != frame[-1]:
                continue                     # CRC 不过就丢
            seq, cmd = frame[3], frame[4]
            yield seq, cmd, frame[5:5 + length]

def main():
    ser = serial.Serial("COM3", 115200, timeout=0.05)   # Linux 下是 /dev/ttyUSB0
    for seq, cmd, payload in frames(ser):
        if cmd != 0x01 or len(payload) != 6:
            continue
        # '<hhh' = 三个小端 int16；STM32 和 PC 都是小端，所以 '<' 可以省略，
        # 但显式写出来更保险（换到大端平台上也不会错）
        setpoint, actual, output = struct.unpack("<hhh", payload)
        print(f"seq={seq:3d}  目标={setpoint:6d}  实际={actual:6d}  输出={output:6d}")

if __name__ == "__main__":
    main()
```

把 `print` 换成往 `matplotlib.animation` 或者 `pyqtgraph.PlotWidget` 里塞数据，
就是第 10 章说的"数据流可视化"。`pyqtgraph` 在 1 kHz 下明显比 `matplotlib` 流畅，
因为它是基于 Qt 的 `QPainter` 直接画，不像 `matplotlib` 每帧重建整个 Figure。

> 🔬 上面这段 Python 我只在 PC 上跑过 `crc8()` 与 `frames()` 的逻辑（喂构造好的字节流），
> **没有接真实串口跑过**，因为板子和 USB-TTL 还没到手。真实串口的时间行为属于待验证。

## 接到 STM32 上要改什么

`protocol.c` 一行都不用改。要自己写的是"串口收发"这一层，大致是：

```c
/* ---- 收：中断里只做一件事——把字节塞进环形缓冲 ---- */
void USART1_IRQHandler(void)
{
    if (USART1->SR & USART_SR_RXNE) {
        uint8_t b = (uint8_t)(USART1->DR & 0xFF);   /* 读 DR 顺便清标志 */
        RingBuf_Push(&g_rx_ring, b);                /* 千万别在这里解析，更别 printf */
    }
}

/* ---- 主循环：出队喂给状态机 ---- */
static ProtoParser g_parser;

void main_loop(void)
{
    uint8_t b;
    while (RingBuf_Pop(&g_rx_ring, &b)) {
        Protocol_ParserFeed(&g_parser, b);          /* 状态机天然处理半包/粘包 */
    }
}
```

两点提醒：

1. **接收中断里不要跑状态机、不要 `printf`**。STM32 的 `printf` 重定向到串口是阻塞发送，
   在中断里调用会把中断拖长，下一个字节来的时候 `DR` 还没读走，就溢出丢包了（第 6 节踩过的坑）。
2. **发送方向**：STM32 这边可以用 `HAL_UART_Transmit()` 直接把 `Protocol_Pack()` 出来的
   `uint8_t buf[PROTO_MAX_FRAME]` 整块发出去，长度就是返回值。发之前记得 `seq++`。
