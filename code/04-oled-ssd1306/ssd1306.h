/**
 ******************************************************************************
 * @file    ssd1306.h
 * @brief   0.96 寸 OLED（SSD1306，128x64，I2C 接口）驱动 —— 头文件
 *
 * 硬件连接（与仓库其它章节统一）：
 *   OLED VCC -> 3.3V       OLED GND -> GND
 *   OLED SCL -> PB6 (I2C1_SCL)
 *   OLED SDA -> PB7 (I2C1_SDA)
 *
 * ⚠️ 上拉电阻：I2C 是开漏总线，SCL/SDA 都必须外接上拉到 3.3V。
 *    模块自带上拉时不用再加；裸屏或模块没焊上拉时必须自己补，
 *    400kHz 建议 2.2kΩ，100kHz 可以用 4.7kΩ（算法见第 06 章第 2 节）。
 *
 * 从机地址：SSD1306 数据手册写的 7 位地址是 0x3C（SA0=0）或 0x3D（SA0=1），
 * 但 HAL 库要求传入"左移一位后"的地址，也就是 0x78 或 0x7A。
 ******************************************************************************
 */

#ifndef __SSD1306_H
#define __SSD1306_H

#include "main.h"          /* CubeMX 生成的工程里 main.h 会包含 stm32f1xx_hal.h */

/* ========================= 可配置项 ========================= */

/* I2C 从机地址：HAL 库要求"7 位地址 << 1"。
 * 模块背后电阻跳线决定 SA0：SA0 接 GND -> 7 位地址 0x3C -> 传 0x78
 *                            SA0 接 VCC -> 7 位地址 0x3D -> 传 0x7A
 * 分不清就两个都试，收到 ACK 的那个才对。 */
#define SSD1306_I2C_ADDR       (0x3C << 1)      /* = 0x78 */

/* 每个字节最多等多久（毫秒）。128 字节 @400kHz 约 2.8ms，
 * 留足余量即可；不要用 HAL_MAX_DELAY，否则总线被拉死会卡住程序 */
#define SSD1306_I2C_TIMEOUT    100U

/* 一次 I2C 传输携带的数据字节上限。
 * 一页 128 字节一次发完，在 100kHz 下约 11.5ms，会超过上面的超时；
 * 而且中途出错就得整页重来，所以拆成小块发。 */
#define SSD1306_CHUNK          32U

/* ========================= 屏幕参数 ========================= */

#define SSD1306_WIDTH          128U    /* 横向 128 列 */
#define SSD1306_HEIGHT         64U     /* 纵向 64 行（= 8 页 x 8 行）*/

#define SSD1306_PAGES          (SSD1306_HEIGHT / 8U)          /* = 8  */
#define SSD1306_BUF_SIZE       (SSD1306_WIDTH * SSD1306_PAGES) /* = 1024 字节 */

/* ========================= 常用命令宏 ========================= */
/* 完整命令表见 SSD1306 数据手册 Rev 1.1 第 8 章 Command Table */

#define SSD1306_CMD_DISPLAY_OFF        0xAEU   /* 关显示（显存内容保留）*/
#define SSD1306_CMD_DISPLAY_ON         0xAFU   /* 开显示 */
#define SSD1306_CMD_SET_MEM_MODE       0x20U   /* 设置寻址模式，后跟 1 字节参数 */
#define SSD1306_CMD_SET_COL_ADDR       0x21U   /* 设置列地址范围，后跟 2 字节 */
#define SSD1306_CMD_SET_PAGE_ADDR      0x22U   /* 设置页地址范围，后跟 2 字节 */
#define SSD1306_CMD_SET_CONTRAST       0x81U   /* 对比度，后跟 1 字节（0x00~0xFF）*/
#define SSD1306_CMD_SEG_REMAP          0xA1U   /* 段重映射：左右方向翻转 */
#define SSD1306_CMD_COM_SCAN_DEC       0xC8U   /* COM 扫描方向：上下方向翻转 */
#define SSD1306_CMD_SET_COM_PINS       0xDAU   /* COM 引脚配置，后跟 1 字节 */
#define SSD1306_CMD_CHARGE_PUMP        0x8DU   /* 电荷泵，后跟 1 字节 */
#define SSD1306_CMD_PAGE_START_BASE    0xB0U   /* 0xB0~0xB7 直接指定页地址（页寻址模式用）*/

/* ========================= 接口函数 ========================= */

/**
 * @brief  初始化 OLED。失败返回 HAL_Error，成功返回 HAL_OK
 * @note   里面会先探测从机地址是否有 ACK，没接屏会返回错误而不是死等
 */
HAL_StatusTypeDef SSD1306_Init(void);

/**
 * @brief  发送一条命令（控制字节 0x00 + 1 字节命令）
 */
HAL_StatusTypeDef SSD1306_WriteCmd(uint8_t cmd);

/**
 * @brief  连续发送 n 字节显示数据（控制字节 0x40 + 数据）
 * @param  data 数据首地址
 * @param  len  字节数
 */
HAL_StatusTypeDef SSD1306_WriteData(const uint8_t *data, uint16_t len);

/**
 * @brief  把显存缓冲全部清成 0（屏幕全黑）
 * @note   只改内存，不改屏幕。要看到效果得再调 SSD1306_Refresh()
 */
void SSD1306_Clear(void);

/**
 * @brief  把 SSD1306_BUF_SIZE 字节显存推到屏幕上
 * @note   按页分块发送，每次最多 SSD1306_CHUNK 字节
 */
HAL_StatusTypeDef SSD1306_Refresh(void);

/**
 * @brief  在显存里画一个点
 * @param  x 0~127，从左到右
 * @param  y 0~63，从上到下
 * @param  on 1 = 点亮，0 = 熄灭
 */
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t on);

/**
 * @brief  在 (x, y) 处显示一个字符串
 * @param  x 起始列 0~127
 * @param  y 起始行 0~63，建议取 8 的倍数（0/8/16/.../56），否则字会骑在两页上
 * @note   字库只覆盖 ASCII 0x20~0x7E；超出的字符显示为空白，不会乱码
 */
void SSD1306_ShowString(uint8_t x, uint8_t y, const char *str);

#endif /* __SSD1306_H */
