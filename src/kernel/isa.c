#include <onix/isa.h>
#include <onix/assert.h>
#include <onix/io.h>

enum
{
    DMA0_CHAN0_ADDR = 0x00,  // 通道 0 起始地址寄存器 (未使用)
    DMA0_CHAN0_COUNT = 0x01, // 通道 0 计数寄存器 (未使用)
    DMA0_CHAN1_ADDR = 0x02,  // 通道 1 起始地址寄存器
    DMA0_CHAN1_COUNT = 0x03, // 通道 1 计数寄存器
    DMA0_CHAN2_ADDR = 0x04,  // 通道 2 起始地址寄存器
    DMA0_CHAN2_COUNT = 0x05, // 通道 2 计数寄存器
    DMA0_CHAN3_ADDR = 0x06,  // 通道 3 起始地址寄存器
    DMA0_CHAN3_COUNT = 0x07, // 通道 3 计数寄存器

    DMA0_STATUS = 0x08,       // 主 DMA 状态寄存器
    DMA0_COMMAND = 0x08,      // 主 DMA 命令寄存器
    DMA0_REQUEST = 0x09,      // 主 DMA 请求寄存器
    DMA0_MASK1 = 0x0A,        // 主 DMA 单通道掩码寄存器
    DMA0_MODE = 0x0B,         // 主 DMA 模式寄存器
    DMA0_RESET = 0x0C,        // 主 DMA 触发器重置寄存器
    DMA0_TEMP = 0x0D,         // 主 DMA 临时寄存器
    DMA0_MASTER_CLEAR = 0x0D, // 主 DMA 主控清除寄存器
    DMA0_MASK_CLEAR = 0x0E,   // 主 DMA 掩码清除寄存器
    DMA0_MASK2 = 0x0F,        // 主 DMA 多通道掩码寄存器

    DMA1_CHAN4_ADDR = 0xC0,  // 通道 4 起始地址寄存器 (未使用)
    DMA1_CHAN4_COUNT = 0xC2, // 通道 4 计数寄存器 (未使用)
    DMA1_CHAN5_ADDR = 0xC4,  // 通道 5 起始地址寄存器
    DMA1_CHAN5_COUNT = 0xC6, // 通道 5 计数寄存器
    DMA1_CHAN6_ADDR = 0xC8,  // 通道 6 起始地址寄存器
    DMA1_CHAN6_COUNT = 0xCA, // 通道 6 计数寄存器
    DMA1_CHAN7_ADDR = 0xCC,  // 通道 7 起始地址寄存器
    DMA1_CHAN7_COUNT = 0xCE, // 通道 7 计数寄存器

    DMA1_STATUS = 0xD0,       // 从 DMA 状态寄存器
    DMA1_COMMAND = 0xD0,      // 从 DMA 命令寄存器
    DMA2_REQUEST = 0xD2,      // 从 DMA 请求寄存器
    DMA1_MASK1 = 0xD4,        // 从 DMA 单通道掩码寄存器
    DMA1_MODE = 0xD6,         // 从 DMA 模式寄存器
    DMA1_RESET = 0xD8,        // 从 DMA 触发器重置寄存器
    DMA1_TEMP = 0xDA,         // 从 DMA 临时寄存器
    DMA1_MASTER_CLEAR = 0xDA, // 从 DMA 主控清除寄存器
    DMA1_MASK_CLEAR = 0xDC,   // 从 DMA 掩码清除寄存器
    DMA1_MASK2 = 0xDE,        // 从 DMA 多通道掩码寄存器

    DMA0_CHAN1_PAGE = 0x83, // 通道 1 页地址寄存器
    DMA0_CHAN2_PAGE = 0x81, // 通道 2 页地址寄存器
    DMA0_CHAN3_PAGE = 0x82, // 通道 3 页地址寄存器

    DMA1_CHAN5_PAGE = 0x8B, // 通道 5 页地址寄存器
    DMA1_CHAN6_PAGE = 0x89, // 通道 6 页地址寄存器
    DMA1_CHAN7_PAGE = 0x8A, // 通道 7 页地址寄存器
};

// 各通道掩码寄存器数组
static u8 DMA_MASKS[] = {
    DMA0_MASK1,
    DMA0_MASK1,
    DMA0_MASK1,
    DMA0_MASK1,
    DMA1_MASK1,
    DMA1_MASK1,
    DMA1_MASK1,
    DMA1_MASK1,
};

// 各通道模式寄存器数组
static u8 DMA_MODES[] = {
    DMA0_MODE,
    DMA0_MODE,
    DMA0_MODE,
    DMA0_MODE,
    DMA1_MODE,
    DMA1_MODE,
    DMA1_MODE,
    DMA1_MODE,
};

// 各通道触发器重置寄存器数组
static u8 DMA_RESETS[] = {
    DMA0_RESET,
    DMA0_RESET,
    DMA0_RESET,
    DMA0_RESET,
    DMA1_RESET,
    DMA1_RESET,
    DMA1_RESET,
    DMA1_RESET,
};

// 各通道起始地址寄存器数组
static u8 DMA_ADDRS[] = {
    DMA0_CHAN0_ADDR,
    DMA0_CHAN1_ADDR,
    DMA0_CHAN2_ADDR,
    DMA0_CHAN3_ADDR,
    DMA1_CHAN4_ADDR,
    DMA1_CHAN5_ADDR,
    DMA1_CHAN6_ADDR,
    DMA1_CHAN7_ADDR,
};

// 各通道计数寄存器数组
static u8 DMA_COUNTS[] = {
    DMA0_CHAN0_COUNT,
    DMA0_CHAN1_COUNT,
    DMA0_CHAN2_COUNT,
    DMA0_CHAN3_COUNT,
    DMA1_CHAN4_COUNT,
    DMA1_CHAN5_COUNT,
    DMA1_CHAN6_COUNT,
    DMA1_CHAN7_COUNT,
};

// 页地址寄存器数组
static u8 DMA_PAGES[] = {
    0,
    DMA0_CHAN1_PAGE,
    DMA0_CHAN2_PAGE,
    DMA0_CHAN3_PAGE,
    0,
    DMA1_CHAN5_PAGE,
    DMA1_CHAN6_PAGE,
    DMA1_CHAN7_PAGE,
};

#define ISA_DMA_CHAN2_READ 0x46
#define ISA_DMA_CHAN2_WRITE 0x4A

#define ISA_DMA0_CHAN2_PAGE 0x81

// 设置 DMA 通道掩码
void isa_dma_mask(u8 channel, bool mask)
{
    assert(channel < 8);
    u16 port = DMA_MASKS[channel];
    u8 value = channel % 4;
    if (!mask)
    {
        value |= 0x4;
    }
    outb(port, value);
}

// 设置 DMA 起始地址
void isa_dma_addr(u8 channel, void *address)
{
    assert(channel < 8);
    u16 port = DMA_ADDRS[channel];

    u32 offset = ((u32)address) % 0x10000;
    if (channel >= 5)
    {
        offset >>= 1;
    }
    outb(port, offset & 0xFF);
    outb(port, (offset >> 8) & 0xFF);

    port = DMA_PAGES[channel];
    outb(port, (u32)address >> 16);
}

// 设置 DMA 传输大小
void isa_dma_size(u8 channel, u32 size)
{
    assert(channel < 8);
    u16 port = DMA_COUNTS[channel];
    if (channel >= 5)
    {
        size >>= 1;
    }

    outb(port, (size - 1) & 0xFF);
    outb(port, ((size - 1) >> 8) & 0xFF);
}

// 设置 DMA 模式
void isa_dma_mode(u8 channel, u8 mode)
{
    assert(channel < 8);
    u16 port = DMA_MODES[channel];
    outb(port, mode | (channel % 4));
}

// 重置 DMA 通道
void isa_dma_reset(u8 channel)
{
    assert(channel < 8);
    u16 port = DMA_RESETS[channel];
    outb(port, 0);
}
