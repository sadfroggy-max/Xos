#include <onix/types.h>
#include <onix/io.h>
#include <onix/sb16.h>
#include <onix/syscall.h>
#include <onix/interrupt.h>
#include <onix/memory.h>
#include <onix/isa.h>
#include <onix/assert.h>
#include <onix/debug.h>
#include <onix/device.h>
#include <onix/string.h>
#include <onix/buffer.h>
#include <onix/task.h>
#include <onix/stdlib.h>
#include <onix/stat.h>
#include <onix/fs.h>
#include <onix/mutex.h>
#include <onix/errno.h>

#define LOGK(fmt, args...) DEBUGK(fmt, ##args)

#define SB_MIXER 0x224      // 混音器端口地址
#define SB_MIXER_DATA 0x225 // 混音器数据端口地址
#define SB_RESET 0x226      // DSP 复位端口
#define SB_READ 0x22A       // DSP 数据读取端口
#define SB_WRITE 0x22C      // DSP 数据写入端口
#define SB_STATE 0x22E      // DSP 状态读取端口
#define SB_INTR16 0x22F     // 16 位 DSP 中断响应端口

#define CMD_STC 0x40  // 设置时间常数
#define CMD_SOSR 0x41 // 设置输出采样率
#define CMD_SISR 0x42 // 设置输入采样率

#define CMD_SINGLE_IN8 0xC8   // 单次 8 位输入传输模式
#define CMD_SINGLE_OUT8 0xC0  // 单次 8 位输出传输模式
#define CMD_SINGLE_IN16 0xB8  // 单次 16 位输入传输模式
#define CMD_SINGLE_OUT16 0xB0 // 单次 16 位输出传输模式

#define CMD_AUTO_IN8 0xCE   // 自动 8 位输入传输模式
#define CMD_AUTO_OUT8 0xC6  // 自动 8 位输出传输模式
#define CMD_AUTO_IN16 0xBE  // 自动 16 位输入传输模式
#define CMD_AUTO_OUT16 0xB6 // 自动 16 位输出传输模式

#define CMD_ON 0xD1      // 打开扬声器
#define CMD_OFF 0xD3     // 关闭扬声器
#define CMD_SP8 0xD0     // 停止 8 位通道播放
#define CMD_RP8 0xD4     // 恢复 8 位通道播放
#define CMD_SP16 0xD5    // 停止 16 位通道播放
#define CMD_RP16 0xD6    // 恢复 16 位通道播放
#define CMD_VERSION 0xE1 // 获取 DSP 版本

#define MODE_MONO8 0x00
#define MODE_STEREO16 0x30

#define STATUS_READ 0x80  // 读缓冲区状态
#define STATUS_WRITE 0x80 // 写缓冲区状态

#define DMA_BUF_ADDR 0x90000 // 缓冲区地址，不能跨越 64K 边界
#define DMA_BUF_SIZE 0x8000  // 缓冲区大小

#define SAMPLE_RATE 44100 // 采样率

typedef struct sb_t
{
    task_t *waiter;
    lock_t lock;
    char *addr; // DMA 缓冲区地址
    u8 mode;    // DSP 模式
    u8 channel; // DMA 通道号
} sb_t;

static sb_t sb16;

static void sb_handler(int vector)
{
    send_eoi(vector);
    sb_t *sb = &sb16;

    inb(SB_INTR16);

    u8 state = inb(SB_STATE);

    LOGK("sb16 handler state 0x%X...\n", state);
    if (sb->waiter)
    {
        task_unblock(sb->waiter, EOK);
        sb->waiter = NULL;
    }
}

static void sb_reset(sb_t *sb)
{
    outb(SB_RESET, 1);
    sleep(1);
    outb(SB_RESET, 0);
    u8 state = inb(SB_READ);
    LOGK("sb16 reset state 0x%x\n", state);
}

static void sb_intr_irq(sb_t *sb)
{
    outb(SB_MIXER, 0x80);
    u8 data = inb(SB_MIXER_DATA);
    if (data != 2)
    {
        outb(SB_MIXER, 0x80);
        outb(SB_MIXER_DATA, 0x2);
    }
}

static void sb_out(u8 cmd)
{
    while (inb(SB_WRITE) & 128)
        ;
    outb(SB_WRITE, cmd);
}

static void sb_turn(sb_t *sb, bool on)
{
    if (on)
        sb_out(CMD_ON);
    else
        sb_out(CMD_OFF);
}

static u32 sb_time_constant(u8 channels, u16 sample)
{
    return 65536 - (256000000 / (channels * sample));
}

static void sb_set_volume(u8 level)
{
    LOGK("set sb16 volume to 0x%02X\n", level);
    outb(SB_MIXER, 0x22);
    outb(SB_MIXER_DATA, level);
}

int sb16_ioctl(sb_t *sb, int cmd, void *args, int flags)
{
    switch (cmd)
    {
    case SB16_CMD_ON:
        sb_reset(sb);    // 复位 DSP
        sb_intr_irq(sb); // 设置中断
        sb_out(CMD_ON);  // 打开声霸卡
        return EOK;
    case SB16_CMD_OFF:
        sb_out(CMD_OFF); // 关闭声霸卡
        return EOK;
    case SB16_CMD_MONO8:
        sb->mode = MODE_MONO8;
        sb->channel = 1;
        isa_dma_reset(sb->channel);
        return EOK;
    case SB16_CMD_STEREO16:
        sb->mode = MODE_STEREO16;
        sb->channel = 5;
        isa_dma_reset(sb->channel);
        return EOK;
    case SB16_CMD_VOLUME:
        sb_set_volume((u32)args & 0xff);
        return EOK;
    default:
        break;
    }
    return -EINVAL;
}

int sb16_write(sb_t *sb, char *data, size_t size)
{
    lock_acquire(&sb->lock);
    assert(size <= DMA_BUF_SIZE);
    memcpy(sb->addr, data, size);

    isa_dma_mask(sb->channel, false);

    isa_dma_addr(sb->channel, sb->addr);
    isa_dma_size(sb->channel, size);

    // 设置采样率
    sb_out(CMD_SOSR);                  // 44100 = 0xAC44
    sb_out((SAMPLE_RATE >> 8) & 0xFF); // 0xAC
    sb_out(SAMPLE_RATE & 0xFF);        // 0x44

    if (sb->mode == MODE_MONO8)
    {
        isa_dma_mode(sb->channel, DMA_MODE_SINGLE | DMA_MODE_WRITE);
        sb_out(CMD_SINGLE_OUT8);
        sb_out(MODE_MONO8);
    }
    else
    {
        isa_dma_mode(sb->channel, DMA_MODE_SINGLE | DMA_MODE_WRITE);
        sb_out(CMD_SINGLE_OUT16);
        sb_out(MODE_STEREO16);
        size >>= 2; // size /= 4
    }

    sb_out((size - 1) & 0xFF);
    sb_out(((size - 1) >> 8) & 0xFF);

    isa_dma_mask(sb->channel, true);

    assert(sb->waiter == NULL);
    sb->waiter = running_task();
    assert(task_block(sb->waiter, NULL, TASK_BLOCKED, TIMELESS) == EOK);

rollback:
    lock_release(&sb->lock);
    return size;
}

void sb16_init()
{
    LOGK("Sound Blaster 16 init...\n");

    sb_t *sb = &sb16;

    sb->addr = (char *)DMA_BUF_ADDR;
    sb->mode = MODE_STEREO16;
    sb->channel = 5;
    lock_init(&sb->lock);

    set_interrupt_handler(IRQ_SB16, sb_handler);
    set_interrupt_mask(IRQ_SB16, true);

    device_install(DEV_CHAR, DEV_SB16, sb, "sb16", 0, sb16_ioctl, NULL, sb16_write);
}
