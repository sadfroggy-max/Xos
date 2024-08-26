#include <onix/types.h>
#include <onix/stdio.h>
#include <onix/assert.h>
#include <onix/debug.h>
#include <onix/rtc.h>
#include <onix/string.h>
#include <onix/interrupt.h>
#include <onix/io.h>
#include <onix/task.h>
#include <onix/errno.h>
#include <onix/syscall.h>
#include <onix/mutex.h>
#include <onix/memory.h>
#include <onix/device.h>
#include <onix/isa.h>
#include <onix/timer.h>

#define LOGK(fmt, args...) DEBUGK(fmt, ##args)

#define DRIVE_NONE 0
#define DRIVE_144M 4
#define DRIVE_INVALID -1

#define TIMEOUT_OUT 1000
#define TIMEOUT_IN 1000
#define TIMEOUT_WAIT 10000
#define MOTOR_WAITING_TIME 1     // 这里的时间实际可能较短，但在模拟器中可以接受
#define MOTOR_INACTIVITY_TIMEOUT 10000 // 马达超时未使用则关闭

#define REG_DOR 0x3F2  // Digital Output Register
#define REG_MSR 0x3F4  // Main Status Register (input)
#define REG_DRS 0x3F4  // Data Rate Select Register (output)
#define REG_DATA 0x3F5 // Data Register
#define REG_DIR 0x3F7  // Digital Input Register (input)
#define REG_CCR 0x3F7  // Configuration Control Register

#define MSR_DRIVE_A 0x1   // 驱动器 A
#define MSR_DRIVE_B 0x2   // 驱动器 B
#define MSR_DRIVE_C 0x4   // 驱动器 C
#define MSR_DRIVE_D 0x8   // 驱动器 D
#define MSR_CMD_BUSY 0x10  // 命令忙状态
#define MSR_DMA_MODE 0x20  // 0 - DMA 传输模式，1 - 非 DMA 模式
#define MSR_IO_DIR 0x40   // 传输方向：0 - CPU -> FDC，1 - FDC -> CPU
#define MSR_DATA_READY 0x80 // 数据寄存器就绪

#define DOR_SELECT_A 0x1  // 选择驱动器 A
#define DOR_SELECT_B 0x2  // 选择驱动器 B
#define DOR_NORM_OP 0x4 // 正常操作状态
#define DOR_ENABLE_IRQ 0x8    // 使能中断
#define DOR_MOTOR_A 0x10  // 启动驱动器 A 的马达
#define DOR_MOTOR_B 0x20  // 启动驱动器 B 的马达
#define DOR_MOTOR_C 0x40  // 启动驱动器 C 的马达
#define DOR_MOTOR_D 0x80  // 启动驱动器 D 的马达

#define DIR_CHANGE 0x80 // 软盘发生改变

#define CMD_SET_PARAMS 0x03     // 设置驱动器参数
#define CMD_WRITE_DATA 0xC5       // 写数据命令
#define CMD_READ_DATA 0xE6        // 读数据命令
#define CMD_RECALIB 0x07       // 重新校准
#define CMD_GET_INT_STATUS 0x08      // 获取中断状态
#define CMD_FORMAT_TRACK 0x4D      // 格式化磁道命令
#define CMD_SEEK_TRACK 0x0F        // 寻道命令
#define CMD_GET_VERSION 0x10     // 获取 FDC 版本
#define CMD_FDC_RESET 0xFE       // FDC 复位

#define FDC_TYPE_82077 0x90 // 扩展型 uPD765B 控制器

#define MOTOR_STATE_OFF 0
#define MOTOR_STATE_DELAY 1
#define MOTOR_STATE_ON 2

#define OP_READ 0
#define OP_WRITE 1

#define SECTOR_SZ 512

#define DMA_BUFFER_ADDR 0x98000 // 不可跨越 64K 边界

#define MAX_RESULT 8

typedef struct fdc_result_t
{
    u8 status0;
    u8 status1;
    u8 status2;
    u8 status3;
    u8 cur_track;
    u8 cur_head;
    u8 cur_sector;
    u8 cur_size;
} fdc_result_t;

typedef struct floppy_ctrl_t
{
    task_t *pending_task; // 等待任务
    timer_t *floppy_timer; // 定时器
    lock_t fdc_lock;    // 控制器锁

    char dev_name[8];
    int floppy_type; // 软盘类型

    u8 dor_register; // 数字输出寄存器

    u8 *dma_buffer; // DMA 缓冲区地址

    union
    {
        u8 num_tracks;    // 磁道数
        u8 num_cylinders; // 柱面数
    };
    u8 num_heads;   // 磁头数
    u8 num_sectors; // 每磁道扇区数
    u8 gap_length;  // GAP3 长度

    u8 cur_drive;   // 当前驱动器号
    u8 cur_status0;     // 状态寄存器 0
    u8 cur_track;   // 当前磁道号
    u8 motor_state;   // 马达状态
    u8 disk_changed; // 软盘变化标志

    union
    {
        u8 status[MAX_RESULT];
        fdc_result_t result;
    };

} floppy_ctrl_t;

// 系统目前只支持一个 1.44M 软盘设备
static floppy_ctrl_t fdc;

// 软盘中断处理函数
static void fd_isr(int irq_num)
{
    send_eoi(irq_num); // 发送中断结束信号；
    LOGK("floppy interrupt triggered...\n");

    floppy_ctrl_t *fd = &fdc;

    if (fd->pending_task)
    {
        task_unblock(fd->pending_task, EOK);
        fd->pending_task = NULL;
    }
}

// 获取软驱类型
static u8 get_drive_type()
{
    u8 type = cmos_read(0x10);
    return (type >> 4) & 0xf; // 主软驱类型在高四位
}

static err_t fdc_outb(u8 data)
{
    u32 timeout = timer_expire_jiffies(TIMEOUT_OUT);

    extern u32 jiffies;

    while (true)
    {
        if (timer_is_expires(timeout))
            return -ETIME;
        u8 msr = inb(REG_MSR) & (MSR_DATA_READY | MSR_IO_DIR);
        LOGK("out state 0x%X %d %d...\n", msr, timeout, jiffies);
        if (msr == MSR_DATA_READY)
        {
            outb(REG_DATA, data);
            return EOK;
        }
        task_yield();
    }
}

static err_t fdc_inb()
{
    u32 timeout = timer_expire_jiffies(TIMEOUT_IN);
    while (true)
    {
        if (timer_is_expires(timeout))
            return -ETIME;
        u8 msr = inb(REG_MSR) & (MSR_DATA_READY | MSR_IO_DIR | MSR_CMD_BUSY);
        if (msr == (MSR_DATA_READY | MSR_IO_DIR | MSR_CMD_BUSY))
            return inb(REG_DATA) & 0xFF;
        task_yield();
    }
}

// 等待软盘中断
static err_t wait_for_interrupt(floppy_ctrl_t *fd)
{
    assert(!fd->pending_task);
    fd->pending_task = running_task();
    return task_block(fd->pending_task, NULL, TASK_BLOCKED, TIMEOUT_WAIT);
}


// 得到执行的结果
static void fd_result(floppy_t *fd, bool sensei)
{
    int n = 0;
    while (n < 7)
    {
        u8 msr = inb(FDC_MSR) & (MSR_READY | MSR_DIO | MSR_BUSY);
        if (msr == MSR_READY)
        {
            break;
        }
        if (msr == (MSR_READY | MSR_DIO | MSR_BUSY))
        {
            fd->st[n++] = fd_inb();
        }
    }

    if (sensei)
    {
        // Send a "sense interrupt status" command
        fd_outb(CMD_SENSEI);
        fd->st0 = fd_inb();
        fd->track = fd_inb();
    }

    // Check for disk changed
    if (inb(FDC_DIR) & DIR_CHANGED)
        fd->changed = true;
}

static void fd_motor_on(floppy_t *fd)
{
    if (fd->motor != MOTOR_OFF)
    {
        fd->motor = MOTOR_ON;
        return;
    }

    LOGK("fd: motor on\n");

    fd->dor |= 0x10 << fd->drive;
    outb(FDC_DOR, fd->dor);
    fd->motor = MOTOR_ON;
    task_sleep(FDC_MOTOR_WAITING);
}

static void fd_motor_timeout(timer_t *timer)
{
    LOGK("floppy motor off timeout\n");
    floppy_t *fd = timer->arg;
    fd->dor &= ~(0x10 << fd->drive);
    outb(FDC_DOR, fd->dor);
    fd->motor = MOTOR_OFF;
    fd->timer = NULL;
}

static void fd_motor_off(floppy_t *fd)
{
    if (fd->motor == MOTOR_ON)
    {
        LOGK("floppy motor off delay\n");
        fd->motor = MOTOR_DELAY;
    }
    if (!fd->timer)
    {
        fd->timer = timer_add(FDC_MOTOR_TIMEOUT, fd_motor_timeout, fd);
    }
    else
    {
        timer_update(fd->timer, FDC_MOTOR_TIMEOUT);
    }
}

// 重新校正，将磁道归零
static void fd_recalibrate(floppy_t *fd)
{
    LOGK("fd: recalibrate\n");

    fd_outb(CMD_RECALIBRATE);
    fd_outb(0x00);

    assert(fd_wait(fd) == EOK);

    fd_result(fd, true);
}

// 设置软盘的控制细节
static void fd_specify(floppy_t *fd)
{
    LOGK("floppy specify...\n");
    fd_outb(CMD_SPECIFY);
    fd_outb(0xDF); // SRT = 3ms, HUT = 240ms
    fd_outb(0x02); // HLT = 16ms, ND = 0
}

// 寻道
static void fd_seek(floppy_t *fd, u8 track, u8 head)
{
    if (fd->track == track)
        return;

    LOGK("fd: seek track %d head %d (current %d)\n", track, head, fd->track);
    fd_outb(CMD_SEEK);
    fd_outb((unsigned char)((head << 2) | fd->drive));
    fd_outb(track);

    assert(fd_wait(fd) == EOK);

    fd_result(fd, true);

    if ((fd->st0 & 0xE0) != 0x20 || fd->track != track)
    {
        panic("fd: seek failed, st0 0x%02x, current %d, target %d\n", fd->st0, fd->track, track);
    }

    // 休眠 15 毫秒
    task_sleep(15);
}

static void lba2chs(floppy_t *fd, idx_t lba, u8 *track, u8 *head, u8 *sector)
{
    *track = lba / (fd->heads * fd->sectors);
    *head = (lba / fd->sectors) % fd->heads;
    *sector = lba % fd->sectors + 1;
}

static err_t fd_transfer(floppy_t *fd, bool mode, void *buf, u8 count, idx_t lba)
{
    u8 track, head, sector;

    // 转换 lba 到 chs cylinder head sector
    lba2chs(fd, lba, &track, &head, &sector);

    // 当前磁道剩余扇区数
    u8 remaining = ((fd->sectors + 1 - sector) + fd->sectors * (fd->heads - head - 1));

    // 这个地方确实有问题，可能存在读取内容较少，外层做了兼容；
    if (remaining < count)
        count = remaining;

    if (mode == FD_WRITE)
        memcpy(fd->buf, buf, count * SECTOR_SIZE);

    // Perform seek if necessary
    fd_seek(fd, track, head);

    // Program data rate (500K/s)
    outb(FDC_CCR, 0);

    // 设置 DMA
    isa_dma_mask(2, false);
    isa_dma_reset(2);

    if (mode == FD_READ)
        isa_dma_mode(2, DMA_MODE_SINGLE | DMA_MODE_READ);
    else
        isa_dma_mode(2, DMA_MODE_SINGLE | DMA_MODE_WRITE);

    // Setup DMA transfer
    isa_dma_addr(2, fd->buf);
    isa_dma_size(2, (u32)count * SECTOR_SIZE);
    isa_dma_mask(2, true);

    // 执行读写请求
    if (mode == FD_READ)
        fd_outb(CMD_READ);
    else
        fd_outb(CMD_WRITE);

    fd_outb((u8)((head << 2) | fd->drive));
    fd_outb(track);
    fd_outb(head);
    fd_outb(sector);
    fd_outb(0x02); // 512 bytes/sector
    fd_outb(fd->sectors);
    fd_outb(fd->gap3);
    fd_outb(0xFF); // DTL = unused

    assert(fd_wait(fd) == EOK); // 等待中断

    fd_result(fd, false);

    if ((fd->result.st0 & 0xC0) == 0)
    {
        // Successful transfer
        if (mode == FD_READ)
            memcpy(buf, fd->buf, count * SECTOR_SIZE);
        return EOK;
    }
    else
    {
        LOGK("fd: xfer error, st0 %02X st1 %02X st2 %02X THS=%d/%d/%d\n",
             fd->result.st0, fd->result.st1, fd->result.st2,
             fd->result.track, fd->result.head, fd->result.sector);
        return -EIO;
    }
}

static err_t fd_read(floppy_t *fd, void *buf, u8 count, idx_t lba)
{
    assert(count + lba < (u32)fd->tracks * fd->heads * fd->sectors);

    lock_acquire(&fd->lock);
    fd_motor_on(fd);

    int ret = fd_transfer(fd, FD_READ, buf, count, lba);

    fd_motor_off(fd);
    lock_release(&fd->lock);
    return ret;
}

static err_t fd_write(floppy_t *fd, void *buf, u8 count, idx_t lba)
{
    assert(count + lba < (u32)fd->tracks * fd->heads * fd->sectors);

    lock_acquire(&fd->lock);
    fd_motor_on(fd);

    int ret = fd_transfer(fd, FD_WRITE, buf, count, lba);

    fd_motor_off(fd);
    lock_release(&fd->lock);
    return ret;
}

// 磁盘控制
static int fd_ioctl(floppy_t *fd, int cmd, void *args, int flags)
{
    switch (cmd)
    {
    case DEV_CMD_SECTOR_START:
        return 0;
    case DEV_CMD_SECTOR_COUNT:
        return (int)fd->tracks * fd->heads * fd->sectors;
    case DEV_CMD_SECTOR_SIZE:
        return SECTOR_SIZE;
    default:
        panic("device command %d can't recognize!!!", cmd);
        break;
    }
}

static err_t fd_reset(floppy_t *fd)
{
    LOGK("floppy setup...\n");
    outb(FDC_DOR, DOR_IRQ | (fd->drive) | (0x10 << fd->drive));
    // assert(fd_wait(fd) == EOK);
    return EOK;
}

static err_t fd_setup(floppy_t *fd)
{
    fd_specify(fd);
    fd_recalibrate(fd);
    return EOK;
}

void floppy_init()
{
    LOGK("floppy disk init...\n");

    floppy_t *fd = &floppy;
    fd->type = fd_type();
    if (!fd->type) // 主驱动器不存在
    {
        LOGK("floppy drive not exists...\n");
        return;
    }

    strcpy(fd->name, "fda");
    if (fd->type != FDT_144M)
    {
        LOGK("floppy %s type %d not supported!!!\n", fd->name, fd->type);
        return;
    }

    LOGK("floppy %s 1.44M init...\n", fd->name);

    // 设置中断处理函数，以及中断屏蔽字
    set_interrupt_handler(IRQ_FLOPPY, fd_handler);
    set_interrupt_mask(IRQ_FLOPPY, true);

    lock_init(&fd->lock);

    fd->waiter = NULL;

    fd->dor = (DOR_IRQ | DOR_NORMAL);

    fd->buf = (u8 *)DMA_BUF_ADDR;

    fd->tracks = 80;
    fd->heads = 2;
    fd->sectors = 18;
    fd->gap3 = 0x1B;

    fd->drive = 0;
    fd->track = 0xFF;

    if (fd_setup(fd) < EOK)
        return;

    device_install(
        DEV_BLOCK, DEV_FLOPPY, fd, fd->name, 0,
        fd_ioctl, fd_read, fd_write);
}
