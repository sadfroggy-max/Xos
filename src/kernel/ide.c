#include <onix/ide.h>
#include <onix/io.h>
#include <onix/printk.h>
#include <onix/stdio.h>
#include <onix/memory.h>
#include <onix/interrupt.h>
#include <onix/task.h>
#include <onix/string.h>
#include <onix/net/types.h>
#include <onix/assert.h>
#include <onix/debug.h>
#include <onix/device.h>
#include <onix/timer.h>
#include <onix/errno.h>
#include <onix/pci.h>

#define LOGK(fmt, ...) DEBUGK(fmt, ##__VA_ARGS__)

#define IDE_TIMEOUT 60000

// IDE 寄存器基址定义
#define IDE_IOBASE_PRIMARY 0x1F0
#define IDE_IOBASE_SECONDARY 0x170

// IDE 寄存器偏移量定义
#define IDE_DATA 0x0000
#define IDE_ERR 0x0001
#define IDE_FEATURE 0x0001
#define IDE_SECTOR 0x0002
#define IDE_LBA_LOW 0x0003
#define IDE_CHS_SECTOR 0x0003
#define IDE_LBA_MID 0x0004
#define IDE_CHS_CYL 0x0004
#define IDE_LBA_HIGH 0x0005
#define IDE_CHS_CYH 0x0005
#define IDE_HDDEVSEL 0x0006
#define IDE_STATUS 0x0007
#define IDE_COMMAND 0x0007
#define IDE_ALT_STATUS 0x0206
#define IDE_CONTROL 0x0206
#define IDE_DEVCTRL 0x0206

// IDE 命令定义
#define IDE_CMD_READ 0x20
#define IDE_CMD_WRITE 0x30
#define IDE_CMD_IDENTIFY 0xEC
#define IDE_CMD_DIAGNOSTIC 0x90
#define IDE_CMD_READ_UDMA 0xC8
#define IDE_CMD_WRITE_UDMA 0xCA
#define IDE_CMD_PIDENTIFY 0xA1
#define IDE_CMD_PACKET 0xA0

// ATAPI 命令定义
#define IDE_ATAPI_CMD_REQUESTSENSE 0x03
#define IDE_ATAPI_CMD_READCAPICITY 0x25
#define IDE_ATAPI_CMD_READ10 0x28

#define IDE_ATAPI_FEATURE_PIO 0
#define IDE_ATAPI_FEATURE_DMA 1

// IDE 控制器状态寄存器
#define IDE_SR_NULL 0x00
#define IDE_SR_ERR 0x01
#define IDE_SR_IDX 0x02
#define IDE_SR_CORR 0x04
#define IDE_SR_DRQ 0x08
#define IDE_SR_DSC 0x10
#define IDE_SR_DWF 0x20
#define IDE_SR_DRDY 0x40
#define IDE_SR_BSY 0x80

// IDE 控制寄存器定义
#define IDE_CTRL_HD15 0x00
#define IDE_CTRL_SRST 0x04
#define IDE_CTRL_NIEN 0x02

// IDE 错误寄存器定义
#define IDE_ER_AMNF 0x01
#define IDE_ER_TK0NF 0x02
#define IDE_ER_ABRT 0x04
#define IDE_ER_MCR 0x08
#define IDE_ER_IDNF 0x10
#define IDE_ER_MC 0x20
#define IDE_ER_UNC 0x40
#define IDE_ER_BBK 0x80

#define IDE_LBA_MASTER 0b11100000
#define IDE_LBA_SLAVE 0b11110000
#define IDE_SEL_MASK 0b10110000

#define IDE_INTERFACE_UNKNOWN 0
#define IDE_INTERFACE_ATA 1
#define IDE_INTERFACE_ATAPI 2

// 总线主控寄存器偏移量
#define BM_COMMAND_REG 0
#define BM_STATUS_REG 2
#define BM_PRD_ADDR 4

// 总线主控命令寄存器定义
#define BM_CR_STOP 0x00
#define BM_CR_START 0x01
#define BM_CR_WRITE 0x00
#define BM_CR_READ 0x08

// 总线主控状态寄存器定义
#define BM_SR_ACT 0x01
#define BM_SR_ERR 0x02
#define BM_SR_INT 0x04
#define BM_SR_DRV0 0x20
#define BM_SR_DRV1 0x40
#define BM_SR_SIMPLEX 0x80

#define IDE_LAST_PRD 0x80000000

// 分区文件系统类型定义
typedef enum PART_FS {
    PART_FS_FAT12 = 1,
    PART_FS_EXTENDED = 5,
    PART_FS_MINIX = 0x80,
    PART_FS_LINUX = 0x83,
} PART_FS;

typedef struct ide_params_t {
    u16 config;
    u16 cylinders;
    u16 reserved1;
    u16 heads;
    u16 reserved2[2];
    u16 sectors;
    u16 reserved3[3];
    u8 serial[20];
    u16 reserved4[3];
    u8 firmware[8];
    u8 model[40];
    u8 drq_sectors;
    u8 reserved5[3];
    u16 capabilities;
    u16 reserved6[10];
    u32 total_lba;
    u16 reserved7;
    u16 mdma_mode;
    u8 reserved8;
    u8 pio_mode;
    u16 reserved9[15];
    u16 major_version;
    u16 minor_version;
    u16 command_sets[6];
    u16 reserved10[31];
    u16 support_settings;
    u16 enable_settings;
    u16 reserved11[101];
    u16 transport_major;
    u16 transport_minor;
    u16 reserved12[31];
    u16 integrity;
} _packed ide_params_t;

ide_ctrl_t controllers[IDE_CTRL_NR];

static int ide_reset_controller(ide_ctrl_t *ctrl);

static void ide_handler(int vector) {
    send_eoi(vector);

    ide_ctrl_t *ctrl = &controllers[vector - IRQ_HARDDISK - 0x20];
    u8 state = inb(ctrl->iobase + IDE_STATUS);
    LOGK("harddisk interrupt vector %d state 0x%x\n", vector, state);

    if (ctrl->waiter) {
        task_unblock(ctrl->waiter, EOK);
        ctrl->waiter = NULL;
    }
}

static void ide_error(ide_ctrl_t *ctrl) {
    u8 error = inb(ctrl->iobase + IDE_ERR);
    if (error) {
        if (error & IDE_ER_BBK) LOGK("bad block\n");
        if (error & IDE_ER_UNC) LOGK("uncorrectable data\n");
        if (error & IDE_ER_MC) LOGK("media change\n");
        if (error & IDE_ER_IDNF) LOGK("id not found\n");
        if (error & IDE_ER_MCR) LOGK("media change requested\n");
        if (error & IDE_ER_ABRT) LOGK("abort\n");
        if (error & IDE_ER_TK0NF) LOGK("track 0 not found\n");
        if (error & IDE_ER_AMNF) LOGK("address mark not found\n");
    }
}

// 硬盘延迟
static void ide_delay()
{
    task_sleep(25);
}

static err_t ide_busy_wait(ide_ctrl_t *ctrl, u8 mask, int timeout_ms)
{
    int expiration = timer_expire_jiffies(timeout_ms);
    while (true)
    {
        // 超时检查
        if (timeout_ms > 0 && timer_is_expires(expiration))
        {
            return -ETIME;
        }

        // 从备用状态寄存器中读取状态
        u8 status = inb(ctrl->iobase + IDE_ALT_STATUS);
        if (status & IDE_SR_ERR) // 检测到错误
        {
            ide_error(ctrl);
            ide_reset_controller(ctrl);
            return -EIO;
        }
        if (status & IDE_SR_BSY) // 控制器忙
        {
            ide_delay();
            continue;
        }
        if ((status & mask) == mask) // 期望状态完成
            return EOK;
    }
}

// 重置硬盘控制器
static err_t ide_reset_controller(ide_ctrl_t *ctrl)
{
    outb(ctrl->iobase + IDE_CONTROL, IDE_CTRL_SRST);
    ide_delay();
    outb(ctrl->iobase + IDE_CONTROL, ctrl->control);
    return ide_busy_wait(ctrl, IDE_SR_NULL, IDE_TIMEOUT);
}

// 选择磁盘
static void ide_select_drive(ide_disk_t *disk)
{
    outb(disk->ctrl->iobase + IDE_HDDEVSEL, disk->selector);
    disk->ctrl->active = disk;
}

// 选择扇区
static void ide_select_sector(ide_disk_t *disk, u32 lba, u8 count)
{
    // 写入功能，可省略
    outb(disk->ctrl->iobase + IDE_FEATURE, 0);

    // 设置扇区数量
    outb(disk->ctrl->iobase + IDE_SECTOR, count);

    // LBA 低字节
    outb(disk->ctrl->iobase + IDE_LBA_LOW, lba & 0xff);
    // LBA 中字节
    outb(disk->ctrl->iobase + IDE_LBA_MID, (lba >> 8) & 0xff);
    // LBA 高字节
    outb(disk->ctrl->iobase + IDE_LBA_HIGH, (lba >> 16) & 0xff);

    // LBA 最高四位 + 磁盘选择
    outb(disk->ctrl->iobase + IDE_HDDEVSEL, ((lba >> 24) & 0xf) | disk->selector);
    disk->ctrl->active = disk;
}

// 从磁盘读取一个扇区到 buf
static void ide_pio_read_sector(ide_disk_t *disk, u16 *buf)
{
    for (size_t i = 0; i < (disk->sector_size / 2); i++)
    {
        buf[i] = inw(disk->ctrl->iobase + IDE_DATA);
    }
}

// 从 buf 写入一个扇区到磁盘
static void ide_pio_write_sector(ide_disk_t *disk, u16 *buf)
{
    for (size_t i = 0; i < (disk->sector_size / 2); i++)
    {
        outw(disk->ctrl->iobase + IDE_DATA, buf[i]);
    }
}

// 磁盘控制
int ide_pio_ioctl(ide_disk_t *disk, int cmd, void *args, int flags)
{
    switch (cmd)
    {
    case DEV_CMD_SECTOR_START:
        return 0;
    case DEV_CMD_SECTOR_COUNT:
        return disk->total_lba;
    case DEV_CMD_SECTOR_SIZE:
        return disk->sector_size;
    default:
        panic("Unrecognized device command %d!!!", cmd);
        break;
    }
}

// PIO 方式读取磁盘
int ide_pio_read(ide_disk_t *disk, void *buf, u8 count, idx_t lba)
{
    assert(count > 0);
    assert(!get_interrupt_state()); // 异步调用，不允许中断

    ide_ctrl_t *ctrl = disk->ctrl;

    lock_acquire(&ctrl->lock);

    int result = -EIO;

    // 选择磁盘
    ide_select_drive(disk);

    // 等待设备就绪
    if ((result = ide_busy_wait(ctrl, IDE_SR_DRDY, IDE_TIMEOUT)) < EOK)
        goto cleanup;

    // 选择扇区
    ide_select_sector(disk, lba, count);

    // 发送读取命令
    outb(ctrl->iobase + IDE_COMMAND, IDE_CMD_READ);

    task_t *current_task = running_task();
    for (size_t i = 0; i < count; i++)
    {
        // 阻塞任务，等待中断
        ctrl->waiter = current_task;
        if ((result = task_block(current_task, NULL, TASK_BLOCKED, IDE_TIMEOUT)) < EOK)
            goto cleanup;

        if ((result = ide_busy_wait(ctrl, IDE_SR_DRQ, IDE_TIMEOUT)) < EOK)
            goto cleanup;

        u32 offset = ((u32)buf + i * SECTOR_SIZE);
        ide_pio_read_sector(disk, (u16 *)offset);
    }
    result = EOK;

cleanup:
    lock_release(&ctrl->lock);
    return result;
}

// PIO 方式写磁盘
int ide_pio_write(ide_disk_t *disk, void *buf, u8 count, idx_t lba)
{
    assert(count > 0);
    assert(!get_interrupt_state()); // 异步调用，不允许中断

    ide_ctrl_t *ctrl = disk->ctrl;

    lock_acquire(&ctrl->lock);

    int result = EOK;

    LOGK("Writing to lba 0x%x\n", lba);

    // 选择磁盘
    ide_select_drive(disk);

    // 等待设备就绪
    if ((result = ide_busy_wait(ctrl, IDE_SR_DRDY, IDE_TIMEOUT)) < EOK)
        goto cleanup;

    // 选择扇区
    ide_select_sector(disk, lba, count);

    // 发送写入命令
    outb(ctrl->iobase + IDE_COMMAND, IDE_CMD_WRITE);
    task_t *current_task = running_task();
    for (size_t i = 0; i < count; i++)
    {
        u32 offset = ((u32)buf + i * SECTOR_SIZE);
        ide_pio_write_sector(disk, (u16 *)offset);

        // 阻塞任务，等待中断
        ctrl->waiter = current_task;
        if ((result = task_block(current_task, NULL, TASK_BLOCKED, IDE_TIMEOUT)) < EOK)
            goto cleanup;

        if ((result = ide_busy_wait(ctrl, IDE_SR_NULL, IDE_TIMEOUT)) < EOK)
            goto cleanup;
    }
    result = EOK;

cleanup:
    lock_release(&ctrl->lock);
    return result;
}

// 分区控制
int ide_pio_part_ioctl(ide_part_t *part, int cmd, void *args, int flags)
{
    switch (cmd)
    {
    case DEV_CMD_SECTOR_START:
        return part->start;
    case DEV_CMD_SECTOR_COUNT:
        return part->count;
    case DEV_CMD_SECTOR_SIZE:
        return part->disk->sector_size;
    default:
        panic("Unrecognized device command %d!!!", cmd);
        break;
    }
}


// 读取分区
int ide_pio_part_read(ide_part_t *part, void *buffer, u8 sector_count, idx_t lba_offset)
{
    return ide_pio_read(part->disk, buffer, sector_count, part->start + lba_offset);
}

// 写入分区
int ide_pio_part_write(ide_part_t *part, void *buffer, u8 sector_count, idx_t lba_offset)
{
    return ide_pio_write(part->disk, buffer, sector_count, part->start + lba_offset);
}

static void ide_configure_dma(ide_ctrl_t *ctrl, int command, char *buffer, u32 length)
{
    // 确保缓冲区不跨页
    assert(((u32)buffer + length) <= ((u32)buffer & (~0xfff)) + PAGE_SIZE);

    // 配置 PRDT
    ctrl->prd.addr = get_paddr((u32)buffer);
    ctrl->prd.len = length | IDE_LAST_PRD;

    // 设置 PRD 地址
    outl(ctrl->bmbase + BM_PRD_ADDR, (u32)&ctrl->prd);

    // 设置 DMA 读写命令
    outb(ctrl->bmbase + BM_COMMAND_REG, command | BM_CR_STOP);

    // 启用中断和错误标志
    outb(ctrl->bmbase + BM_STATUS_REG, inb(ctrl->bmbase + BM_STATUS_REG) | BM_SR_INT | BM_SR_ERR);
}

// 启动 DMA
static void ide_begin_dma(ide_ctrl_t *ctrl)
{
    outb(ctrl->bmbase + BM_COMMAND_REG, inb(ctrl->bmbase + BM_COMMAND_REG) | BM_CR_START);
}

static err_t ide_terminate_dma(ide_ctrl_t *ctrl)
{
    // 停止 DMA 传输
    outb(ctrl->bmbase + BM_COMMAND_REG, inb(ctrl->bmbase + BM_COMMAND_REG) & (~BM_CR_START));

    // 读取 DMA 状态
    u8 status = inb(ctrl->bmbase + BM_STATUS_REG);

    // 清除中断和错误标志
    outb(ctrl->bmbase + BM_STATUS_REG, status | BM_SR_INT | BM_SR_ERR);

    // 检查是否发生错误
    if (status & BM_SR_ERR)
    {
        LOGK("IDE DMA error detected %02X\n", status);
        return -EIO;
    }
    return EOK;
}

err_t ide_udma_read(ide_disk_t *disk, void *buffer, u8 sector_count, idx_t lba_offset)
{
    LOGK("Performing DMA read at LBA 0x%x\n", lba_offset);

    int result = 0;
    ide_ctrl_t *ctrl = disk->ctrl;

    lock_acquire(&ctrl->lock);

    // 配置 DMA 读取
    ide_configure_dma(ctrl, BM_CR_READ, buffer, sector_count * SECTOR_SIZE);

    // 选择扇区
    ide_select_sector(disk, lba_offset, sector_count);

    // 发出 UDMA 读取命令
    outb(disk->ctrl->iobase + IDE_COMMAND, IDE_CMD_READ_UDMA);

    ide_begin_dma(ctrl);

    disk->ctrl->waiter = running_task();
    if ((result = task_block(disk->ctrl->waiter, NULL, TASK_BLOCKED, IDE_TIMEOUT)) < EOK)
    {
        LOGK("IDE DMA error occurred!!! %d\n", result);
    }

    assert(ide_terminate_dma(ctrl) == EOK);

    lock_release(&ctrl->lock);
    return result;
}

err_t ide_udma_write(ide_disk_t *disk, void *buffer, u8 sector_count, idx_t lba_offset)
{
    LOGK("Performing DMA write at LBA 0x%x\n", lba_offset);
    int result = EOK;
    ide_ctrl_t *ctrl = disk->ctrl;

    lock_acquire(&ctrl->lock);

    // 配置 DMA 写入
    ide_configure_dma(ctrl, BM_CR_WRITE, buffer, sector_count * SECTOR_SIZE);

    // 选择扇区
    ide_select_sector(disk, lba_offset, sector_count);

    // 发出 UDMA 写入命令
    outb(disk->ctrl->iobase + IDE_COMMAND, IDE_CMD_WRITE_UDMA);

    ide_begin_dma(ctrl);

    disk->ctrl->waiter = running_task();
    if ((result = task_block(disk->ctrl->waiter, NULL, TASK_BLOCKED, IDE_TIMEOUT)) < EOK)
    {
        LOGK("IDE DMA error occurred!!! %d\n", result);
    }

    assert(ide_terminate_dma(ctrl) == EOK);

    lock_release(&ctrl->lock);
    return result;
}

// 读取 ATAPI 数据包
static int ide_atapi_packet_read_pio(ide_disk_t *disk, u8 *packet, int packet_length, void *buffer, size_t buffer_size)
{
    // 等待磁盘准备好
    // ide_busy_wait(disk->ctrl, IDE_SR_NULL);

    lock_acquire(&disk->ctrl->lock);

    // 配置寄存器
    outb(disk->ctrl->iobase + IDE_FEATURE, IDE_ATAPI_FEATURE_PIO);
    outb(disk->ctrl->iobase + IDE_SECTOR, 0);
    outb(disk->ctrl->iobase + IDE_LBA_LOW, 0);
    outb(disk->ctrl->iobase + IDE_LBA_MID, (buffer_size & 0xFF));
    outb(disk->ctrl->iobase + IDE_LBA_HIGH, (buffer_size >> 8) & 0xFF);
    outb(disk->ctrl->iobase + IDE_HDDEVSEL, disk->selector & 0x10);

    // 发送 ATAPI 命令
    outb(disk->ctrl->iobase + IDE_COMMAND, IDE_CMD_PACKET);

    int ret = EOF;
    // 等待设备准备好
    if ((ret = ide_busy_wait(disk->ctrl, IDE_SR_DRDY, IDE_TIMEOUT)) < 0)
        goto exit;

    // 写入 packet 内容
    u16 *packet_ptr = (u16 *)packet;
    for (size_t i = 0; i < packet_length / 2; i++)
    {
        outw(disk->ctrl->iobase + IDE_DATA, packet_ptr[i]);
    }

    // 阻塞等待磁盘完成数据写入
    task_t *current_task = running_task();
    disk->ctrl->waiter = current_task;
    if ((ret = task_block(current_task, NULL, TASK_BLOCKED, IDE_TIMEOUT)) < 0)
        goto exit;

    int remaining_size = buffer_size;

    packet_ptr = (u16 *)buffer;
    int buffer_index = 0;
    while (remaining_size > 0)
    {
        // 等待磁盘准备好
        if ((ret = ide_busy_wait(disk->ctrl, IDE_SR_NULL, IDE_TIMEOUT)) < 0)
            goto exit;

        // 读取可用字节数
        int byte_count = inb(disk->ctrl->iobase + IDE_LBA_HIGH) << 8;
        byte_count |= inb(disk->ctrl->iobase + IDE_LBA_MID);

        assert(byte_count >= 0);

        if (byte_count == 0)
            break;
        assert(remaining_size <= byte_count);
        // 读取字节数据
        for (size_t i = 0; i < byte_count / 2; i++)
        {
            packet_ptr[buffer_index++] = inw(disk->ctrl->iobase + IDE_DATA);
        }
        remaining_size -= byte_count;
    }
    ret = buffer_size - remaining_size;

exit:
    lock_release(&disk->ctrl->lock);
    return ret;
}


sstatic int ide_atapi_packet_read_dma(ide_disk_t *disk, u8 *pkt, int pktlen, void *buf, size_t bufsize)
{
    // 等待磁盘空闲
    // ide_busy_wait(disk->ctrl, IDE_SR_NULL);

    // 加锁以保证线程安全
    lock_acquire(&disk->ctrl->lock);

    // 初始化 DMA 设置
    ide_setup_dma(disk->ctrl, BM_CR_READ, buf, bufsize);

    // 配置必要的寄存器
    outb(disk->ctrl->iobase + IDE_FEATURE, IDE_ATAPI_FEATURE_DMA);
    outb(disk->ctrl->iobase + IDE_SECTOR, 0);
    outb(disk->ctrl->iobase + IDE_LBA_LOW, 0);
    outb(disk->ctrl->iobase + IDE_LBA_MID, 0);
    outb(disk->ctrl->iobase + IDE_LBA_HIGH, 0);
    outb(disk->ctrl->iobase + IDE_HDDEVSEL, disk->selector & 0x10);

    // 发出 ATAPI 命令
    outb(disk->ctrl->iobase + IDE_COMMAND, IDE_CMD_PACKET);

    int result = EOF;
    // 检查磁盘状态，确保设备就绪
    if ((result = ide_busy_wait(disk->ctrl, IDE_SR_DRDY, IDE_TIMEOUT)) < 0)
        goto error;

    // 将包数据写入数据寄存器
    u16 *data = (u16 *)pkt;
    for (size_t i = 0; i < pktlen / 2; i++)
    {
        outw(disk->ctrl->iobase + IDE_DATA, data[i]);
    }

    // 启动 DMA 操作
    ide_start_dma(disk->ctrl);

    // 等待 DMA 操作完成
    task_t *task = running_task();
    disk->ctrl->waiter = task;
    if ((result = task_block(task, NULL, TASK_BLOCKED, IDE_TIMEOUT)) < 0)
        goto error;

    // 确保 DMA 正常停止并无错误
    assert(ide_stop_dma(disk->ctrl) == EOK);
    result = bufsize;

error:
    // 释放锁
    lock_release(&disk->ctrl->lock);
    return result;
}

// 读取 ATAPI 容量
static int ide_atapi_read_capacity(ide_disk_t *disk)
{
    u8 packet[12];
    u32 buffer[2];
    u32 block_count;
    u32 block_size;

    memset(packet, 0, sizeof(packet));
    packet[0] = IDE_ATAPI_CMD_READCAPICITY;

    // 根据控制器类型选择读取函数
    int (*read_func)() = ide_atapi_packet_read_pio;
    if (disk->ctrl->iotype == IDE_TYPE_UDMA)
        read_func = ide_atapi_packet_read_dma;

    int result = read_func(disk, packet, sizeof(packet), buffer, sizeof(buffer));
    if (result < 0)
        return result;
    if (result != sizeof(buffer))
        return -EIO;

    block_count = ntohl(buffer[0]);
    block_size = ntohl(buffer[1]);

    if (block_size != disk->sector_size)
    {
        LOGK("CD block size warning %d\n", block_size);
        return 0;
    }
    return block_count;
}

// ATAPI 设备读
static int ide_atapi_read(ide_disk_t *disk, void *buf, int count, idx_t lba)
{
    u8 packet[12];
    if (count > 0xffff)
        return -EIO;

    memset(packet, 0, sizeof(packet));

    packet[0] = IDE_ATAPI_CMD_READ10;
    packet[2] = lba >> 24;
    packet[3] = (lba >> 16) & 0xFF;
    packet[4] = (lba >> 8) & 0xFF;
    packet[5] = lba & 0xFF;
    packet[7] = (count >> 8) & 0xFF;
    packet[8] = count & 0xFF;

    int (*read_func)() = ide_atapi_packet_read_pio;
    if (disk->ctrl->iotype == IDE_TYPE_UDMA)
        read_func = ide_atapi_packet_read_dma;

    return read_func(disk, packet, sizeof(packet), buf, count * disk->sector_size);
}

// 设备探测
static err_t ide_probe_device(ide_disk_t *disk)
{
    outb(disk->ctrl->iobase + IDE_HDDEVSEL, disk->selector & IDE_SEL_MASK);
    ide_delay();

    outb(disk->ctrl->iobase + IDE_SECTOR, 0x55);
    outb(disk->ctrl->iobase + IDE_CHS_SECTOR, 0xAA);

    outb(disk->ctrl->iobase + IDE_SECTOR, 0xAA);
    outb(disk->ctrl->iobase + IDE_CHS_SECTOR, 0x55);

    outb(disk->ctrl->iobase + IDE_SECTOR, 0x55);
    outb(disk->ctrl->iobase + IDE_CHS_SECTOR, 0xAA);

    u8 sector_count = inb(disk->ctrl->iobase + IDE_SECTOR);
    u8 sector_index = inb(disk->ctrl->iobase + IDE_CHS_SECTOR);

    if (sector_count == 0x55 && sector_index == 0xAA)
        return EOK;
    return -EIO;
}

// 检测设备类型
static int ide_interface_type(ide_disk_t *disk)
{
    outb(disk->ctrl->iobase + IDE_COMMAND, IDE_CMD_DIAGNOSTIC);
    if (ide_busy_wait(disk->ctrl, IDE_SR_NULL, IDE_TIMEOUT) < EOK)
        return IDE_INTERFACE_UNKNOWN;

    outb(disk->ctrl->iobase + IDE_HDDEVSEL, disk->selector & IDE_SEL_MASK);
    ide_delay();

    u8 sector_count = inb(disk->ctrl->iobase + IDE_SECTOR);
    u8 sector_index = inb(disk->ctrl->iobase + IDE_LBA_LOW);
    if (sector_count != 1 || sector_index != 1)
        return IDE_INTERFACE_UNKNOWN;

    u8 cylinder_low = inb(disk->ctrl->iobase + IDE_CHS_CYL);
    u8 cylinder_high = inb(disk->ctrl->iobase + IDE_CHS_CYH);
    u8 state = inb(disk->ctrl->iobase + IDE_STATUS);

    if (cylinder_low == 0x14 && cylinder_high == 0xeb)
        return IDE_INTERFACE_ATAPI;

    if (cylinder_low == 0 && cylinder_high == 0 && state != 0)
        return IDE_INTERFACE_ATA;

    return IDE_INTERFACE_UNKNOWN;
}

// 修正字符串顺序
static void ide_fixstrings(char *buf, u32 len)
{
    for (size_t i = 0; i < len; i += 2)
    {
        register char temp = buf[i];
        buf[i] = buf[i + 1];
        buf[i + 1] = temp;
    }
    buf[len - 1] = '\0';
}

// 识别 IDE 设备
static err_t ide_identify(ide_disk_t *disk, u16 *buf)
{
    LOGK("Identifying disk %s...\n", disk->name);
    lock_acquire(&disk->ctrl->lock);
    ide_select_drive(disk);

    // ide_select_sector(disk, 0, 0);
    u8 command = IDE_CMD_IDENTIFY;
    if (disk->interface == IDE_INTERFACE_ATAPI)
    {
        command = IDE_CMD_PIDENTIFY;
    }

    outb(disk->ctrl->iobase + IDE_COMMAND, command);

    int status = EOF;
    if ((status = ide_busy_wait(disk->ctrl, IDE_SR_NULL, IDE_TIMEOUT)) < EOK)
        goto end;

    ide_params_t *params = (ide_params_t *)buf;

    ide_pio_read_sector(disk, buf);

    ide_fixstrings(params->serial, sizeof(params->serial));
    LOGK("Disk %s serial number %s\n", disk->name, params->serial);

    ide_fixstrings(params->firmware, sizeof(params->firmware));
    LOGK("Disk %s firmware version %s\n", disk->name, params->firmware);

    ide_fixstrings(params->model, sizeof(params->model));
    LOGK("Disk %s model number %s\n", disk->name, params->model);

    if (disk->interface == IDE_INTERFACE_ATAPI)
    {
        status = EOK;
        goto end;
    }

    if (params->total_lba == 0)
    {
        status = -EIO;
        goto end;
    }
    LOGK("Disk %s total lba %d\n", disk->name, params->total_lba);

    disk->total_lba = params->total_lba;
    disk->cylinders = params->cylinders;
    disk->heads = params->heads
    disk->sectors = params->sectors;
    ret = EOK;

rollback:
    lock_release(&disk->ctrl->lock);
    return ret;
}

static void ide_part_init(ide_disk_t *disk, u16 *buf)
{
    // 检查磁盘是否有效
    if (disk->total_lba == 0)
        return;

    // 读取MBR（主引导记录）
    ide_pio_read(disk, buf, 1, 0);

    // 解析MBR内容
    boot_sector_t *boot = (boot_sector_t *)buf;

    for (size_t i = 0; i < IDE_PART_NR; i++)
    {
        part_entry_t *entry = &boot->entry[i];
        ide_part_t *part = &disk->parts[i];

        if (entry->count == 0)
            continue;

        snprintf(part->name, sizeof(part->name), "%s%d", disk->name, i + 1);

        LOGK("Partition %s\n", part->name);
        LOGK("    Bootable flag: %d\n", entry->bootable);
        LOGK("    Start sector: %d\n", entry->start);
        LOGK("    Sector count: %d\n", entry->count);
        LOGK("    Filesystem ID: 0x%x\n", entry->system);

        part->disk = disk;
        part->count = entry->count;
        part->system = entry->system;
        part->start = entry->start;

        if (entry->system == PART_FS_EXTENDED)
        {
            LOGK("Extended partitions are not supported.\n");

            boot_sector_t *eboot = (boot_sector_t *)(buf + SECTOR_SIZE);
            ide_pio_read(disk, (void *)eboot, 1, entry->start);

            for (size_t j = 0; j < IDE_PART_NR; j++)
            {
                part_entry_t *eentry = &eboot->entry[j];
                if (eentry->count == 0)
                    continue;

                LOGK("Extended partition %d of partition %d\n", j, i);
                LOGK("    Bootable flag: %d\n", eentry->bootable);
                LOGK("    Start sector: %d\n", eentry->start);
                LOGK("    Sector count: %d\n", eentry->count);
                LOGK("    Filesystem ID: 0x%x\n", eentry->system);
            }
        }
    }
}

// IDE控制器初始化
static void ide_ctrl_init()
{
    int iotype = IDE_TYPE_PIO;
    int bmbase = 0;

    pci_device_t *device = pci_find_device_by_class(PCI_CLASS_STORAGE_IDE);
    if (device != NULL)
    {
        pci_bar_t bar;
        int ret = pci_find_bar(device, &bar, PCI_BAR_TYPE_IO);
        assert(ret == EOK);

        LOGK("PCI Device found at 0x%x with I/O base 0x%x, size %d\n",
             device, bar.iobase, bar.size);

        pci_enable_busmastering(device);

        iotype = IDE_TYPE_UDMA;
        bmbase = bar.iobase;
    }

    u16 *buf = (u16 *)alloc_kpage(1);
    for (size_t cidx = 0; cidx < IDE_CTRL_NR; cidx++)
    {
        ide_ctrl_t *ctrl = &controllers[cidx];
        snprintf(ctrl->name, sizeof(ctrl->name), "ide%u", (unsigned)cidx);
        lock_init(&ctrl->lock);
        ctrl->active = NULL;
        ctrl->waiter = NULL;
        ctrl->iotype = iotype;
        ctrl->bmbase = bmbase + cidx * 8;

        if (cidx == 1)
        {
            ctrl->iobase = IDE_IOBASE_SECONDARY;
        }
        else
        {
            ctrl->iobase = IDE_IOBASE_PRIMARY;
        }

        ctrl->control = inb(ctrl->iobase + IDE_CONTROL);

        for (size_t didx = 0; didx < IDE_DISK_NR; didx++)
        {
            ide_disk_t *disk = &ctrl->disks[didx];
            snprintf(disk->name, sizeof(disk->name), "hd%c", 'a' + cidx * 2 + didx);
            disk->ctrl = ctrl;
            disk->master = (didx == 0);
            disk->selector = (didx == 0) ? IDE_LBA_MASTER : IDE_LBA_SLAVE;

            if (ide_probe_device(disk) < 0)
            {
                LOGK("IDE device %s not found.\n", disk->name);
                continue;
            }

            disk->interface = ide_interface_type(disk);
            LOGK("IDE device %s has interface type %d.\n", disk->name, disk->interface);

            if (disk->interface == IDE_INTERFACE_UNKNOWN)
                continue;

            if (disk->interface == IDE_INTERFACE_ATA)
            {
                disk->sector_size = SECTOR_SIZE;
                if (ide_identify(disk, buf) == EOK)
                {
                    ide_part_init(disk, buf);
                }
            }
            else if (disk->interface == IDE_INTERFACE_ATAPI)
            {
                LOGK("Device %s uses the ATAPI interface.\n", disk->name);
                disk->sector_size = CD_SECTOR_SIZE;
                if (ide_identify(disk, buf) == EOK)
                {
                    disk->total_lba = ide_atapi_read_capacity(disk);
                    LOGK("Device %s has %d total LBA.\n", disk->name, disk->total_lba);
                }
            }
        }
    }
    free_kpage((u32)buf, 1);
}

static void ide_install()
{
    void *read = ide_pio_read;
    void *write = ide_pio_write;

    for (size_t cidx = 0; cidx < IDE_CTRL_NR; cidx++)
    {
        ide_ctrl_t *ctrl = &controllers[cidx];
        if (ctrl->iotype == IDE_TYPE_UDMA)
        {
            read = ide_udma_read;
            write = ide_udma_write;
        }

        for (size_t didx = 0; didx < IDE_DISK_NR; didx++)
        {
            ide_disk_t *disk = &ctrl->disks[didx];
            if (disk->total_lba == 0)
                continue;

            if (disk->interface == IDE_INTERFACE_ATA)
            {
                dev_t dev = device_install(
                    DEV_BLOCK, DEV_IDE_DISK, disk, disk->name, 0,
                    ide_pio_ioctl, read, write);

                for (size_t i = 0; i < IDE_PART_NR; i++)
                {
                    ide_part_t *part = &disk->parts[i];
                    if (part->count == 0)
                        continue;

                    device_install(
                        DEV_BLOCK, DEV_IDE_PART, part, part->name, dev,
                        ide_pio_part_ioctl, ide_pio_part_read, ide_pio_part_write);
                }
            }
            else if (disk->interface == IDE_INTERFACE_ATAPI)
            {
                device_install(
                    DEV_BLOCK, DEV_IDE_CD, disk, disk->name, 0,
                    ide_pio_ioctl, ide_atapi_read, NULL);
            }
        }
    }
}


// IDE硬盘初始化
void ide_init()
{
    LOGK("Initializing IDE...\n");

    // 设置硬盘中断处理程序，并启用中断
    set_interrupt_handler(IRQ_HARDDISK, ide_handler);
    set_interrupt_handler(IRQ_HARDDISK2, ide_handler);
    set_interrupt_mask(IRQ_HARDDISK, true);
    set_interrupt_mask(IRQ_HARDDISK2, true);
    set_interrupt_mask(IRQ_CASCADE, true);

    // 初始化IDE控制器
    ide_ctrl_init();

    // 安装IDE设备
    ide_install();
}
