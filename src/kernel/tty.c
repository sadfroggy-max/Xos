#include <onix/tty.h>
#include <onix/device.h>
#include <onix/assert.h>
#include <onix/fifo.h>
#include <onix/task.h>
#include <onix/mutex.h>
#include <onix/debug.h>
#include <onix/errno.h>
#include <onix/syscall.h>

#define LOGK(fmt, args...) DEBUGK(fmt, ##args)

extern task_t *task_table[TASK_NR]; // 任务表
static tty_t typewriter;

// 发送 SIGINT 信号给前台进程组
int tty_intr()
{
    tty_t *tty = &typewriter;
    if (tty->pgid == 0)
    {
        return 0;
    }
    for (size_t i = 0; i < TASK_NR; i++)
    {
        task_t *task = task_table[i];
        if (!task || task->pgid != tty->pgid)
            continue;
        kill(task->pid, SIGINT);
    }
    return 0;
}

int tty_rx_notify(char *ch, bool ctrl, bool shift, bool alt)
{
    if (*ch == '\r')
    {
        *ch = '\n';
    }

    if (!ctrl)
    {
        return 0;
    }

    tty_t *tty = &typewriter;
    switch (*ch)
    {
    case 'c':
    case 'C':
        LOGK("CTRL + C Pressed\n");
        tty_intr();
        *ch = '\n';
        return 0;
    case 'l':
    case 'L':
        // 清除屏幕
        device_write(tty->wdev, "\x1b[2J\x1b[H\r", 7, 0, 0);
        *ch = '\r';
        return 0;
    default:
        return 1;
    }
}

int tty_read(tty_t *tty, char *buf, u32 count)
{
    return device_read(tty->rdev, buf, count, 0, 0);
}

int tty_write(tty_t *tty, char *buf, u32 count)
{
    return device_write(tty->wdev, buf, count, 0, 0);
}

int tty_ioctl(tty_t *tty, int cmd, void *args, int flags)
{
    if (cmd == TIOCSPGRP)
    {
        tty->pgid = (pid_t)args;
        return 0;
    }
    return -EINVAL;
}

int sys_stty()
{
    return -ENOSYS;
}

int sys_gtty()
{
    return -ENOSYS;
}

void tty_init()
{
    device_t *device = NULL;

    tty_t *tty = &typewriter;

    // 设置键盘为输入设备
    device = device_find(DEV_KEYBOARD, 0);
    tty->rdev = device->dev;

    // 设置控制台为输出设备
    device = device_find(DEV_CONSOLE, 0);
    tty->wdev = device->dev;

    device_install(DEV_CHAR, DEV_TTY, tty, "tty", 0, tty_ioctl, tty_read, tty_write);
}
