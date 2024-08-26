#include <onix/fpu.h>
#include <onix/task.h>
#include <onix/cpu.h>
#include <onix/interrupt.h>
#include <onix/arena.h>
#include <onix/debug.h>
#include <onix/assert.h>

#define LOGK(fmt, args...) DEBUGK(fmt, ##args)

task_t *last_fpu_task = NULL;

// 验证系统是否支持 FPU
bool fpu_check()
{
    cpu_version_t version;
    cpu_version(&version);
    if (!version.FPU)
        return false;

    u32 status_word = 0x55AA;
    u32 result;
    asm volatile(
        "movl %%cr0, %%edx\n" // 读取 cr0 寄存器
        "andl %%ecx, %%edx\n" // 清除 EM 和 TS 标志位以启用 FPU
        "movl %%edx, %%cr0\n" // 更新 cr0 寄存器

        "fninit\n"    // 初始化 FPU
        "fnstsw %1\n" // 保存状态字到 status_word

        "movl %1, %%eax\n" // 将状态字载入 eax
        : "=a"(result)     // 将 eax 保存到 result
        : "m"(status_word), "c"(~(CR0_EM | CR0_TS)));
    return result == 0; // 如果状态字为 0，则表示 FPU 可用
}

// 获取 cr0 寄存器的值
u32 get_cr0()
{
    u32 cr0;
    asm volatile("movl %%cr0, %0" : "=r"(cr0)); // 将 cr0 的值保存到 cr0 变量中
    return cr0;
}

// 设置 cr0 寄存器
void set_cr0(u32 cr0)
{
    asm volatile("movl %0, %%cr0" ::"r"(cr0)); // 将 cr0 的值写入寄存器
}

// 激活 FPU
void fpu_enable(task_t *task)
{
    // LOGK("激活 FPU...\n");

    set_cr0(get_cr0() & ~(CR0_EM | CR0_TS));

    // 如果当前任务和上次使用 FPU 的任务相同，则无需恢复浮点环境
    if (last_fpu_task == task)
        return;

    // 如果上一个任务使用了 FPU，则保存它的浮点环境
    if (last_fpu_task && (last_fpu_task->flags & TASK_FPU_ENABLED))
    {
        assert(last_fpu_task->fpu);
        asm volatile("fnsave (%%eax)" ::"a"(last_fpu_task->fpu));
        last_fpu_task->flags &= ~TASK_FPU_ENABLED;
    }

    last_fpu_task = task;

    // 如果当前任务已经有 FPU 环境，则恢复它
    if (task->fpu)
    {
        asm volatile("frstor (%%eax)" ::"a"(task->fpu));
    }
    else
    {
        // 否则，初始化一个新的浮点环境
        asm volatile(
            "fnclex\n"
            "fninit\n");

        LOGK("为任务 0x%p 创建 FPU 状态\n", task);
        task->fpu = (fpu_t *)kmalloc(sizeof(fpu_t));
        task->flags |= (TASK_FPU_ENABLED | TASK_FPU_USED);
    }
}

// 禁用 FPU
void fpu_disable(task_t *task)
{
    set_cr0(get_cr0() | (CR0_EM | CR0_TS));
}

// FPU 异常处理函数
void fpu_handler(int vector)
{
    // LOGK("处理 FPU 异常...\n");
    assert(vector == INTR_NM);
    task_t *task = running_task();
    assert(task->uid);

    fpu_enable(task);
}

// 初始化 FPU
void fpu_init()
{
    LOGK("初始化 FPU...\n");

    bool has_fpu = fpu_check();
    last_fpu_task = NULL;
    assert(has_fpu);

    if (has_fpu)
    {
        // 设置 FPU 异常处理程序
        set_exception_handler(INTR_NM, fpu_handler);
        // 配置 CR0 寄存器
        set_cr0(get_cr0() | CR0_EM | CR0_TS | CR0_NE);
    }
    else
    {
        LOGK("系统不支持 FPU...\n");
    }
}
