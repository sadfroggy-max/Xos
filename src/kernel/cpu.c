#include <onix/cpu.h>

// 检测是否支持 cpuid 指令
bool cpu_check_cpuid()
{
    bool is_supported;
    asm volatile(
        "pushfl \n"                // 保存 eflags
        "pushfl \n"                // 复制 eflags
        "xor $0x00200000, (%%esp)\n" // 翻转 ID 位
        "popfl \n"                 // 将修改后的值写回 eflags

        "pushfl \n"                // 再次获取 eflags
        "pop %%eax \n"             // 将 eflags 存入 eax
        "xor (%%esp), %%eax \n"    // 比较前后的 ID 位
        "and $0x00200000, %%eax \n" // 提取 ID 位
        "shr $21, %%eax \n"        // 右移 21 位，判断是否支持

        "popfl \n"                 // 恢复原始 eflags
        : "=a"(is_supported));     // 将结果存入 is_supported
    return is_supported;
}

// 获取 CPU 供应商 ID 字符串
void cpu_vendor_id(cpu_vendor_t *vendor_info)
{
    u32 *info = (u32 *)vendor_info->info; // 提取 info 字段的指针
    asm volatile(
        "cpuid \n"
        : "=a"(info[0]),  // EAX 寄存器值
          "=b"(info[1]),  // EBX 寄存器值
          "=d"(info[2]),  // EDX 寄存器值
          "=c"(info[3])   // ECX 寄存器值
        : "a"(0));        // CPUID 功能号 0
    vendor_info->info[12] = 0; // 确保字符串末尾的终止符
}

// 获取 CPU 版本信息
void cpu_version(cpu_version_t *version_info)
{
    u32 *info = (u32 *)version_info;
    asm volatile(
        "cpuid \n"
        : "=a"(info[0]),  // EAX 寄存器值
          "=b"(info[1]),  // EBX 寄存器值
          "=c"(info[2]),  // ECX 寄存器值
          "=d"(info[3])   // EDX 寄存器值
        : "a"(1));        // CPUID 功能号 1
}

