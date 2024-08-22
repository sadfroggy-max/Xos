#include "hyc.h"

#define LOGK(fmt, args...) DEBUGK(fmt, ##args)

int sys_test()
{
    char ch;
    device_t *device;

    device = device_find(DEV_IDE_CD, 0);
    if (!device)
        return 0;

    void *buf = (void *)alloc_kpage(1);
    memset(buf, 0, PAGE_SIZE);
    device_read(device->dev, buf, 2, 0, 0);
    free_kpage((u32)buf, 1);
    return EOK;
}
