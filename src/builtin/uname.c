#include "../include/xos/types.h"
#include "../include/xos/stdio.h"
#include "../include/xos/syscall.h"
#include "../include/xos/string.h"
#include "../include/xos/uname.h"

int main(int argc, char const *argv[], char const *envp[])
{
    utsname_t name;
    int ret = uname(&name);
    if (ret < 0)
    {
        printf(strerror(ret));
        return ret;
    }
    printf("%s_%s\n", name.sysname, name.version);
    return 0;
}
