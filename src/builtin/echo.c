#ifdef XOS
#include "../include/xos/types.h"
#include "../include/xos/stdio.h"
#include "../include/xos/syscall.h"
#include "../include/xos/string.h"
#else
#include "../include/xos/stdio.h"
#include "../include/xos/string.h"
#endif

int main(int argc, char const *argv[])
{
    for (size_t i = 1; i < argc; i++)
    {
        printf(argv[i]);
        if (i < argc - 1)
        {
            printf(" ");
        }
    }
    printf("\n");
    return 0;
}
