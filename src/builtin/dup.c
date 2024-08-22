#include "../include/xos/stdio.h"
#include "../include/xos/syscall.h"
#include "../include/xos/fs.h"
#include "../include/xos/errno.h"

int main(int argc, char const *argv[])
{
    char ch;
    while (true)
    {
        int n = read(STDIN_FILENO, &ch, 1);
        if (n < EOK)
        {
            break;
        }
        if (ch == '\n')
        {
            write(STDOUT_FILENO, &ch, 1);
            break;
        }
        write(STDOUT_FILENO, &ch, 1);
        write(STDOUT_FILENO, &ch, 1);
    }
    return 0;
}
