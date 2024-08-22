#include "../include/xos/syscall.h"
#include "../include/xos/signal.h"
#include "../include/xos/stdlib.h"

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        return -1;
    }
    return kill(atoi(argv[1]), SIGTERM);
}