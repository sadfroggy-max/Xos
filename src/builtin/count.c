#include "../include/xos/stdio.h"
#include "../include/xos/syscall.h"

int main(int argc, char const *argv[])
{
    int counter = 1;
    while (counter)
    {
        printf("hello onix %d\a\n", counter++);
        sleep(1000);
    }
    return 0;
}
