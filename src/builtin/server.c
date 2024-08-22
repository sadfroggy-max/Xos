#include "../include/xos/types.h"
#include "../include/xos/stdio.h"
#include "../include/xos/syscall.h"
#include "../include/xos/string.h"
#include "../include/xos/fs.h"
#include "../include/xos/net.h"

#define BUFLEN 0x1000

static char buf[BUFLEN];

int main(int argc, char const *argv[])
{
    int fd = socket(AF_INET, SOCK_RAW, PROTO_IP);

    msghdr_t msg;
    iovec_t iov;

    msg.name = NULL;
    msg.namelen = 0;

    msg.iov = &iov;
    msg.iovlen = 1;

    iov.base = buf;
    iov.size = sizeof(buf);

    printf("receiving...\n");
    int ret = recvmsg(fd, &msg, 0);
    printf("recvmsg %d\n", ret);

    ip_t *ip = (ip_t *)iov.base;

    printf("recv ip %r -> %r : %s\n", ip->src, ip->dst, ip->payload);

    ip_addr_t addr;
    ip_addr_copy(addr, ip->dst);
    ip_addr_copy(ip->dst, ip->src);
    ip_addr_copy(ip->src, addr);

    memcpy(ip->payload, "this is ack message", 19);
    iov.size = sizeof(ip_t) + 19;

    sendmsg(fd, &msg, 0);

    close(fd);
    return EOK;
}
