#ifndef XOS_NET_PKT_H
#define XOS_NET_PKT_H

#include "./types.h"
#include "./pbuf.h"
#include "../list.h"

typedef struct pkt_pcb_t
{
    list_node_t node;

    eth_addr_t laddr;
    eth_addr_t raddr;
    netif_t *netif;

    int protocol;

    list_t rx_pbuf_list;
    struct task_t *rx_waiter;
} pkt_pcb_t;

err_t pkt_input(netif_t *netif, pbuf_t *pbuf);

#endif
