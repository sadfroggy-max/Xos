#ifndef XOS_NET_DHCP_H
#define XOS_NET_DHCP_H

#include "../net.h"

void dhcp_start(netif_t *netif);
void dhcp_stop(netif_t *netif);

#endif
