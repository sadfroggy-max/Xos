#ifndef XOS_NET_PORT_H
#define XOS_NET_PORT_H

#include "../types.h"
#include "../bitmap.h"

typedef struct port_map_t
{
    bitmap_t map;
    u32 buf;
} port_map_t;

// 获取端口号
err_t port_get(port_map_t *map, u16 port);

// 释放端口号
void port_put(port_map_t *map, u16 port);

// 初始化端口号位图
void port_init(port_map_t *map);

#endif