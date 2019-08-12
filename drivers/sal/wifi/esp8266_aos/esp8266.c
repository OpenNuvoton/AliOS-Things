/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */


#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "aos/kernel.h"
#include "ulog/ulog.h"
#include "aos/yloop.h"

#include <sal_arch.h>
#include <sal_ipaddr.h>
#include <sal_import.h>
#include "config_esp8266.h"

#define TAG "sal_esp8266"

extern char net_ipaddr[16], net_mask[16], net_gw[16];

static uint8_t inited = 0;
static link_t g_link[NETM_SOCKET_NUM];
static kmutex_t g_link_mutex;
static netconn_data_input_cb_t g_netconn_data_input_cb=NULL;

static int fd_to_linkid(int fd)
{
    int link_id;
	kstat_t stat = RHINO_SUCCESS;

	stat = krhino_mutex_lock(&g_link_mutex, 0xffffffff);
  if ( !( (stat == RHINO_SUCCESS) || (stat == RHINO_MUTEX_OWNER_NESTED)) )
		 return;

    for (link_id = 0; link_id < NETM_SOCKET_NUM; link_id++) {
        if (g_link[link_id].fd == fd) {
            break;
        }
    }

	krhino_mutex_unlock(&g_link_mutex);
  return link_id;
}

static void change_linkid_status(int linkid, int status)
{
	kstat_t stat = RHINO_SUCCESS;
	
	stat = krhino_mutex_lock(&g_link_mutex, 0xffffffff);
  if ( !( (stat == RHINO_SUCCESS) || (stat == RHINO_MUTEX_OWNER_NESTED)) )
		 return;

  g_link[linkid].fd = status;

	krhino_mutex_unlock(&g_link_mutex);
}

static int esp8266_uart_init(void)
{
    int ret = 0;
		
		ret = netm_softreset();
	   if (ret < 0) {
        return ret;
    }
		
		#ifdef CONFIG_NETM_FLOW_CONTROL
		ret = netm_uart_flowctl();
    if (ret < 0) {
        return ret;
    }
		#endif
		
		ret = netm_close_echo(0);
    if (ret < 0) {
        return ret;
    }
		
		ret = netm_get_version();
    if (ret < 0) {
        return ret;
    }

    ret = netm_set_mode(1);

    if (ret < 0) {
        return ret;
    }

    ret = netm_set_mux_mode(1, 0);

    if (ret < 0) {
        return ret;
    }

    return 0;
}

static void esp8266_get_ip_delayed_action(void *arg)
{
    aos_post_event(EV_WIFI, CODE_WIFI_ON_GOT_IP, 0xdeaddead);
}


int esp8266_wifi_got_ip(const char *ssid, const char *psw)
{
    char rsp[LINE_LEN] = {0};

    int ret = netm_connect_to_ap(ssid, psw);

    if (ret < 0) {
        LOGE(TAG, "esp8266 config error try again");
        return ret;
    }

    ret = netm_get_local_ip(rsp, strlen(rsp));

    if (ret < 0) {
        LOGE(TAG, "esp8266 get ip error");
        return ret;
    }

    LOGD(TAG, "esp8266 got ip %s \r\n", rsp);
    /*delay 5 seconds to post got ip event*/
    aos_post_delayed_action(100, esp8266_get_ip_delayed_action, NULL);
    return 0;
}

static void esp8266_wifi_module_socket_data_handle(int linkid, void *data, size_t len)
{
    /*
    +IPD,0,44,106.15.213.199,5684:
    */
    if (g_netconn_data_input_cb && ((g_link[linkid].fd >= 0))) {
        if (g_netconn_data_input_cb(g_link[linkid].fd, data, len, NULL, 0)) {
            LOGE(TAG, " %s socket %d get data len %d fail to post to sal, drop it",
                 __func__, g_link[linkid].fd, len);
        }
    }

    return;
}

/*****************************************
* esp8266_wifi_module interface
*****************************************/
int HAL_SAL_Init(void)
{
    int ret = 0;
    int i;

    if (inited) {
        LOGI(TAG, "esp8266 wifi module have already inited \r\n");
        return 0;
    }

    memset(g_link, 0, sizeof(g_link));

    for (i = 0; i < NETM_SOCKET_NUM; i++) {
        g_link[i].fd = -1;
    }

	if ( krhino_mutex_create(&g_link_mutex, "link_mutex") != RHINO_SUCCESS )
    {
		LOGE(TAG, "Creating cmd mutex failed (%s %d).", __func__, __LINE__);
        goto err;
    }

    ret = esp8266_uart_init();

    if (ret<0) {
        LOGE(TAG, "%s %d failed \r\n", __func__, __LINE__);
        goto err;
    }

    at_read_cb_handle(0, esp8266_wifi_module_socket_data_handle, NULL);

    inited = 1;

    return 0;
err:

    krhino_mutex_del(&g_link_mutex);

    return -1;
}


int HAL_SAL_Deinit()
{
    if (!inited) {
        return 0;
    }

    krhino_mutex_del(&g_link_mutex);

    inited = 0;
    return 0;
}

int HAL_SAL_DomainToIp(char *domain, char ip[16])
{
    int ret = -1;

    if (!inited) {
        LOGE(TAG, "%s esp8266 wifi module haven't init yet \r\n", __func__);
        return -1;
    }

    if (NULL == domain || NULL == ip) {
        LOGE(TAG, "invalid input at %s \r\n", __func__);
        return -1;
    }
#if 1
    ret = netm_parse_domain(domain, ip);

    if (ret < 0) {
        LOGE(TAG, "domain to ip fail %s \r\n", __func__);
        return -1;
    }
#else
		printf("DNS query: %s(length:%d), we modify to 106.15.83.29 forcely.\r\n", domain, strlen(domain));
		sprintf(ip, "106.15.83.29");
#endif

    return 0;
}

int HAL_SAL_Start(sal_conn_t *conn)
{
    int  linkid = 0;
    int ret = -1;
    netm_conn_e net_type = 0;
	kstat_t stat = RHINO_SUCCESS;

    if (!inited) {
        LOGE(TAG, "%s SAL wifi haven't init yet \r\n", __func__);
        return -1;
    }

    if (!conn || !conn->addr) {
        LOGE(TAG, "%s %d - invalid input \r\n", __func__, __LINE__);
        return -1;
    }

    stat = krhino_mutex_lock(&g_link_mutex, 0xffffffff);
		if ( !( (stat == RHINO_SUCCESS) || (stat == RHINO_MUTEX_OWNER_NESTED)) )
			return -1;

    for (linkid = 0; linkid < NETM_SOCKET_NUM; linkid++) {
        if (g_link[linkid].fd >= 0) {
            continue;
        }

        g_link[linkid].fd = conn->fd;
        break;
    }

    krhino_mutex_unlock(&g_link_mutex);

    if (linkid >= NETM_SOCKET_NUM) {
        LOGE(TAG, "No link available for now, %s failed. \r\n", __func__);
        return -1;
    }

    switch (conn->type) {
        case TCP_SERVER:
            net_type = AT_TYPE_TCP_SERVER;
            break;

        case TCP_CLIENT:
            net_type = AT_TYPE_TCP_CLIENT;
            break;

        case UDP_UNICAST:
            net_type = AT_TYPE_UDP_UNICAST;
            break;

        case SSL_CLIENT:
        case UDP_BROADCAST:
        default:
            break;
    }

    ret = netm_connect(linkid, net_type, conn->addr, conn->r_port);

    if (ret < 0) {
        LOGE(TAG, "conn_start fail %s", __func__);
        change_linkid_status(linkid, -1);
        return -1;
    }

    return 0;

}

int HAL_SAL_Close(int fd, int32_t remote_port)
{
    int  linkid = 0;
    int  ret = 0;

    if (!inited) {
        LOGE(TAG, "%s esp8266 wifi module haven't init yet \r\n", __func__);
        return -1;
    }

    linkid = fd_to_linkid(fd);

    if (linkid >= NETM_SOCKET_NUM) {
        LOGE(TAG, "No connection found for fd (%d) in %s \r\n", fd, __func__);
        return -1;
    }

    ret = netm_disconnect(linkid);

    if (ret < 0) {
        LOGE(TAG, "conn_close %d fail %s \r\n", fd, __func__);
        return -1;
    }

    change_linkid_status(linkid, -1);

    return ret;
}

int HAL_SAL_Send(int fd, uint8_t *data, uint32_t len,
                 char remote_ip[16], int32_t remote_port, int32_t timeout)
{
    int  linkid;
    int  ret = -1;
    int  retry = 0;

    if (!inited) {
        LOGE(TAG, "%s esp8266 wifi module haven't init yet \r\n", __func__);
        return -1;
    }

    linkid = fd_to_linkid(fd);

    if (linkid >= NETM_SOCKET_NUM) {
        LOGE(TAG, "No connection found for fd (%d) in %s \r\n", fd, __func__);
        return -1;
    }

    do {
        ret = netm_mux_send(linkid, data, len);

        if (ret < 0) {
            retry++;

            if (retry > 3) {
								printf("[%s %d]\r\n", __func__, __LINE__);
                break;
            }
        }
    } while (ret < 0);

    if (ret < 0) {
        LOGE(TAG, "%s esp8266 wifi module send fail\r\n", __func__);
        return ret;
    }

    return 0;
}

int HAL_SAL_RegisterNetconnDataInputCb(netconn_data_input_cb_t cb)
{
    if (cb) {
        g_netconn_data_input_cb = cb;
    } else {
        LOGE(TAG, "%s esp8266 wifi input cb register err", __func__);
    }

    return 0;
}

int sal_device_init()
{
		int ret;
		
		#ifdef ESP8266_USE_HARD_RESET
		ret = netm_hardreset();  
    if (ret < 0) {
        return ret;
    } 
		#endif

    ret = netm_init();

    if (ret < 0) {
        LOGE(TAG, "esp8266 netm_init err");
        return -1;
    }

    return 0;
}

