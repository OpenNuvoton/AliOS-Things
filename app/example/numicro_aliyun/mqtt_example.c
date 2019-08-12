/*
 * Copyright (C) 2015-2018 Alibaba Group Holding Limited
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <k_api.h>

#include "iot_import.h"
#include "iot_export.h"
#include "app_entry.h"

#include "numicro_m487_led_button.h"

#define BUILD_DEV 3

#if (BUILD_DEV==0)

#define PRODUCT_KEY             "a1Ll7sjheeL"
#define PRODUCT_SECRET          "dEfhsQipc8Jez7gm"
#define DEVICE_NAME             "nsUqaKB3YTWZWbYpqxKP"
#define DEVICE_SECRET           "n2W2TYxBoJE7qHoaRl42KWzCZQE7lvFO"

#elif (BUILD_DEV==1)
//DEV0
#define PRODUCT_KEY				"a1Ll7sjheeL"
#define DEVICE_NAME             "NCC0Dpq4z3EmKal1UZxe"
#define DEVICE_SECRET           "RLIZpPxsFgvDh9yChNjzcA2DcRPAhFpf"
#define PRODUCT_SECRET          "dEfhsQipc8Jez7gm"

#elif (BUILD_DEV==2)
//APP0
#define PRODUCT_KEY				"a1Ll7sjheeL"
#define DEVICE_NAME             "PfrfuKsWweTxOnuG8wo4"
#define DEVICE_SECRET           "EBsP2YuU486ybGOXiNcGlrtKQWMQs48H"
#define PRODUCT_SECRET          "dEfhsQipc8Jez7gm"

#elif (BUILD_DEV==3)
//DEV1
#define PRODUCT_KEY				"a1Ll7sjheeL"
#define DEVICE_NAME             "5cDCjmKHMyUlGKxgECIa"
#define DEVICE_SECRET           "gI5NBydQN8dLNgJp5GMEbrJ8b1gFb9Tc"
#define PRODUCT_SECRET          "dEfhsQipc8Jez7gm"

#elif (BUILD_DEV==4)
//APP1
#define PRODUCT_KEY				"a1Ll7sjheeL"
#define DEVICE_NAME             "Z9TiBGz2gz2yB0GC31FW"
#define DEVICE_SECRET           "OQS5VUMwT0vDnAwT8EI9kCCQ8MKZcL3i"
#define PRODUCT_SECRET          "dEfhsQipc8Jez7gm"

#else
//Default
#define PRODUCT_KEY             "a1MZxOdcBnO"
#define PRODUCT_SECRET          "h4I4dneEFp7EImTv"
#define DEVICE_NAME             "test_01"
#define DEVICE_SECRET           "t9GmMf2jb3LgWfXBaZD2r3aJrfVWBv56"
#endif

/* These are pre-defined topics */
#define TOPIC_UPDATE            "/"PRODUCT_KEY"/"DEVICE_NAME"/update"
#define TOPIC_ERROR             "/"PRODUCT_KEY"/"DEVICE_NAME"/update/error"
#define TOPIC_GET               "/"PRODUCT_KEY"/"DEVICE_NAME"/get"
#define TOPIC_DATA               "/"PRODUCT_KEY"/"DEVICE_NAME"/data"

#define MQTT_MSGLEN             (1024)

#define EXAMPLE_TRACE(fmt, ...)  \
    do { \
        HAL_Printf("%s|%03d :: ", __func__, __LINE__); \
        HAL_Printf(fmt, ##__VA_ARGS__); \
        HAL_Printf("%s", "\r\n"); \
    } while(0)

static kbuf_queue_t gUsrEventQueue;
#define DEF_MAX_MSG_LENGTH	10
static uint8_t g_u8MsgQBuf[ DEF_MAX_MSG_LENGTH*sizeof(E_UserEvent) ];

static int mqtt_publish_message (void *pclient, const char* topic, iotx_mqtt_topic_info_t* psTopicMsg )
{
    int rc = 0;

    if ( !psTopicMsg || !pclient || !topic )
        goto exit_mqtt_publish_message;

    rc = IOT_MQTT_Publish ( pclient, topic, psTopicMsg );
    if (rc < 0) {
        EXAMPLE_TRACE("error occur when publish");
        goto exit_mqtt_publish_message;
    }

    EXAMPLE_TRACE("packet-id=%lu, publish topic msg=%s", (uint32_t)rc, (char*)psTopicMsg->payload);

    /* handle the MQTT packet received from TCP or SSL connection */
    IOT_MQTT_Yield ( pclient, 200 );

    return rc;

exit_mqtt_publish_message:

    return -1;
}

static int handle_user_command(iotx_mqtt_topic_info_pt topic_info)
{
    if ( !strcmp(topic_info->payload, "LED*=ON") )
    {
        led_control(e_LED_R, 1);
        led_control(e_LED_G, 1);
        led_control(e_LED_Y, 1);
        goto exit_handle_user_command;
    }
    else if ( !strcmp(topic_info->payload, "LED*=OFF") )
    {
        led_control(e_LED_R, 0);
        led_control(e_LED_G, 0);
        led_control(e_LED_Y, 0);
        goto exit_handle_user_command;
    }
    else if ( !strcmp(topic_info->payload, "LEDR=ON") )
    {
        led_control(e_LED_R, 1);
        goto exit_handle_user_command;
    }
    else if ( !strcmp(topic_info->payload, "LEDR=OFF") )
    {
        led_control(e_LED_R, 0);
        goto exit_handle_user_command;
    }
    else if ( !strcmp(topic_info->payload, "LEDY=ON") )
    {
        led_control(e_LED_Y, 1);
        goto exit_handle_user_command;
    }
    else if ( !strcmp(topic_info->payload, "LEDY=OFF") )
    {
        led_control(e_LED_Y, 0);
        goto exit_handle_user_command;
    }
    else if ( !strcmp(topic_info->payload, "LEDG=ON") )
    {
        led_control(e_LED_G, 1);
        goto exit_handle_user_command;
    }
    else if ( !strcmp(topic_info->payload, "LEDG=OFF") )
    {
        led_control(e_LED_G, 0);
        goto exit_handle_user_command;
    }
    return 0;

exit_handle_user_command:
    return 1;
}

int handle_button_event(E_UserEvent eBtn)
{
	int ret = krhino_buf_queue_send(&gUsrEventQueue, (void *)&eBtn, sizeof(E_UserEvent));
	if (ret != RHINO_SUCCESS)
		EXAMPLE_TRACE( "krhino_buf_queue_send result\n" );
    return ret;

exit_handle_button_event:
    return -1;
}

void event_handle(void *pcontext, void *pclient, iotx_mqtt_event_msg_pt msg)
{
    uintptr_t packet_id = (uintptr_t)msg->msg;
    iotx_mqtt_topic_info_pt topic_info = (iotx_mqtt_topic_info_pt)msg->msg;

    switch (msg->event_type) {
    case IOTX_MQTT_EVENT_UNDEF:
        EXAMPLE_TRACE("undefined event occur.");
        break;

    case IOTX_MQTT_EVENT_DISCONNECT:
        EXAMPLE_TRACE("MQTT disconnect.");
        break;

    case IOTX_MQTT_EVENT_RECONNECT:
        EXAMPLE_TRACE("MQTT reconnect.");
        break;

    case IOTX_MQTT_EVENT_SUBCRIBE_SUCCESS:
        EXAMPLE_TRACE("subscribe success, packet-id=%u", (unsigned int)packet_id);
        break;

    case IOTX_MQTT_EVENT_SUBCRIBE_TIMEOUT:
        EXAMPLE_TRACE("subscribe wait ack timeout, packet-id=%u", (unsigned int)packet_id);
        break;

    case IOTX_MQTT_EVENT_SUBCRIBE_NACK:
        EXAMPLE_TRACE("subscribe nack, packet-id=%u", (unsigned int)packet_id);
        break;

    case IOTX_MQTT_EVENT_UNSUBCRIBE_SUCCESS:
        EXAMPLE_TRACE("unsubscribe success, packet-id=%u", (unsigned int)packet_id);
        break;

    case IOTX_MQTT_EVENT_UNSUBCRIBE_TIMEOUT:
        EXAMPLE_TRACE("unsubscribe timeout, packet-id=%u", (unsigned int)packet_id);
        break;

    case IOTX_MQTT_EVENT_UNSUBCRIBE_NACK:
        EXAMPLE_TRACE("unsubscribe nack, packet-id=%u", (unsigned int)packet_id);
        break;

    case IOTX_MQTT_EVENT_PUBLISH_SUCCESS:
        EXAMPLE_TRACE("publish success, packet-id=%u", (unsigned int)packet_id);
        break;

    case IOTX_MQTT_EVENT_PUBLISH_TIMEOUT:
        EXAMPLE_TRACE("publish timeout, packet-id=%u", (unsigned int)packet_id);
        break;

    case IOTX_MQTT_EVENT_PUBLISH_NACK:
        EXAMPLE_TRACE("publish nack, packet-id=%u", (unsigned int)packet_id);
        break;

    case IOTX_MQTT_EVENT_PUBLISH_RECEIVED:
        if ( !handle_user_command(topic_info) )
            EXAMPLE_TRACE("topic message arrived but without any related handle: topic=%.*s, topic_msg=%.*s",
                          topic_info->topic_len,
                          topic_info->ptopic,
                          topic_info->payload_len,
                          topic_info->payload);
        break;

    case IOTX_MQTT_EVENT_BUFFER_OVERFLOW:
        EXAMPLE_TRACE("buffer overflow, %s", msg->msg);
        break;

    default:
        EXAMPLE_TRACE("Should NOT arrive here.");
        break;
    }
}

static void _demo_message_arrive(void *pcontext, void *pclient, iotx_mqtt_event_msg_pt msg)
{
    iotx_mqtt_topic_info_pt     ptopic_info = (iotx_mqtt_topic_info_pt) msg->msg;

    switch (msg->event_type) {
    case IOTX_MQTT_EVENT_PUBLISH_RECEIVED:
        /* print topic name and topic message */
        EXAMPLE_TRACE("----");
        EXAMPLE_TRACE("PacketId: %d", ptopic_info->packet_id);
        EXAMPLE_TRACE("Topic: '%.*s' (Length: %d)",
                      ptopic_info->topic_len,
                      ptopic_info->ptopic,
                      ptopic_info->topic_len);
        EXAMPLE_TRACE("Payload: '%.*s' (Length: %d)",
                      ptopic_info->payload_len,
                      ptopic_info->payload,
                      ptopic_info->payload_len);
        EXAMPLE_TRACE("----");
        break;
    default:
        EXAMPLE_TRACE("Should NOT arrive here.");
        break;
    }
}


int mqtt_client(void)
{
    int rc, msg_len, cnt = 0, counter = 0;
    void *pclient;
    iotx_conn_info_pt pconn_info;
    iotx_mqtt_param_t mqtt_params;
    iotx_mqtt_topic_info_t topic_msg;
    char msg_pub[128];
    kstat_t kret;

	if ( (kret=krhino_buf_queue_create( &gUsrEventQueue, "gUsrEventQueue", &g_u8MsgQBuf[0], sizeof(g_u8MsgQBuf), sizeof(E_UserEvent)) ) != RHINO_SUCCESS )
    {
        EXAMPLE_TRACE("Creating gUsrEventQueue failed (%s %d).", __func__, __LINE__);
		return -1;
	}

    /* Device AUTH */
    if (0 != IOT_SetupConnInfo(PRODUCT_KEY, DEVICE_NAME, DEVICE_SECRET, (void **)&pconn_info)) {
        EXAMPLE_TRACE("AUTH request failed!");
        return -1;
    }

    /* Initialize MQTT parameter */
    memset(&mqtt_params, 0x0, sizeof(mqtt_params));

    mqtt_params.port = pconn_info->port;
    mqtt_params.host = pconn_info->host_name;
    mqtt_params.client_id = pconn_info->client_id;
    mqtt_params.username = pconn_info->username;
    mqtt_params.password = pconn_info->password;
    mqtt_params.pub_key = pconn_info->pub_key;

    mqtt_params.request_timeout_ms = 2000;
    mqtt_params.clean_session = 0;
    mqtt_params.keepalive_interval_ms = 60000;
    mqtt_params.read_buf_size = MQTT_MSGLEN;
    mqtt_params.write_buf_size = MQTT_MSGLEN;

    mqtt_params.handle_event.h_fp = event_handle;
    mqtt_params.handle_event.pcontext = NULL;


    /* Construct a MQTT client with specify parameter */
    pclient = IOT_MQTT_Construct(&mqtt_params);
    if (NULL == pclient) {
        EXAMPLE_TRACE("MQTT construct failed");
        return -1;
    }

    /* Initialize topic information */
    memset(&topic_msg, 0x0, sizeof(iotx_mqtt_topic_info_t));
    strcpy(msg_pub, "update: hello! start!");

    topic_msg.qos = IOTX_MQTT_QOS1;
    topic_msg.retain = 0;
    topic_msg.dup = 0;
    topic_msg.payload = (void *)msg_pub;
    topic_msg.payload_len = strlen(msg_pub);

    rc = IOT_MQTT_Publish(pclient, TOPIC_UPDATE, &topic_msg);
    if (rc < 0) {
        IOT_MQTT_Destroy(&pclient);
        EXAMPLE_TRACE("error occur when publish");
        return -1;
    }

    EXAMPLE_TRACE("\n publish message: \n topic: %s\n payload: \%s\n rc = %d", TOPIC_UPDATE, topic_msg.payload, rc);

    /* Subscribe the specific topic */
    rc = IOT_MQTT_Subscribe(pclient, TOPIC_DATA, IOTX_MQTT_QOS1, _demo_message_arrive, NULL);
    if (rc < 0) {
        IOT_MQTT_Destroy(&pclient);
        EXAMPLE_TRACE("IOT_MQTT_Subscribe() failed, rc = %d", rc);
        return -1;
    }

    IOT_MQTT_Yield(pclient, 200);

    HAL_SleepMs(2000);

    /* Initialize topic information */
    memset(msg_pub, 0x0, 128);
    strcpy(msg_pub, "data: hello! start!");
    memset(&topic_msg, 0x0, sizeof(iotx_mqtt_topic_info_t));
    topic_msg.qos = IOTX_MQTT_QOS1;
    topic_msg.retain = 0;
    topic_msg.dup = 0;
    topic_msg.payload = (void *)msg_pub;
    topic_msg.payload_len = strlen(msg_pub);

    rc = IOT_MQTT_Publish(pclient, TOPIC_DATA, &topic_msg);
    EXAMPLE_TRACE("\n publish message: \n topic: %s\n payload: \%s\n rc = %d", TOPIC_DATA, topic_msg.payload, rc);

    IOT_MQTT_Yield(pclient, 200);

    while(1) {
		int received_msgsize=0;
		E_UserEvent msgbuf;
        kret = krhino_buf_queue_recv(&gUsrEventQueue, 1000/*AOS_WAIT_FOREVER*/, (void *)&msgbuf, &received_msgsize );
        if ( kret == RHINO_BLK_TIMEOUT )
		{
            IOT_MQTT_Yield(pclient, 200);
			continue;
		}
        else if ( kret !=  RHINO_SUCCESS ) {
            EXAMPLE_TRACE("recv msg fail %d", kret);
            break;
        }
		msg_len = 0;
        /* Generate topic message */
        switch (msgbuf)
        {
        case e_BTN_SW2:
            msg_len = snprintf(msg_pub, sizeof(msg_pub), "SW2 pressed!");
            break;

        case e_BTN_SW3:
            msg_len = snprintf(msg_pub, sizeof(msg_pub), "SW3 pressed!");
            break;

        default:
            EXAMPLE_TRACE("error msg type");
            break;
        }

        if (msg_len < 0) {
            EXAMPLE_TRACE("Error occur! Exit program");
            break;
        }

        topic_msg.payload = (void *)msg_pub;
        topic_msg.payload_len = msg_len;
        rc = mqtt_publish_message (pclient, TOPIC_DATA, &topic_msg );

    }
    
    IOT_MQTT_Yield(pclient, 200);

    IOT_MQTT_Unsubscribe(pclient, TOPIC_DATA);

    IOT_MQTT_Yield(pclient, 200);

    IOT_MQTT_Destroy(&pclient);

	kret = krhino_buf_queue_del(&gUsrEventQueue);
    if( kret != RHINO_SUCCESS)
		EXAMPLE_TRACE( "Delete gUsrEventQueue failed (%s %d).", __func__, __LINE__);

    return 0;
}

int linkkit_main(void *paras)
{
    IOT_SetLogLevel(IOT_LOG_DEBUG);

    HAL_SetProductKey(PRODUCT_KEY);
    HAL_SetDeviceName(DEVICE_NAME);
    HAL_SetDeviceSecret(DEVICE_SECRET);
    HAL_SetProductSecret(PRODUCT_SECRET);
    /* Choose Login Server */
    int domain_type = IOTX_CLOUD_REGION_SHANGHAI;
    IOT_Ioctl(IOTX_IOCTL_SET_DOMAIN, (void *)&domain_type);

    /* Choose Login  Method */
    int dynamic_register = 0;
    IOT_Ioctl(IOTX_IOCTL_SET_DYNAMIC_REGISTER, (void *)&dynamic_register);

    mqtt_client();
    IOT_DumpMemoryStats(IOT_LOG_DEBUG);
    IOT_SetLogLevel(IOT_LOG_NONE);

    EXAMPLE_TRACE("out of sample!");

    return 0;
}
