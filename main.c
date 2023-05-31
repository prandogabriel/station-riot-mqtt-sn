/*
 * Copyright (C) 2023 Gabriel Prando
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @author      Gabriel Prando <gprando55@gmail.com>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "shell.h"
#include "msg.h"
#include "net/emcute.h"
#include "net/ipv6/addr.h"
#include "thread.h"
#include "net/netif.h"
// #include "ztimer.h"
#include "xtimer.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "net/gnrc/rpl.h"
#include "net/gnrc/rpl/dodag.h"
#include "net/gnrc/rpl/structs.h"

#ifndef EMCUTE_ID
#define EMCUTE_ID ("gertrud")
#endif

#define EMCUTE_PRIO (THREAD_PRIORITY_MAIN - 1)

#define NUMOFSUBS (16U)
#define TOPIC_MAXLEN (64U)

#ifndef BORDER_ROUTER_IPV6_ADDR1
#define BORDER_ROUTER_IPV6_ADDR1 "2001:db8::1"
#endif

#ifndef BORDER_ROUTER_IPV6_ADDR2
#define BORDER_ROUTER_IPV6_ADDR2 "2001:db8::2"
#endif

// struct that contains sensors
typedef struct sensors
{
    int temperature;
    int humidity;
    int windDirection;
    int windIntensity;
    int rainHeight;
} t_sensors;

static char stack[THREAD_STACKSIZE_DEFAULT];
static msg_t queue[8];

static emcute_sub_t subscriptions[NUMOFSUBS];

static void *emcute_thread(void *arg)
{
    (void)arg;
    emcute_run(CONFIG_EMCUTE_DEFAULT_PORT, EMCUTE_ID);
    return NULL; /* should never be reached */
}

static int connect_to_gateway(ipv6_addr_t *gateway_addr)
{
    sock_udp_ep_t gw = {.family = AF_INET6, .port = CONFIG_EMCUTE_DEFAULT_PORT};
    char *topic = NULL;
    char *message = NULL;
    size_t len = 0;

    char addr_str[IPV6_ADDR_MAX_STR_LEN];

    ipv6_addr_to_str(addr_str, gateway_addr, IPV6_ADDR_MAX_STR_LEN);

    if (ipv6_addr_from_str((ipv6_addr_t *)&gw.addr.ipv6, addr_str) == NULL)
    {
        printf("error parsing IPv6 address\n");
        return 1;
    }
    if (emcute_con(&gw, true, topic, message, len, 0) != EMCUTE_OK)
    {
        printf("error: unable to connect to [%s]:%i\n", addr_str, (int)gw.port);
        return 1;
    }
    printf("Successfully connected to gateway at [%s]:%i\n",
           addr_str, (int)gw.port);

    return 0;
}

static int pub_message(char *topic, char *data, int qos)
{
    emcute_topic_t t;
    unsigned flags = EMCUTE_QOS_0;

    switch (qos)
    {
    case 1:
        flags |= EMCUTE_QOS_1;
        break;
    case 2:
        flags |= EMCUTE_QOS_2;
        break;
    default:
        flags |= EMCUTE_QOS_0;
        break;
    }

    // step 1: get topic id
    t.name = topic;
    if (emcute_reg(&t) != EMCUTE_OK)
    {
        puts("error: unable to obtain topic ID");
        return 1;
    }

    // step 2: publish data
    if (emcute_pub(&t, data, strlen(data), flags) != EMCUTE_OK)
    {
        printf("error: unable to publish data to topic '%s [%i]'\n",
               t.name, (int)t.id);
        return 1;
    }

    printf("published %s on topic %s\n", data, topic);

    return 0;
}

int rand_val(int min, int max)
{
    srand(1);
    return (rand() % (int)((max - min + 1) * 100)) / 100 + min;
}

static void gen_sensors_values(t_sensors *sensors)
{
    sensors->temperature = rand_val(-50, 50);
    sensors->humidity = rand_val(0, 100);
    sensors->windDirection = rand_val(0, 360);
    sensors->windIntensity = rand_val(0, 100);
    sensors->rainHeight = rand_val(0, 50);
}

static kernel_pid_t initialize_rpl(void)
{
    kernel_pid_t iface_pid = 6;
    if (gnrc_netif_get_by_pid(iface_pid) == NULL)
    {
        printf("unknown interface specified\n");
        return -1;
    }

    gnrc_rpl_init(iface_pid);
    printf("successfully initialized RPL on interface %d\n", iface_pid);
    return iface_pid;
}

static ipv6_addr_t *get_best_ranked_ipv6(void)
{
    sock_udp_ep_t local = SOCK_IPV6_EP_ANY;
    sock_udp_t sock;

    local.port = 0xabcd;

    if (sock_udp_create(&sock, &local, NULL, 0) < 0)
    {
        puts("Error creating UDP sock");
        return NULL;
    }

    sock_udp_ep_t remote = {.family = AF_INET6};
    ssize_t res;

    remote.port = 8000;

    const char *ipv6_addresses[2] = {BORDER_ROUTER_IPV6_ADDR1, BORDER_ROUTER_IPV6_ADDR2};
    int current_ip_index = 0;
    ipv6_addr_t *addr = NULL;
    while (1)
    {
        sleep(50);
        if (ipv6_addr_from_str((ipv6_addr_t *)&remote.addr.ipv6, ipv6_addresses[current_ip_index]) == NULL)
        {
            printf("error parsing IPv6 address\n");
            return NULL;
        }
        if (sock_udp_send(&sock, "gateway_ipv6_request", sizeof("gateway_ipv6_request"), &remote) < 0)
        {
            puts("Error sending message");
        }
        else
        {
            printf("Successfully sending, waiting response...\n");
            char client_buffer[256];
            if ((res = sock_udp_recv(&sock, client_buffer, sizeof(client_buffer), 1 * US_PER_SEC,
                                     NULL)) < 0)
            {
                if (res == -ETIMEDOUT)
                {
                    puts("Timed out");
                }
                else
                {
                    puts("Error receiving message");
                }

                // Switch to next IP address on communication failure.
                current_ip_index = (current_ip_index + 1) % 2;
            }
            else
            {
                printf("Received data: ");
                puts(client_buffer);

                if (ipv6_addr_from_str(addr, client_buffer) == NULL)
                {
                    printf("Received invalid IPv6, continue trying...\n");
                }
                else
                {
                    printf("Received valid IPv6, terminating...\n");
                    break;
                }
            }
        }
    }

    sock_udp_close(&sock);

    return addr;
}

static int start(void)
{
    // Fazer a request UDP para pegar o melhor o IP do gateway
    ipv6_addr_t *gateway_addr = get_best_ranked_ipv6();

    // Conectar nesse IP
    connect_to_gateway(gateway_addr);
    // Começar a publicar
    // Caso dê erro, procurar o IP do outro Gateway
    // conectar novamente
    // A cada x tempo preciso verificar qual IP está melhor para conectar

    // sensors struct
    t_sensors sensors;
    // name of the topic
    char *topic = "sensor/values";

    // json that it will published
    char json[128];

    while (1)
    {
        // updates sensor values
        gen_sensors_values(&sensors);

        // fills the json document
        sprintf(json, "{\"id\": \"1\",  \"temperature\": "
                      "\"%d\", \"humidity\": \"%d\", \"windDirection\": \"%d\", "
                      "\"windIntensity\": \"%d\", \"rainHeight\": \"%d\"}",
                sensors.temperature, sensors.humidity,
                sensors.windDirection, sensors.windIntensity, sensors.rainHeight);

        // publish to the topic
        pub_message(topic, json, 0);

        // it sleeps for five seconds
        // ztimer_sleep(ZTIMER_MSEC, 5000);
        xtimer_usleep(5 * US_PER_SEC);
    }

    return 0;
}

int main(void)
{
    puts("MQTT-SN application\n");

    /* the main thread needs a msg queue to be able to run `ping`*/
    msg_init_queue(queue, ARRAY_SIZE(queue));

    /* initialize our subscription buffers */
    memset(subscriptions, 0, (NUMOFSUBS * sizeof(emcute_sub_t)));

    /* start the emcute thread */
    thread_create(stack, sizeof(stack), EMCUTE_PRIO, 0,
                  emcute_thread, NULL, "emcute");

    /* Initialize RPL interface */
    kernel_pid_t iface_pid = initialize_rpl();
    if (iface_pid == -1)
    {
        return -1;
    }

    /* Start the application*/
    start();

    /* should be never reached */
    return 0;
}