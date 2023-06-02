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

#define MAX_PUB_ERRORS 3
#define NUMOFSUBS (16U)
#define TOPIC_MAXLEN (64U)

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

// Função para formatar o endereço IPv6.
static bool format_ipv6_address(gnrc_rpl_instance_t *instance, char *new_addr_str, size_t size)
{
    // Buffer to hold the IPv6 address string.
    char addr_str[IPV6_ADDR_MAX_STR_LEN];

    // Convert the IPv6 address to a string.
    ipv6_addr_to_str(addr_str, &(instance->dodag.dodag_id), IPV6_ADDR_MAX_STR_LEN);

    // Count up to the third ':'.
    int count = 0;
    for (char *c = addr_str; *c != '\0'; c++)
    {
        if (*c == ':')
        {
            count++; // TODO alterar aqui para 3
            if (count == 2)
            {
                *c = '\0'; // Terminate the string after the third ':'.
                break;
            }
        }
    }

    // Make sure the new address won't exceed the buffer size.
    if (strlen(addr_str) + 4 < size)
    {
        // Format the new IPv6 address string.
        sprintf(new_addr_str, "%s::1", addr_str);
        return true;
    }
    else
    {
        printf("Error: Buffer overflow.\n");
        return false;
    }
}

// Função para criar um socket UDP.
static int create_udp_socket(sock_udp_ep_t *local, sock_udp_t *sock)
{
    local->port = 0xabcd;

    if (sock_udp_create(sock, local, NULL, 0) < 0)
    {
        puts("Error creating UDP sock");
        return -1;
    }

    return 0;
}

// Função para enviar uma solicitação para o gateway IPv6.
static int send_ipv6_request(sock_udp_t *sock, const char *message, const char *addr_str)
{
    sock_udp_ep_t remote = {.family = AF_INET6};
    remote.port = 8000;

    if (ipv6_addr_from_str((ipv6_addr_t *)&remote.addr.ipv6, addr_str) == NULL)
    {
        printf("Error parsing IPv6 address\n");
        return -1;
    }

    if (sock_udp_send(sock, message, sizeof(message), &remote) < 0)
    {
        puts("Error sending message");
        return -1;
    }

    printf("Successfully sent, waiting for response...\n");
    return 0;
}

static int receive_data(sock_udp_t *sock, char *buffer, size_t buffer_size)
{
    int res;
    if ((res = sock_udp_recv(sock, buffer, buffer_size, 1 * US_PER_SEC, NULL)) < 0)
    {
        if (res == -ETIMEDOUT)
        {
            puts("Timed out");
        }
        else
        {
            puts("Error receiving message");
        }
    }
    else
    {
        printf("Received data: ");
        puts(buffer);
    }
    return res;
}

static int process_data(char *buffer)
{
    ipv6_addr_t addr;
    /*
    for (int i = 0; i < (int)strlen(buffer); ++i)
    {
        printf("Character: '%c', ASCII: %d\n", buffer[i], (unsigned char)buffer[i]);
    }
    */
    if (ipv6_addr_from_str(&addr, buffer) == NULL)
    {
        printf("Received invalid IPv6, continue trying...\n");
    }
    else
    {
        printf("Received valid IPv6, terminating...\n");
        return 0;
    }
    return -1;
}

static void send_udp_and_receive_data(sock_udp_t *sock, char *new_addr_str)
{
    char client_buffer[256];
    uint8_t i = 0;
    while (i < 5)
    {
        sleep(5);
        if (send_ipv6_request(sock, "gateway_ipv6_request", new_addr_str) == 0)
        {
            memset(client_buffer, 0, sizeof(client_buffer));
            if (receive_data(sock, client_buffer, sizeof(client_buffer)) >= 0)
            {
                if (process_data(client_buffer) == 0)
                {
                    break;
                }
            }
        }
        i++;
    }
}

static ipv6_addr_t *get_gateway_ipv6(void)
{
    uint8_t instance_id = 1; // O ID da instância que você deseja obter.

    gnrc_rpl_instance_t *instance = NULL;

    bool ip_setted = false;
    char new_addr_str[43]; // Aumente o tamanho para 43 para acomodar "::1".
    sleep(30);
    while (instance == NULL && !ip_setted)
    {
        instance = gnrc_rpl_instance_get(instance_id);

        if (instance != NULL)
        {
            ip_setted = format_ipv6_address(instance, new_addr_str, sizeof(new_addr_str));

            // Buffer para armazenar a string do endereço IPv6.
            char addr_str[IPV6_ADDR_MAX_STR_LEN];

            // Converte o endereço IPv6 para uma string.
            ipv6_addr_to_str(addr_str, &(instance->dodag.dodag_id), IPV6_ADDR_MAX_STR_LEN);

            // Conta até o terceiro ':'.
            int count = 0;
            for (char *c = addr_str; *c != '\0'; c++)
            {
                if (*c == ':')
                {
                    count++;
                    if (count == 3)
                    {
                        *c = '\0'; // Termina a string após o terceiro ':'.
                        break;
                    }
                }
            }

            // Certifique-se de que o novo endereço não excederá o tamanho do buffer.
            if (ip_setted)
            {
                printf("New DODAG IPv6 address: %s\n", new_addr_str);
            }
        }
        else
        {
            printf("Instance or DODAG not found.\n");
        }
    }

    sock_udp_ep_t local = SOCK_IPV6_EP_ANY;
    sock_udp_t sock;

    create_udp_socket(&local, &sock);

    ipv6_addr_t *addr = malloc(sizeof(ipv6_addr_t));

    send_udp_and_receive_data(&sock, new_addr_str);

    sock_udp_close(&sock);

    return addr;
}

static void reconnect_to_gateway(void)
{
    ipv6_addr_t *gateway_addr = get_gateway_ipv6();

    uint8_t connected = 0;
    while (connected != 0)
    {
        connected = connect_to_gateway(gateway_addr);
        printf("try connect result %d  /n", connected);
    }
}

static int start(void)
{
    reconnect_to_gateway();

    t_sensors sensors;

    char *topic = "sensor/values";

    // json that it will published
    char json[128];

    int pub_errors = 0;

    while (1)
    {
        gen_sensors_values(&sensors);

        sprintf(json, "{\"id\": \"1\",  \"temperature\": "
                      "\"%d\", \"humidity\": \"%d\", \"windDirection\": \"%d\", "
                      "\"windIntensity\": \"%d\", \"rainHeight\": \"%d\"}",
                sensors.temperature, sensors.humidity,
                sensors.windDirection, sensors.windIntensity, sensors.rainHeight);

        // publish to the topic
        if (pub_message(topic, json, 0) != 0)
        {
            printf("increase pub errors /n");
            // Incrementa o contador de erros e verifica se o máximo foi atingido
            if (++pub_errors >= MAX_PUB_ERRORS)
            {
                printf("Failed to publish message after %d attempts, reconnecting...\n", pub_errors);
                // TODO aqui remover a interface do RPL para que sete uma nova.. e dar um sleep de uns 30 sec
                reconnect_to_gateway();
                // Reseta o contador de erros
                pub_errors = 0;
            }
        }
        else
        {
            // Reseta o contador de erros se a publicação for bem sucedida
            pub_errors = 0;
        }

        // it sleeps for five seconds
        // ztimer_sleep(ZTIMER_MSEC, 5000);
        xtimer_usleep(5 * US_PER_SEC);
    }

    return 0;
}

/*
static int start(void)
{
    // Fazer a request UDP para pegar o melhor o IP do gateway
    ipv6_addr_t *gateway_addr = get_gateway_ipv6();

    // Conectar nesse IP

    uint8_t connected = 0;
    while (connected != 0)
    {
        connected = connect_to_gateway(gateway_addr);
        printf("try connect result %d  /n", connected);
    }


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
*/

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