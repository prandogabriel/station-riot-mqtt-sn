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

// for sensors on iot lab
#include "lpsxxx.h"
#include "lpsxxx_params.h"
#include "lsm303dlhc.h"
#include "lsm303dlhc_params.h"

#ifndef EMCUTE_ID
#define EMCUTE_ID ("station")
#endif

#define EMCUTE_PRIO (THREAD_PRIORITY_MAIN - 1)

#define MAX_PUB_ERRORS 3
#define NUMOFSUBS (16U)
#define TOPIC_MAXLEN (64U)

#define CLIENT_BUFFER_SIZE (128)
static char client_buffer[CLIENT_BUFFER_SIZE];
#define BUFFER_SIZE 256
static char json[BUFFER_SIZE];

// struct that contains sensors
typedef struct sensors
{
    uint16_t pressure;
    int16_t temperature;

    lsm303dlhc_3d_data_t accelerometer;
    lsm303dlhc_3d_data_t magnetometer;

} t_sensors;

static char stack[THREAD_STACKSIZE_DEFAULT];
static char stack2[THREAD_STACKSIZE_DEFAULT];

static msg_t queue[8];
#if BOARD_ID == 1
static lpsxxx_t lpsxxx;
#endif
static lsm303dlhc_t lsm303dlhc;

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
    printf("try connect to gateway with ip: %s\n", addr_str);
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
    uint16_t pres = 0;
    int16_t temp = 0;
    lsm303dlhc_3d_data_t mag_value;
    lsm303dlhc_3d_data_t acc_value;

    lsm303dlhc_read_acc(&lsm303dlhc, &acc_value);
    lsm303dlhc_read_mag(&lsm303dlhc, &mag_value);
#if BOARD_ID == 1
    lpsxxx_read_temp(&lpsxxx, &temp);
    lpsxxx_read_pres(&lpsxxx, &pres);
#endif

    /*
    printf("Accelerometer x: %i y: %i z: %i\n",
           acc_value.x_axis, acc_value.y_axis, acc_value.z_axis);
    printf("Magnetometer x: %i y: %i z: %i\n",
           mag_value.x_axis, mag_value.y_axis, mag_value.z_axis);
    xtimer_usleep(500 * US_PER_MS);

    // printf("Pressure: %uhPa, Temperature: %i.%u°C\n",
    //      pres, (temp / 100), (temp % 100));
    */

    sensors->temperature = temp;
    sensors->pressure = pres;

    sensors->magnetometer = mag_value;
    sensors->accelerometer = acc_value;
}

static kernel_pid_t initialize_rpl(void)
{
    kernel_pid_t iface_pid = 6;
    netif_t *iface = netif_iter(NULL);
    if (iface != NULL)
    {
        char name[4];
        netif_get_name(iface, name);
        iface_pid = name[0] - 48;
    }
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
            if (count == 3)
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

static int receive_data(sock_udp_t *sock)
{
    int res;
    printf("Successfully sending, waiting response...\n");

    if ((res = sock_udp_recv(sock, client_buffer, sizeof(client_buffer), 1 * US_PER_SEC, NULL)) < 0)
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
        puts(client_buffer);
    }
    return res;
}

static int process_data(char *buffer, ipv6_addr_t *addr)
{
    /*
    for (int i = 0; i < (int)strlen(buffer); ++i)
    {
        printf("Character: '%c', ASCII: %d\n", buffer[i], (unsigned char)buffer[i]);
    }
    */
    if (ipv6_addr_from_str(addr, buffer) == NULL)
    {
        printf("Received invalid IPv6, continue trying... \n");
    }
    else
    {
        puts(buffer);
        printf("Received valid IPv6, terminating...\n");
        return 0;
    }
    return -1;
}
static void send_udp_and_receive_data(sock_udp_t *sock, char *new_addr_str, ipv6_addr_t *new_gw_addr)
{
    uint8_t i = 0;
    while (i < 5)
    {
        if (send_ipv6_request(sock, "gateway_ipv6_request", new_addr_str) == 0)
        {
            memset(client_buffer, 0, sizeof(client_buffer));
            if (receive_data(sock) >= 0)
            {
                if (process_data(client_buffer, new_gw_addr) == 0)
                {
                    break;
                }
            }
        }
        i++;
        sleep(5);
    }

    printf("new_addr_str: %s\n", new_addr_str);
}

static ipv6_addr_t *get_gateway_ipv6(void)
{
    uint8_t instance_id = 1; // O ID da instância que você deseja obter.

    gnrc_rpl_instance_t *instance = NULL;

    bool ip_setted = false;
    char new_addr_str[43]; // Aumente o tamanho para 43 para acomodar "::1".

    sleep(10);

    while (instance == NULL && !ip_setted)
    {
        for (uint8_t i = 0; i < GNRC_RPL_INSTANCES_NUMOF; ++i)
        {
            if (gnrc_rpl_instances[i].state == 0)
            {
                continue;
            }
            instance = gnrc_rpl_instance_get(gnrc_rpl_instances[i].id);

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
                sleep(10);
            }
        }
    }

    sock_udp_ep_t local = SOCK_IPV6_EP_ANY;
    sock_udp_t sock;

    create_udp_socket(&local, &sock);

    ipv6_addr_t *addr = malloc(sizeof(ipv6_addr_t));

    send_udp_and_receive_data(&sock, new_addr_str, addr);

    sock_udp_close(&sock);

    return addr;
}

static void reconnect_to_gateway(void)
{
    ipv6_addr_t *gateway_addr = get_gateway_ipv6();

    uint8_t connected = -1;
    uint8_t connect_errors_count = 0;
    while (connected != 0)
    {
        connected = connect_to_gateway(gateway_addr);
        printf("try connect result %d  \n", connected);

        if (connected != 0)
        {
            connect_errors_count++;
            printf("error on connect, count %d  \n", connect_errors_count);
        }

        if (connect_errors_count >= 3)
        {
            printf("max errors, get gateway ipv6 again %d  \n", connected);
            gateway_addr = get_gateway_ipv6();
        }
    }
}

static int remove_gnrc_rpl_instance(uint8_t instance_id)
{
    gnrc_rpl_instance_t *inst;

    if ((inst = gnrc_rpl_instance_get(instance_id)) == NULL)
    {
        printf("error: could not find the instance (%d)\n", instance_id);
        return 1;
    }

    if (gnrc_rpl_instance_remove(inst) == false)
    {
        printf("error: could not remove instance (%d)\n", instance_id);
        return 1;
    }

    printf("success: removed instance (%d)\n", instance_id);
    return 0;
}

static void *gateway_thread(void *arg)
{
    (void)arg;
    reconnect_to_gateway();

    t_sensors sensors;

    char *topic = "sensor/values";

    int pub_errors = 0;

    while (1)
    {
        gen_sensors_values(&sensors);

        snprintf(json, BUFFER_SIZE,
                 "{"
                 "\"temperature\": \"%u\","
                 "\"pressure\": \"%u\","
                 "\"accelerometer\": {\"x\": \"%d\", \"y\": \"%d\", \"z\": \"%d\"},"
                 "\"magnetometer\": {\"x\": \"%d\", \"y\": \"%d\", \"z\": \"%d\"}"
                 "}",
                 sensors.temperature,
                 sensors.pressure,
                 sensors.accelerometer.x_axis,
                 sensors.accelerometer.y_axis,
                 sensors.accelerometer.z_axis,
                 sensors.magnetometer.x_axis,
                 sensors.magnetometer.y_axis,
                 sensors.magnetometer.z_axis);

        // publish to the topic
        if (pub_message(topic, json, 0) != 0)
        {
            printf("increase pub errors \n");
            // Incrementa o contador de erros e verifica se o máximo foi atingido
            if (++pub_errors >= MAX_PUB_ERRORS)
            {
                printf("Failed to publish message after %d attempts, reconnecting...\n", pub_errors);

                remove_gnrc_rpl_instance(1);

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

        // ztimer_sleep(ZTIMER_MSEC, 5000);
        xtimer_usleep(10 * US_PER_SEC);
    }

    return NULL;
}

static void init_sensors(void)
{
    lsm303dlhc_init(&lsm303dlhc, &lsm303dlhc_params[0]);
    lsm303dlhc_enable(&lsm303dlhc);
#if BOARD_ID == 1
    lpsxxx_init(&lpsxxx, &lpsxxx_params[0]);
    lpsxxx_enable(&lpsxxx);
#endif
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

    init_sensors();

    thread_create(stack2, sizeof(stack), EMCUTE_PRIO, 0,
                  gateway_thread, NULL, "gateway");

    /* start shell */
    puts("All up, running the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(NULL, line_buf, SHELL_DEFAULT_BUFSIZE);

    /* should be never reached */
    return 0;
}