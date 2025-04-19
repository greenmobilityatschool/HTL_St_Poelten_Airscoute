//
// Created by Benedikt Walter on 19.01.24.
//

#ifndef MY_PROJECT_MYWIFI_H
#define MY_PROJECT_MYWIFI_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include <string>
//wifi libs
#include "lwipopts.h"
#include "pico/cyw43_arch.h"
#include "lwip/ip4_addr.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"


#ifdef ERROR_WIFI_LOG
#define ERROR_WIFI(fmt, ...) printf("ERROR-WIFI: " fmt "\n", ##__VA_ARGS__)
#else
#define ERROR_WIFI(fmt, ...)
#endif

#ifdef DEBUG_WIFI_LOG
#define DEBUG_WIFI(fmt, ...) printf("DEBUG-WIFI: " fmt "\n", ##__VA_ARGS__)
#else
#define DEBUG_WIFI(fmt, ...)
#endif


class myWIFI {
private:
    bool trying_to_connect = false;
    int connected = CYW43_LINK_DOWN;
public:
    myWIFI() = default;
    int getConnected();
    void poll();
    int init();
    int scanAndConnect();
    int connect(std::string, std::string);
    void disconnect();
    void emergencyReset();
    bool connectToAP(const char* ssid, const char* password);
};

static int scan_result(void *env, const cyw43_ev_scan_result_t *result);


#endif //MY_PROJECT_MYWIFI_H
