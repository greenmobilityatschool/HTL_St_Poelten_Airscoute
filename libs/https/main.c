/*
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include <tusb.h>

#define WIFI_SSID "iPhone von Benedikt"
#define WIFI_PASSWORD "12345678"
#define TLS_CLIENT_SERVER        "www.gm4s.eu"
/*#define TLS_CLIENT_HTTP_REQUEST  "GET /api/test HTTP/1.1\r\n" \
                                 "Host: " TLS_CLIENT_SERVER "\r\n" \
                                 "Connection: close\r\n" \
                                 "\r\n"*/
#define TLS_CLIENT_HTTP_REQUEST  "POST /api/addMarker HTTP/1.1\r\n" \
                                 "Host: " TLS_CLIENT_SERVER "\r\n" \
                                 "Content-Type: application/json\r\n" \
                                 "Content-Length: 203\r\n" \
                                 "Connection: close\r\n" \
                                 "\r\n" \
                                 "{\"token\":\"86ea63a5-4ea6-4bd1-88f0-bb370970dd16\"," \
                                 "\"measured_at\":\"2024-10-23T12:28:02.379+00:00\"," \
                                 "\"lat\":48.2072620612573,\"long\":15.61750700948781,\"co2\":3,\"hum\":4," \
                                 "\"temp\":5,\"part_2_5\":6,\"part_5\":7,\"part_10\":8}"
#define TLS_CLIENT_TIMEOUT_SECS  6000

extern bool run_tls_client_test(unsigned char const* cert, unsigned int cert_len, char const* server, char const* request, int timeout);

int main() {
    stdio_init_all();

    while (!tud_cdc_connected());
    printf("Serial Monitor connected\n");

    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return 1;
    }
    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect\n");
        return 1;
    }
    bool pass = run_tls_client_test(NULL, 0, TLS_CLIENT_SERVER, TLS_CLIENT_HTTP_REQUEST, TLS_CLIENT_TIMEOUT_SECS);
    if (pass) {
        printf("Test passed\n");
    } else {
        printf("Test failed\n");
    }
    /* sleep a bit to let usb stdio write out any buffer to host */
    sleep_ms(100);

    cyw43_arch_deinit();
    printf("All done\n");
    return pass ? 0 : 1;
}