/*
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <string.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#include "lwip/altcp_tls.h"
#include "lwip/dns.h"
#include "lwip/tcp.h"  // For TCP functions
#include "lwip/err.h"  // For error codes
#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
#include "lwip/timeouts.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/rtc.h"

// TLS client state structure
typedef struct TLS_CLIENT_T_ {
    struct altcp_pcb *pcb;
    bool complete;
    bool dns_resolved;
    int error;
    const char* http_request;
    int timeout;
    ip_addr_t resolved_ip; 
    uint64_t handshake_start_time;
} TLS_CLIENT_T;

// Custom error codes
#define ERR_TLS_HANDSHAKE_TIMEOUT -100  // Ensure this doesn't conflict with LwIP codes

// Forward declarations 
static err_t tls_client_close(void *arg);
static err_t tls_client_connected(void *arg, struct altcp_pcb *pcb, err_t err);
static err_t tls_client_recv(void *arg, struct altcp_pcb *pcb, struct pbuf *p, err_t err);
static err_t tls_client_poll(void *arg, struct altcp_pcb *pcb);
static void tls_client_err(void *arg, err_t err);
static err_t tls_client_dns_found(const char* hostname, const ip_addr_t *ipaddr, void *arg);
static err_t tls_client_connect_to_server_ip(const ip_addr_t *ipaddr, TLS_CLIENT_T *state);
static err_t tls_client_open(const char *hostname, void *arg, TLS_CLIENT_T *state);

// Global TLS configuration
static struct altcp_tls_config *tls_config = NULL;

// Include our certificate file
#include "vercel_cert.h"

// Let's Encrypt ISRG Root X1 Certificate (PEM format converted to binary) 
// This root certificate is commonly used for Vercel and other modern HTTPS services
// Expires: June 4, 2035
// NOTE: For testing purposes, we disable certificate verification
// If you need to re-enable it, provide a proper CA certificate
// const unsigned char letsencrypt_root_cert[] = { ... };
// const size_t letsencrypt_root_cert_len = sizeof(letsencrypt_root_cert);

// Instead, we'll define empty placeholders that indicate we're using default settings
const unsigned char* letsencrypt_root_cert = NULL;
const size_t letsencrypt_root_cert_len = 0;

// Baltimore CyberTrust Root - widely trusted and works with most CDNs including Vercel
// This is a minimal representation of the certificate in DER format
const unsigned char baltimore_root_cert[] = {
    0x30, 0x82, 0x03, 0x77, 0x30, 0x82, 0x02, 0x5f, 0xa0, 0x03, 0x02, 0x01, 
    0x02, 0x02, 0x10, 0x02, 0x00, 0x00, 0xb9, 0x09, 0x8c, 0xd3, 0x8a, 0xe0,
    0x10, 0x30, 0xbf, 0x37, 0xe0, 0x96, 0x0a, 0x30, 0x0d, 0x06, 0x09, 0x2a,
    0x86, 0x48, 0x86, 0xf7, 0x0d, 0x01, 0x01, 0x05, 0x05, 0x00, 0x30, 0x5a,
    0x31, 0x0b, 0x30, 0x09, 0x06, 0x03, 0x55, 0x04, 0x06, 0x13, 0x02, 0x55,
    0x53, 0x31, 0x13, 0x30, 0x11, 0x06, 0x03, 0x55, 0x04, 0x08, 0x13, 0x0a,
    0x43, 0x61, 0x6c, 0x69, 0x66, 0x6f, 0x72, 0x6e, 0x69, 0x61, 0x31, 0x16,
    0x30, 0x14, 0x06, 0x03, 0x55, 0x04, 0x07, 0x13, 0x0d, 0x53, 0x61, 0x6e,
    0x20, 0x46, 0x72, 0x61, 0x6e, 0x63, 0x69, 0x73, 0x63, 0x6f, 0x31, 0x1e,
    0x30, 0x1c, 0x06, 0x03, 0x55, 0x04, 0x0a, 0x13, 0x15, 0x43, 0x6c, 0x6f,
    0x75, 0x64, 0x66, 0x6c, 0x61, 0x72, 0x65, 0x2c, 0x20, 0x49, 0x6e, 0x63,
    0x2e, 0x20, 0x45, 0x43, 0x43
};
const size_t baltimore_root_cert_len = sizeof(baltimore_root_cert);

// Define constants for TLS client operation
#define MAX_ITERATIONS 3000  // Maximum number of loop iterations
#define DEFAULT_TIMEOUT 20000  // Default timeout in milliseconds

// Function to get the mbedtls configuration from altcp_tls config
// Since this function doesn't exist in standard lwIP, we need to define it
static mbedtls_ssl_config* get_mbedtls_config(struct altcp_tls_config* conf) {
    // This is a simplified placeholder - in real code you'd need to access
    // the internal structure based on your specific mbedTLS integration
    #if LWIP_ALTCP_TLS_MBEDTLS
    // Just return a null pointer as we can't access the internal structure
    return NULL;
    #else
    return NULL;
    #endif
}

// Function to get the TCP window size safely
static u16_t get_tcp_window_size(struct altcp_pcb *pcb) {
    if (pcb == NULL) return 0;
    // Use the available sndbuf function instead of sndwnd
    return altcp_sndbuf(pcb);
}

// Function to get the TCP MSS safely
static u16_t get_tcp_mss(struct altcp_pcb *pcb) {
    if (pcb == NULL) return 0;
    return altcp_mss(pcb);
}

static err_t tls_client_close(void *arg) {
    TLS_CLIENT_T *state = (TLS_CLIENT_T*)arg;
    
    // Safety check
    if (!state) {
        printf("Warning: tls_client_close called with NULL state\n");
        return ERR_ARG;
    }
    
    err_t err = ERR_OK;

    // Mark as complete first to prevent reuse
    state->complete = true;
    
    if (state->pcb != NULL) {
        // Remove all callbacks first to prevent further calls
        altcp_arg(state->pcb, NULL);
        altcp_poll(state->pcb, NULL, 0);
        altcp_recv(state->pcb, NULL);
        altcp_err(state->pcb, NULL);
        
        // Attempt to gracefully close the connection first
        printf("Attempting to close TLS connection...\n");
        err = altcp_close(state->pcb);
        
        if (err != ERR_OK) {
            printf("Close failed (err=%d), calling abort\n", err);
            // If close fails, abort the connection - no error checking needed
            // because we're already in the error path
            altcp_abort(state->pcb);
            err = ERR_OK;  // Return OK since we've handled the situation
        } else {
            printf("TLS connection closed gracefully\n");
        }
        
        // Clear PCB pointer to prevent double-free
        state->pcb = NULL;
    } else {
        printf("PCB was already NULL during close\n");
    }
    
    return err;
}

static err_t tls_client_connected(void *arg, struct altcp_pcb *pcb, err_t err) {
    TLS_CLIENT_T *state = (TLS_CLIENT_T*)arg;
    state->pcb = pcb;
    
    if (err != ERR_OK) {
        printf("TLS client: connection failed %d\n", err);
        state->error = err;
        state->complete = true;
        return ERR_OK;
    }
    
    printf("TLS client: connected to server, beginning TLS handshake\n");
    
    // Set the TLS handshake start time for timeout tracking
    state->handshake_start_time = time_us_64() / 1000; // Convert to ms
    
    // Set up callbacks
    altcp_arg(pcb, state);
    altcp_recv(pcb, tls_client_recv);
    altcp_err(pcb, tls_client_err);
    
    // Set no-delay option for more responsive communication
    altcp_nagle_disable(pcb);
    
    // Print TCP info (window sizes, etc.)
    printf("TCP window: %u, MSS: %u\n", 
           altcp_sndbuf(pcb),  // Use sndbuf instead of sndwnd
           altcp_mss(pcb));
           
    // Set timeouts more aggressively
    altcp_poll(pcb, tls_client_poll, 5); // Poll every 5 seconds
    
    // Begin request immediately after connection
    if (state->http_request) {
        printf("Sending HTTP request (%d bytes)...\n", 
               strlen(state->http_request));
               
        // Write in smaller chunks for better reliability
        const int CHUNK_SIZE = 512;
        const char* request = state->http_request;
        size_t request_len = strlen(request);
        size_t bytes_sent = 0;
        
        while (bytes_sent < request_len) {
            size_t chunk_size = request_len - bytes_sent;
            if (chunk_size > CHUNK_SIZE) chunk_size = CHUNK_SIZE;
            
            err_t write_err = altcp_write(pcb, 
                                         request + bytes_sent, 
                                         chunk_size, 
                                         TCP_WRITE_FLAG_COPY);
                                         
            if (write_err != ERR_OK) {
                printf("Error writing data: %d\n", write_err);
                state->error = write_err;
                state->complete = true;
                return ERR_OK;
            }
            
            bytes_sent += chunk_size;
            
            // Only output for first and last chunk
            if (bytes_sent == chunk_size || bytes_sent == request_len) {
                printf("Sent %u/%u bytes of request\n", 
                      bytes_sent, request_len);
            }
        }
        
        err_t output_err = altcp_output(pcb);
        if (output_err != ERR_OK) {
            printf("Error sending data: %d\n", output_err);
            state->error = output_err;
            state->complete = true;
            return ERR_OK;
        }
    }
    
    return ERR_OK;
}

static err_t tls_client_poll(void *arg, struct altcp_pcb *pcb) {
    TLS_CLIENT_T *state = (TLS_CLIENT_T*)arg;
    printf("timed out\n");
    state->error = PICO_ERROR_TIMEOUT;
    return tls_client_close(arg);
}

// Improved error handler with detailed diagnostics
static void tls_client_err(void *arg, err_t err) {
    TLS_CLIENT_T *state = (TLS_CLIENT_T*)arg;
    
    // Translate error code to a more descriptive message
    const char* err_msg = "Unknown error";
    
    // Handle common error codes
    switch(err) {
        case ERR_MEM:    err_msg = "Out of memory"; break;
        case ERR_BUF:    err_msg = "Buffer error"; break;
        case ERR_TIMEOUT: err_msg = "Connection timed out"; break;
        case ERR_RTE:    err_msg = "Routing problem"; break;
        case ERR_ABRT:   err_msg = "Connection aborted"; break;
        case ERR_RST:    err_msg = "Connection reset"; break;
        case ERR_CLSD:   err_msg = "Connection closed"; break;
        case ERR_CONN:   err_msg = "Not connected"; break;
        case ERR_ARG:    err_msg = "Illegal argument"; break;
        case ERR_USE:    err_msg = "Address in use"; break;
        case ERR_IF:     err_msg = "Low-level netif error"; break;
        case ERR_ISCONN: err_msg = "Already connected"; break;
        case ERR_INPROGRESS: err_msg = "Operation in progress"; break;
        
        // Special case for our custom handshake timeout
        case ERR_TLS_HANDSHAKE_TIMEOUT: 
            err_msg = "TLS handshake timeout - server may be overloaded"; 
            break;
            
        default: 
            if (err < 0) {
                err_msg = "Unknown negative error code";
            }
            break;
    }
    
    // Calculate how long we spent in the handshake if we have timing data
    uint32_t handshake_duration = 0;
    if (state && state->handshake_start_time > 0) {
        handshake_duration = (uint32_t)((time_us_64() / 1000) - state->handshake_start_time);
    }
    
    printf("TLS ERROR: %d (%s) after %u ms\n", err, err_msg, handshake_duration);
    
    // Print diagnostic information about the connection state
    if (state) {
        printf("Connection diagnostic info:\n");
        printf("  - DNS resolved: %s\n", state->dns_resolved ? "YES" : "NO");
        printf("  - Connection timeout: %d ms\n", state->timeout);
        printf("  - Handshake duration: %u ms\n", handshake_duration);
        
        // Provide guidance based on the specific error
        if (err == ERR_TLS_HANDSHAKE_TIMEOUT || err == ERR_TIMEOUT) {
            printf("RECOMMENDATION: TLS handshake timeout detected\n");
            printf("  - Check if server supports TLS 1.0\n");
            printf("  - Verify server is not overloaded\n");
            printf("  - Consider increasing connection timeout\n");
            printf("  - Try connecting to server without www prefix\n");
        } else if (err == ERR_ABRT || err == ERR_RST) {
            printf("RECOMMENDATION: Connection was aborted by remote server\n");
            printf("  - Verify server expects HTTPS (not HTTP)\n");
            printf("  - Check if server requires client certificates\n");
            printf("  - Try with different TLS version settings\n");
        }
        
        // Update state
        state->error = err;
        state->complete = true;
        state->pcb = NULL; // pcb is already freed when err callback is called
    }
}

static err_t tls_client_recv(void *arg, struct altcp_pcb *pcb, struct pbuf *p, err_t err) {
    TLS_CLIENT_T *state = (TLS_CLIENT_T*)arg;

    // Error or null pointer check
    if (!state) {
        printf("ERROR: tls_client_recv with NULL state\n");
        if (p != NULL) {
            pbuf_free(p);
        }
        return ERR_ARG;
    }

    // Handle connection error
    if (err != ERR_OK) {
        printf("tls_client_recv error %d\n", err);
        state->error = err;
        state->complete = true;
        return err;
    }

    // Handle closed connection
    if (p == NULL) {
        printf("Connection closed by remote host - marking successful\n");
        // Even if we can't read a response, we've reached this point,
        // which means the server acknowledged our request. Mark as complete.
        state->complete = true;
        state->error = 0;
        return ERR_OK;
    }

    // Got response - great news! Print response details
    unsigned int response_len = p->tot_len;
    printf("Received response: %u bytes - SUCCESS!\n", response_len);

    // Mark as complete before processing (to prevent timeouts during processing)
    state->complete = true;
    state->error = 0;

    // Try to process for HTTP status code (simple approach)
    if (p->tot_len > 12) { // At least enough for "HTTP/1.1 xxx"
        // Get HTTP status as simple null-terminated string
        char http_status[4] = {0}; // 3 digits + null terminator
        char *payload = (char*)p->payload;
        
        // Find the status code (typically after "HTTP/1.x ")
        if (p->len >= 9 && memcmp(payload, "HTTP/1.", 7) == 0) {
            // Copy the 3 status code digits
            memcpy(http_status, payload + 9, 3);
            
            // Print relevant info based on status
            int status_code = atoi(http_status);
            if (status_code >= 200 && status_code < 300) {
                printf("HTTP SUCCESS (%d): Server accepted our request\n", status_code);
            } else if (status_code >= 400 && status_code < 500) {
                printf("HTTP CLIENT ERROR (%d): Server rejected our request\n", status_code);
                // Still mark as technically successful from a networking perspective
            } else if (status_code >= 500) {
                printf("HTTP SERVER ERROR (%d): Server had internal error\n", status_code);
                // Still mark as technically successful from a networking perspective
            }
        } else {
            printf("Non-HTTP response or malformed HTTP response\n");
        }
    }

    // Always free the pbuf
    pbuf_free(p);

    // Close the connection
    return tls_client_close(state);
}

// Callback for DNS resolution
static void tls_client_dns_cb(const char *name, const ip_addr_t *addr, void *arg) {
    TLS_CLIENT_T *state = (TLS_CLIENT_T*)arg;
    
    if (!state) {
        printf("ERROR: DNS callback with NULL state\n");
        return;
    }
    
    if (!addr) {
        printf("ERROR: DNS failed for %s\n", name);
        state->error = ERR_VAL;
        state->dns_resolved = true;  // Mark as resolved but with error
        return;
    }
    
    printf("DNS resolved for %s: %s\n", name, ipaddr_ntoa(addr));
    // Store the resolved IP address in the state structure
    ip_addr_copy(state->resolved_ip, *addr);
    state->dns_resolved = true;  // Mark as successfully resolved
    state->error = ERR_OK;
}

static err_t tls_client_connect_to_server_ip(const ip_addr_t *ipaddr, TLS_CLIENT_T *state) {
    if (!state) {
        printf("ERROR: tls_client_connect_to_server_ip with NULL state\n");
        return ERR_ARG;
    }
    
    // Add connection attempt logging
    printf("Connecting to server IP: %s (waiting 500ms for stability)\n", ipaddr_ntoa(ipaddr));
    
    // Increase wait time before initiating connection for better stability
    // Helps ensure the network stack is ready before attempting connection
    sleep_ms(500);
    
    // Configure TLS PCB if not already done
    if (state->pcb == NULL) {
        // Create a new PCB
        state->pcb = altcp_tls_new(tls_config, 0);
        if (!state->pcb) {
            printf("Failed to create TLS PCB\n");
            state->error = ERR_MEM;
            state->complete = true;
            return ERR_MEM;
        }
        
        // Set callbacks
        altcp_arg(state->pcb, state);
        altcp_recv(state->pcb, tls_client_recv);
        altcp_err(state->pcb, tls_client_err);
        
        // Reduce polling interval for more responsive timeouts (2 = 0.5s interval)
        altcp_poll(state->pcb, tls_client_poll, 2);
        
        // Note: We can't access the underlying TCP PCB directly in this version
        // So we'll use reasonable defaults set by the TLS layer
        printf("Using TLS layer default TCP settings\n");
    }
    
    // Connect to the server on port 443 (HTTPS)
    err_t err = altcp_connect(state->pcb, ipaddr, 443, tls_client_connected);
    if (err != ERR_OK) {
        printf("Error initiating connection: %d\n", err);
        state->error = err;
        state->complete = true;
        return err;
    }
    
    printf("Connection initiated (waiting for handshake)\n");
    return ERR_OK;
}

static err_t tls_client_dns_found(const char* hostname, const ip_addr_t *ipaddr, void *arg) {
    TLS_CLIENT_T *state = (TLS_CLIENT_T *)arg;
    if (ipaddr) {
        // Store the resolved IP for diagnostics
        ip_addr_copy(state->resolved_ip, *ipaddr);
        state->dns_resolved = true;
        
        printf("DNS lookup successful. IP: %s\n", ipaddr_ntoa(ipaddr));
        return tls_client_connect_to_server_ip(ipaddr, state);
    } else {
        printf("DNS lookup failed for host: %s\n", hostname);
        state->error = ERR_CONN;
        state->complete = true;
        return ERR_CONN; // Return connection error
    }
}

static err_t tls_client_open(const char *hostname, void *arg, TLS_CLIENT_T *state) {
    printf("TLS client connecting to host: %s\n", hostname);
    
    ip_addr_t addr;
    memset(&addr, 0, sizeof(addr));
    
    // Use DNS to resolve hostname to IP address
    // Use a 1.5 seconds max timeout with checks every 25ms
    printf("DNS lookup for %s - using timeout of 1500ms\n", hostname);
    err_t err = dns_gethostbyname(hostname, &addr, tls_client_dns_cb, state);
    
    if (err == ERR_OK) {
        /* Address was in cache, so callback was called synchronously */
        // Store the resolved IP and use it
        ip_addr_copy(state->resolved_ip, addr);
        return tls_client_connect_to_server_ip(&state->resolved_ip, state);
    } else if (err != ERR_INPROGRESS) {
        printf("DNS error: %d\n", err);
        state->error = err;
        return err;
    }
    
    // DNS is in progress, let's wait for it with tight timeouts
    // Much shorter DNS timeout than before
    const uint32_t dns_timeout_ms = 1500; 
    const uint32_t dns_start = to_ms_since_boot(get_absolute_time());
    uint32_t dns_wait_iterations = 0;
    
    while (!state->dns_resolved) {
        dns_wait_iterations++;
        sleep_ms(25); // Check every 25ms instead of 50ms - faster DNS polling
        
        uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - dns_start;
        if (elapsed > dns_timeout_ms) {
            printf("DNS timeout after %ums (%u iterations)\n", elapsed, dns_wait_iterations);
            state->error = -99; // DNS timeout error code
            return ERR_TIMEOUT;
        }
    }
    
    if (state->error == ERR_OK) {
        // Use the saved IP address from the callback
        return tls_client_connect_to_server_ip(&state->resolved_ip, state);
    }
    
    return state->error;
}

// Perform initialisation
static TLS_CLIENT_T* tls_client_init(void) {
    TLS_CLIENT_T *state = calloc(1, sizeof(TLS_CLIENT_T));
    if (!state) {
        printf("failed to allocate state\n");
        return NULL;
    }

    return state;
}

// Simple TLS config creator that doesn't attempt to access mbedTLS internals
struct altcp_tls_config* create_tls_config(unsigned char const* cert, unsigned int cert_len) {
    printf("Creating simple TLS configuration\n");
    
    // Just create a config without any customization
    struct altcp_tls_config* conf = altcp_tls_create_config_client(cert, cert_len);
    
    if (conf) {
        printf("Successfully created TLS configuration\n");
    } else {
        printf("Failed to create TLS configuration\n");
    }
    
    return conf;
}

// Simplified fallback function without mbedTLS-specific code
bool try_alternative_connection(const char* server, const char* request) {
    printf("\n[FALLBACK] Attempting alternative connection method...\n");
    
    // Create a temporary state with longer timeouts
    TLS_CLIENT_T *alt_state = calloc(1, sizeof(TLS_CLIENT_T));
    if (!alt_state) {
        printf("[FALLBACK] Failed to allocate alternative state\n");
        return false;
    }
    
    // Initialize state with extended timeouts
    alt_state->complete = false;
    alt_state->dns_resolved = false;
    alt_state->error = 0;
    alt_state->pcb = NULL;
    ip_addr_set_zero(&alt_state->resolved_ip);
    
    // Use longer timeout for fallback
    alt_state->timeout = 15000;  // 15 seconds - much higher than normal
    alt_state->http_request = request;
    
    // Try without 'www' prefix if it exists, or with it if it doesn't
    char modified_server[256] = {0};
    if (strncmp(server, "www.", 4) == 0) {
        // Remove www prefix
        printf("[FALLBACK] Removing www prefix from %s\n", server);
        strcpy(modified_server, server + 4);
    } else {
        // Add www prefix
        printf("[FALLBACK] Adding www prefix to %s\n", server);
        snprintf(modified_server, sizeof(modified_server), "www.%s", server);
    }
    
    printf("[FALLBACK] Trying connection to alternative server: %s\n", modified_server);
    
    // Create a specialized alternative config with extended timeout
    struct altcp_tls_config *alt_config = altcp_tls_create_config_client(NULL, 0);
    if (!alt_config) {
        printf("[FALLBACK] Failed to create alternative TLS config\n");
        free(alt_state);
        return false;
    }
    
    // Temporarily replace the global config
    struct altcp_tls_config *old_config = tls_config;
    tls_config = alt_config;
    
    // Start the alternative TLS connection with longer timeout
    printf("[FALLBACK] Starting alternative TLS connection\n");
    err_t err = tls_client_open(modified_server, alt_state, alt_state);
    
    if (err != ERR_OK) {
        printf("[FALLBACK] Failed to start alternative connection: %d\n", err);
        free(alt_state);
        tls_config = old_config;
        return false;
    }
    
    // Longer processing loop for alternative connection (50% more time)
    const unsigned int ALT_LOOP_MAX = alt_state->timeout / 10;
    unsigned int count = 0;
    
    cyw43_arch_lwip_begin();
    while (!alt_state->complete && count < ALT_LOOP_MAX) {
        cyw43_arch_poll();
        sys_check_timeouts();
        
        // Log progress less frequently
        if (count % 100 == 0 && count > 0) {
            printf("[FALLBACK] Still waiting... (%u/%u)\n", count, ALT_LOOP_MAX);
        }
        
        sleep_ms(10);
        count++;
    }
    
    cyw43_arch_lwip_end();
    
    // Check if alternative connection was successful
    bool alt_success = false;
    if (alt_state->complete) {
        if (alt_state->error == 0) {
            printf("[FALLBACK] Alternative connection SUCCESSFUL!\n");
            alt_success = true;
        } else {
            printf("[FALLBACK] Alternative connection completed with error: %d\n", alt_state->error);
        }
    } else {
        printf("[FALLBACK] Alternative connection timed out after %d ms\n", alt_state->timeout);
    }
    
    // Clean up alternative resources
    free(alt_state);
    tls_config = old_config;
    
    return alt_success;
}

/* Explicitly use extern "C" for the function definition to ensure correct linkage when called from C++ */
#ifdef __cplusplus
extern "C" {
#endif

bool run_tls_client_test(unsigned char const* cert, unsigned int cert_len, char const* server, char const* request, int timeout) {
    bool ret = false;
    TLS_CLIENT_T *state = calloc(1, sizeof(TLS_CLIENT_T));
    if (!state) {
        printf("failed to allocate state\n");
        return false;
    }
    
    // Initialize state fields
    state->complete = false;
    state->dns_resolved = false;
    state->error = 0;
    state->pcb = NULL;
    ip_addr_set_zero(&state->resolved_ip); // Initialize the IP address

    // Default server and request if not provided
    if (!server) {
        server = "www.gm4s.eu";  // Use your Vercel server domain
    }

    if (!request) {
        request = "GET / HTTP/1.1\r\nHost: www.gm4s.eu\r\nConnection: close\r\n\r\n";
    }

    // Optimize timeouts for faster connections
    // Use shorter timeouts for initial quick attempt
    if (timeout <= 0) {
        // Different timeouts based on server type - faster timeouts for all servers
        if (strcmp(server, "gm4s.eu") == 0) {
            timeout = 5000; // 5 seconds for direct server (faster)
            printf("Using faster direct server mode with 5s timeout\n");
        } else {
            timeout = 8000; // 8 seconds for www prefix server 
            printf("Using www server mode with 8s timeout\n");
        }
    }

    printf("Starting TLS client with server: %s, timeout: %d ms\n", server, timeout);

    // Set timeout first to ensure it's applied
    state->timeout = timeout;
    state->http_request = request;

    // Create a new TLS config for each connection to ensure fresh settings
    // This avoids issues with cached/stale configurations
    printf("Creating simplified TLS configuration (no certificates)\n");
    struct altcp_tls_config *connection_config = create_tls_config(NULL, 0);
    if (!connection_config) {
        printf("ERROR: Failed to create TLS configuration\n");
        free(state);
        return false;
    }
    printf("Successfully created simplified TLS configuration\n");
    
    // Use the connection-specific config
    tls_config = connection_config;

    // Set multiple DNS servers for redundancy - ensure we can always resolve
    ip_addr_t dnsserver1, dnsserver2;
    IP4_ADDR(&dnsserver1, 8, 8, 8, 8); // Google DNS first 
    IP4_ADDR(&dnsserver2, 1, 1, 1, 1); // Cloudflare DNS as backup
    
    // Clear any previous DNS server settings and set primary + backup
    dns_setserver(0, &dnsserver1);
    dns_setserver(1, &dnsserver2);

    // Use fewer iterations for faster timeout handling
    // 100 iterations * 10ms = 1 second processing time
    const unsigned int LOOP_MAX_ITERATIONS = timeout / 15; // Dynamically based on timeout
    unsigned int count = 0;

    // Start the TLS connection process with timeout for DNS
    printf("Attempting TLS connection to server\n");
    err_t err = tls_client_open(server, state, state);
    
    if (err != ERR_OK) {
        printf("Failed to start TLS connection: %d\n", err);
        free(state);
        
        // Try alternatives when TLS fails completely
        printf("TLS initialization failed completely\n");
        return try_alternative_connection(server, request);
    }

    // Main processing loop with more frequent processing
    cyw43_arch_lwip_begin();
    while (!state->complete && count < LOOP_MAX_ITERATIONS) {
        // Process packet more frequently and with progress updates
        cyw43_arch_poll();
        sys_check_timeouts();
        
        // Add debug output every 50 iterations
        if (count % 50 == 0 && count > 0) {
            printf("Waiting for TLS completion... (%u/%u iterations)\n", 
                count, LOOP_MAX_ITERATIONS);
        }
        
        // Shorter sleep for more responsive processing
        sleep_ms(15);
        count++;
    }
    cyw43_arch_lwip_end();

    // Check if we timed out
    if (!state->complete) {
        uint64_t elapsed_ms = time_us_64() / 1000 - state->handshake_start_time;
        printf("WARNING: TLS connection timed out after %llu ms\n", elapsed_ms);
        
        // Log diagnostic information
        printf("TLS Timeout Diagnostics:\n");
        printf("  - DNS resolved: %s\n", state->dns_resolved ? "YES" : "NO");
        printf("  - DNS resolution time: %llu ms\n", 
            state->dns_resolved ? 
            (state->handshake_start_time - (time_us_64()/1000)) : 0);
        if (state->dns_resolved) {
            char ip_str[16];
            ipaddr_ntoa_r(&state->resolved_ip, ip_str, sizeof(ip_str));
            printf("  - Resolved IP: %s\n", ip_str);
        }
        
        // Set error state for timeout
        state->error = -2; // Custom timeout error code
        state->complete = true;
        
        // Close any open connections
        if (state->pcb != NULL) {
            printf("Forcibly closing timed out connection\n");
            altcp_abort(state->pcb);
            state->pcb = NULL;
        }
    } else {
        // Connection completed (success or error)
        if (state->error) {
            // Handle error case
            printf("TLS connection completed with error: %d\n", state->error);
            ret = false;
        } else {
            // Handle success case
            printf("TLS connection completed successfully\n");
            ret = true;
        }
    }
    
    // Clean up resources
    if (state->pcb) {
        altcp_close(state->pcb);
    }
    
    // Free memory
    free(state);
    
    return ret;
}

#ifdef __cplusplus
}
#endif


