#ifndef MBEDTLS_CONFIG_TLS_CLIENT_H
#define MBEDTLS_CONFIG_TLS_CLIENT_H

#include "mbedtls_config_examples_common.h"

// Make sure SSL client mode is enabled
#define MBEDTLS_SSL_CLI_C

// Enable TLS functionality
#define MBEDTLS_SSL_TLS_C

// Enable certificate support
#define MBEDTLS_X509_CRT_PARSE_C
#define MBEDTLS_X509_USE_C

// Enable necessary crypto
#define MBEDTLS_AES_C
#define MBEDTLS_SHA256_C
#define MBEDTLS_MD_C
#define MBEDTLS_PK_C

#endif /* MBEDTLS_CONFIG_TLS_CLIENT_H */ 