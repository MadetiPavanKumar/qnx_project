#ifndef TELEMETRY_TRANSPORT_H
#define TELEMETRY_TRANSPORT_H

#include <stddef.h>
#include <stdint.h>

/* ================================================================
 * NETWORK TRANSPORT - the only file in this project that does
 * sockets/TLS/HTTP (items 13/31/32/33/42). ONLY EVER called from
 * system/telemetry_service.c, on the Telemetry Service thread -
 * every timeout below is bounded (config.h), but even a bounded
 * multi-second wait here would be unacceptable on any real-time
 * task's thread, which is why this is never called from anywhere
 * else.
 *
 * Uses OpenSSL (libssl/libcrypto) for TLS over a manually-managed
 * BSD socket (not curl, not system() - item 31), with a persistent
 * connection reused across calls (item 32) and certificate
 * verification always enabled (SSL_CTX_set_verify(..., SSL_VERIFY_
 * PEER, ...) + the platform's default trust store - see telemetry_
 * transport.c). Credentials come from the environment
 * (SUPABASE_URL, SUPABASE_ANON_KEY) - never hard-coded, never logged
 * (item 42).
 *
 * IMPORTANT - NOT VERIFIED ON QNX: this was written against, and
 * compiles/links cleanly against, standard OpenSSL 3.x headers/libs
 * (verified in this environment). Whether YOUR QNX 8.0 image has
 * libssl/libcrypto available (and their headers, and a CA bundle to
 * verify against) has not been checked - that's a build-environment
 * fact this pass has no way to confirm. If QNX SDP 8.0's io-pkt/
 * pkgsrc doesn't have them for your image, this file's OpenSSL calls
 * are the only thing that would need to change - the header's
 * function contracts below (telemetry_transport_post() in
 * particular) are what telemetry_service.c depends on, not any
 * OpenSSL type, so swapping in a different TLS library only touches
 * this one file's implementation.
 * ================================================================ */

/* Reads SUPABASE_URL (e.g. "https://xxxxx.supabase.co") and
   SUPABASE_ANON_KEY from the environment. Call once, before the
   Telemetry Service thread starts (see main.c). Returns 1 if both
   are set and the URL parses as host+https; returns 0 otherwise -
   callers should treat 0 as "stay OFFLINE forever, don't retry" per
   item 37 (the robot must run fine with zero cloud config), not a
   fatal error. Logs exactly one explanatory message (never the key
   itself) via logger_log_err() when returning 0. */
int telemetry_transport_init(void);

/* 1 if telemetry_transport_init() found usable config, 0 otherwise -
   telemetry_service.c checks this once per cycle instead of calling
   init() repeatedly. */
int telemetry_transport_is_configured(void);

typedef struct
{
    int      success;      /* 1 = 2xx HTTP response actually received */
    int      http_status;  /* 0 if no response was ever received (connect/
                               send/TLS failure, or timeout) */
    uint32_t connect_ms;   /* 0 if an existing persistent connection was
                               reused - no new connect happened */
    uint32_t send_ms;
    uint32_t response_ms;  /* time spent waiting for + reading the response */
} telemetry_transport_result_t;

/* Uploads `json_body` as a single PostgREST request to
   https://<host>/rest/v1/<table>, with apikey/Authorization headers
   from the configured anon key. Reuses the persistent TLS connection
   when possible (item 32); on any I/O error the connection is closed
   and ONE fresh reconnect is attempted within this same call before
   giving up for this call (item 36's retry is telemetry_service.c's
   job at the next cycle, not this function's). Every network wait is
   bounded by TELEMETRY_CONNECT_TIMEOUT_MS/SEND_TIMEOUT_MS/
   RECV_TIMEOUT_MS (config.h) - never blocks indefinitely (item 33).
   `out` is always fully populated, even on failure. Only ever called
   from the Telemetry Service thread. */
void telemetry_transport_post(const char *table, const char *json_body,
                               size_t json_len,
                               telemetry_transport_result_t *out);

/* Closes the persistent connection, if any. telemetry_service.c calls
   this after a failed upload, so the NEXT attempt starts from a known
   clean state rather than continuing to reuse a socket that may be
   half-broken. Safe to call even if nothing is connected. */
void telemetry_transport_close(void);

#endif /* TELEMETRY_TRANSPORT_H */
