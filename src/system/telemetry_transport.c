#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netdb.h>

#include <openssl/ssl.h>
#include <openssl/err.h>

#include "telemetry_transport.h"
#include "../common/config.h"
#include "../common/logger.h"

/* ---- Configuration, read once by telemetry_transport_init() ---- */
#define TT_HOST_MAX  256
#define TT_KEY_MAX   512

static char host[TT_HOST_MAX]  = "";
static char api_key[TT_KEY_MAX] = "";
static int  configured = 0;

/* ---- Persistent connection state - only ever touched from the
   Telemetry Service thread (see telemetry_transport.h), so no lock
   is needed here. ---- */
static SSL_CTX *ssl_ctx  = NULL;
static SSL      *ssl      = NULL;
static int       sock_fd  = -1;

static uint32_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL));
}

int telemetry_transport_is_configured(void)
{
    return configured;
}

void telemetry_transport_close(void)
{
    if (ssl)
    {
        SSL_shutdown(ssl);
        SSL_free(ssl);
        ssl = NULL;
    }
    if (sock_fd >= 0)
    {
        close(sock_fd);
        sock_fd = -1;
    }
}

/* Strips "https://" and any trailing path/slash from `url`, writing
   just the host into `out` (size out_size). Returns 0 on success, -1
   if `url` isn't "https://something". Supabase's REST endpoint is
   always TLS on 443 - there is no http:// case to support. */
static int parse_host(const char *url, char *out, size_t out_size)
{
    static const char prefix[] = "https://";
    size_t prefix_len = sizeof(prefix) - 1;

    if (strncmp(url, prefix, prefix_len) != 0) return -1;

    const char *host_start = url + prefix_len;
    const char *end = host_start;
    while (*end && *end != '/' && *end != ':') end++;

    size_t len = (size_t)(end - host_start);
    if (len == 0 || len >= out_size) return -1;

    memcpy(out, host_start, len);
    out[len] = '\0';
    return 0;
}

int telemetry_transport_init(void)
{
    const char *url = getenv("SUPABASE_URL");
    const char *key = getenv("SUPABASE_ANON_KEY");

    if (!url || !key || url[0] == '\0' || key[0] == '\0')
    {
        logger_log_err("[Telemetry] SUPABASE_URL / SUPABASE_ANON_KEY not set - "
                       "telemetry will stay OFFLINE (local robot operation is "
                       "unaffected). Set both environment variables and "
                       "restart to enable cloud telemetry.");
        configured = 0;
        return 0;
    }

    if (parse_host(url, host, sizeof(host)) != 0)
    {
        logger_log_err("[Telemetry] SUPABASE_URL must look like "
                       "'https://<project>.supabase.co' - telemetry will "
                       "stay OFFLINE");
        configured = 0;
        return 0;
    }

    snprintf(api_key, sizeof(api_key), "%s", key); /* never logged */

    SSL_CTX *new_ctx = SSL_CTX_new(TLS_client_method());
    if (!new_ctx)
    {
        logger_log_err("[Telemetry] SSL_CTX_new failed - telemetry will "
                       "stay OFFLINE");
        configured = 0;
        return 0;
    }

    /* Certificate verification is NEVER disabled - a robot uploading
       telemetry to an unverified endpoint is a real security risk,
       not just a correctness nicety (item 42's spirit applies here
       too, not just to the key itself). Uses the platform's default
       trust store; if your QNX image has no CA bundle installed,
       this will make every connect attempt fail closed (telemetry
       stays OFFLINE) rather than silently connecting insecurely -
       install a CA bundle rather than relaxing this. */
    SSL_CTX_set_verify(new_ctx, SSL_VERIFY_PEER, NULL);
    if (!SSL_CTX_set_default_verify_paths(new_ctx))
    {
        logger_log_err("[Telemetry] SSL_CTX_set_default_verify_paths failed "
                       "- no CA trust store found. Telemetry will attempt "
                       "connections but they will fail TLS verification "
                       "(fails closed, not insecurely) until a CA bundle is "
                       "available on this system.");
    }

    ssl_ctx = new_ctx;
    configured = 1;
    return 1;
}

/* Waits up to `timeout_ms` for `fd` to become ready. `for_write`
   selects POLLOUT-equivalent (connect completion) vs POLLIN
   (response data). Returns 1 if ready, 0 on timeout, -1 on error. */
static int wait_ready(int fd, int for_write, int timeout_ms)
{
    fd_set set;
    FD_ZERO(&set);
    FD_SET(fd, &set);

    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int rc = for_write ? select(fd + 1, NULL, &set, NULL, &tv)
                        : select(fd + 1, &set, NULL, NULL, &tv);
    if (rc < 0) return -1;
    if (rc == 0) return 0;
    return 1;
}

/* TCP connect with a real, bounded timeout (item 33) - the socket is
   temporarily non-blocking just for this, then switched back to
   blocking with SO_RCVTIMEO/SO_SNDTIMEO set, which is what bounds
   every subsequent SSL_connect()/SSL_write()/SSL_read() below (they
   all go through this same fd's send()/recv()). Returns 0 on
   success, -1 on failure (fd is closed on failure). */
static int tcp_connect(const char *hostname, int timeout_ms)
{
    struct addrinfo hints, *res = NULL;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    if (getaddrinfo(hostname, "443", &hints, &res) != 0 || !res)
    {
        return -1;
    }

    int fd = -1;
    int connected = 0;

    for (struct addrinfo *ai = res; ai != NULL; ai = ai->ai_next)
    {
        fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0) continue;

        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);

        int rc = connect(fd, ai->ai_addr, ai->ai_addrlen);
        if (rc == 0)
        {
            connected = 1;
        }
        else if (errno == EINPROGRESS)
        {
            if (wait_ready(fd, 1, timeout_ms) == 1)
            {
                int soerr = 0;
                socklen_t len = sizeof(soerr);
                if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &soerr, &len) == 0 && soerr == 0)
                {
                    connected = 1;
                }
            }
        }

        if (connected)
        {
            fcntl(fd, F_SETFL, flags); /* back to blocking */

            struct timeval tv;
            tv.tv_sec  = TELEMETRY_SEND_TIMEOUT_MS / 1000;
            tv.tv_usec = (TELEMETRY_SEND_TIMEOUT_MS % 1000) * 1000;
            setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

            tv.tv_sec  = TELEMETRY_RECV_TIMEOUT_MS / 1000;
            tv.tv_usec = (TELEMETRY_RECV_TIMEOUT_MS % 1000) * 1000;
            setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

            break;
        }

        close(fd);
        fd = -1;
    }

    freeaddrinfo(res);

    if (!connected) return -1;

    sock_fd = fd;
    return 0;
}

/* Establishes a fresh TLS connection (TCP connect + TLS handshake),
   replacing any existing one. Returns 0 on success, -1 on failure
   (all state cleaned up on failure - telemetry_transport_close() is
   safe to call again afterward). */
static int reconnect(void)
{
    telemetry_transport_close();

    if (tcp_connect(host, TELEMETRY_CONNECT_TIMEOUT_MS) != 0)
    {
        return -1;
    }

    ssl = SSL_new(ssl_ctx);
    if (!ssl)
    {
        close(sock_fd);
        sock_fd = -1;
        return -1;
    }

    SSL_set_fd(ssl, sock_fd);
    SSL_set_tlsext_host_name(ssl, host); /* SNI */

    X509_VERIFY_PARAM *param = SSL_get0_param(ssl);
    X509_VERIFY_PARAM_set1_host(param, host, 0); /* hostname verification */

    if (SSL_connect(ssl) != 1)
    {
        telemetry_transport_close();
        return -1;
    }

    return 0;
}

/* Sends `len` bytes in full, bounded by the socket's SO_SNDTIMEO
   (set in tcp_connect()). Returns 0 on success, -1 on any error
   (including a partial-then-stalled write). */
static int send_all(const char *data, size_t len)
{
    size_t sent = 0;
    while (sent < len)
    {
        int n = SSL_write(ssl, data + sent, (int)(len - sent));
        if (n <= 0) return -1;
        sent += (size_t)n;
    }
    return 0;
}

/* Reads an HTTP/1.1 response into `buf` (capacity `cap`), stopping
   once it has the full header block AND either a Content-Length
   worth of body or (absent that header) the header block itself is
   taken as the complete response - correct for PostgREST's
   "Prefer: return=minimal" replies (empty body), which is the only
   response shape this client needs to handle; NOT a general-purpose
   HTTP client. Returns total bytes read, or -1 on error/timeout
   (bounded by SO_RCVTIMEO). Null-terminates `buf` on success. */
static int recv_response(char *buf, size_t cap)
{
    size_t total = 0;
    int header_end = -1;
    long content_length = -1;

    while (total < cap - 1)
    {
        int n = SSL_read(ssl, buf + total, (int)(cap - 1 - total));
        if (n <= 0) break; /* error, timeout, or peer closed */

        total += (size_t)n;
        buf[total] = '\0';

        if (header_end < 0)
        {
            char *marker = strstr(buf, "\r\n\r\n");
            if (marker)
            {
                header_end = (int)(marker - buf) + 4;

                char *cl = strstr(buf, "Content-Length:");
                if (!cl) cl = strstr(buf, "content-length:");
                if (cl) content_length = strtol(cl + 15, NULL, 10);
            }
        }

        if (header_end >= 0)
        {
            long body_so_far = (long)total - header_end;
            if (content_length <= 0 || body_so_far >= content_length)
            {
                return (int)total; /* complete response */
            }
        }
    }

    return (total > 0) ? (int)total : -1;
}

static int parse_status(const char *response)
{
    /* "HTTP/1.1 201 Created\r\n..." */
    const char *sp = strchr(response, ' ');
    if (!sp) return 0;
    return (int)strtol(sp + 1, NULL, 10);
}

static int do_one_request(const char *table, const char *json_body, size_t json_len,
                           telemetry_transport_result_t *out)
{
    char request[TELEMETRY_MAX_PAYLOAD_BYTES + 512];
    int req_len = snprintf(request, sizeof(request),
        "POST /rest/v1/%s HTTP/1.1\r\n"
        "Host: %s\r\n"
        "apikey: %s\r\n"
        "Authorization: Bearer %s\r\n"
        "Content-Type: application/json\r\n"
        "Prefer: return=minimal\r\n"
        "Connection: keep-alive\r\n"
        "Content-Length: %zu\r\n"
        "\r\n"
        "%s",
        table, host, api_key, api_key, json_len, json_body);

    if (req_len < 0 || (size_t)req_len >= sizeof(request)) return -1;

    uint32_t send_start = now_ms();
    if (send_all(request, (size_t)req_len) != 0) return -1;
    out->send_ms = now_ms() - send_start;

    char response[TELEMETRY_MAX_RESPONSE_BYTES];
    uint32_t recv_start = now_ms();
    int n = recv_response(response, sizeof(response));
    out->response_ms = now_ms() - recv_start;

    if (n <= 0) return -1;

    out->http_status = parse_status(response);
    out->success = (out->http_status >= 200 && out->http_status < 300);
    return 0;
}

void telemetry_transport_post(const char *table, const char *json_body,
                               size_t json_len,
                               telemetry_transport_result_t *out)
{
    memset(out, 0, sizeof(*out));

    if (!configured)
    {
        return; /* stays all-zero/failure - see telemetry_service.c */
    }

    if (!ssl || sock_fd < 0)
    {
        uint32_t connect_start = now_ms();
        if (reconnect() != 0)
        {
            out->connect_ms = now_ms() - connect_start;
            return;
        }
        out->connect_ms = now_ms() - connect_start;
    }

    if (do_one_request(table, json_body, json_len, out) == 0) return;

    /* One retry on a fresh connection - covers the common "server
       closed our idle keep-alive connection" case without treating
       it as a real failure (item 36 - this is not the exponential
       backoff, that's telemetry_service.c's job for a genuinely
       unreachable server; this is just "that particular socket was
       stale"). */
    telemetry_transport_close();
    uint32_t connect_start = now_ms();
    if (reconnect() != 0)
    {
        out->connect_ms += now_ms() - connect_start;
        memset(out, 0, sizeof(*out)); /* fully failed - report a clean zero result */
        return;
    }
    out->connect_ms += now_ms() - connect_start;

    if (do_one_request(table, json_body, json_len, out) != 0)
    {
        telemetry_transport_close(); /* don't leave a known-bad connection around */
        out->success = 0;
    }
}
