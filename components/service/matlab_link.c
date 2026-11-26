/*
 * ============================================================================
 * matlab_link.c - Trien khai giao thuc UDP 2 chieu cho He A & Burst SRAM He B
 * ============================================================================
 */

#include "matlab_link.h"
#include "config.h"
#include "mission_ctrl.h"
#include "sensor_service.h"
#include "wifi_ap.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"

#include <errno.h>
#include <string.h>

static const char *TAG = "matlab_link";

#define PKT_MAGIC_0         0xAAU
#define PKT_MAGIC_1         0x55U
#define PKT_HDR_SIZE        5U      /* magic0, magic1, type, len_lo, len_hi */
#define PKT_OVERHEAD        6U      /* hdr(5) + xor_cs(1) */

/* Command IDs (MATLAB -> ESP32) */
#define CMD_PING            0x01U
#define CMD_START_A         0x02U
#define CMD_START_B         0x03U
#define CMD_STOP            0x04U
#define CMD_SET_PARAMS      0x05U
#define CMD_GET_PARAMS      0x06U
#define CMD_BURST_REQ       0x07U
#define CMD_BURST_DONE      0x08U

/* Packet IDs (ESP32 -> MATLAB) */
#define PKT_ACK             0x80U
#define PKT_STATUS          0x81U
#define PKT_PARAMS          0x82U
#define PKT_LIVE_SAMPLE     0x83U
#define PKT_BURST_CHUNK     0x84U

#define MATLAB_TASK_STACK   4096
#define MATLAB_TASK_PRIO    5
#define MATLAB_TASK_CORE    0

static portMUX_TYPE        s_lock          = portMUX_INITIALIZER_UNLOCKED;
static int                 s_sock          = -1;
static TaskHandle_t        s_rx_task       = NULL;
static volatile bool       s_task_stop_req = false;
static volatile bool       s_peer_known    = false;
static struct sockaddr_in  s_peer_addr     = {0};
static app_link_state_t    s_app_state     = APP_STATE_IDLE;
static matlab_action_t     s_pending_act   = MATLAB_ACT_NONE;

typedef struct __attribute__((packed)) {
    uint8_t             app_state;
    uint8_t             sensor_profile;
    uint8_t             calib_valid;
    uint8_t             sta_connected;
    robot_run_summary_t summary;
} status_payload_t;

_Static_assert(sizeof(status_payload_t) == 48, "status_payload_t must be 48 bytes");

typedef struct __attribute__((packed)) {
    uint16_t start_chunk;
    uint16_t count_chunks;
} burst_req_payload_t;

static uint8_t calc_xor(const uint8_t *buf, size_t len)
{
    uint8_t cs = 0;
    for (size_t i = 0; i < len; i++) {
        cs ^= buf[i];
    }
    return cs;
}

static void send_framed_packet(uint8_t pkt_type, const void *payload, uint16_t payload_len)
{
    int sock;
    struct sockaddr_in dest;
    bool has_peer;

    portENTER_CRITICAL(&s_lock);
    sock     = s_sock;
    has_peer = s_peer_known;
    dest     = s_peer_addr;
    portEXIT_CRITICAL(&s_lock);

    if (sock < 0 || !has_peer) {
        return;
    }

    static uint8_t tx_buf[1200];
    if ((size_t)payload_len + PKT_OVERHEAD > sizeof(tx_buf)) {
        return;
    }

    tx_buf[0] = PKT_MAGIC_0;
    tx_buf[1] = PKT_MAGIC_1;
    tx_buf[2] = pkt_type;
    tx_buf[3] = (uint8_t)(payload_len & 0xFFU);
    tx_buf[4] = (uint8_t)((payload_len >> 8) & 0xFFU);
    if (payload_len > 0 && payload != NULL) {
        memcpy(&tx_buf[5], payload, payload_len);
    }
    tx_buf[5 + payload_len] = calc_xor(&tx_buf[2], (size_t)payload_len + 3U);

    sendto(sock, tx_buf, (size_t)payload_len + PKT_OVERHEAD, 0,
           (const struct sockaddr *)&dest, sizeof(dest));
}

static void send_ack(uint8_t cmd, uint8_t err_code)
{
    uint8_t pl[2] = { cmd, err_code };
    send_framed_packet(PKT_ACK, pl, sizeof(pl));
}

void matlab_link_send_status_now(void)
{
    status_payload_t st;
    memset(&st, 0, sizeof(st));

    portENTER_CRITICAL(&s_lock);
    st.app_state = (uint8_t)s_app_state;
    portEXIT_CRITICAL(&s_lock);

    st.sensor_profile = (uint8_t)sensor_service_get_profile();
    st.calib_valid    = sensor_service_has_valid_calib() ? 1U : 0U;
    st.sta_connected  = wifi_ap_has_station() ? 1U : 0U;
    data_logger_get_summary(&st.summary);
    /* Dam bao sample_count phan anh chinh xac bo dem SRAM hien tai khi o WAIT_BURST */
    if (st.app_state == (uint8_t)APP_STATE_WAIT_BURST) {
        st.summary.sample_count = data_logger_count();
    }

    send_framed_packet(PKT_STATUS, &st, sizeof(st));
}

static void send_params_now(void)
{
    outer_smc_params_t p;
    mission_ctrl_get_params(&p);
    send_framed_packet(PKT_PARAMS, &p, sizeof(p));
}

static void handle_burst_request(uint16_t start_chunk, uint16_t count_chunks)
{
    uint32_t total_samples = data_logger_count();
    uint16_t total_chunks  = (uint16_t)((total_samples + SYSTEM_B_BURST_CHUNK_SAMPLES - 1U)
                                        / SYSTEM_B_BURST_CHUNK_SAMPLES);

    if (total_chunks == 0) {
        /* Khong co mau nao trong SRAM: gui 1 goi rong bao hieu ket thuc */
        uint16_t hdr[3] = { 0, 0, 0 };
        send_framed_packet(PKT_BURST_CHUNK, hdr, sizeof(hdr));
        ESP_LOGW(TAG, "Burst requested but SRAM buffer is empty");
        return;
    }

    if (start_chunk >= total_chunks) {
        return;
    }

    uint16_t end_chunk;
    if (count_chunks == 0 || (uint32_t)start_chunk + count_chunks > total_chunks) {
        end_chunk = total_chunks;
    } else {
        end_chunk = (uint16_t)(start_chunk + count_chunks);
    }

    ESP_LOGI(TAG, "Bursting SRAM chunks [%u..%u) of %u (%lu samples total)",
             start_chunk, end_chunk, total_chunks, (unsigned long)total_samples);

    /* Buffer tren stack/static cho 1 chunk: 6 B header + 20 * 52 B = 1046 B */
    static uint8_t chunk_pl[6 + SYSTEM_B_BURST_CHUNK_SAMPLES * sizeof(robot_log_sample_t)];

    for (uint16_t c = start_chunk; c < end_chunk && !s_task_stop_req; c++) {
        uint32_t base_idx = (uint32_t)c * SYSTEM_B_BURST_CHUNK_SAMPLES;
        uint16_t cnt = 0;
        robot_log_sample_t *dst_samples = (robot_log_sample_t *)&chunk_pl[6];

        for (uint16_t i = 0; i < SYSTEM_B_BURST_CHUNK_SAMPLES; i++) {
            if (base_idx + i >= total_samples) break;
            if (data_logger_get(base_idx + i, &dst_samples[cnt])) {
                cnt++;
            }
        }

        chunk_pl[0] = (uint8_t)(c & 0xFFU);
        chunk_pl[1] = (uint8_t)((c >> 8) & 0xFFU);
        chunk_pl[2] = (uint8_t)(total_chunks & 0xFFU);
        chunk_pl[3] = (uint8_t)((total_chunks >> 8) & 0xFFU);
        chunk_pl[4] = (uint8_t)(cnt & 0xFFU);
        chunk_pl[5] = (uint8_t)((cnt >> 8) & 0xFFU);

        uint16_t pl_len = (uint16_t)(6U + (size_t)cnt * sizeof(robot_log_sample_t));
        send_framed_packet(PKT_BURST_CHUNK, chunk_pl, pl_len);

        /* Nhip nghi 4 ms giua cac goi UDP de tranh tran bo dem LwIP/PC */
        vTaskDelay(pdMS_TO_TICKS(4));
    }
}

static void process_command(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    app_link_state_t cur_state;
    portENTER_CRITICAL(&s_lock);
    cur_state = s_app_state;
    portEXIT_CRITICAL(&s_lock);

    switch (cmd) {
        case CMD_PING:
            matlab_link_send_status_now();
            break;

        case CMD_START_A:
            if (cur_state == APP_STATE_IDLE) {
                portENTER_CRITICAL(&s_lock);
                s_pending_act = MATLAB_ACT_START_A;
                portEXIT_CRITICAL(&s_lock);
                send_ack(cmd, 0);
                ESP_LOGI(TAG, "MATLAB command: START System A (5S + Live)");
            } else {
                send_ack(cmd, 1); /* Busy */
            }
            break;

        case CMD_START_B:
            if (cur_state == APP_STATE_IDLE) {
                portENTER_CRITICAL(&s_lock);
                s_pending_act = MATLAB_ACT_START_B;
                portEXIT_CRITICAL(&s_lock);
                send_ack(cmd, 0);
                ESP_LOGI(TAG, "MATLAB command: START System B (7S + SRAM Burst)");
            } else {
                send_ack(cmd, 1); /* Busy */
            }
            break;

        case CMD_STOP:
            mission_ctrl_request_stop();
            send_ack(cmd, 0);
            ESP_LOGW(TAG, "MATLAB command: EMERGENCY STOP");
            break;

        case CMD_SET_PARAMS:
            if (len == sizeof(outer_smc_params_t)) {
                outer_smc_params_t p;
                memcpy(&p, payload, sizeof(p));
                mission_ctrl_set_params(&p);
                send_ack(cmd, 0);
                send_params_now();
            } else {
                send_ack(cmd, 2); /* Bad length */
            }
            break;

        case CMD_GET_PARAMS:
            send_params_now();
            break;

        case CMD_BURST_REQ:
            if (cur_state == APP_STATE_WAIT_BURST || data_logger_count() > 0) {
                uint16_t start_c = 0, count_c = 0;
                if (len >= sizeof(burst_req_payload_t)) {
                    burst_req_payload_t req;
                    memcpy(&req, payload, sizeof(req));
                    start_c = req.start_chunk;
                    count_c = req.count_chunks;
                }
                send_ack(cmd, 0);
                handle_burst_request(start_c, count_c);
            } else {
                send_ack(cmd, 3); /* No burst data available */
            }
            break;

        case CMD_BURST_DONE:
            portENTER_CRITICAL(&s_lock);
            s_pending_act = MATLAB_ACT_BURST_DONE;
            portEXIT_CRITICAL(&s_lock);
            send_ack(cmd, 0);
            ESP_LOGI(TAG, "MATLAB confirmed BURST DONE");
            break;

        default:
            send_ack(cmd, 0xFF);
            break;
    }
}

static void matlab_rx_task(void *pvParameters)
{
    (void)pvParameters;
    uint8_t rx_buf[256];
    struct sockaddr_in src_addr;

    ESP_LOGI(TAG, "MATLAB UDP link listening on port %d -> TX port %d",
             MATLAB_CMD_PORT, MATLAB_DATA_PORT);

    while (!s_task_stop_req) {
        socklen_t socklen = sizeof(src_addr);
        int len = recvfrom(s_sock, rx_buf, sizeof(rx_buf), 0,
                           (struct sockaddr *)&src_addr, &socklen);

        if (len < (int)PKT_OVERHEAD) {
            continue;
        }

        if (rx_buf[0] != PKT_MAGIC_0 || rx_buf[1] != PKT_MAGIC_1) {
            continue;
        }

        uint8_t  cmd    = rx_buf[2];
        uint16_t pl_len = (uint16_t)rx_buf[3] | ((uint16_t)rx_buf[4] << 8);
        if ((int)(PKT_OVERHEAD + pl_len) != len) {
            continue;
        }

        uint8_t cs = calc_xor(&rx_buf[2], (size_t)pl_len + 3U);
        if (cs != rx_buf[5 + pl_len]) {
            ESP_LOGW(TAG, "Bad checksum on cmd 0x%02X", cmd);
            continue;
        }

        /* Ghi nhan IP cua PC/MATLAB (luon gui tra ve cong MATLAB_DATA_PORT = 8888) */
        portENTER_CRITICAL(&s_lock);
        s_peer_addr.sin_family = AF_INET;
        s_peer_addr.sin_addr   = src_addr.sin_addr;
        s_peer_addr.sin_port   = htons(MATLAB_DATA_PORT);
        s_peer_known           = true;
        portEXIT_CRITICAL(&s_lock);

        process_command(cmd, &rx_buf[5], pl_len);
    }

    s_rx_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t matlab_link_start(void)
{
    if (s_sock >= 0 && s_rx_task != NULL) {
        return ESP_OK;
    }

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket: errno %d", errno);
        return ESP_FAIL;
    }

    struct sockaddr_in local_addr = {0};
    local_addr.sin_family      = AF_INET;
    local_addr.sin_port        = htons(MATLAB_CMD_PORT);
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind UDP port %d: errno %d", MATLAB_CMD_PORT, errno);
        close(sock);
        return ESP_FAIL;
    }

    struct timeval tv = { .tv_sec = 0, .tv_usec = 150000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    portENTER_CRITICAL(&s_lock);
    s_sock          = sock;
    s_peer_known    = false;
    s_pending_act   = MATLAB_ACT_NONE;
    s_task_stop_req = false;
    portEXIT_CRITICAL(&s_lock);

    BaseType_t ok = xTaskCreatePinnedToCore(
        matlab_rx_task,
        "matlab_udp_rx",
        MATLAB_TASK_STACK,
        NULL,
        MATLAB_TASK_PRIO,
        &s_rx_task,
        MATLAB_TASK_CORE
    );
    if (ok != pdPASS) {
        close(sock);
        s_sock = -1;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void matlab_link_stop(void)
{
    s_task_stop_req = true;

    /* Cho task thoat sach (toi da ~200 ms nho SO_RCVTIMEO = 150 ms) */
    for (int i = 0; i < 30 && s_rx_task != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(15));
    }

    portENTER_CRITICAL(&s_lock);
    int sock     = s_sock;
    s_sock       = -1;
    s_peer_known = false;
    portEXIT_CRITICAL(&s_lock);

    if (sock >= 0) {
        close(sock);
    }
}

void matlab_link_set_app_state(app_link_state_t state)
{
    portENTER_CRITICAL(&s_lock);
    s_app_state = state;
    portEXIT_CRITICAL(&s_lock);
}

bool matlab_link_is_peer_known(void)
{
    bool known;
    portENTER_CRITICAL(&s_lock);
    known = s_peer_known;
    portEXIT_CRITICAL(&s_lock);
    return known;
}

matlab_action_t matlab_link_take_action(void)
{
    matlab_action_t act;
    portENTER_CRITICAL(&s_lock);
    act           = s_pending_act;
    s_pending_act = MATLAB_ACT_NONE;
    portEXIT_CRITICAL(&s_lock);
    return act;
}

void matlab_link_send_live_sample(const robot_log_sample_t *sample)
{
    if (sample == NULL) return;
    send_framed_packet(PKT_LIVE_SAMPLE, sample, sizeof(robot_log_sample_t));
}
