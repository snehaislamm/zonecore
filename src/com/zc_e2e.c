#include "zc/zc_e2e.h"

#include <string.h>

uint8_t zc_crc8_j1850(const uint8_t *data, uint32_t len, uint8_t init, bool final_xor)
{
    uint8_t crc = init;
    for (uint32_t i = 0U; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0U; b < 8U; b++) {
            crc = ((crc & 0x80U) != 0U) ? (uint8_t)((uint8_t)(crc << 1) ^ 0x1DU)
                                        : (uint8_t)(crc << 1);
        }
    }
    return final_xor ? (uint8_t)(crc ^ 0xFFU) : crc;
}

static uint8_t e2e_crc(uint16_t data_id, const uint8_t frame[8])
{
    uint8_t buf[9];
    buf[0] = (uint8_t)(data_id & 0xFFU);
    buf[1] = (uint8_t)(data_id >> 8);
    (void)memcpy(&buf[2], &frame[1], 7U);
    return zc_crc8_j1850(buf, 9U, 0xFFU, true);
}

void zc_e2e_tx_init(zc_e2e_tx_t *tx, uint16_t data_id)
{
    tx->data_id = data_id;
    tx->counter = 0U;
}

void zc_e2e_protect(zc_e2e_tx_t *tx, uint8_t frame[8])
{
    frame[1] = (uint8_t)((frame[1] & 0xF0U) | (tx->counter & 0x0FU));
    frame[0] = e2e_crc(tx->data_id, frame);
    tx->counter = (uint8_t)((tx->counter + 1U) & 0x0FU);
}

void zc_e2e_rx_init(zc_e2e_rx_t *rx, uint16_t data_id, uint8_t max_delta, uint16_t timeout_ms)
{
    (void)memset(rx, 0, sizeof(*rx));
    rx->data_id = data_id;
    rx->max_delta = (max_delta == 0U) ? 1U : max_delta;
    rx->timeout_ms = timeout_ms;
    rx->status = ZC_E2E_NO_DATA_YET;
}

zc_e2e_status_t zc_e2e_check(zc_e2e_rx_t *rx, const uint8_t frame[8])
{
    zc_e2e_status_t st;
    const uint8_t ctr = frame[1] & 0x0FU;
    if (e2e_crc(rx->data_id, frame) != frame[0]) {
        st = ZC_E2E_CRC_ERR;
    } else if (!rx->have_rx) {
        st = ZC_E2E_OK;
    } else {
        const uint8_t delta = (uint8_t)((ctr - rx->last_counter) & 0x0FU);
        if (delta == 0U) {
            st = ZC_E2E_REPEATED;
        } else if (delta == 1U) {
            st = ZC_E2E_OK;
        } else if (delta <= rx->max_delta) {
            st = ZC_E2E_OK_SOME_LOST;
        } else {
            st = ZC_E2E_WRONG_SEQ;
        }
    }

    if ((st == ZC_E2E_OK) || (st == ZC_E2E_OK_SOME_LOST)) {
        rx->last_counter = ctr;
        rx->have_rx = true;
        rx->age_ms = 0U;
    } else if (st == ZC_E2E_WRONG_SEQ) {
        rx->last_counter = ctr; /* resynchronise, but flag this frame */
        rx->err_count++;
    } else if (st == ZC_E2E_CRC_ERR) {
        rx->err_count++;
    } else {
        /* repeated: ignore, do not refresh age */
    }
    rx->status = st;
    return st;
}

void zc_e2e_rx_tick(zc_e2e_rx_t *rx, uint16_t dt_ms)
{
    if (rx->age_ms < (uint16_t)(0xFFFFU - dt_ms)) {
        rx->age_ms = (uint16_t)(rx->age_ms + dt_ms);
    }
    if (rx->have_rx && (rx->age_ms > rx->timeout_ms)) {
        rx->status = ZC_E2E_TIMEOUT;
    }
}

bool zc_e2e_usable(const zc_e2e_rx_t *rx)
{
    /* Corrupt / out-of-sequence frames are dropped (they never refresh age),
     * so the last *valid* payload stays usable until the deadline expires. */
    return rx->have_rx && (rx->age_ms <= rx->timeout_ms);
}
