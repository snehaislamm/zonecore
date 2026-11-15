/**
 * @file zc_e2e.h
 * @brief End-to-end communication protection, modelled on AUTOSAR E2E
 *        Profile 1: CRC-8/SAE-J1850 over (Data ID, payload) + 4-bit alive
 *        counter + reception timeout. Detects corruption, repetition, loss,
 *        delay and masquerade (wrong Data ID) per ISO 26262-6 Annex D.
 *
 * Frame layout (8 bytes): byte0 = CRC, byte1[3:0] = counter, rest = payload.
 */
#ifndef ZC_E2E_H
#define ZC_E2E_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ZC_E2E_OK = 0,
    ZC_E2E_OK_SOME_LOST, /**< counter jumped but within max_delta */
    ZC_E2E_REPEATED,
    ZC_E2E_WRONG_SEQ,
    ZC_E2E_CRC_ERR,
    ZC_E2E_TIMEOUT,
    ZC_E2E_NO_DATA_YET
} zc_e2e_status_t;

typedef struct {
    uint16_t data_id;
    uint8_t  counter;
} zc_e2e_tx_t;

typedef struct {
    uint16_t        data_id;
    uint8_t         max_delta;   /**< tolerated lost frames + 1 */
    uint16_t        timeout_ms;
    uint8_t         last_counter;
    bool            have_rx;
    uint16_t        age_ms;
    zc_e2e_status_t status;
    uint32_t        err_count;
} zc_e2e_rx_t;

uint8_t zc_crc8_j1850(const uint8_t *data, uint32_t len, uint8_t init, bool final_xor);

void zc_e2e_tx_init(zc_e2e_tx_t *tx, uint16_t data_id);
void zc_e2e_protect(zc_e2e_tx_t *tx, uint8_t frame[8]);

void zc_e2e_rx_init(zc_e2e_rx_t *rx, uint16_t data_id, uint8_t max_delta, uint16_t timeout_ms);
zc_e2e_status_t zc_e2e_check(zc_e2e_rx_t *rx, const uint8_t frame[8]);
/** Advance reception age; latches ZC_E2E_TIMEOUT when age exceeds timeout. */
void zc_e2e_rx_tick(zc_e2e_rx_t *rx, uint16_t dt_ms);
/** True when the last valid data may be consumed by the application. */
bool zc_e2e_usable(const zc_e2e_rx_t *rx);

#endif
