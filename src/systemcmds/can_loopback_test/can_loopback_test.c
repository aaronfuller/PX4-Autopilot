/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 ****************************************************************************/

/**
 * @file can_loopback_test.c
 *
 * FDCAN1 <-> FDCAN2 self-loopback test.
 *
 * Requires a physical jumper on the DUT:
 *   CAN1_H <-> CAN2_H
 *   CAN1_L <-> CAN2_L
 *
 * Requires that uavcan is stopped before running (uavcan owns FDCAN when
 * active). Reboot or 'uavcan start fw' will restore normal operation.
 *
 * Prints "CAN_LOOPBACK: PASS" on success or "CAN_LOOPBACK: FAIL <reason>"
 * on failure.  Exit status: 0 = pass, non-zero = fail.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

#include <hardware/stm32_fdcan.h>
#include <hardware/stm32h7x3xx_memorymap.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "arm_internal.h"        /* putreg32 / getreg32 macros */

/* Message RAM allocation.  FDCAN1 and FDCAN2 share the same physical RAM
 * (STM32_CANRAM_BASE, 10 KB).  We split it in half: FDCAN1 gets the first
 * 256 bytes (64 words), FDCAN2 gets the next 256 bytes.
 *
 * Per FDCAN we allocate:
 *   Standard filter list: 1 word  (1 filter, accept-all)
 *   RX FIFO 0:           16 words (4 elements x 4 words for classic CAN)
 *   TX Buffers:          16 words (4 elements x 4 words for classic CAN)
 *   Total:               33 words (padded to 64 for alignment/headroom)
 */
#define FDCAN1_MSGRAM_OFFSET   0x0000U   /* byte offset from CANRAM base */
#define FDCAN2_MSGRAM_OFFSET   0x0100U   /* +256 bytes = 64 words */

#define FDCAN_FILT_OFFSET      0x0000U   /* +0 words */
#define FDCAN_RXF0_OFFSET      0x0004U   /* +1 word */
#define FDCAN_TXBUF_OFFSET     0x0044U   /* +17 words (1 filt + 16 RX) */

#define RX_FIFO_NUM_ELEMENTS   4
#define TX_BUF_NUM_ELEMENTS    4
#define CAN_ELEMENT_WORDS      4         /* 2 header + 2 data words (8 byte payload) */

/* Test parameters */
#define TEST_TIMEOUT_MS        200       /* per-frame RX timeout */

/* Bit timing for 1 Mbit/s at FDCAN kernel clock of 24 MHz (STM32_FDCANCLK = HSE):
 * Total TQ per bit = 24
 * BRP = 1 (NBRP = 0), TSEG1 = 17 (NTSEG1 = 16), TSEG2 = 6 (NTSEG2 = 5), SJW = 6 (NSJW = 5)
 * Sample point = (1 + 17) / 24 = 75%
 */
/* NBTP.NTSEG2 occupies bits [6:0] — not defined by the header, hardcode. */
#define NBTP_1MBIT_24MHZ  ((0U  << FDCAN_NBTP_NBRP_SHIFT)  |  \
                           (16U << FDCAN_NBTP_NTSEG1_SHIFT) |  \
                           (5U  << 0)                       |  \
                           (5U  << FDCAN_NBTP_NSJW_SHIFT))

/* Standard filter element: accept-all range filter, store to RX FIFO 0. */
#define STD_FILTER_ACCEPT_ALL  ((0U      << 30) |   /* SFT   = range               */ \
                                (1U      << 27) |   /* SFEC  = store in RX FIFO 0  */ \
                                (0x000U  << 16) |   /* SFID1 = 0x000               */ \
                                (0x7FFU  <<  0))    /* SFID2 = 0x7FF               */

/* GFC: reject non-matching standard and extended frames, reject all remote */
#define GFC_REJECT_NON_MATCHING   ((3U << 4) | (3U << 2) | (1U << 1) | (1U << 0))

/* CCCR bits we use (some already defined in hardware/stm32_fdcan.h) */
#ifndef FDCAN_CCCR_CSR
#define FDCAN_CCCR_CSR   (1U << 4)   /* Clock Stop Request */
#endif
#ifndef FDCAN_CCCR_CSA
#define FDCAN_CCCR_CSA   (1U << 3)   /* Clock Stop Acknowledge */
#endif


/* --------------------------------------------------------------------------
 * Low-level FDCAN helpers
 * -------------------------------------------------------------------------- */

typedef struct {
	uint32_t base;                /* STM32_FDCAN1_BASE or STM32_FDCAN2_BASE */
	uint32_t msgram_offset;       /* byte offset from CANRAM base */
} fdcan_ctx_t;

static const fdcan_ctx_t FDCAN1_CTX = { STM32_FDCAN1_BASE, FDCAN1_MSGRAM_OFFSET };
static const fdcan_ctx_t FDCAN2_CTX = { STM32_FDCAN2_BASE, FDCAN2_MSGRAM_OFFSET };


static int fdcan_enter_init(const fdcan_ctx_t *c)
{
	uint32_t cccr;
	int timeout = 10000;

	/* Set INIT bit and wait for it to latch */
	cccr = getreg32(c->base + STM32_FDCAN_CCCR_OFFSET);
	putreg32(cccr | FDCAN_CCCR_INIT, c->base + STM32_FDCAN_CCCR_OFFSET);

	while (--timeout > 0) {
		cccr = getreg32(c->base + STM32_FDCAN_CCCR_OFFSET);
		if (cccr & FDCAN_CCCR_INIT) {
			break;
		}
	}
	if (timeout <= 0) {
		return -1;
	}

	/* Enable configuration change */
	putreg32(cccr | FDCAN_CCCR_INIT | FDCAN_CCCR_CCE, c->base + STM32_FDCAN_CCCR_OFFSET);
	return 0;
}


static int fdcan_leave_init(const fdcan_ctx_t *c)
{
	uint32_t cccr;
	int timeout = 100000;

	cccr = getreg32(c->base + STM32_FDCAN_CCCR_OFFSET);
	cccr &= ~(FDCAN_CCCR_INIT | FDCAN_CCCR_CCE);
	putreg32(cccr, c->base + STM32_FDCAN_CCCR_OFFSET);

	while (--timeout > 0) {
		if ((getreg32(c->base + STM32_FDCAN_CCCR_OFFSET) & FDCAN_CCCR_INIT) == 0) {
			return 0;
		}
	}
	return -1;
}


static int fdcan_init(const fdcan_ctx_t *c)
{
	uint32_t canram = STM32_CANRAM_BASE + c->msgram_offset;

	if (fdcan_enter_init(c) < 0) {
		return -1;
	}

	/* Bit timing */
	putreg32(NBTP_1MBIT_24MHZ, c->base + STM32_FDCAN_NBTP_OFFSET);

	/* Configure standard filter list (1 filter starting at our msgram base) */
	putreg32((c->msgram_offset + FDCAN_FILT_OFFSET) |
	         (1U << FDCAN_SIDFC_LSS_SHIFT),
	         c->base + STM32_FDCAN_SIDFC_OFFSET);

	/* Write the accept-all filter element into RAM */
	putreg32(STD_FILTER_ACCEPT_ALL, canram + FDCAN_FILT_OFFSET);

	/* No extended filters */
	putreg32(0, c->base + STM32_FDCAN_XIDFC_OFFSET);

	/* Reject non-matching frames + reject remote frames */
	putreg32(GFC_REJECT_NON_MATCHING, c->base + STM32_FDCAN_GFC_OFFSET);

	/* RX FIFO 0 config */
	putreg32((c->msgram_offset + FDCAN_RXF0_OFFSET) |
	         ((uint32_t)RX_FIFO_NUM_ELEMENTS << 16),
	         c->base + STM32_FDCAN_RXF0C_OFFSET);

	/* No RX FIFO 1 or dedicated RX buffers */
	putreg32(0, c->base + STM32_FDCAN_RXF1C_OFFSET);
	putreg32(0, c->base + STM32_FDCAN_RXBC_OFFSET);

	/* RX element size: bits [10:8] = FIFO0 element size, 0 = 8-byte data */
	putreg32(0, c->base + STM32_FDCAN_RXESC_OFFSET);

	/* TX buffer config: TX FIFO mode, 4 elements, no dedicated TX buffers */
	putreg32((c->msgram_offset + FDCAN_TXBUF_OFFSET) |
	         ((uint32_t)TX_BUF_NUM_ELEMENTS << FDCAN_TXBC_TFQS_SHIFT),
	         c->base + STM32_FDCAN_TXBC_OFFSET);

	/* TX element size: bits [2:0] = 0 = 8-byte data */
	putreg32(0, c->base + STM32_FDCAN_TXESC_OFFSET);

	/* No TX event FIFO */
	putreg32(0, c->base + STM32_FDCAN_TXEFC_OFFSET);

	/* Leave init mode and enter normal operation */
	return fdcan_leave_init(c);
}


/* Enqueue an 8-byte, standard-ID, classic-CAN frame into TX FIFO slot 0.
 * Returns 0 on success, -1 if TX FIFO is full.
 */
static int fdcan_send(const fdcan_ctx_t *c, uint32_t std_id, const uint8_t *data, uint8_t dlc)
{
	uint32_t canram = STM32_CANRAM_BASE + c->msgram_offset;
	uint32_t tx_base = canram + FDCAN_TXBUF_OFFSET;
	uint32_t txfqs;
	uint32_t put_index;

	if (dlc > 8) { dlc = 8; }

	txfqs = getreg32(c->base + STM32_FDCAN_TXFQS_OFFSET);
	if (txfqs & (1U << 21)) { return -1; }             /* TFQF = FIFO full */

	put_index = (txfqs >> 16) & 0x1F;                  /* TFQPI: put index */

	/* T0: standard ID left-justified in bits [28:18], no XTD, no RTR */
	uint32_t *slot = (uint32_t *)(tx_base + put_index * CAN_ELEMENT_WORDS * 4);
	slot[0] = (std_id & 0x7FF) << 18;
	/* T1: DLC in [19:16], no FDF, no BRS, no marker */
	slot[1] = ((uint32_t)dlc & 0x0F) << 16;
	/* T2, T3: data bytes 0..7 */
	uint32_t d02 = 0, d34 = 0;
	for (int i = 0; i < 4 && i < dlc; i++) { d02 |= ((uint32_t)data[i])     << (8 * i); }
	for (int i = 4; i < 8 && i < dlc; i++) { d34 |= ((uint32_t)data[i]) << (8 * (i - 4)); }
	slot[2] = d02;
	slot[3] = d34;

	/* Request transmission for this slot */
	putreg32(1U << put_index, c->base + STM32_FDCAN_TXBAR_OFFSET);
	return 0;
}


/* Try to receive one frame from RX FIFO 0.
 * On success returns 0 and populates *std_id, data[], *dlc.
 * Returns -1 if the FIFO is empty.
 */
static int fdcan_recv(const fdcan_ctx_t *c, uint32_t *std_id, uint8_t *data, uint8_t *dlc)
{
	uint32_t canram = STM32_CANRAM_BASE + c->msgram_offset;
	uint32_t rx_base = canram + FDCAN_RXF0_OFFSET;
	uint32_t rxf0s;
	uint32_t get_index;

	rxf0s = getreg32(c->base + STM32_FDCAN_RXF0S_OFFSET);
	if ((rxf0s & FDCAN_RXF0S_F0FL) == 0) {
		return -1;
	}
	get_index = (rxf0s >> 8) & 0x3F;

	uint32_t *slot = (uint32_t *)(rx_base + get_index * CAN_ELEMENT_WORDS * 4);
	uint32_t r0 = slot[0];
	uint32_t r1 = slot[1];
	uint32_t d02 = slot[2];
	uint32_t d34 = slot[3];

	if (r0 & (1U << 30)) {
		/* Extended ID — we don't expect these in this test */
		*std_id = 0xFFFFFFFF;
	} else {
		*std_id = (r0 >> 18) & 0x7FF;
	}
	*dlc = (r1 >> 16) & 0x0F;

	for (int i = 0; i < 4; i++) { data[i]     = (d02 >> (8 * i))     & 0xFF; }
	for (int i = 0; i < 4; i++) { data[i + 4] = (d34 >> (8 * i))     & 0xFF; }

	/* Acknowledge the FIFO slot */
	putreg32(get_index, c->base + STM32_FDCAN_RXF0A_OFFSET);
	return 0;
}


/* Millisecond monotonic timestamp using clock_gettime. */
static uint64_t millis(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}


/* Send a test frame on tx, wait up to TEST_TIMEOUT_MS for it on rx.
 * Returns 0 on success, non-zero (with reason printed) on failure.
 */
static int loopback_one(const fdcan_ctx_t *tx, const fdcan_ctx_t *rx,
                        const char *label, uint32_t can_id,
                        const uint8_t *payload, uint8_t dlc)
{
	if (fdcan_send(tx, can_id, payload, dlc) != 0) {
		PX4_ERR("%s TX FIFO full", label);
		return 1;
	}

	uint64_t deadline = millis() + TEST_TIMEOUT_MS;
	uint32_t got_id = 0;
	uint8_t  got_data[8] = {0};
	uint8_t  got_dlc = 0;

	while (millis() < deadline) {
		if (fdcan_recv(rx, &got_id, got_data, &got_dlc) == 0) {
			if (got_id != can_id) {
				PX4_ERR("%s ID mismatch: sent 0x%03x, got 0x%03x",
				        label, (unsigned)can_id, (unsigned)got_id);
				return 2;
			}
			if (got_dlc != dlc) {
				PX4_ERR("%s DLC mismatch: sent %u, got %u",
				        label, (unsigned)dlc, (unsigned)got_dlc);
				return 3;
			}
			if (memcmp(got_data, payload, dlc) != 0) {
				PX4_ERR("%s payload mismatch", label);
				return 4;
			}
			return 0;
		}
	}
	PX4_ERR("%s RX timeout (no frame in %d ms) — check jumpers",
	        label, TEST_TIMEOUT_MS);
	return 5;
}


static void print_usage(void)
{
	PRINT_MODULE_DESCRIPTION(
		"### Description\n"
		"FDCAN1 <-> FDCAN2 self-loopback test.  Requires physical jumper:\n"
		"  CAN1_H <-> CAN2_H\n"
		"  CAN1_L <-> CAN2_L\n"
		"\n"
		"Requires uavcan stopped (`uavcan stop`) before running.\n"
		"Prints CAN_LOOPBACK: PASS on success, CAN_LOOPBACK: FAIL on failure."
	);
	PRINT_MODULE_USAGE_NAME_SIMPLE("can_loopback_test", "command");
}


__EXPORT int can_loopback_test_main(int argc, char *argv[]);

int can_loopback_test_main(int argc, char *argv[])
{
	if (argc > 1 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "help") == 0)) {
		print_usage();
		return 0;
	}

	PX4_INFO("Initializing FDCAN1 and FDCAN2 for loopback test...");

	if (fdcan_init(&FDCAN1_CTX) != 0) {
		PX4_ERR("FDCAN1 init failed");
		printf("CAN_LOOPBACK: FAIL fdcan1_init\n");
		return 1;
	}
	if (fdcan_init(&FDCAN2_CTX) != 0) {
		PX4_ERR("FDCAN2 init failed");
		printf("CAN_LOOPBACK: FAIL fdcan2_init\n");
		fdcan_enter_init(&FDCAN1_CTX);
		return 1;
	}

	/* Test patterns */
	const struct {
		uint32_t id;
		uint8_t  data[8];
		uint8_t  dlc;
	} patterns[] = {
		{ 0x123, { 0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44 }, 8 },
		{ 0x456, { 0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE }, 8 },
		{ 0x001, { 0x00 },                                           1 },
		{ 0x7FF, { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }, 8 },
	};
	int total_frames = 0;
	int fail_count   = 0;

	for (unsigned i = 0; i < sizeof(patterns) / sizeof(patterns[0]); i++) {
		char label[32];
		snprintf(label, sizeof(label), "pat%u CAN1->CAN2", i);
		if (loopback_one(&FDCAN1_CTX, &FDCAN2_CTX, label,
		                 patterns[i].id, patterns[i].data, patterns[i].dlc) != 0) {
			fail_count++;
		}
		total_frames++;

		snprintf(label, sizeof(label), "pat%u CAN2->CAN1", i);
		if (loopback_one(&FDCAN2_CTX, &FDCAN1_CTX, label,
		                 patterns[i].id, patterns[i].data, patterns[i].dlc) != 0) {
			fail_count++;
		}
		total_frames++;
	}

	/* Leave FDCAN in INIT mode so uavcan can restart cleanly. */
	fdcan_enter_init(&FDCAN1_CTX);
	fdcan_enter_init(&FDCAN2_CTX);

	if (fail_count == 0) {
		printf("CAN_LOOPBACK: PASS %d/%d frames\n", total_frames, total_frames);
		return 0;
	} else {
		printf("CAN_LOOPBACK: FAIL %d/%d frames failed\n",
		       fail_count, total_frames);
		return 1;
	}
}
