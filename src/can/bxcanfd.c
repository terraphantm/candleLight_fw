/*

The MIT License (MIT)

Copyright (c) 2016 Hubert Denkmair

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include "can.h"
#include "config.h"
#include "gpio.h"
#include "hal_include.h"
#include "stm32g0b1xx.h"
#include "timer.h"

void can_init(can_data_t *channel, FDCAN_GlobalTypeDef *instance)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {
		.PeriphClockSelection = RCC_PERIPHCLK_FDCAN,
		.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1,
	};

	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	__HAL_RCC_FDCAN_CLK_ENABLE(); // TODO: make sure we only do this once ?

	// Setup GPIO
	// TODO needs to changed for two channel setup
	if (instance == FDCAN1) {
		GPIO_InitTypeDef itd = {
			.Pin = GPIO_PIN_0 | GPIO_PIN_1,
			.Mode = GPIO_MODE_AF_PP,
			.Pull = GPIO_NOPULL,
			.Speed = GPIO_SPEED_FREQ_LOW,
			.Alternate = GPIO_AF3_FDCAN1,
		};

		__HAL_RCC_GPIOD_CLK_ENABLE();
		HAL_GPIO_Init(GPIOD, &itd);
	} else if (instance == FDCAN2) {
		GPIO_InitTypeDef itd = {
			.Pin = GPIO_PIN_4 | GPIO_PIN_5,
			.Mode = GPIO_MODE_AF_PP,
			.Pull = GPIO_NOPULL,
			.Speed = GPIO_SPEED_FREQ_LOW,
			.Alternate = GPIO_AF3_FDCAN1,
		};

		__HAL_RCC_GPIOC_CLK_ENABLE();
		HAL_GPIO_Init(GPIOC, &itd);
	}

	// setup CAN FD useing the HAL lib
	channel->channel.Instance = instance;
	channel->channel.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	channel->channel.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
	channel->channel.Init.Mode = FDCAN_MODE_NORMAL;
	channel->channel.Init.AutoRetransmission = DISABLE;
	channel->channel.Init.TransmitPause = DISABLE;
	channel->channel.Init.ProtocolException = ENABLE;
	channel->channel.Init.NominalPrescaler = 8;
	channel->channel.Init.NominalSyncJumpWidth = 1;
	channel->channel.Init.NominalTimeSeg1 = 13;
	channel->channel.Init.NominalTimeSeg2 = 2;
	channel->channel.Init.DataPrescaler = 2;
	channel->channel.Init.DataSyncJumpWidth = 4;
	channel->channel.Init.DataTimeSeg1 = 15;
	channel->channel.Init.DataTimeSeg2 = 4;
	channel->channel.Init.StdFiltersNbr = 0;
	channel->channel.Init.ExtFiltersNbr = 0;
	channel->channel.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	HAL_FDCAN_Init(&channel->channel);
}

bool can_set_bittiming(can_data_t *channel, uint16_t brp, uint8_t phase_seg1, uint8_t phase_seg2, uint8_t sjw)
{
	if ((brp > 0) && (brp <= 1024) &&
		(phase_seg1 > 0) && (phase_seg1 <= 16) &&
		(phase_seg2 > 0) && (phase_seg2 <= 8) &&
		(sjw > 0) && (sjw <= 4)) {
		channel->channel.Init.NominalSyncJumpWidth = sjw;
		channel->channel.Init.NominalTimeSeg1 = phase_seg1;
		channel->channel.Init.NominalTimeSeg2 = phase_seg2;
		channel->channel.Init.NominalPrescaler = brp;
		return true;
	} else {
		return false;
	}

	return 0;
}

bool can_set_data_bittiming(can_data_t *channel, uint16_t brp, uint8_t phase_seg1, uint8_t phase_seg2, uint8_t sjw)
{
	if ((brp > 0) && (brp <= 32) &&
		(phase_seg1 > 0) && (phase_seg1 <= 32) &&
		(phase_seg2 > 0) && (phase_seg2 <= 16) &&
		(sjw > 0) && (sjw <= 16)) {
		channel->channel.Init.DataSyncJumpWidth = sjw;
		channel->channel.Init.DataTimeSeg1 = phase_seg1;
		channel->channel.Init.DataTimeSeg2 = phase_seg2;
		channel->channel.Init.DataPrescaler = brp;
		return true;
	} else {
		return false;
	}
}

void can_enable(can_data_t *channel, uint32_t mode)
{
	if (mode & GS_CAN_MODE_ONE_SHOT) {
		channel->channel.Init.AutoRetransmission = ENABLE;
	} else {
		channel->channel.Init.AutoRetransmission = DISABLE;
	}

	if ((mode & (GS_CAN_MODE_LISTEN_ONLY | GS_CAN_MODE_LOOP_BACK)) ==
		(GS_CAN_MODE_LISTEN_ONLY | GS_CAN_MODE_LOOP_BACK)) {
		channel->channel.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
	} else if (mode & GS_CAN_MODE_LISTEN_ONLY) {
		channel->channel.Init.Mode = FDCAN_MODE_BUS_MONITORING;
	} else if (mode & GS_CAN_MODE_LOOP_BACK) {
		channel->channel.Init.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK;
	} else {
		channel->channel.Init.Mode = FDCAN_MODE_NORMAL;
	}

	if (mode & GS_CAN_MODE_FD) {
		channel->channel.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
	} else {
		channel->channel.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	}

	HAL_FDCAN_Init(&channel->channel);

	/* Configure reception filter to Rx FIFO 0 on both FDCAN instances */
	FDCAN_FilterTypeDef sFilterConfig = {
		.IdType = FDCAN_STANDARD_ID,
		.FilterIndex = 0,
		.FilterType = FDCAN_FILTER_RANGE,
		.FilterConfig = FDCAN_FILTER_DISABLE,
		.FilterID1 = 0x000,
		.FilterID2 = 0x7FF,
	};

	HAL_FDCAN_ConfigFilter(&channel->channel, &sFilterConfig);

	/*
	 * Configure global filter on both FDCAN instances:
	 * Filter all remote frames with STD and EXT ID
	 * Reject non matching frames with STD ID and EXT ID
	 */
	HAL_FDCAN_ConfigGlobalFilter(&channel->channel,
								 FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
								 FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

#ifdef nCANSTBY_Pin
	HAL_GPIO_WritePin(nCANSTBY_Port, nCANSTBY_Pin, !GPIO_INIT_STATE(nCANSTBY_Active_High));
#endif

	// Start CAN using HAL
	HAL_FDCAN_Start(&channel->channel);
}

void can_disable(can_data_t *channel)
{
	HAL_FDCAN_Stop(&channel->channel);

#ifdef nCANSTBY_Pin
	HAL_GPIO_WritePin(nCANSTBY_Port, nCANSTBY_Pin, GPIO_INIT_STATE(nCANSTBY_Active_High));
#endif
}

bool can_is_enabled(can_data_t *channel)
{
	return channel->channel.State == HAL_FDCAN_STATE_BUSY;
}

bool can_receive(can_data_t *channel, struct gs_host_frame *rx_frame)
{
	FDCAN_RxHeaderTypeDef RxHeader;
	uint32_t timestamp_us = timer_get();

	if (HAL_FDCAN_GetRxMessage(&channel->channel, FDCAN_RX_FIFO0, &RxHeader, rx_frame->canfd->data) != HAL_OK) {
		return false;
	}

	rx_frame->channel = channel->nr;
	rx_frame->flags = 0;
	rx_frame->can_id = RxHeader.Identifier;

	if (RxHeader.IdType == FDCAN_EXTENDED_ID) {
		rx_frame->can_id |= CAN_EFF_FLAG;
	}

	if (RxHeader.RxFrameType == FDCAN_REMOTE_FRAME) {
		rx_frame->can_id |= CAN_RTR_FLAG;
	}

	rx_frame->can_dlc = (RxHeader.DataLength & 0x000F0000) >> 16;

	if (RxHeader.FDFormat == FDCAN_FD_CAN) {
		rx_frame->canfd_ts->timestamp_us = timestamp_us;

		/* this is a CAN-FD frame */
		rx_frame->flags = GS_CAN_FLAG_FD;
		if (RxHeader.BitRateSwitch == FDCAN_BRS_ON) {
			rx_frame->flags |= GS_CAN_FLAG_BRS;
		}

		if (RxHeader.ErrorStateIndicator == FDCAN_ESI_PASSIVE) {
			rx_frame->flags |= GS_CAN_FLAG_ESI;
		}
	} else {
		rx_frame->classic_can_ts->timestamp_us = timestamp_us;
	}

	return true;
}

bool can_is_rx_pending(can_data_t *channel)
{
	return (HAL_FDCAN_GetRxFifoFillLevel(&channel->channel, FDCAN_RX_FIFO0) >= 1);
}

bool can_send(can_data_t *channel, struct gs_host_frame *frame)
{
	FDCAN_TxHeaderTypeDef TxHeader;

	TxHeader.DataLength = frame->can_dlc << 16;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	TxHeader.TxFrameType = frame->can_id & CAN_RTR_FLAG ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;

	if (frame->can_id & CAN_EFF_FLAG) {
		TxHeader.IdType = FDCAN_EXTENDED_ID;
		TxHeader.Identifier = frame->can_id & 0x1FFFFFFF;
	} else {
		TxHeader.IdType = FDCAN_STANDARD_ID;
		TxHeader.Identifier = frame->can_id & 0x7FF;
	}

	if (frame->flags & GS_CAN_FLAG_FD) {
		TxHeader.FDFormat = FDCAN_FD_CAN;
		if (frame->flags & GS_CAN_FLAG_BRS) {
			TxHeader.BitRateSwitch = FDCAN_BRS_ON;
		} else {
			TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
		}
	} else {
		TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
		TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	}

	if (HAL_FDCAN_AddMessageToTxFifoQ(&channel->channel, &TxHeader, frame->canfd->data) != HAL_OK) {
		return false;
	}

	return true;
}

uint32_t can_get_error_status(can_data_t *channel)
{
	uint32_t err = channel->channel.Instance->PSR;

	/* Write 7 to LEC so we know if it gets set to the same thing again */
	channel->channel.Instance->PSR = 7;

	return err;
}

static bool status_is_active(uint32_t err)
{
	return !(err & (FDCAN_PSR_BO | FDCAN_PSR_EP));
}

bool can_parse_error_status(can_data_t *channel, struct gs_host_frame *frame, uint32_t err)
{
	uint32_t last_err = channel->reg_esr_old;
	/*
	 * We build up the detailed error information at the same time as
	 * we decide whether there's anything worth sending. This variable
	 * tracks that final result.
	 */
	bool should_send = false;

	channel->reg_esr_old = err;

	frame->echo_id = 0xFFFFFFFF;
	frame->can_id  = CAN_ERR_FLAG;
	frame->can_dlc = CAN_ERR_DLC;
	frame->classic_can->data[0] = CAN_ERR_LOSTARB_UNSPEC;
	frame->classic_can->data[1] = CAN_ERR_CRTL_UNSPEC;
	frame->classic_can->data[2] = CAN_ERR_PROT_UNSPEC;
	frame->classic_can->data[3] = CAN_ERR_PROT_LOC_UNSPEC;
	frame->classic_can->data[4] = CAN_ERR_TRX_UNSPEC;
	frame->classic_can->data[5] = 0;
	frame->classic_can->data[6] = 0;
	frame->classic_can->data[7] = 0;

	/* We transitioned from passive/bus-off to active, so report the edge. */
	if (!status_is_active(last_err) && status_is_active(err)) {
		frame->can_id |= CAN_ERR_CRTL;
		frame->classic_can->data[1] |= CAN_ERR_CRTL_ACTIVE;
		should_send = true;
	}

	if (err & FDCAN_PSR_BO) {
		if (!(last_err & FDCAN_PSR_BO)) {
			/* We transitioned to bus-off. */
			frame->can_id |= CAN_ERR_BUSOFF;
			should_send = true;
		}
	}

	/*
	 * The Linux sja1000 driver puts these counters here. Seems like
	 * as good a place as any.
	 */
	// TX error count
	frame->classic_can->data[6] = ((channel->channel.Instance->ECR & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos);
	// RX error count
	frame->classic_can->data[7] = ((channel->channel.Instance->ECR & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos);

	if (err & FDCAN_PSR_EP) {
		if (!(last_err & FDCAN_PSR_EP)) {
			frame->can_id |= CAN_ERR_CRTL;
			frame->classic_can->data[1] |= CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE;
			should_send = true;
		}
	} else if (err & FDCAN_PSR_EW) {
		if (!(last_err & FDCAN_PSR_EW)) {
			frame->can_id |= CAN_ERR_CRTL;
			frame->classic_can->data[1] |= CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING;
			should_send = true;
		}
	}

	uint8_t lec = err & FDCAN_PSR_LEC;
	switch (lec) {
		case 0x01: /* stuff error */
			frame->can_id |= CAN_ERR_PROT;
			frame->classic_can->data[2] |= CAN_ERR_PROT_STUFF;
			should_send = true;
			break;
		case 0x02: /* form error */
			frame->can_id |= CAN_ERR_PROT;
			frame->classic_can->data[2] |= CAN_ERR_PROT_FORM;
			should_send = true;
			break;
		case 0x03: /* ack error */
			frame->can_id |= CAN_ERR_ACK;
			should_send = true;
			break;
		case 0x04: /* bit recessive error */
			frame->can_id |= CAN_ERR_PROT;
			frame->classic_can->data[2] |= CAN_ERR_PROT_BIT1;
			should_send = true;
			break;
		case 0x05: /* bit dominant error */
			frame->can_id |= CAN_ERR_PROT;
			frame->classic_can->data[2] |= CAN_ERR_PROT_BIT0;
			should_send = true;
			break;
		case 0x06: /* CRC error */
			frame->can_id |= CAN_ERR_PROT;
			frame->classic_can->data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ;
			should_send = true;
			break;
		default: /* 0=no error, 7=no change */
			break;
	}

	return should_send;
}
