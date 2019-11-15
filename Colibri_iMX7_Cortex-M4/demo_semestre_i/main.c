/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

///////////////////////////////////////////////////////////////////////////////
//  Includes
///////////////////////////////////////////////////////////////////////////////

/* Standar includes */
#include <string.h>
#include <stdio.h>

/* Board and drivers */
#include "board.h"
#include "flexcan.h"
#include "mu_imx.h"
#include "rpmsg/rpmsg_rtos.h"
#include "debug_console_imx.h"

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/*-----------------------------------------------------------*/

#define TX_MSG_BUF_NUM    8
#define RX_MSG_BUF_NUM    9

#define CAN_TX_IDENTIFIER     0x123
#define CAN_RX_IDENTIFIER     0x321
#define CAN_GLOBAL_MASK       0x321

/* The number of items the queue can hold at once. */
#define CAN_RX_MSG_QUEUE_LENGTH ( 10 )

#define MAX_RPMSG_LENGTH 512

/*-----------------------------------------------------------*/

/* Initialize CAN communication */
static void can_init(void);
/* Task processing queue of incomming CAN messages */
static void can_process_rx_queue_task(void *pv_param);

/* Receives RPMsg control messages */ 
static void rpmsg_receive_task(void *pv_param);

/*-----------------------------------------------------------*/

/* Tx and Rx message configuration buffers */
static flexcan_msgbuf_t *txMsgBufPtr;
static flexcan_msgbuf_t *rxMsgBufPtr;
/* The queue used for received messages */
static QueueHandle_t rx_can_msg_queue;

/* RPMsg communication started */
volatile uint8_t send_rpmsg = 0U;

/* Communication channel for RPMsg */
struct rpmsg_channel *app_chnl = NULL;
struct remote_device *rdev = NULL;

/*-----------------------------------------------------------*/

/*!
 * @brief Main function
 */
int main(void)
{
    /* Initialize demo application pins setting and clock setting. */
    hardware_init();
    /* Initialize CAN communication */
    can_init();

    /* Prepare for the MU Interrupt */
    MU_Init(BOARD_MU_BASE_ADDR);
    NVIC_SetPriority(BOARD_MU_IRQ_NUM, 3);
    NVIC_EnableIRQ(BOARD_MU_IRQ_NUM);

    /* Create the Rx CAN message queue */
    rx_can_msg_queue = xQueueCreate(CAN_RX_MSG_QUEUE_LENGTH, sizeof(flexcan_msgbuf_t));

    /* Create a rpmsg control task. */
    xTaskCreate(rpmsg_receive_task, "Reeive RPMsg commands", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL);

    /* Create the task to process the CAN messages */
    xTaskCreate(can_process_rx_queue_task,         /* The function that implements the task. */
                "Process CAN message queue",  /* The text name assigned to the task - for debug only as it is not used by the kernel. */
                configMINIMAL_STACK_SIZE,     /* The size of the stack to allocate to the task. */
                NULL,                         /* The parameter passed to the task - not used in this simple case. */
                tskIDLE_PRIORITY+1,           /* The priority assigned to the task. */
                NULL);                        /* The task handle is not required, so NULL is passed. */

    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();

    PRINTF("Should never reach this point\r\n");

    /* Should never reach this point. */
    while (true);
}

/*!
 * @brief Initialize CAN communication
 */
void can_init(void)
{
    const flexcan_timing_t timing_table[] = {
        {7, 3, 7, 7, 6},  /* 125 kHz from 24 MHz OSC */
        {3, 3, 7, 7, 6},  /* 250 kHz from 24 MHz OSC */
        {1, 3, 7, 7, 6},  /* 500 kHz from 24 MHz OSC */
        {0, 3, 7, 7, 6},  /* 1   MHz from 24 MHz OSC */
    };

    flexcan_init_config_t initConfig = {
        .timing = timing_table[0],
        .operatingMode = flexcanNormalMode,
        .maxMsgBufNum  = 16
    };

    /* Initialize FlexCAN module. */
    FLEXCAN_Init(BOARD_FLEXCAN_BASEADDR, &initConfig);
    /* Enable FlexCAN Clock. */
    FLEXCAN_Enable(BOARD_FLEXCAN_BASEADDR);
    /* Set FlexCAN to use Global mask mode. */
    FLEXCAN_SetRxMaskMode(BOARD_FLEXCAN_BASEADDR, flexcanRxMaskGlobal);
    /* Set FlexCAN global mask. */
    FLEXCAN_SetRxGlobalMask(BOARD_FLEXCAN_BASEADDR, ~CAN_ID_STD(CAN_GLOBAL_MASK));

    /* Clear Tx and Rx message buffer interrupt pending bit. */
    FLEXCAN_ClearMsgBufStatusFlag(BOARD_FLEXCAN_BASEADDR, TX_MSG_BUF_NUM);
    FLEXCAN_ClearMsgBufStatusFlag(BOARD_FLEXCAN_BASEADDR, RX_MSG_BUF_NUM);
    /* Enable Tx and Rx message buffer interrupt. */
    FLEXCAN_SetMsgBufIntCmd(BOARD_FLEXCAN_BASEADDR, TX_MSG_BUF_NUM, true);
    FLEXCAN_SetMsgBufIntCmd(BOARD_FLEXCAN_BASEADDR, RX_MSG_BUF_NUM, true);

    /* Initialize Global variable. */
    txMsgBufPtr = FLEXCAN_GetMsgBufPtr(BOARD_FLEXCAN_BASEADDR, TX_MSG_BUF_NUM);
    rxMsgBufPtr = FLEXCAN_GetMsgBufPtr(BOARD_FLEXCAN_BASEADDR, RX_MSG_BUF_NUM);

    /* Setup Rx MsgBuf to receive Frame. */
    rxMsgBufPtr->idStd = CAN_RX_IDENTIFIER;
    rxMsgBufPtr->code  = flexcanRxEmpty;

    txMsgBufPtr->prio  = 0x0; /* We don't use local priority */
    txMsgBufPtr->idStd = CAN_TX_IDENTIFIER; /* Set Tx Identifier */
    txMsgBufPtr->idExt = 0x0; /* We don't use Extend Id. */
    txMsgBufPtr->dlc   = 0x1; /* Send only 1 byte data. */
    txMsgBufPtr->rtr   = 0x0; /* Send data frame. */
    txMsgBufPtr->ide   = 0x0; /* Frame format is standard. */
    txMsgBufPtr->srr   = 0x1; /* Don't care in standard id mode. */

    /* Set FlexCAN interrupt priority. */
    NVIC_SetPriority(BOARD_FLEXCAN_IRQ_NUM, 3);
    /* Enable FlexCAN interrupt. */
    NVIC_EnableIRQ(BOARD_FLEXCAN_IRQ_NUM);
}

/*!
 * @brief Takes elements from the Rx CAN queue to process them
 */
void can_process_rx_queue_task(void *pv_param)
{
    flexcan_msgbuf_t rx_message;
    // char message[MAX_RPMSG_LENGTH];

    // PRINTF("%s - %s\r\n", __FUNCTION__, __TIME__);

    while (true)
    {
        /* Wait until something arrives in the queue */
        xQueueReceive(rx_can_msg_queue, &rx_message, portMAX_DELAY);
#if 0
        PRINTF("%s, id=0x%3x\n\r", __FUNCTION__, rx_message.idStd);
        if (send_rpmsg)
        {
            /* Copy string to RPMsg tx buffer */
            sprintf(message, "id=%lu, payload=%lu %lu\r\n", rx_message.id, rx_message.word0, rx_message.word1);
            PRINTF("%s, message = %s\r\n", __FUNCTION__, message);

            // rpmsg_transmit(message, strlen(message));
        }
#endif


        // PRINTF("Message received: %d\n\r", count++);
        // PRINTF("DLC=%d, mb_idx=0x%3x\n\r", rx_message.dlc, rx_message.idStd);
        // PRINTF("RX MB data:\n\r");
        // PRINTF("%02X %02X %02X %02X %02X %02X %02X %02X \n\r", rx_message.data0,
        //                                                        rx_message.data1,
        //                                                        rx_message.data2,
        //                                                        rx_message.data3,
        //                                                        rx_message.data4,
        //                                                        rx_message.data5,
        //                                                        rx_message.data6,
        //                                                        rx_message.data7);
   }
}

/*!
 * @brief Tx and Rx CAN Message interruption handler
 */
void BOARD_FLEXCAN_HANDLER(void)
{
    /* Solve Tx interrupt */
    if (FLEXCAN_GetMsgBufStatusFlag(BOARD_FLEXCAN_BASEADDR, TX_MSG_BUF_NUM))
    {
        FLEXCAN_ClearMsgBufStatusFlag(BOARD_FLEXCAN_BASEADDR, TX_MSG_BUF_NUM);
    }

    /* Solve Rx interrupt */
    if (FLEXCAN_GetMsgBufStatusFlag(BOARD_FLEXCAN_BASEADDR, RX_MSG_BUF_NUM))
    {
        /* Lock message buffer for receive data. */
        FLEXCAN_LockRxMsgBuf(BOARD_FLEXCAN_BASEADDR, RX_MSG_BUF_NUM);

        /* Send message data to queue */
        xQueueSendFromISR(rx_can_msg_queue, rxMsgBufPtr, 0U);

        /* Unock message buffer for receive data */
        FLEXCAN_UnlockAllRxMsgBuf(BOARD_FLEXCAN_BASEADDR);

        FLEXCAN_ClearMsgBufStatusFlag(BOARD_FLEXCAN_BASEADDR, RX_MSG_BUF_NUM);
    }
}

void rpmsg_receive(char *received_msg, int *length)
{
    unsigned long src;
    int result;

    void *rx_buf; /* pointer to rx shared buffer */

    /* Get RPMsg rx buffer with message */
    result = rpmsg_rtos_recv_nocopy(app_chnl->rp_ept, &rx_buf, length, &src, 0xFFFFFFFF);
    assert(result == 0);
    // PRINTF("length=%d, src=%d\r\n", *length, src);

    /* Each RPMSG buffer can carry less than 512 payload */
    assert(*length < MAX_RPMSG_LENGTH);
    /* Copy string from RPMsg rx buffer */
    memcpy(received_msg, rx_buf, *length);
    /* End string by '\0' */
    received_msg[*length] = 0; 

    /* Release held RPMsg rx buffer */
    result = rpmsg_rtos_recv_nocopy_free(app_chnl->rp_ept, rx_buf);
    assert(result == 0);
}

void rpmsg_transmit(char *transmit_msg, int length)
{
    void *tx_buf;
    unsigned long size;
    int result;

    /* Get tx buffer from RPMsg */
    tx_buf = rpmsg_rtos_alloc_tx_buffer(app_chnl->rp_ept, &size);
    assert(tx_buf);

    /* Copy string to RPMsg tx buffer */
    memcpy(tx_buf, transmit_msg, length);

    /* Echo back received message with nocopy send */
    result = rpmsg_rtos_send_nocopy(app_chnl->rp_ept, tx_buf, length, 1024);
    assert(result == 0);
}

/*!
 * @brief Task handling the reception of commands through RPMsg
 */
void rpmsg_receive_task(void *pv_param)
{
    char received_msg[MAX_RPMSG_LENGTH];
    int length, result;

    PRINTF("%s - %s\r\n", __FUNCTION__, __TIME__);

    /* Initializes the rpmsg driver resources */
    /* RPMSG Init as REMOTE */
    result = rpmsg_rtos_init(0, &rdev, RPMSG_MASTER, &app_chnl);
    assert(result == 0);
    PRINTF("Name service handshake is done, M4 has setup a rpmsg channel [%d ---> %d]\r\n", app_chnl->src, app_chnl->dst);

    while (1)
    {
        rpmsg_receive(received_msg, &length);
        if (strcmp(received_msg, "start") == 0)
        {
            send_rpmsg = 1;
            rpmsg_transmit("OK", 2);
            PRINTF("start!\r\n");
        }
        else
        {
            rpmsg_transmit("NACK", 2);
        }
    }
}


/*!
 * @brief MU Interrrupt ISR
 */
void BOARD_MU_HANDLER(void)
{
    /*
     * Calls into rpmsg_handler provided by middleware at
     * middleware\multicore\open-amp\porting\imx7d_m4\platform.c
     */
    rpmsg_handler();
}
