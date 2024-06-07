#pragma once

//CAN Task Priorities
#define CAN_MSGPRINT_TASK_PRIO 4
#define CAN_MSGGEN_TASK_PRIO 3
#define CAN_CTRL_TASK_PRIO 1
#define CANOPEN_TASK_PRIO 3
#define CANOPEN_TMR_TASK_PRIO 4
#define CANOPEN_INTERRUPT_TASK_PRIO 2
#define CANOPEN_SDO_TASK_PRIO 1

// CAN Node ID
#define CO_NODE_ID 2

// Serial Baud Rate
#define SERIAL_BAUD 2000000

// Task Delays
#define CANOPEN_CTRL_TASK_DELAY 1      // Start CAN and for read and write operations
#define CANOPEN_MAIN_TASK_DELAY 1      // CAN Open Init and CO_TMR_TASK, CO_INT_TASK init, and other non blocking functions go here
#define CANOPEN_TIMER_TASK_DELAY 1     // Can Open SYNC function happens here
#define CANOPEN_INTERRUPT_TASK_DELAY 1 // CAN Open rx and tx interrupt

// Allocate memory for tasks
#define CO_CTRL_TASK_STACK_DEPTH 128
#define CO_MAIN_TASK_STACK_DEPTH 256
#define CO_TIMER_TASK_STACK_DEPTH 256
#define CO_INTERRUPT_TASK_STACK_DEPTH 256
#define CO_SDO_TASK_STACK_DEPTH 128

// CANOpen settings
#define NMT_CONTROL               \
    CO_NMT_STARTUP_TO_OPERATIONAL \
    | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION
#define FIRST_HB_TIME 500
#define SDO_SRV_TIMEOUT_TIME 1000
#define SDO_CLI_TIMEOUT_TIME 1000
#define SDO_CLI_BLOCK false
#define OD_STATUS_BITS nullptr

#include <Arduino.h>
#include <time.h>
#include <Arduino_CAN.h>
#include <CANopen.h>
#include <Arduino_FreeRTOS.h>
#include <ArduinoLog.h>
#include "OD.h"

CanBitRate can_bitrate = CanBitRate::BR_250k;

typedef struct CANMessages
{
    CanMsg const message;
} TX_Messages, RX_Messages;

static SemaphoreHandle_t CAN_ctrl_task_sem;

static const int CAN_TX_queue_len = 10;
static const int CAN_RX_queue_len = 10;

QueueHandle_t CAN_TX_queue;
QueueHandle_t CAN_RX_queue;

TaskHandle_t can_ctrl_task, co_main_task, co_sdo_task, co_tmr_task, co_interrupt_task, loop_task;

/* Global variables and objects */

/* CANopen object */
CO_t *CO = nullptr;

// Global variables
uint32_t time_old, time_current;
CO_ReturnError_t err;

// Tash and helper functions
static void CAN_ctrl_task(void *args);
static void CO_main(void *arg);
static void CO_tmrTask(void *arg);
static void CO_interruptTask(void *arg);
static void CO_SDOTask(void *arg);
CO_SDO_abortCode_t read_SDO(CO_SDOclient_t *SDO_C, uint8_t nodeID, uint16_t index, uint8_t subIndex, uint16_t *buf, size_t bufSize, size_t *readSize);
CO_SDO_abortCode_t write_SDO(CO_SDOclient_t *SDO_C, uint8_t nodeID, uint16_t index, uint8_t subIndex, uint8_t *data, size_t dataSize);
