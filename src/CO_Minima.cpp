#include <CO_Minima.hpp>

void setup()
{
    Serial.begin(SERIAL_BAUD);
    Log.begin(LOG_LEVEL_VERBOSE, &Serial);
    delay(2000);

    Log.noticeln("CANOpen Setup");

    // Create Semaphore
    CAN_ctrl_task_sem = xSemaphoreCreateBinary();

    // Create CAN RX and TX queues
    CAN_TX_queue = xQueueCreate(CAN_TX_queue_len, sizeof(TX_Messages));
    CAN_RX_queue = xQueueCreate(CAN_TX_queue_len, sizeof(RX_Messages));

    // Give Semaphore to CAN CTRL
    xSemaphoreGive(CAN_ctrl_task_sem);

    // Create CAN_CTRL Task MARK: CAN CTRL TASK
    auto const ctrl_task_create = xTaskCreate(CAN_ctrl_task, static_cast<const char *>("CAN_ctrl"), CO_CTRL_TASK_STACK_DEPTH, nullptr, CAN_CTRL_TASK_PRIO, &can_ctrl_task);
    if (ctrl_task_create != pdPASS)
    {
        Log.errorln("Failed to create 'can_ctrl' thread");
    }
    else
    {
        Log.noticeln("Created can_ctrl Thread");
    }

    // Create CO_MAIN Task MARK: CO MAIN TASK
    auto const co_main_create = xTaskCreate(CO_main, static_cast<const char *>("CO_MAIN"), CO_MAIN_TASK_STACK_DEPTH, nullptr, CANOPEN_TASK_PRIO, &co_main_task);
    if (co_main_create != pdPASS)
        Log.errorln("Failed to create 'co_main' thread");
    else
        Log.noticeln("Created co_main thread");

    Log.noticeln("Starting Scheduler...");

    vTaskStartScheduler();

    Log.errorln("How am I here???");
}

// This function sets up the CAN bus using Arduino CAN and provides some error handling for CAN bus errors.  TX and RX queues
// are used to receive and transmit data from the CAN controller in the Arduino minima MARK: CAN CTRL
static void CAN_ctrl_task(void *args)
{
    CANMessages tx_msg;
    CanMsg rx_message; // Might require const

    xSemaphoreTake(CAN_ctrl_task_sem, portMAX_DELAY);

    // MARK: CAN Begin
    if (!CAN.begin(CanBitRate::BR_500k))
    {
        Log.errorln("CAN.begin(...) failed.");
        for (;;)
        {
        }
    }
    Log.noticeln("CAN Driver installed");
    Log.noticeln("CAN Driver started");

    // clear out any old messages on bus
    unsigned long currentTime = millis();
    while ((millis() - currentTime) < 100)
    {
        if (CAN.available())
        {
            rx_message = CAN.read();
        }
    }

    while (1)
    {
        if (CAN.available())
        {
            // MARK: CAN Read
            rx_message = CAN.read();

            if (uxQueueSpacesAvailable(CAN_RX_queue))
            {
                xQueueSend(CAN_RX_queue, (void *)&rx_message, 10);
            }
            else
            {
                xQueueSend(CAN_RX_queue, (void *)&rx_message, 10);
                Log.warningln(" RX Queue Full");
            }
        }

        if (xQueueReceive(CAN_TX_queue, (void *)&tx_msg, 0) == pdTRUE)
        {
            //MARK: CAN WRITE
            if (int const rc = CAN.write(tx_msg.message); rc < 0)
            {
                Log.warningln("CAN.write(...) failed with error code %d", rc);
            }
        }

        // if (CAN.isError())
        // {
        //     CAN.clearError();
        //     Log.noticeln("Error cleared");
        // }
        // Log.noticeln("CAN CTRL TASK");
        vTaskDelay(CANOPEN_CTRL_TASK_DELAY / portTICK_PERIOD_MS);
    }
    xSemaphoreGive(CAN_ctrl_task_sem);
    vTaskDelete(nullptr);
}

// CO_main is also provided by CANopenNode in example documentation for basic implementation of CANopenNode.  This function
// has been modified to start the CO_tmr and CO_interrupt tasks that handle processing of messages and receiving of messages
// MARK: CO MAIN
static void CO_main(void *arg)
{
    CO_ReturnError_t err;
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
    uint32_t heapMemoryUsed;
    void *CANptr = nullptr;
    uint8_t pendingNodeId = CO_NODE_ID;
    uint8_t activeNodeId = CO_NODE_ID;
    uint16_t pendingBitRate = 500;

    // Allocate memory

    CO_config_t *config_ptr = nullptr;
    CO = CO_new(config_ptr, &heapMemoryUsed);
    if (CO == nullptr)
    {
        Log.errorln("Can't allocate memory");
        return;
    }
    else
    {
        Log.noticeln("Allocated %d bytes for CANopen objects", heapMemoryUsed);
    }

    while (reset != CO_RESET_APP)
    {
        /* CANopen communication reset - initialize CANopen objects *******************/
        Log.warningln("CANopenNode - Reset Communication...");

        /* Wait rt_thread. */
        CO->CANmodule->CANnormal = false;

        /* Enter CAN configuration. */
        CO_CANsetConfigurationMode((void *)&CANptr);
        CO_CANmodule_disable(CO->CANmodule);

        /* initialize CANopen */
        err = CO_CANinit(CO, CANptr, pendingBitRate);

        if (err != CO_ERROR_NO)
        {
            Log.errorln("Error: CAN initialization failed: %d", err);
            return;
        }

        CO_LSS_address_t lssAddress = {.identity = {
                                           .vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
                                           .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
                                           .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
                                           .serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber}};

        err = CO_LSSinit(CO, &lssAddress, &pendingNodeId, &pendingBitRate);
        if (err != CO_ERROR_NO)
        {
            Log.errorln("Error: LSS slave initialization failed: %d", err);
            return;
        }

        activeNodeId = pendingNodeId;
        uint32_t errInfo = 0;

        err = CO_CANopenInit(CO,                                         /* CANopen object */
                             nullptr,                                    /* alternate NMT */
                             nullptr,                                    /* alternate em */
                             OD,                                         /* Object dictionary */
                             OD_STATUS_BITS,                             /* Optional OD_statusBits */
                             static_cast<CO_NMT_control_t>(NMT_CONTROL), /* CO_NMT_control_t */
                             FIRST_HB_TIME,                              /* firstHBTime_ms */
                             SDO_SRV_TIMEOUT_TIME,                       /* SDOserverTimeoutTime_ms */
                             SDO_CLI_TIMEOUT_TIME,                       /* SDOclientTimeoutTime_ms */
                             SDO_CLI_BLOCK,                              /* SDOclientBlockTransfer */
                             activeNodeId,
                             &errInfo);

        if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS)
        {
            if (err == CO_ERROR_OD_PARAMETERS)
            {
                Log.errorln("Error: Object Dictionary entry 0x%X", errInfo);
            }
            else
            {
                Log.errorln("Error: CANopen initialization failed: %d", err);
            }
            return;
        }

        err = CO_CANopenInitPDO(CO, CO->em, OD, activeNodeId, &errInfo);
        if (err != CO_ERROR_NO)
        {
            if (err = CO_ERROR_OD_PARAMETERS)
            {
                Log.errorln("Error: Object Dictionary entry 0x%X", errInfo);
            }
            else
            {
                Log.errorln("Error: PDO initialization failed: %d", err);
            }
            return;
        }

        /* Configure Timer interrupt function for execution every 1 millisecond */
        auto const co_tmr_create = xTaskCreate(CO_tmrTask, static_cast<const char *>("CO_TMR"), CO_TIMER_TASK_STACK_DEPTH, nullptr, CANOPEN_TMR_TASK_PRIO, &co_tmr_task);
        if (co_tmr_create != pdPASS)
            Log.errorln("Failed to create 'co_tmr' thread");
        else
            Log.noticeln("Created co_tmr thread");

        auto const co_int_create = xTaskCreate(CO_interruptTask, static_cast<const char *>("CO_INT"), CO_INTERRUPT_TASK_STACK_DEPTH, nullptr, CANOPEN_INTERRUPT_TASK_PRIO, &co_interrupt_task);
        if (co_int_create != pdPASS)
            Log.errorln("Failed to create 'co_int' thread");
        else
            Log.noticeln("Created co_int thread");

        if (!CO->nodeIdUnconfigured)
        {
            Log.noticeln("CAN ID Configured to %d", activeNodeId);
        }
        else
        {
            Log.errorln("CANopenNode - Node-ID not initialized");
        }

        CO_CANsetNormalMode(CO->CANmodule);

        reset = CO_RESET_NOT;

        Log.noticeln("CANOpenNode - Running...");
        fflush(stdout);

        while (reset == CO_RESET_NOT)
        {
            uint32_t timeDifference_us = 1000;

            reset = CO_process(CO, false, timeDifference_us, nullptr);

            // OD_PERSIST_COMM.x6000_counter++;

            // Serial.println(OD_PERSIST_COMM.x6000_counter);

            // if (OD_PERSIST_COMM.x6001_counterFromNode != 0)

            // {

            // Serial.println("Received RPDO");

            // Serial.println(OD_PERSIST_COMM.x6001_counterFromNode);
            // }

            /* Nonblocking application code may go here. */

            /* Process automatic storage */

            /* optional sleep for short time */

            // Serial.println("Inside CO PROCESS");

            vTaskDelay(CANOPEN_MAIN_TASK_DELAY / portTICK_PERIOD_MS);
        }
    }

    CO_CANsetConfigurationMode((void *)&CANptr);
    CO_delete(CO);

    Log.noticeln("CANopenNode finished");
    vTaskDelete(nullptr);
}

//MARK: CO TMR 
static void CO_tmrTask(void *arg)
{
    static unsigned long prevTime = 0;
    while (1)
    {
        if (!CO->nodeIdUnconfigured && CO->CANmodule->CANnormal)
        {
            bool_t syncWas = false;
            /* get time difference since last function call */
            uint32_t timeDifference_us = 1000;

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
            syncWas = CO_process_SYNC(CO, timeDifference_us, nullptr);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
            CO_process_RPDO(CO, syncWas, timeDifference_us, nullptr);
            // Serial.println("RPDO Process");
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
            CO_process_TPDO(CO, syncWas, timeDifference_us, nullptr);
            // Serial.println("PDO Process Running");
#endif
            /* Further I/O or nonblocking application code may go here. */
        }
        vTaskDelay(CANOPEN_TIMER_TASK_DELAY / portTICK_PERIOD_MS);
    }
    vTaskDelete(nullptr);
}

//MARK: CO INTERRUPT
static void CO_interruptTask(void *arg)
{
    while (1)
    {
        CO_CANinterrupt(CO->CANmodule);
        vTaskDelay(CANOPEN_INTERRUPT_TASK_DELAY / portTICK_PERIOD_MS);
    }

    vTaskDelete(nullptr);
}

void loop()
{

}