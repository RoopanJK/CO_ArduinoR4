# CO_ArduinoR4


Completed:

1. Hearbeat producer.
2. TPDO working verified.


TODO:

1. Potential Bug: SDO read request message is not sent along with the index and subindex, opened an issue in CanOpenNode git.
2. NMT Management.
3. LSS Slave has to be tested.
4. Deeper understanding of vTaskDelay's for each task.
5. RPDO has to be verified


Important notes:

1. OD file has to placed inside /lib/CANopenNode, as platform IO build flags hasn't been configured yet.