#ifndef __CONFIG_H
#define __CONFIG_H

/* -------------------------------------------------------------------------- */
/*  Device Role Selection (uncomment one)                                     */
/* -------------------------------------------------------------------------- */
#define ROLE_MANAGER      1
#define ROLE_COLLECTOR_1  2
#define ROLE_COLLECTOR_2  3

#define DEVICE_ROLE  ROLE_MANAGER

/* -------------------------------------------------------------------------- */
/*  RS485 Address                                                            */
/* -------------------------------------------------------------------------- */
#if DEVICE_ROLE == ROLE_MANAGER
  #define DEVICE_ADDR  0x01
#elif DEVICE_ROLE == ROLE_COLLECTOR_1
  #define DEVICE_ADDR  0x02
#elif DEVICE_ROLE == ROLE_COLLECTOR_2
  #define DEVICE_ADDR  0x03
#else
  #error "DEVICE_ROLE must be ROLE_MANAGER, ROLE_COLLECTOR_1, or ROLE_COLLECTOR_2"
#endif

/* -------------------------------------------------------------------------- */
/*  Timing Parameters (ms)                                                   */
/* -------------------------------------------------------------------------- */
#define POLL_INTERVAL_MS    10000   /* Manager polling cycle                 */
#define RETRY_TIMEOUT_MS    500     /* Manager retry timeout                  */
#define FRAME_TIMEOUT_MS    100     /* Inter-byte frame timeout               */

/* -------------------------------------------------------------------------- */
/*  Sensor Intervals (ms)                                                    */
/* -------------------------------------------------------------------------- */
#define DHT_INTERVAL_MS     5000    /* Collector-1 DHT read interval          */
#define MPU6050_INTERVAL_MS 2000    /* Collector-2 MPU6050 read interval      */

/* -------------------------------------------------------------------------- */
/*  Data Store Depth                                                         */
/* -------------------------------------------------------------------------- */
#define STORE_DEPTH         10

/* -------------------------------------------------------------------------- */
/*  RS485 Max Frame Payload (bytes, excluding header/tail/CRC)               */
/* -------------------------------------------------------------------------- */
#define RS485_MAX_PAYLOAD   32

#endif /* __CONFIG_H */
