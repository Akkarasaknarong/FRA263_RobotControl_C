#ifndef INC_MODBUS_H_
#define INC_MODBUS_H_

#include "stm32g4xx_hal.h"
#include "datatype.h"
extern Robot_t Robot ;

/* ── Config ─────────────────────────────────────── */
#define MODBUS_SLAVE_ADDRESS    21
#define MODBUS_REGISTER_COUNT   0x32   /* 50 registers */
#define MODBUS_BUFFER_SIZE      300

/* ── Heartbeat ───────────────────────────────────── */
#define HEARTBEAT_ROBOT  22881u
#define HEARTBEAT_PC     18537u

/* ── Register map ───────────────────────────────── */
#define REG_HEARTBEAT        0x00
#define REG_MODE             0x01
#define REG_GRIPPER_MANUAL   0x02
#define REG_GRIPPER_SEQ      0x03
#define REG_GRIPPER_AUTO_EN  0x04
#define REG_JOG              0x05
#define REG_TEST_TYPE        0x06
#define REG_PERF_VEL         0x07
#define REG_PERF_ACC         0x08
#define REG_PREC_INIT        0x09
#define REG_PREC_FINAL       0x0A
#define REG_PREC_REPEAT      0x0B
#define REG_PICKPLACE_START  0x12
#define REG_PICKPLACE_COUNT  0x22
#define REG_P2P_UNIT         0x23
#define REG_P2P_TARGET       0x24
#define REG_SOFT_STOP        0x25
#define REG_SENSORS          0x26
#define REG_TASK             0x27
#define REG_POSITION         0x28
#define REG_VELOCITY         0x29
#define REG_ACCELERATION     0x30
#define REG_EMERGENCY        0x31

/* ── Mode bits ──────────────────────────────────── */
#define MODE_HOME      0x0001
#define MODE_JOG       0x0002
#define MODE_AUTO      0x0004
#define MODE_SET_HOME  0x0008
#define MODE_TEST      0x0010

/* ── Union 16-bit ↔ 2×8-bit ────────────────────── */
//typedef union {
//    uint16_t U16;
//    uint8_t  U8[2];
//} u16u8_t;

/* ── Modbus State ───────────────────────────────── */
typedef enum {
    Modbus_state_Init,
    Modbus_state_Idle,
    Modbus_state_Emission,
    Modbus_state_Reception,
    Modbus_state_ControlAndWaiting
} ModbusStateTypedef;

/* ── Handle ─────────────────────────────────────── */
typedef struct {
    uint8_t             slaveAddress;
    u16u8_t            *RegisterAddress;
    uint32_t            RegisterSize;
    UART_HandleTypeDef *huart;
    TIM_HandleTypeDef  *htim;       /* TIM16: 2t one-pulse */

    uint8_t  Flag_T15TimeOut;
    uint8_t  Flag_T35TimeOut;
    uint8_t  Flag_URev;             /* byte received flag */

    ModbusStateTypedef Mstatus;

    uint8_t  Rxframe[MODBUS_BUFFER_SIZE];
    uint8_t  Txframe[MODBUS_BUFFER_SIZE];
    uint8_t  TxCount;

    struct {
        uint8_t  MessageBufferRx[MODBUS_BUFFER_SIZE + 3];
        uint16_t RxTail;
        uint8_t  MessageBufferTx[MODBUS_BUFFER_SIZE + 3];
        uint16_t TxTail;
    } modbusUartStructure;

} ModbusHandleTypedef;

/* ── API ─────────────────────────────────────────── */
void Modbus_init(ModbusHandleTypedef *hmodbus, u16u8_t *regs);
void Modbus_Protocal_Worker(void);

#endif
