#include "modbus.h"
#include "basesystem_interface.h"

static ModbusHandleTypedef *hMB;
extern volatile uint8_t  dbg_crc_fail;
extern volatile uint16_t dbg_crc_calc;
extern volatile uint16_t dbg_crc_recv;
extern volatile uint8_t  dbg_tx_count;
extern volatile uint8_t dbg_worker_state;
/* ══════════════════════════════════════════════════
   CRC-16 / Modbus  (lookup-table, เร็วกว่า bit-bang)
 ══════════════════════════════════════════════════ */
static const uint8_t auchCRCLo[] = {
    0x00,0xC0,0xC1,0x01,0xC3,0x03,0x02,0xC2,0xC6,0x06,0x07,0xC7,0x05,0xC5,0xC4,0x04,
    0xCC,0x0C,0x0D,0xCD,0x0F,0xCF,0xCE,0x0E,0x0A,0xCA,0xCB,0x0B,0xC9,0x09,0x08,0xC8,
    0xD8,0x18,0x19,0xD9,0x1B,0xDB,0xDA,0x1A,0x1E,0xDE,0xDF,0x1F,0xDD,0x1D,0x1C,0xDC,
    0x14,0xD4,0xD5,0x15,0xD7,0x17,0x16,0xD6,0xD2,0x12,0x13,0xD3,0x11,0xD1,0xD0,0x10,
    0xF0,0x30,0x31,0xF1,0x33,0xF3,0xF2,0x32,0x36,0xF6,0xF7,0x37,0xF5,0x35,0x34,0xF4,
    0x3C,0xFC,0xFD,0x3D,0xFF,0x3F,0x3E,0xFE,0xFA,0x3A,0x3B,0xFB,0x39,0xF9,0xF8,0x38,
    0x28,0xE8,0xE9,0x29,0xEB,0x2B,0x2A,0xEA,0xEE,0x2E,0x2F,0xEF,0x2D,0xED,0xEC,0x2C,
    0xE4,0x24,0x25,0xE5,0x27,0xE7,0xE6,0x26,0x22,0xE2,0xE3,0x23,0xE1,0x21,0x20,0xE0,
    0xA0,0x60,0x61,0xA1,0x63,0xA3,0xA2,0x62,0x66,0xA6,0xA7,0x67,0xA5,0x65,0x64,0xA4,
    0x6C,0xAC,0xAD,0x6D,0xAF,0x6F,0x6E,0xAE,0xAA,0x6A,0x6B,0xAB,0x69,0xA9,0xA8,0x68,
    0x78,0xB8,0xB9,0x79,0xBB,0x7B,0x7A,0xBA,0xBE,0x7E,0x7F,0xBF,0x7D,0xBD,0xBC,0x7C,
    0xB4,0x74,0x75,0xB5,0x77,0xB7,0xB6,0x76,0x72,0xB2,0xB3,0x73,0xB1,0x71,0x70,0xB0,
    0x50,0x90,0x91,0x51,0x93,0x53,0x52,0x92,0x96,0x56,0x57,0x97,0x55,0x95,0x94,0x54,
    0x9C,0x5C,0x5D,0x9D,0x5F,0x9F,0x9E,0x5E,0x5A,0x9A,0x9B,0x5B,0x99,0x59,0x58,0x98,
    0x88,0x48,0x49,0x89,0x4B,0x8B,0x8A,0x4A,0x4E,0x8E,0x8F,0x4F,0x8D,0x4D,0x4C,0x8C,
    0x44,0x84,0x85,0x45,0x87,0x47,0x46,0x86,0x82,0x42,0x43,0x83,0x41,0x81,0x80,0x40
};
static const uint8_t auchCRCHi[] = {
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40
};

static uint16_t CRC16(uint8_t *buf, uint16_t len)
{
    uint8_t hi = 0xFF, lo = 0xFF, idx;
    while (len--) {
        idx = lo ^ *buf++;
        lo  = hi ^ auchCRCHi[idx];
        hi  = auchCRCLo[idx];
    }
    return (uint16_t)(hi << 8 | lo);
}

/* ══════════════════════════════════════════════════
   FC handlers  (Rxframe layout: [FC][reg_hi][reg_lo][data...])
   — slave addr และ CRC ถูกดึงออกก่อนเข้ามาที่นี่แล้ว
 ══════════════════════════════════════════════════ */

static void ModbusErrorReply(uint8_t fc, uint8_t errorCode)
{
    hMB->Txframe[0] = fc | 0x80;
    hMB->Txframe[1] = errorCode;
    hMB->TxCount    = 2;
}

/* FC 0x03 — Read Holding Registers */
static void fc03(void)
{
    uint16_t start = (uint16_t)((hMB->Rxframe[1] << 8) | hMB->Rxframe[2]);
    uint16_t qty   = (uint16_t)((hMB->Rxframe[3] << 8) | hMB->Rxframe[4]);

    if (qty < 1 || qty > 0x7D) {
        ModbusErrorReply(0x03, 0x03);   /* Illegal Data Value */
        return;
    }
    if (start >= hMB->RegisterSize || (start + qty) > hMB->RegisterSize) {
        ModbusErrorReply(0x03, 0x02);   /* Illegal Data Address */
        return;
    }

    hMB->Txframe[0] = 0x03;
    hMB->Txframe[1] = (uint8_t)(qty * 2);
    for (uint16_t i = 0; i < qty; i++) {
        hMB->Txframe[2 + 2*i]     = hMB->RegisterAddress[start + i].U8[1]; /* hi */
        hMB->Txframe[2 + 2*i + 1] = hMB->RegisterAddress[start + i].U8[0]; /* lo */
    }
    hMB->TxCount = 2 + qty * 2;


}

/* FC 0x06 — Write Single Register */
static void fc06(void)
{
    uint16_t addr  = (uint16_t)((hMB->Rxframe[1] << 8) | hMB->Rxframe[2]);
    uint16_t value = (uint16_t)((hMB->Rxframe[3] << 8) | hMB->Rxframe[4]);
    hMB->RegisterAddress[addr].U16 = value;  // ← breakpoint

    if (addr >= hMB->RegisterSize) {
        ModbusErrorReply(0x06, 0x02);
        return;
    }

    hMB->RegisterAddress[addr].U16 = value;

    /* echo payload กลับ (slave addr + CRC เติมใน Emission) */
    memcpy(hMB->Txframe, hMB->Rxframe, 5);   /* FC + addr(2) + value(2) */
    hMB->TxCount = 5;
}

/* FC 0x10 — Write Multiple Registers */
static void fc10(void)
{
    uint16_t start    = (uint16_t)((hMB->Rxframe[1] << 8) | hMB->Rxframe[2]);
    uint16_t qty      = (uint16_t)((hMB->Rxframe[3] << 8) | hMB->Rxframe[4]);
    /* Rxframe[5] = byte count, Rxframe[6..] = data */

    if (qty < 1 || qty > 0x7B) {
        ModbusErrorReply(0x10, 0x03);
        return;
    }
    if (start >= hMB->RegisterSize || (start + qty) > hMB->RegisterSize) {
        ModbusErrorReply(0x10, 0x02);
        return;
    }

    for (uint16_t i = 0; i < qty; i++) {
        hMB->RegisterAddress[start + i].U8[1] = hMB->Rxframe[6 + 2*i];     /* hi */
        hMB->RegisterAddress[start + i].U8[0] = hMB->Rxframe[6 + 2*i + 1]; /* lo */
    }

    /* response: FC + start(2) + qty(2) */
    hMB->Txframe[0] = 0x10;
    hMB->Txframe[1] = hMB->Rxframe[1];
    hMB->Txframe[2] = hMB->Rxframe[2];
    hMB->Txframe[3] = hMB->Rxframe[3];
    hMB->Txframe[4] = hMB->Rxframe[4];
    hMB->TxCount    = 5;
}

static void Modbus_frame_response(void)
{
    uint8_t fc = hMB->Rxframe[0];

    // 🌟 1. ดึง Address ที่ PC กำลังจะเขียนออกมาดู (ใช้วิธีต่อ Byte บน-ล่าง)
    uint16_t addr = (uint16_t)((hMB->Rxframe[1] << 8) | hMB->Rxframe[2]);

    switch (fc) {   /* Rxframe[0] = Function Code */
        case 0x03: fc03(); break;
        case 0x06: fc06(); break;
        case 0x10: fc10(); break;
        default:   ModbusErrorReply(hMB->Rxframe[0], 0x01); break;
    }

    if (fc == 0x06 || fc == 0x10) {
        // ถ้าระบบไม่ได้พ่น Error ออกมา (Txframe[0] ต้องเท่ากับ fc เดิม)
        if (hMB->Txframe[0] == fc) {

            // 🌟 2. ดักไว้เลย! ถ้าไม่ใช่ Address 0x00 (Heartbeat) ถึงจะยอมยกธง
            if (addr != 0x00) {

            	// Has New data
//                Basesystem_Data.has_new_data = 1;
//                Robot.robot_has_new_data_from_basesystem = 1;
                if (Robot.Robot_Status == Ready_recieve_Basesystem){
                	   Robot.Data_from_Basesystem = have_data_Basesystem ;
                }

                // separate Auto | P2P, Sequence
                if (addr >= 18 && addr <= 34) {
					Basesystem_Data._Auto.Type = AUTO_TYPE_SEQUENCE;
				} else if (addr == 35 || addr == 36) {
					Basesystem_Data._Auto.Type = AUTO_TYPE_P2P;
				}
            }

        }
    }
}

/* ══════════════════════════════════════════════════
   Emission — ประกอบ frame แล้วส่งผ่าน DMA
   Layout: [slave_addr][FC][data...][CRC_lo][CRC_hi]
 ══════════════════════════════════════════════════ */
static void Modbus_Emission(void)
{
    uint8_t  *tx   = hMB->modbusUartStructure.MessageBufferTx;
    uint16_t  plen = hMB->TxCount;   /* ความยาว payload (FC + data) */

    /* byte[0] = slave address */
    tx[0] = hMB->slaveAddress;

    /* byte[1..plen] = Txframe */
    memcpy(&tx[1], hMB->Txframe, plen);

    /* total ก่อน CRC = 1 (addr) + plen */
    uint16_t data_len = 1 + plen;

    /* คำนวณ CRC จาก addr ถึง byte สุดท้ายของ payload */
    u16u8_t crc;
    crc.U16 = CRC16(tx, data_len);

    /* ต่อท้าย: CRC low byte ก่อน (Modbus Little-Endian) */
    tx[data_len]     = crc.U8[0];   /* low  */
    tx[data_len + 1] = crc.U8[1];   /* high */

    uint16_t total = data_len + 2;
    hMB->modbusUartStructure.TxTail = total;

    /* ส่งผ่าน DMA */
    HAL_UART_Transmit_DMA(hMB->huart, tx, total);
}

/* ══════════════════════════════════════════════════
   Public API
 ══════════════════════════════════════════════════ */
void Modbus_init(ModbusHandleTypedef *hmodbus, u16u8_t *regs)
{
    hMB = hmodbus;
    hMB->RegisterAddress = regs;
    hMB->Mstatus         = Modbus_state_Init;
    hMB->TxCount         = 0;
    hMB->Flag_T15TimeOut = 0;
    hMB->Flag_T35TimeOut = 0;
    hMB->Flag_URev       = 0;
    hMB->modbusUartStructure.RxTail = 0;

    /* เริ่มรับผ่าน DMA + idle-line detection */
    HAL_UARTEx_ReceiveToIdle_DMA(hMB->huart,
        hMB->modbusUartStructure.MessageBufferRx,
        MODBUS_BUFFER_SIZE);

    /* ปิด half-transfer interrupt ของ DMA ที่ฝั่ง RX
       (ไม่งั้น callback จะถูกเรียกตอนครึ่งทางด้วย) */
    __HAL_DMA_DISABLE_IT(hMB->huart->hdmarx, DMA_IT_HT);
}

void Modbus_Protocal_Worker(void)
{
	dbg_worker_state = (uint8_t)hMB->Mstatus;
    switch (hMB->Mstatus) {

    default:
    case Modbus_state_Init:
        hMB->Mstatus = Modbus_state_Idle;
        break;

    /* ── รอ frame เข้า หรือรอส่ง ── */
    case Modbus_state_Idle:
        if (hMB->Flag_T15TimeOut) {
            hMB->Mstatus = Modbus_state_ControlAndWaiting;
        }
        break;

    /* ── ประมวลผล frame ที่รับมา ── */
    case Modbus_state_ControlAndWaiting: {
        uint16_t len = hMB->modbusUartStructure.RxTail;
        uint8_t *buf = hMB->modbusUartStructure.MessageBufferRx;

        /* ต้องมีอย่างน้อย: addr(1) + FC(1) + CRC(2) = 4 bytes */
        if (len < 4) goto reset_rx;

        /* ตรวจ CRC */
        u16u8_t crc;
        crc.U16 = CRC16(buf, len - 2);
        dbg_crc_calc = crc.U16;
        dbg_crc_recv = (uint16_t)(buf[len-1] << 8 | buf[len-2]);

        if (crc.U8[0] != buf[len-2] || crc.U8[1] != buf[len-1]) {
            dbg_crc_fail++;
            goto reset_rx;
        }

        /* ตรวจ slave address */
        if (buf[0] == hMB->slaveAddress) {
            /* copy FC + data เข้า Rxframe (ตัด slave addr และ CRC ออก) */
            memcpy(hMB->Rxframe, &buf[1], len - 3);
            Modbus_frame_response();
            dbg_tx_count = hMB->TxCount;
        }
        /* ถ้า address ไม่ตรง → ไม่ตอบ (broadcast ปกติ) */

        /* ส่งกลับถ้ามี response */
        if (hMB->TxCount > 0) {
            Modbus_Emission();
            hMB->TxCount = 0;
        }

    reset_rx:
        hMB->modbusUartStructure.RxTail = 0;
        hMB->Flag_T15TimeOut = 0;
        hMB->Flag_URev       = 0;
        hMB->Mstatus         = Modbus_state_Idle;

        /* restart รับ frame ถัดไป */
        HAL_UARTEx_ReceiveToIdle_DMA(hMB->huart,
            hMB->modbusUartStructure.MessageBufferRx,
            MODBUS_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(hMB->huart->hdmarx, DMA_IT_HT);
        break;
    }

    /* ── ไม่ใช้แล้ว ── */
    case Modbus_state_Reception:
        hMB->Mstatus = Modbus_state_Idle;
        break;

    case Modbus_state_Emission:
        hMB->Mstatus = Modbus_state_Idle;
        break;
    }
}
