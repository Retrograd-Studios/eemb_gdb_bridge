#if !defined(_MSC_VER)
#include <sys/time.h>
#endif // _MSC_VER

// #if defined(_WIN32)
// #include "src/win32/win32_socket.h"
// #endif // _WIN32

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <unistd.h>

#include <stlink.h>
#include "uart.h"

#include "logging.h"
#include "register.h"

// #include "sim.h"
#include "modbus_wrapper.h"

// REQUEST 1byte(0)_0_12bytes(UuID)_2byte(cmd)_4bytes(addr)_1byte(datasize)_Xbytes(data)_2bytes(crc)
// NEW: 1byte(0)_0_12bytes(UuID)_1byte(cmd_DEBUG)_1byte(DEBUG_COMMAND)_4bytes(addr)_1byte(datasize)_Xbytes(data)_2bytes(crc)
// RESPONSE 1b(0)_1b(cmd)_4b(dataSize)_Xb(data)_2b(crc) (1+1+4+256+2)264
// define somewhere as global variable


static uint8_t LOG_STATUS = 0;

int32_t parseResponse(stlink_t *sl, debug_commands_e cmd)
{
    if (sl->msgStatus)
    {
        ELOG("ANSWER NOT RECEIVED for cmd {%d}\n", cmd);
        return (-1);    
    }
    ssize_t i = 0;
    uint8_t rxbuf[RESPONSE_SIZE];
    memset(rxbuf, 0, RESPONSE_SIZE);

    #ifndef SIMULATOR
    uint8_t * prrxbuf = getRxBuff(sl->modbus); 
    memcpy(rxbuf, prrxbuf, RESPONSE_SIZE);
    #else
    memcpy(rxbuf, getRxBuff_gdb(sl->modbus), RESPONSE_SIZE);
    #endif 

    if (rxbuf[i++])
    {
        ELOG("First byte is not 0\n");
        return (-1);
    }

    if (MBL_DBG_CMD_GDB != (type_command_e) (rxbuf[i]))
    {
        ELOG("DebugCMD wrong {%s}: ", rxbuf[i]);
        return (-1);
    }
    i++;
    if (cmd != (debug_commands_e) rxbuf[i]){
        // DLOG("CMD_ERROR packageCMD{%s} : transferCMD{%s}\n");
        ELOG("CMD ERROR {%d}: \n", rxbuf[i]);
        return (-1);
    }
    i++;
    sl->q_len = read_uint32(rxbuf, i);
    i += 4;
    if (sl->q_len >= (RESPONSE_SIZE - 8)) 
    {
        ELOG("SIZE ERROR {%d}: \n", sl->q_len);
        return (-1);
    } 
    memcpy(sl->q_buf, &rxbuf[i], sl->q_len);
    i += sl->q_len;

    return sl->q_len;
}

uint32_t send_recv_uart(stlink_t *sl, debug_commands_e cmd, uint32_t memaddr, uint8_t* txbuf, uint16_t txsize) {
    
    // uint32_t dataSize;
    uint16_t cmdDebug = ((uint16_t) cmd << 8) | MBL_DBG_CMD_GDB ;
     
    #ifndef SIMULATOR
    sendMsg(sl->modbus, sl->uid[0], sl->uid[1], sl->uid[2], cmdDebug, memaddr, txbuf, txsize, PKG_WAIT_TIME_MS);

    volatile uint8_t busy = isBusy(sl->modbus);
    while(busy)
    {
        busy = isBusy(sl->modbus);
        usleep(500);
    }
    sl->msgStatus = isError(sl->modbus) ? 1 : 0;
    #else
    sendMsg_gdb(sl->modbus, sl->uid[0], sl->uid[1], sl->uid[2], cmdDebug, memaddr, txbuf, txsize, 3000);
    #endif 

    // dataSize = read_uint32(sl->q_buf, 2); // DELETE ALL BELOW
    #ifndef SIMULATOR
    return (read_uint32(getRxBuff(sl->modbus), 0));
    #else
    return (read_uint32(getRxBuff_sim(sl->modbus), 0));
    #endif 
    
}

int32_t uart_read_debug32(stlink_t *sl, uint32_t addr, uint32_t *data)
{ 
    
    send_recv_uart(sl, DEBUG_READDEBUGREG, addr, 0, 0); 
    int32_t size = parseResponse(sl, DEBUG_READDEBUGREG);
    if (size < 0)
    {
        return -1;
    }
    memcpy(data, sl->q_buf, sl->q_len);

    return 0;
}



int32_t uart_write_debug32(stlink_t *sl, uint32_t addr, uint32_t data)
{
    uint8_t txData[BUF_MAX_SIZE];

    write_uint32(txData, data);
    send_recv_uart(sl, DEBUG_WRITEDEBUGREG, addr,  txData, 4);
    int32_t size = parseResponse(sl, DEBUG_WRITEDEBUGREG);

    if ((sl->q_len == TXDATASIZE_STATUS) & (sl->q_buf[0] == TXDATA_SUCCESS))
    {
        memset(sl->q_buf, 0, 4);
        return 0;
    }
    memset(sl->q_buf, 0, 4);
    return -1;
}

int32_t uart_read_mem32(stlink_t *sl, uint32_t addr, uint16_t len) {
    
    uint8_t txData[BUF_MAX_SIZE];

    if ( addr == 0x00 && LOG_STATUS)
    {
        uart_print_log(sl);
        // return 0;
    }


    // uint32_t dataSize = 4;
    // uint8_t *rxData[dataSize];
    
    send_recv_uart(sl, DEBUG_READMEM_32BIT, addr,  0, len);
    int32_t size = parseResponse(sl, DEBUG_READMEM_32BIT);
    
     return(size < 0 ? -1 : 0);

}

int32_t uart_write_mem32(stlink_t *sl, uint32_t addr, uint16_t len) 
{
    uint8_t txData[RESPONSE_SIZE];

    memcpy(txData, sl->q_buf, len);

    send_recv_uart(sl, DEBUG_WRITEMEM_32BIT, addr,  txData, len);

    int32_t size = parseResponse(sl, DEBUG_WRITEMEM_32BIT);
    if ((sl->q_len == TXDATASIZE_STATUS) & (sl->q_buf[0] == TXDATA_SUCCESS))
    {
        memset(sl->q_buf, 0, 4);
        return 0;
    }
    memset(sl->q_buf, 0, 4);
    return -1;
} // DEBUG_WRITEMEM_32BIT

int32_t uart_write_mem8(stlink_t *sl, uint32_t addr, uint16_t len) // len??
{
    uint8_t txData[BUF_MAX_SIZE];

    memcpy(txData, (sl->q_buf), len);
    send_recv_uart(sl, DEBUG_WRITEMEM_32BIT, addr,  txData, len);
    int32_t size = parseResponse(sl, DEBUG_WRITEMEM_32BIT);

    if ((sl->q_len == TXDATASIZE_STATUS) & (sl->q_buf[0] == TXDATA_SUCCESS))
    {
        memset(sl->q_buf, 0, 4);
        return 0;
    }
    memset(sl->q_buf, 0, 4);
    return -1;
} // DEBUG_WRITEMEM_8BIT

int32_t uart_step(stlink_t* sl) {
    int32_t res;
    send_recv_uart(sl, DEBUG_STEPCORE, 0, 0, 0);
    res = parseResponse(sl, DEBUG_STEPCORE);
    if (res < 0){return -1;}

    if ((sl->q_len == TXDATASIZE_STATUS) & (sl->q_buf[0] == TXDATA_SUCCESS))
    {
        memset(sl->q_buf, 0, 4);
        DLOG("Step sent!\n"); 
        return 0;
    }
    else if ((sl->q_len == TXDATASIZE_STATUS) & (sl->q_buf[0] == TXDATA_FAILURE))
    {
        ELOG("Failure\n"); 
    }
    memset(sl->q_buf, 0, 4);
    
    return(-1);                                                       
}

int32_t uart_read_all_regs(stlink_t *sl, struct stlink_reg *regp) // rework needed
 {
    send_recv_uart(sl, DEBUG_READALLREGS, 0, 0, 0);

    int32_t size = parseResponse(sl, DEBUG_READALLREGS);

    for (ssize_t i = 0; i < 16; i++) {
        regp->r[i] = read_uint32(sl->q_buf, i * 4);
        }
    regp->xpsr       = read_uint32(sl->q_buf, 64);
    regp->main_sp    = read_uint32(sl->q_buf, 68);
    regp->process_sp = read_uint32(sl->q_buf, 72);
    regp->rw         = read_uint32(sl->q_buf, 76);
    regp->rw2        = read_uint32(sl->q_buf, 80);

    return(size < 0 ? -1 : 0 );
    
} // DEBUG_APIV1_READALLREGS

int32_t uart_write_reg(stlink_t *sl, uint32_t reg, int32_t idx){
    uint8_t data[8];
    write_uint32(data, reg);
    write_uint32(&data[4] , idx);

    send_recv_uart(sl, DEBUG_WRITEREG, 0, data, 8);
    int32_t size = parseResponse(sl, DEBUG_WRITEREG);

    if ((sl->q_len == TXDATASIZE_STATUS) & (sl->q_buf[0] == TXDATA_SUCCESS))
    {
        memset(sl->q_buf, 0, 4);
        return 0;
    }
    memset(sl->q_buf, 0, 4);
    return -1;
} // DEBUG_APIV1_WRITEREG 

int32_t uart_read_reg(stlink_t *sl, int32_t r_idx, struct stlink_reg *regp)
{
    uint32_t reg_data; 
    uint8_t txbuf[RESPONSE_SIZE];
    write_uint32(txbuf, r_idx);
    send_recv_uart(sl, DEBUG_READREG, 0, txbuf, 4);
    int32_t size = parseResponse(sl, DEBUG_READREG);
    reg_data = read_uint32(sl->q_buf, 0);

    switch (r_idx) {
    case 16:
        regp->xpsr = reg_data;
        break;
    case 17:
        regp->main_sp = reg_data;
        break;
    case 18:
        regp->process_sp = reg_data;
        break;
    case 19:
        regp->rw = reg_data; // XXX ?(primask, basemask etc.) in read_unsupported_reg
        break;
    case 20:
        regp->rw2 = reg_data; // XXX ?(primask, basemask etc.)
        break;
    default:
        regp->r[r_idx] = reg_data;
    }

    return(size < 0 ? -1 : 0);
}

int32_t uart_write_unsupported_reg(stlink_t *sl, uint32_t val, int32_t r_idx, struct stlink_reg *regp)
{
    int32_t ret;

    if (r_idx >= 0x1C && r_idx <= 0x1F) { // primask, basepri, faultmask, or control
        /* These are held in the same register */
        ret = uart_read_unsupported_reg(sl, 0x14, regp);

        if (ret == -1) { return(ret); }

        val = (uint8_t)(val >> 24);

        switch (r_idx) {
        case 0x1C: /* control */
            val = (((uint32_t)val) << 24) |
                  (((uint32_t)regp->faultmask) << 16) |
                  (((uint32_t)regp->basepri) << 8) |
                  ((uint32_t)regp->primask);
            break;
        case 0x1D: /* faultmask */
            val = (((uint32_t)regp->control) << 24) |
                  (((uint32_t)val) << 16) |
                  (((uint32_t)regp->basepri) << 8) |
                  ((uint32_t)regp->primask);
            break;
        case 0x1E: /* basepri */
            val = (((uint32_t)regp->control) << 24) |
                  (((uint32_t)regp->faultmask) << 16) |
                  (((uint32_t)val) << 8) |
                  ((uint32_t)regp->primask);
            break;
        case 0x1F: /* primask */
            val = (((uint32_t)regp->control) << 24) |
                  (((uint32_t)regp->faultmask) << 16) |
                  (((uint32_t)regp->basepri) << 8) |
                  ((uint32_t)val);
            break;
        }

        r_idx = 0x14;
    
    }

    write_uint32(sl->q_buf, val);

    ret = uart_write_mem32(sl, STLINK_REG_DCRDR, 4);

    if (ret == -1) { return(ret); }

    sl->q_buf[0] = (unsigned char)r_idx;
    sl->q_buf[1] = 0;
    sl->q_buf[2] = 0x01;
    sl->q_buf[3] = 0;

    return(uart_write_mem32(sl, STLINK_REG_DCRSR, 4));
}

int32_t uart_read_unsupported_reg(stlink_t *sl, int32_t r_idx, struct stlink_reg *regp)
{
    uint32_t r;
    int32_t ret;

    sl->q_buf[0] = (unsigned char)r_idx;

    for (int32_t i = 1; i < 4; i++) sl->q_buf[i] = 0;

    ret = uart_write_mem32(sl, STLINK_REG_DCRSR, 4);

    if (ret == -1) { return(ret); }

    ret = uart_read_mem32(sl, STLINK_REG_DCRDR, 4);

    if (ret == -1) { return(ret); }

    r = read_uint32(sl->q_buf, 0);
    DLOG("r_idx (%2d) = 0x%08x\n", r_idx, r);

    switch (r_idx) {
    case 0x14:
        regp->primask = (uint8_t)(r & 0xFF);
        regp->basepri = (uint8_t)((r >> 8) & 0xFF);
        regp->faultmask = (uint8_t)((r >> 16) & 0xFF);
        regp->control = (uint8_t)((r >> 24) & 0xFF);
        break;
    case 0x21:
        regp->fpscr = r;
        break;
    default:
        regp->s[r_idx - 0x40] = r;
        break;
    }

    return(0);
}
int32_t uart_read_all_unsupported_regs(stlink_t *sl, struct stlink_reg *regp) {
    int32_t ret;

    ret = uart_read_unsupported_reg(sl, 0x14, regp);

    if (ret == -1) { return(ret); }

    ret = uart_read_unsupported_reg(sl, 0x21, regp);

    if (ret == -1) { return(ret); }

    for (int32_t i = 0; i < 32; i++) {
        ret = uart_read_unsupported_reg(sl, 0x40 + i, regp);

        if (ret == -1) { return(ret); }
    }

    return(0);
}
int32_t uart_run(stlink_t *sl, enum run_type type)
{
    int32_t res;

     
    send_recv_uart(sl, DEBUG_RUNCORE, 0, 0, 0);
    res = parseResponse(sl, DEBUG_RUNCORE);
    if ((sl->q_len == TXDATASIZE_STATUS) & (sl->q_buf[0] == TXDATA_SUCCESS))
    {
        memset(sl->q_buf, 0, 4);
        return 0;
    }
    memset(sl->q_buf, 0, 4);
    return (-1);
    
}

int32_t uart_force_debug(stlink_t *sl) {

    int32_t res;
    send_recv_uart(sl, DEBUG_FORCEDEBUG, 0, 0, 0);
    res = parseResponse(sl, DEBUG_FORCEDEBUG);
    if (res < 0){return -1;}

    if ((sl->q_len == TXDATASIZE_STATUS) & (sl->q_buf[0] == TXDATA_SUCCESS))
    {
        DLOG("Monitor Mode Debug Enabled!\n");
        memset(sl->q_buf, 0, 4); 
        return 0;
    }
    else if ((sl->q_len == TXDATASIZE_STATUS) & (sl->q_buf[0] == TXDATA_FAILURE)) // Not realized
    {
        ELOG("Failure! Monitor Mode Debug is not Enabled!"); 
    }
    memset(sl->q_buf, 0, 4);
    
    return(-1);    
}

// int32_t uart_halt(stlink_t *sl) {

//     int32_t res;
//     send_recv_uart(sl, DEBUG_PAUSE, 0, 0, 0);
//     res = parseResponse(sl, DEBUG_PAUSE);
//     if (res < 0){return -1;}

//     if ((sl->q_len == TXDATASIZE_STATUS) & (sl->q_buf[0] == TXDATA_SUCCESS))
//     {
//         DLOG("Paused!\n"); 
//         return 0;
//     }
//     else if ((sl->q_len == TXDATASIZE_STATUS) & (sl->q_buf[0] == TXDATA_FAILURE))
//     {
//         ELOG("Failure!\n"); 
//     }
    
//     return(-1);    
// }

void uart_close(stlink_t* sl) {
    if (!sl) { return; }

    // destroyModbus(sl->modbus);
}

int32_t uart_exit_debug_mode(stlink_t *sl) {

    int32_t res;
    uint32_t demcr_reg;

    uart_read_debug32(sl, STLINK_REG_CM3_DEMCR, &demcr_reg);
    
    demcr_reg &= ~STLINK_REG_CM3_DEMCR_DBGMON_ENA;
    res = uart_write_debug32(sl, STLINK_REG_CM3_DEMCR, demcr_reg);
    res |= uart_write_debug32(sl, STLINK_REG_CFSR, ~0x1F); // 5 first bits are empty
    DLOG("Monitor Mode Debug Disabled!\n");    
    return(res);    
}

int32_t uart_core_id(stlink_t * sl) {
    uint8_t data[RESPONSE_SIZE];
    // NEED TO READ CORE ID somehow
    // And place it in debug
    send_recv_uart(sl, DEBUG_READCOREID, 0, 0, 0);
    int32_t size = parseResponse(sl, DEBUG_READCOREID);
    if (size < 0){return -1;} 
    memcpy(data, sl->q_buf, 12);

    // Check for alingment later (could be wrong)
    
    memcpy(&sl->uid[0], data, 4);
    memcpy(&sl->uid[1], &data[4], 4);
    memcpy(&sl->uid[2], &data[8], 4);
    sl->core_id = sl->uid[2];
    return(0);
}

int32_t uart_status(stlink_t *sl) {
    int32_t result;
    uint32_t status = 0; 

    result = uart_read_debug32(sl, STLINK_REG_DFSR, &status);
    DLOG("core status: %08X\n", status);


   
    if ((status >> DEBUGMON_HALTED)& 0x1) {
        sl->core_stat = TARGET_HALTED;
    } else if ((status >> DEBUGMON_BKPT)& 0x1) {
        sl->core_stat = TARGET_HALTED;
    } else if ((status >> DEBUGMON_DWTTRAP) & 0x1){
        sl->core_stat = TARGET_HALTED;
    } else{
        sl->core_stat = TARGET_RUNNING;
    }
 
    return(result);
}

int32_t uart_print_log(stlink_t *sl)
{
    int32_t res;
    send_recv_uart(sl, DEBUG_PRINT_LOG, 0, 0, 0);
    res = parseResponse(sl, DEBUG_PRINT_LOG);
    if (res < 0){return -1;}
    if (sl->q_len > BUF_MAX_SIZE) {return -1;}
    

    
    
    DLOG(sl->q_buf);
    
    return(0); 
}
int32_t uart_changeSettings(stlink_t *sl);
int32_t uart_reset(stlink_t *sl)
{
    int32_t res;
    send_recv_uart(sl, DEBUG_RESET, 0, 0, 0);
    res = parseResponse(sl, DEBUG_RESET);
    if (res < 0){return -1;}

    if ((sl->q_len == TXDATASIZE_STATUS) & (sl->q_buf[0] == TXDATA_SUCCESS))
    {
        DLOG("Reset sent!\n"); 
        memset(sl->q_buf, 0, 4);

        return 0;
    }
    else if ((sl->q_len == TXDATASIZE_STATUS) & (sl->q_buf[0] == TXDATA_FAILURE))
    {
        ELOG("Failure. Reset not sent.");
    }
    memset(sl->q_buf, 0, 4);
    return(-1); 
}
int32_t uart_changeSettings(stlink_t *sl)
{
    uart_settings_t *cfg = &sl->dbgSettings; 
    
    uint8_t txData[BUF_MAX_SIZE];
    uint16_t txSize = 0;
    const uint32_t baudRates[5] = {9600, 19200, 38400, 115200, 921600};

    *(uint32_t*)txData = baudRates[cfg->work_baudRate];
    txSize += 4;
    txData[txSize++] = cfg->work_parity;
    txData[txSize++] = cfg->work_stopBits; 

    send_recv_uart(sl, DEBUG_CHANGE_SETTINGS, 0,  txData, txSize);
    int32_t size = parseResponse(sl, DEBUG_CHANGE_SETTINGS);

    if ((sl->q_len == TXDATASIZE_STATUS) & (sl->q_buf[0] == TXDATA_SUCCESS))
    {
        memset(sl->q_buf, 0, 4);
        return 0;
    }
    
    return -1;
}

static uart_backend_t backend = {
    uart_close,// uart_close DOES NOTHING
    uart_exit_debug_mode, // 
    NULL,// uart_enter_swd_mode,
    NULL,// NULL, // don't enter_jtag_mode here...
    NULL,// uart_exit_dfu_mode,
    uart_core_id,// 
    uart_reset,// uart_reset,
    NULL,// uart_jtag_reset,
    uart_run,
    uart_status,// uart_status,
    NULL,// uart_version,
    uart_read_debug32,
    uart_read_mem32,
    uart_write_debug32,
    uart_write_mem32,
    uart_write_mem8,
    uart_read_all_regs,
    uart_read_reg,
    uart_read_all_unsupported_regs,
    uart_read_unsupported_reg,
    uart_write_unsupported_reg, // NOT IMPLEMENTED
    uart_write_reg,
    uart_step,
    NULL,// uart_current_mode,
    uart_force_debug,// 
    NULL,// uart_target_voltage,
    NULL,// uart_set_swdclk,
    NULL,// uart_enable_trace,
    NULL,// uart_disable_trace,
    NULL,// uart_read_trace
    uart_changeSettings
};
#ifdef SIMULATOR
extern Modbus_sim * modbus_gdb;
extern Modbus_sim * modbus_dev_sim;
#endif

uint8_t open_uart(stlink_t* sl) {
    uart_settings_t *s = &sl->dbgSettings;
    uint8_t ret = connectUart(sl->modbus, s->comName, s->init_baudRate, s->init_parity, s->init_stopBits);
    
    if (!ret) 
    {
        ILOG("Reading Core ID\n");
        uart_core_id(sl);
    };
    return ret;
}

uint8_t reopen_uart(stlink_t* sl)
{
    uart_settings_t *s = &sl->dbgSettings;

    uint8_t ret = connectUart(sl->modbus, s->comName, s->work_baudRate, s->work_parity, s->work_stopBits);
    if (!ret) 
    {
        ILOG("Reading Core ID\n");
        uart_core_id(sl);
    };
    return ret;
}
uint8_t getPortString(stlink_t * sl, char* cPorts)
{
    return getComPortList(sl->modbus, cPorts);
}

uint8_t newModbus(stlink_t * sl)
{
    int32_t modbus = createModbus(); 
    sl->modbus = modbus;
    sl->backend = &backend;
}

