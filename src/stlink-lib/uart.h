
#pragma once

#include <stdint.h>
#include <commands.h>

#define RESPONSE_SIZE 256
#define BUF_MAX_SIZE 256
#define PKG_WAIT_TIME_MS 20
#define TXDATASIZE_STATUS (uint32_t) (1)
#define TXDATA_SUCCESS  (uint8_t) (0xCF)
#define TXDATA_FAILURE  (uint8_t) (0xA5)

int32_t parseResponse(stlink_t *sl, debug_commands_e cmd);

uint32_t send_recv_uart(stlink_t *sl, debug_commands_e cmd, uint32_t memaddr, uint8_t* txbuf, uint16_t txsize);

int32_t uart_run(stlink_t *sl, enum run_type type);
int32_t uart_read_debug32(stlink_t *sl, uint32_t addr, uint32_t *data); // DEBUG_READMEM_32BIT 
int32_t uart_write_debug32(stlink_t *sl, uint32_t addr, uint32_t data); // DEBUG_WRITEMEM_32BIT
int32_t uart_write_mem32(stlink_t *sl, uint32_t addr, uint16_t len); // DEBUG_WRITEMEM_32BIT
int32_t uart_write_mem8(stlink_t *sl, uint32_t addr, uint16_t len); // DEBUG_WRITEMEM_8BIT
int32_t uart_read_mem32(stlink_t *sl, uint32_t addr, uint16_t len); // DEBUG_READMEM_32BIT
int32_t uart_step(stlink_t* sl);
int32_t uart_read_all_regs(stlink_t *sl, struct stlink_reg *regp);
int32_t uart_write_reg(stlink_t *sl, uint32_t reg, int32_t idx);
int32_t uart_read_reg(stlink_t *sl, int32_t r_idx, struct stlink_reg *regp);
int32_t uart_write_unsupported_reg(stlink_t *sl, uint32_t val, int32_t r_idx, struct stlink_reg *regp);
int32_t uart_read_unsupported_reg(stlink_t *sl, int32_t r_idx, struct stlink_reg *regp);
int32_t uart_read_all_unsupported_regs(stlink_t *sl, struct stlink_reg *regp);
int32_t uart_print_log(stlink_t *sl);

extern uint8_t open_uart(stlink_t* sl);
extern uint8_t reopen_uart(stlink_t* sl);
extern uint8_t getPortString(stlink_t * sl, char* cPorts);
extern uint8_t newModbus(stlink_t * sl);
// Modbus* createModbus(char* portName);
// void destroyModbus(Modbus* modbus);
// void sendMsg(Modbus* modbus, uint32_t uid0, uint32_t uid1, uint32_t uid2, uint8_t cmd, uint32_t memAddr,
//              const uint8_t *data, uint32_t size, uint32_t timeout);
// uint8_t* getRxBuff(Modbus* modbus);
