#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


int32_t createModbus( void );

uint8_t getComPortList(int32_t descriptor, char *cPorts);

void sendMsg(int32_t descriptor, uint32_t uid0, uint32_t uid1, uint32_t uid2, uint16_t cmd, uint32_t memAddr,
             const uint8_t *data, uint32_t size, uint32_t timeout);

uint8_t connectUart(uint32_t descriptor, const char* portName, uint8_t baudRateId, uint8_t parity, uint8_t stopBits);

uint8_t* getRxBuff(int32_t descriptor);

uint8_t isBusy(int32_t descriptor);
uint8_t isError(int32_t descriptor);

#ifdef __cplusplus
}
#endif


