
//-- cpp2c

#include "modbus_wrapper.h"
#include <stdlib.h>
#include "ModbusRTU_XSerial.h"
// }

#include "xserial.hpp"
#include <vector>
#include <memory>

std::vector<std::unique_ptr<ModbusRTU_XSerial::ModbusRTU>> ModbusInstances;
static uint8_t isErrorFound = 1;

void find_baudRate(int32_t descriptor)
{
   auto& modbus = ModbusInstances[descriptor];
   static uint32_t baudRates[5] =
	{
		9600,
		19200,
		38400,
		115200,
		921600
	};
   isErrorFound = 1;
   uint32_t real_baudRate;
   static uint32_t vUuids[3];
   uint32_t retryCount = 0;
   static uint32_t baudRateId = 0;
   
   modbus->serialResetSettings(baudRateId, 0, 0);
   modbus->sendMsg(0, 0, 0, 0x2722, 0, nullptr, 0, 30); // DEBUG_READCOREID
   
   while(isErrorFound)
            {
               if (!modbus->isBusy)
               {
                  
                  if (modbus->isError)
                  {
                     if (retryCount < 5)
                     {
                           retryCount++;
                           modbus->serialResetSettings(baudRateId, 0, 0);
                           modbus->sendMsg(0, 0, 0,  0x2722,  0, nullptr, 0, 30);
                     }
                     else if (baudRateId < 4)
                     {
                           retryCount = 0;
                           baudRateId++;
                           modbus->serialResetSettings(baudRateId, 0, 0);
                           modbus->sendMsg(0, 0, 0,  0x2722,  0, nullptr, 0, 30);
                     }
                     else
                     {
                           retryCount = 0;
                           baudRateId = 0;
                           break;
                     }

                  }
                  else
                  {    
                     const uint8_t* const rxBuf = modbus->rxBuf;

                     uint32_t rIt = 1;
                     rIt += 2; // CMD
                     rIt += 4; // SIZE
                     uint32_t uuid0 = *(uint32_t*)&rxBuf[rIt];
                     rIt += 4;
                     uint32_t uuid1 = *(uint32_t*)&rxBuf[rIt];
                     rIt += 4;
                     uint32_t uuid2 = *(uint32_t*)&rxBuf[rIt];
                     rIt += 4;

                     vUuids[0] = uuid0;
                     vUuids[1] = uuid1;
                     vUuids[2] = uuid2;
                     isErrorFound = 0;
                  }
               }
            }
   return ;
}

void userConnection(int32_t descriptor, uint8_t baudRateId, uint8_t parity, uint8_t stopBits)
{
   auto& modbus = ModbusInstances[descriptor];
   static uint32_t vUuids[3];
   uint8_t isErrorFound = 1;
   uint32_t retryCount = 0;
   
   modbus->serialResetSettings(baudRateId, parity, stopBits);
   modbus->sendMsg(0, 0, 0, 0x2722, 0, nullptr, 0, 30); // DEBUG_READCOREID
   
   while(isErrorFound)
            {
               if (!modbus->isBusy)
               {
                  
                  if (modbus->isError)
                  {
                     // if (retryCount <= MDB_MAX_TRIES)
                     // {
                     //       retryCount++;
                     //       modbus->serialResetSettings(baudRateId, parity, stopBits);
                     //       modbus->sendMsg(0, 0, 0,  0x2722,  0, nullptr, 0, 30);
                     // }
                     // else
                     // {
                     retryCount = 0;
                     baudRateId = 0;
                     modbus->isBusy = 0;
                     break;
                     // }
                  }
                  else
                  {    
                     const uint8_t* const rxBuf = modbus->rxBuf;

                     uint32_t rIt = 1;
                     rIt += 2; // CMD
                     rIt += 4; // SIZE
                     uint32_t uuid0 = *(uint32_t*)&rxBuf[rIt];
                     rIt += 4;
                     uint32_t uuid1 = *(uint32_t*)&rxBuf[rIt];
                     rIt += 4;
                     uint32_t uuid2 = *(uint32_t*)&rxBuf[rIt];
                     rIt += 4;

                     vUuids[0] = uuid0;
                     vUuids[1] = uuid1;
                     vUuids[2] = uuid2;
                     isErrorFound = 0;
                  }
               }
            }
   return ;  
}

extern "C" int32_t createModbus() {


   ModbusInstances.push_back(std::make_unique<ModbusRTU_XSerial::ModbusRTU>(0));

   uint32_t descriptor = ModbusInstances.size() - 1;

   // auto& modbus = ModbusInstances[descriptor];
   // if (modbus->isError)
   // {
   //    return 0x13;
   // }
   // userConnection(descriptor, baudRateId, parity, stopBits);
   // if (isErrorFound)
   // {
   //    find_baudRate(descriptor);
   // }

   // if (isErrorFound)
   // {
   //    return 0x13;
   // }
   return descriptor;

}


extern "C" uint8_t connectUart(uint32_t descriptor, const char* portName, uint8_t baudRateId, uint8_t parity, uint8_t stopBits)
{
   auto& modbus = ModbusInstances[descriptor];
   // #ifdef _WIN32
   // thr.detach();
   // sched_param sch;
   // int policy;
   // sch.sched_priority = 24;
   // pthread_getschedparam(thr.native_handle(), &policy, &sch);
   // pthread_setschedparam(thr.native_handle(), SCHED_FIFO, &sch); // smth not working
   // #endif // _WIN32
   uint8_t portFound = 0;
   modbus->serialPort.getListSerialPorts(modbus->ports);
   
   for (uint8_t i = 0; i < modbus->ports.size(); i++)
   {
      if (modbus->ports[i] == portName)
      {
         portFound = 1;
         modbus->isError = false;
         modbus->serialSetSettings(portName, baudRateId, parity, stopBits);
         break;
      }
   }
   if (!portFound){
      for (auto& port : modbus->ports)
      {
         size_t pos = port.find("COM");
         if ( pos == 0 )
         {
            portName = port.c_str();
            modbus->isError = false;
            modbus->serialSetSettings(portName, baudRateId, parity, stopBits);
            if (!isError) {break;}
         }
      }
   }
   if (modbus->isError)
   {
      return modbus->isError;
   }

   userConnection(descriptor, baudRateId, parity, stopBits);
   return modbus->isError;
}

extern "C" uint8_t getComPortList(int32_t descriptor, char *cPorts)
{  
   // Returns encoded string, not ports array.
   // cPorts up too 255 chars.
   uint8_t cPortsIterator = 0;
   std::vector<std::string>portscpp;
   auto& modbus = ModbusInstances[descriptor];
   modbus->serialPort.getListSerialPorts(portscpp);
   
   for (uint8_t i = 0; i < portscpp.size(); i++)
   {
      const uint8_t portSize = portscpp[i].size();
      if (cPortsIterator + 1 + portSize >= 254) { break; }
      strcpy(&cPorts[cPortsIterator], portscpp[i].c_str());
      cPortsIterator += portSize;
      cPorts[cPortsIterator++] = ';';
   }
   cPorts[cPortsIterator++] = '\0';
   return cPortsIterator;
}

extern "C" uint8_t* getRxBuff(int32_t descriptor) {
   auto& modbus = ModbusInstances[descriptor];
   return modbus->rxBuf;
}

extern "C" void sendMsg(int32_t descriptor, uint32_t uid0, uint32_t uid1, uint32_t uid2, uint16_t cmd, uint32_t memAddr,
             const uint8_t *data, uint32_t size, uint32_t timeout) {
      auto& modbus = ModbusInstances[descriptor];
      modbus->sendMsg(uid0, uid1, uid2, cmd, memAddr, data, size, timeout);
    
}


extern "C" uint8_t isBusy(int32_t descriptor) {
   auto& modbus = ModbusInstances[descriptor];
   return modbus->isBusy;
}

extern "C" uint8_t isError(int32_t descriptor) {
   auto& modbus = ModbusInstances[descriptor];
   return modbus->isError;
}