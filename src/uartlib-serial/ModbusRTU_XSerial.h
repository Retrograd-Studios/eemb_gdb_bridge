

#pragma once

#include <string>
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
//#include <pthread.h>

#include <xserial.hpp>

#define FSR_CORE_USE_XSERIAL
#define MDB_MAX_TRIES	5
#define MDB_MAX_STEP_TRIES 1

#define TX_BUF_SIZE		(1 + 4 + 4 + 4 + 1 + 4 + 4 + 256 + 2)
#define RX_BUF_SIZE		(1 + 4 + 4 + 4 + 1 + 4 + 4 + 256 + 2)

namespace ModbusRTU_XSerial {


#define MBL_TIMEOUT	( 1000 )
#define MBL_REQ_TIME	( 350 )




	constexpr uint32_t baudRates[5] =
	{
		9600,
		19200,
		38400,
		115200,
		921600
	};

	constexpr xserial::ComPort::eParity parities[3] = {
			xserial::ComPort::eParity::COM_PORT_NOPARITY,
			xserial::ComPort::eParity::COM_PORT_EVENPARITY,
			xserial::ComPort::eParity::COM_PORT_ODDPARITY
	};

	constexpr xserial::ComPort::eStopBit stopBits[2] = {
		xserial::ComPort::eStopBit::COM_PORT_ONESTOPBIT,
		xserial::ComPort::eStopBit::COM_PORT_TWOSTOPBITS
	};

	struct AU8
	{
		uint8_t arr[8];
	};



	class ModbusRTU 
	{
	private:
		
		std::thread thr;
		std::atomic<bool> isTerminated;
		uint32_t waiting;
		std::stringstream logsS;
		void loop();
		void parsing();
		void sendFromQueue();
		void crc16calcByte(uint16_t* crc, uint8_t val);
		void crc16calc(uint16_t* crc, uint8_t* addr, uint32_t len);

		uint32_t RX_bufSize = 0;

	public:
		std::vector<std::string> ports;
		xserial::ComPort serialPort;
		uint8_t txBuf[TX_BUF_SIZE];
		uint32_t txSize;
		uint8_t rxBuf[RX_BUF_SIZE];
		uint8_t tryCount;
		uint32_t timeout;
		uint8_t stepTries;
		std::atomic<bool> isBusy;
		std::atomic<bool> isError;

		explicit ModbusRTU() = delete;

		ModbusRTU( uint8_t foo ):
			tryCount(0),
			stepTries(0),
			isTerminated(false),
			thr([&]() { loop(); }),
			waiting(0),
			isError(false), logsS("")
		{

			isError = foo; //asm("nop");
		}

		void serialResetSettings(uint8_t baudRateNum = 0, uint8_t parityNum = 0, uint8_t stopBitNum = 0)
		{
			const std::string& port = serialPort.getCurrentComPort();
			serialSetSettings(port, baudRateNum, parityNum, stopBitNum);
		}


		void serialSetSettings(const std::string& port, uint8_t baudRateNum = 0, uint8_t parityNum = 0, uint8_t stopBitNum = 0)
		{
			if (!serialPort.open(port, baudRates[baudRateNum], parities[parityNum], 8, stopBits[stopBitNum]))
			{
				isError = true;
			}
		}

		void sendMsg(uint32_t uid0, uint32_t uid1, uint32_t uid2, uint16_t cmd, uint32_t memAddr,
			const uint8_t* data, uint32_t size, uint32_t timeout);

		bool openPort()
		{
			return  serialPort.getStateComPort();
		}

		std::string getLogs() {
			return logsS.str();
		};

		bool isOpen() {
			return serialPort.getStateComPort();
		}


		~ModbusRTU()
		{
			isTerminated = true;
			thr.join();
		}
	};

}