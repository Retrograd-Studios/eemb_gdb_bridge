
// extern "C"{
#include <ModbusRTU_XSerial.h>
// }
#include <iostream>


namespace ModbusRTU_XSerial {



	void ModbusRTU::loop()
	{


		while (!isTerminated)
		{

			if (!isOpen())
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				continue;
			}

			//todo...


			if (waiting)
			{
				if (!isError)
				{
					parsing();
				}
				if (waiting)
				{
					waiting -= 1;
					if (!waiting)
					{
						tryCount++;
						if (tryCount >= MDB_MAX_TRIES)
						{
							isError = true;
							isBusy = false;
						}
						else 
						{
							waiting = this->timeout;
							if (tryCount != 0)
							{
								std::cerr << "RetryCountWaitTimout: " << (int32_t) tryCount << "\n";
							}
							serialPort.write((char*)this->txBuf, this->txSize);
						}
					}
				}
				}
				

			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		serialPort.close();

	}


	void ModbusRTU::crc16calc(uint16_t* crc, uint8_t* addr, uint32_t len)
	{
		*crc = 0xFFFF;

		while (len--)
			crc16calcByte(crc, *(addr++));
	}

	void ModbusRTU::crc16calcByte(uint16_t* crc, uint8_t val)
	{
		int i;

		*crc ^= (uint16_t)val;
		for (i = 0; i < 8; ++i) {
			if (*crc & 1)
				*crc = (*crc >> 1) ^ 0xA001;
			else
				*crc = (*crc >> 1);
		}
	}




	void ModbusRTU::sendMsg(uint32_t uid0, uint32_t uid1, uint32_t uid2, 
		uint16_t cmd, uint32_t memAddr, 
		const uint8_t* data, uint32_t size, uint32_t timeout)
	{
		tryCount = 0;
		isBusy = true;
		isError = false;
		uint32_t mSize = 0;

		uint8_t* const msg = txBuf;

		msg[0] = 0;
		mSize++;

		*(uint32_t*)&msg[mSize] = uid0;
		mSize += 4;
		*(uint32_t*)&msg[mSize] = uid1;
		mSize += 4;
		*(uint32_t*)&msg[mSize] = uid2;
		mSize += 4;

		*(uint16_t*)&msg[mSize] = cmd;
		mSize += 2;

		*(uint32_t*)&msg[mSize] = memAddr;
		mSize += 4;

		*(uint32_t*)&msg[mSize] = size;
		mSize += 4;

		if (data != nullptr && size)
		{
			memcpy(&msg[mSize], data, size);
			mSize += size;
		}

		crc16calc((uint16_t*)&msg[mSize], msg, mSize);
		mSize += 2;

		if (serialPort.write((char*)msg, mSize ))
		{
			waiting = timeout;
			this->timeout = timeout;
			memcpy(this->txBuf, msg, mSize);
			this->txSize = mSize;
		}
		else
		{
			isError = true;
			isBusy = false;
		}
		// ModbusRTU::tryCount += 1;
	}



	void ModbusRTU::parsing()
	{
		uint32_t len = serialPort.bytesToRead();
		uint8_t errFlag = 0;
		uint16_t rxSize = 0;
		while (len)
		{
			uint8_t buf[RX_BUF_SIZE];
			uint16_t prevRxSize = len;
			while (prevRxSize != rxSize)  {
				prevRxSize = rxSize;
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
				const uint16_t cRxSize = serialPort.read((char*)&buf[rxSize], RX_BUF_SIZE);
				rxSize += cRxSize;
			}

			if (!rxSize)
			{
				errFlag = 1;
				break;
			}

			if (rxSize < 3)
			{
				errFlag = 1;
				// isError = true;
				// isBusy = false;
				break;
			}

			uint16_t crc = 0;
			uint16_t crc0 = *(uint16_t*)&buf[rxSize - 2];
			crc16calc(&crc, buf, rxSize - 2);

			if (crc != crc0)
			{
				std::cout << "CRC error!\n";
				logsS << "CRC error!\n";
				errFlag = 1;
				// isError = true;
				// isBusy = false;
				break;
			}
			
			// if ( buf[0] == 0 && buf[1] == 0x27 && (buf[2] & 0x80) )
			// {
			// 	if (this->stepTries < MDB_MAX_STEP_TRIES){
				// if (stepTries != 0)
				// {
				// 	std::cerr << "Retry: " << (char) stepTries << "\n";
				// }
			 	
				// std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				// waiting = this->timeout;
				// this->stepTries++;
				// serialPort.write((char*)this->txBuf, this->txSize);
				// return;
				// }
				// else{
				// 	isError = true;
				// 	errFlag = 0;
				// }
			// }
			memcpy(rxBuf, buf, rxSize);
			RX_bufSize = rxSize;

			waiting = 0;


			isBusy = 0;
			isError = 0;
			tryCount = 0;
			//std::this_thread::sleep_for(std::chrono::milliseconds(1));
			len = serialPort.bytesToRead();// bytesAvailable();
		}
	if (errFlag)
	{
		tryCount++;
		if (tryCount != 0)
		{
			std::cerr << "RetryCountBadPacket: " << (int32_t) tryCount << "\n";
		}
		if (tryCount >= MDB_MAX_TRIES)
		{
			tryCount = 0;
			isError = true;
			isBusy = false;
		}
		else 
		{
			waiting = 0;
			waiting = this->timeout;
			serialPort.write((char*)this->txBuf, this->txSize);
		}
		
	}


	}
}

//#endif // FSR_CORE_USE_XSERIAL