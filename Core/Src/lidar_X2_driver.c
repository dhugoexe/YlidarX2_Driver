#include "lidar_X2_driver.h"
#include <stm32g431xx.h>

uint8_t rxBuffer[BUFFER_SIZE];
uint8_t LidarUartFlag = 0;
extern HAL_StatusTypeDef status;

uint8_t currentSpeed = 50;

lidar_trame_t lidar_trame = {0};


// message: A5 5A 05 00 00 40 81
int init_lidar(void)
{
	//lidar_setSpeed(LIDAR_MOTOR_STOP);
	status = HAL_UART_Receive_DMA(&huart3, rxBuffer, BUFFER_SIZE);
	if(LidarUartFlag){
		LidarUartFlag = 0;
		//lidar_setSpeed(LIDAR_MOTOR_NORMAL_SPEED);
		if(rxBuffer[0] == LIDAR_START_FLAG1 && rxBuffer[1] == LIDAR_START_FLAG2)
		{
			return 1;
		}
	}
	return 0;
}



lidar_point_t lidar_scan_loop(void)
{
	lidar_point_t point = {0};

	status = HAL_UART_Receive(&huart3, rxBuffer, 32, 1000);

	if(rxBuffer[0] == HEADER_BYTE1 && rxBuffer[1] == HEADER_BYTE2)
	{
		uint8_t sample = rxBuffer[3];
		uint8_t sampleData[sample*2];
		memcpy(sampleData, &rxBuffer[10], sample*2);
		point = lidar_DataProcessing(sampleData, sample);

		if (point.isCorrect)
		{
			printf("Distance: %.2f mm, Angle: %.2f°\r\n",
					point.distance,
					point.angle);
		}else {printf("Error incorrect point \r\n");}
	}

}

lidar_point_t lidar_DataProcessing(uint8_t lidar_data, uint8_t sample)
{
	lidar_point_t point = {0};

	lidar_trame = lidar_extractDataFromTrame(lidar_data);
	point.distance = lidar_calculatingDistance(lidar_trame.Si, lidar_trame.LSN);
	point.angle = lidar_calculatingAngle(lidar_trame.FSA, lidar_trame.LSA, lidar_trame.LSN);
	point.isCorrect = lidar_checksum(lidar_data);

	return point;
}

float lidar_calculatingDistance(uint8_t* data, int sample)
{
	float sum = 0.0f;
	for (int i = 0; i < sample; i++)
	{
		sum += (data[i * 2] + data[i * 2 + 1]) / 4.0f;
	}

	return sum / sample;
}

float lidar_calculatingAngle(uint8_t FSA_data, uint8_t LSA_data, int samples)
{
	float sum = 0.0f;

	float angleFSA = (FSA_data >> 1) / 64;
	float angleLSA = (LSA_data >> 1) / 64;

	for (int i = 0; i < samples; i++)
	{
		sum += ((angleLSA - angleFSA) / (samples -1)) * (i-1) + angleFSA;
	}
	return sum/samples;

}

int lidar_checksum(const uint8_t *data)
{
	uint16_t checksum = 0;

	for (size_t i = 0; i < sizeof(data); i++) {
		checksum ^= (uint16_t)data[i];
	}
	return checksum == data[8] + data[9];

}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		LidarUartFlag = 1;
	}
}

/*

int lidar_setSpeed(uint8_t speed)
{
	if (speed >= currentSpeed)
	{
		while(currentSpeed != speed)
		{
			htim16.Instance->CCR1=(4250-1)*currentSpeed/100;
			htim16.Instance->CCR2=(4250-1)*(100-currentSpeed)/100;
			currentSpeed++;
			HAL_Delay(100);

		}
	}
	else if (speed <= currentSpeed)
	{
		while(currentSpeed != speed)
		{
			htim16.Instance->CCR1=(4250-1)*currentSpeed/100;
			htim16.Instance->CCR2=(4250-1)*(100-currentSpeed)/100;
			currentSpeed--;
			HAL_Delay(100);
		}
	}
	return currentSpeed;
}
 */

lidar_trame_t lidar_extractDataFromTrame(uint8_t* buffer)  
{
	lidar_trame_t lidar_trame = {0};

	if (buffer == NULL) {
		return lidar_trame;
	}

	lidar_trame.PH = (uint16_t)buffer[1] << 8 | buffer[0];
	lidar_trame.CT = (uint16_t)buffer[3] << 8 | buffer[2];
	lidar_trame.LSN = (uint16_t)buffer[5] << 8 | buffer[4];
	lidar_trame.FSA = (uint16_t)buffer[7] << 8 | buffer[6];
	lidar_trame.LSA = (uint16_t)buffer[9] << 8 | buffer[8];
	lidar_trame.CS = (uint16_t)buffer[11] << 8 | buffer[10];

	if (lidar_trame.LSN > 0) {
		lidar_trame.Si = (uint16_t*)malloc(lidar_trame.LSN * sizeof(uint16_t));

		if (lidar_trame.Si != NULL) {
			for (uint16_t i = 0; i < lidar_trame.LSN; i++) {
				size_t offset = 12 + (i * 2);
				lidar_trame.Si[i] = (uint16_t)buffer[offset + 1] << 8 | buffer[offset];
			}
		}
	}

	return lidar_trame;
}

