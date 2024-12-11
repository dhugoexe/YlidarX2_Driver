#include "lidar_X2_driver.h"
#include <stm32g431xx.h>
#include "math.h"

uint8_t rxBuffer[BUFFER_SIZE];
uint8_t rxByte;
static int rxIndex = 0;
uint8_t LidarUartFlag = 0;
extern HAL_StatusTypeDef status;

uint8_t currentSpeed = 50;

lidar_trame_t lidar_trame = {0};
Lidar_RxState_t state = WAIT_HEADER_1;

static volatile uint32_t callback_count = 0;




void lidar_init(void) {

	while (!LidarUartFlag)
	{
		status = HAL_UART_Receive_IT(&huart3, &rxByte, 1);
	}

}

void lidar_scan_loop(void)
{
	static lidar_point_t point = {0};

	if(LidarUartFlag) {
		point = lidar_DataProcessing(rxBuffer, lidar_trame.LSN);
		LidarUartFlag = 0;  // Reset du flag
	}

}

lidar_point_t lidar_DataProcessing(uint8_t lidar_data, uint8_t sample)
{
	lidar_point_t point = {0};

	lidar_trame = lidar_extractDataFromTrame(lidar_data);
	point.distance = lidar_calculatingDistance(lidar_trame.Si, lidar_trame.LSN);
	point.angle = lidar_calculatingAngle(lidar_trame.FSA, lidar_trame.LSA, lidar_trame.LSN, point.distance);
	//point.isCorrect = lidar_checksum(lidar_data);
	point.isCorrect = 1;

	return point;
}

float lidar_calculatingAngle(uint8_t FSA_data, uint8_t LSA_data, int samples, float distance)
{

    float angleFSA = ((uint16_t)FSA_data >> 1) / 64.0f;
    float angleLSA = ((uint16_t)LSA_data >> 1) / 64.0f;

    float angle_diff = angleLSA - angleFSA;
    if(angle_diff < 0) {
        angle_diff += 360.0f;
    }

    float angle = angleFSA + (angle_diff / 2.0f);

    if(distance != 0) {
        float angCorrect = atan2f(21.8f * (155.3f - distance),
                               155.3f * distance) * 180.0f / 3.14159f;
        return angle + angCorrect;
    }

    return angle;
}
float lidar_calculatingDistance(uint8_t* data, int sample)
{
    uint16_t dist_value = (uint16_t)data[sample*2+1] << 8 | data[sample*2];
    return dist_value / 4.0f;
}
int lidar_checksum(const uint8_t* data, size_t len)
{
    uint16_t checksum = 0;
    for(size_t i = 0; i < len-2; i++) {  // -2 car CS ne participe pas au XOR
        checksum ^= data[i];
    }
    return checksum == (data[len-2] | (data[len-1] << 8));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	callback_count++;
	if(huart->Instance == USART3) {
		switch(state) {
		case WAIT_HEADER_1:
			if(rxByte == HEADER_BYTE1) {
				printf("Byte reçu: 0x%02X\r\n", rxByte); // Debug
				state = WAIT_HEADER_2;
				rxIndex = 0;
			}
			break;

		case WAIT_HEADER_2:
			printf("Second byte: 0x%02X\r\n", rxByte); // Debug
			if(rxByte == HEADER_BYTE2) {
				state = RECEIVE_DATA;
			} else {
				state = WAIT_HEADER_1;
			}
			break;

		case RECEIVE_DATA:
			rxBuffer[rxIndex++] = rxByte;

			if(rxIndex >= BUFFER_SIZE) {
				state = WAIT_HEADER_1;
				lidar_trame = lidar_extractDataFromTrame(rxBuffer);


				lidar_point_t point = lidar_DataProcessing(rxBuffer, lidar_trame.LSN);
				printf("Distance: %.2f mm, Angle: %.2f°\r\n",
								point.distance,
								point.angle);
				LidarUartFlag = 1;
				memset(rxBuffer, 0, BUFFER_SIZE);
			}
			break;
		}

		HAL_UART_Receive_IT(&huart3, &rxByte, 1);
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

    if (buffer == NULL) return lidar_trame;

    // Reconstruction correcte des valeurs 16 bits en little endian
    lidar_trame.PH = (uint16_t)buffer[1] << 8 | buffer[0];  // Doit être 0x55AA
    lidar_trame.CT = buffer[2];  // Un seul byte
    lidar_trame.LSN = buffer[3]; // Un seul byte
    lidar_trame.FSA = (uint16_t)buffer[5] << 8 | buffer[4];
    lidar_trame.LSA = (uint16_t)buffer[7] << 8 | buffer[6];
    lidar_trame.CS = (uint16_t)buffer[9] << 8 | buffer[8];


    if (lidar_trame.LSN > 0) {
        lidar_trame.Si = malloc(lidar_trame.LSN * sizeof(uint16_t));
        if (lidar_trame.Si != NULL) {
            for (uint8_t i = 0; i < lidar_trame.LSN; i++) {
                // Les données commencent à l'offset 10
                size_t offset = 10 + (i * 2);
                lidar_trame.Si[i] = (uint16_t)buffer[offset+1] << 8 | buffer[offset];
            }
        }
    }

    return lidar_trame;
}

