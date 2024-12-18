#include "lidar_X2_driver.h"
#include <stm32g431xx.h>
#include "math.h"

uint8_t rxBuffer[BUFFER_SIZE];
uint8_t lidarBuffer[BUFFER_SIZE];
uint8_t rxByte;
static int rxIndex = 0;
uint8_t LidarUartFlag = 0;
extern HAL_StatusTypeDef status;

uint8_t currentSpeed = 50;

lidar_point_t point = {0};
lidar_trame_t lidar_trame = {0};
Lidar_RxState_t state = WAIT_HEADER_1;

static volatile uint32_t callback_count = 0;

static uint8_t prev_byte = 0;
static uint8_t potential_header = 0;


void lidar_init(void) {

	while (!LidarUartFlag)
	{
		status = HAL_UART_Receive_IT(&huart3, &rxByte, 1);
	}

}

void lidar_scan_loop(void)
{

	if(LidarUartFlag) {
		lidar_trame = lidar_extractDataFromTrame(lidarBuffer);
		point = lidar_DataProcessing(lidar_trame);
		if (point.distance < LIDAR_DETECTION_RANGE)
		{
			printf("Distance: %.2f mm, Angle: %.2f°\r\n",
							point.distance,
							point.angle);
					LidarUartFlag = 0;

		}

	}

}

lidar_point_t lidar_DataProcessing(lidar_trame_t lidar_trame)
{
	lidar_point_t pointTemp = {0};

	pointTemp.distance = lidar_calculatingDistance(lidar_trame.Si_distance, lidar_trame.LSN);
	pointTemp.angle = lidar_calculatingAngle(lidar_trame.FSA, lidar_trame.LSA, lidar_trame.LSN, point.distance);
	//point.isCorrect = lidar_checksum(lidar_data);
	pointTemp.isCorrect = 1;

	return pointTemp;
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
    float sum = 0;
    for(int i = 0; i < sample; i++)
    {
        // Construire la valeur 16-bit en little-endian
        uint16_t Si = (uint16_t)data[i*2+1] << 8 | data[i*2];  // Exemple: 0x6FE5 depuis E5 6F
        sum += (float)Si / 4.0f;  // Division par 4 selon la formule Distance = Si/4
    }
    return sum / sample;
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
                state = WAIT_HEADER_2;
                rxIndex = 0;
                rxBuffer[rxIndex++] = rxByte;  // Store header1
            }
            break;

        case WAIT_HEADER_2:
            if(rxByte == HEADER_BYTE2) {
                rxBuffer[rxIndex++] = rxByte;  // Store header2
                state = RECEIVE_DATA;
            } else {
                state = WAIT_HEADER_1;
                rxIndex = 0;
            }
            break;

        case RECEIVE_DATA:
            if(rxByte == HEADER_BYTE1) {
                prev_byte = rxByte;
                potential_header = 1;
            }
            // Check pour le second byte du header
            else if(potential_header && rxByte == HEADER_BYTE2 && prev_byte == HEADER_BYTE1) {
                // On a trouvé une nouvelle séquence de header complète
                if(rxIndex > 2) {  // Sauvegarder seulement si on a des données
                    LidarUartFlag = 1;
                    memcpy(lidarBuffer, rxBuffer, rxIndex - 1); // -1 car on ne veut pas le HEADER_BYTE1
                }
                // Démarrer un nouveau buffer avec le header
                state = RECEIVE_DATA;
                rxIndex = 0;
                rxBuffer[rxIndex++] = HEADER_BYTE1;
                rxBuffer[rxIndex++] = HEADER_BYTE2;
                potential_header = 0;
            }
            else {
                // Si c'était un faux HEADER_BYTE1
                if(potential_header && rxByte != HEADER_BYTE2) {
                    potential_header = 0;
                }
                rxBuffer[rxIndex++] = rxByte;

                if(rxIndex >= BUFFER_SIZE) {
                    state = WAIT_HEADER_1;
                    rxIndex = 0;
                }
            }
            break;
        }

        prev_byte = rxByte;
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



lidar_trame_t lidar_extractDataFromTrame(uint8_t* buffer) {
    lidar_trame_t lidar_trame = {0};

    if (buffer == NULL) return lidar_trame;

    // Extract and validate packet header
    lidar_trame.PH = (uint16_t)buffer[1] << 8 | buffer[0];
    if (lidar_trame.PH != 0x55AA) {
        return lidar_trame;
    }

    lidar_trame.CT = buffer[2];
    lidar_trame.isStartOfScan = (lidar_trame.CT & 0x01) == 1;

    lidar_trame.LSN = buffer[3];
    lidar_trame.FSA = (uint16_t)buffer[5] << 8 | buffer[4];
    lidar_trame.LSA = (uint16_t)buffer[7] << 8 | buffer[6];
    lidar_trame.CS = (uint16_t)buffer[9] << 8 | buffer[8];

    if (lidar_trame.LSN > 0 && lidar_trame.LSN <= MAX_LSN) {
        for (uint8_t i = 0; i < lidar_trame.LSN; i++) {
            size_t offset = 10 + (i * 2);
            uint16_t raw_data = (uint16_t)buffer[offset+1] << 8 | buffer[offset];
            lidar_trame.Si_distance[i] = raw_data >> 1;
            lidar_trame.Si_interference[i] = raw_data & 0x01;
        }
    }

    return lidar_trame;
}

