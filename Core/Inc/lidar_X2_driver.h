#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include "usart.h"
#include "tim.h"

#define MATH_PI 3.141592

#define BUFFER_SIZE (12 + MAX_LSN * 2)

#define RX_HEADER_SIZE 7
#define INFO_BUFFER_SIZE 20

#define LIDAR_START_FLAG1    0xA5
#define LIDAR_START_FLAG2    0x5A
#define LIDAR_POINT_PER_PACK 40
#define MAX_LSN 256

#define HEADER_BYTE1  0xAA
#define HEADER_BYTE2  0x55

#define TIMER_TIM_BASE 4250-1
// TIM 16 - Channel 1
#define LIDAR_MOTOR_STOP 50
#define LIDAR_MOTOR_NORMAL_SPEED 35

#define LIDAR_DETECTION_RANGE 100
typedef enum {
    WAIT_HEADER_1,
    WAIT_HEADER_2,
    RECEIVE_DATA
} Lidar_RxState_t;

typedef struct {
    float angle;
    float distance;
    int isCorrect; // 0 ou 1 suivant le checksum
} lidar_point_t;

typedef struct {
    uint16_t PH;
    uint8_t CT;
    uint8_t LSN;
    uint16_t FSA;
    uint16_t LSA;
    uint16_t CS;
    uint16_t Si_distance[MAX_LSN];
    uint8_t Si_interference[MAX_LSN];
    int isStartOfScan;
} lidar_trame_t;



void lidar_init(void);
void lidar_scan_loop(void);

int lidar_checksum(const uint8_t* data, size_t len);
lidar_point_t lidar_DataProcessing(lidar_trame_t lidar_trame);
float lidar_calculatingDistance(uint8_t* data, int sample);
float lidar_calculatingAngle(uint8_t FSA_data, uint8_t LSA_data, int samples, float distance);
lidar_trame_t lidar_extractDataFromTrame(uint8_t* buffer);
