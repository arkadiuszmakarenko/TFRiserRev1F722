#include "stm32f7xx_hal.h"

typedef enum
{
  RTC_READ = 0U,
  RTC_WRITE,
  RTC_BUSY
} RTC_MSM6242_StateTypeDef;


typedef struct {

	RTC_MSM6242_StateTypeDef rtc_state;
	int8_t ctl_d;
	int8_t ctl_e;
	int8_t ctl_f;
	//
	uint32_t busy_tick;

} RTC_MSM6242_TypeDef;


uint8_t RTC_Read(uint8_t address,RTC_HandleTypeDef *hrtc);
void RTC_Write(uint8_t address,uint8_t value,RTC_HandleTypeDef *hrtc);
//extern void RTC_Process();
extern void RTC_Init(RTC_HandleTypeDef *hrtc);
void RTC_M6242_Init();

