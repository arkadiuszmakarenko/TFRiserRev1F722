#include <rtc_msm6242.h>

RTC_HandleTypeDef *_hrtc;
static RTC_MSM6242_TypeDef rtc = { 0 };
static uint8_t ignoreWeekdayWrite = 0;
static uint32_t busyflagTimeout = 0;


void RTC_M6242_Init() {

	rtc.ctl_f =0x4;

}


	uint8_t RTC_Read(uint8_t address, RTC_HandleTypeDef *hrtc) {
		uint8_t data = 0;

		RTC_TimeTypeDef time = { 0 };
		RTC_DateTypeDef date = { 0 };
		HAL_RTC_GetTime(hrtc, &time, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(hrtc, &date, RTC_FORMAT_BIN);

		switch (address) {
		case 0x0:
			data = time.Seconds % 10;
			break;
		case 0x1:
			data = (time.Seconds / 10) % 10;
			break;

		case 0x2:
			data = time.Minutes % 10;
			break;

		case 0x3:
			data = (time.Minutes / 10) % 10;
			break;

		case 0x4:
			data = time.Hours % 10;
			break;

		case 0x5:
			data = (time.Hours / 10) % 10;
			break;

		case 0x6:
			data = date.Date % 10;
			break;

		case 0x7:
			data = (date.Date / 10) % 10;
			break;

		case 0x8:
			data = date.Month % 10;
			break;

		case 0x9:
			data = (date.Month / 10) % 10;
			break;

		case 0xA:
			data = date.Year % 10;
			break;

		case 0xB:
			data = (date.Year / 10) % 10;
			break;

		case 0xC:
			data = date.WeekDay;
			break;

			//Control registers
		case 0xD:
			data = rtc.ctl_d;
			break;
		case 0xE:
			data = rtc.ctl_e;
			break;
		case 0xF:
			data = rtc.ctl_f;
			break;
		}

		return data;
	}

	void RTC_Write(uint8_t address, uint8_t value, RTC_HandleTypeDef *hrtc) {
		uint8_t DoNotSetClock = 0;
		RTC_TimeTypeDef time = { 0 };
		RTC_DateTypeDef date = { 0 };
		HAL_RTC_GetTime(hrtc, &time, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(hrtc, &date, RTC_FORMAT_BIN);


		switch (address) {
		case 0x0:
			time.Seconds = (((time.Seconds / 10) % 10) * 10) + value;
			break;
		case 0x1:
			time.Seconds = ((time.Seconds) % 10 + (value * 10));
			break;

		case 0x2:
			time.Minutes = (((time.Minutes / 10) % 10) * 10) + value;
			break;

		case 0x3:
			time.Minutes = ((time.Minutes) % 10 + (value * 10));
			break;

		case 0x4:
			time.Hours = (((time.Hours / 10) % 10) * 10) + value;
			break;

		case 0x5:
			time.Hours = ((time.Hours) % 10 + ((value & 0x3) * 10));
			break;

		case 0x6:
			date.Date = (((date.Date / 10) % 10) * 10) + value;
			break;

		case 0x7:
			date.Date = ((date.Date) % 10 + (value * 10));
			break;

		case 0x8:
			date.Month = (((date.Month / 10) % 10) * 10) + value;
			break;

		case 0x9:
			date.Month = ((date.Month) % 10 + (value * 10));
			break;

		case 0xA: //10
			date.Year = (((date.Year / 10) % 10) * 10) + value;
			break;

		case 0xB: //11
			date.Year = ((date.Year) % 10 + (value * 10));
			break;

		case 0xC: //12
			if (ignoreWeekdayWrite != 1)
				date.WeekDay = value;
			ignoreWeekdayWrite = 0;
			break;

			//Control registers
		case 0xD: //13
			;
			uint8_t bit = (value >> 0) & 1U; // check for hold flag
			if (bit ==1)
			{
				busyflagTimeout = 1;
				//rtc.ctl_d = 0x3;
				rtc.ctl_d = 0x1;
			}


			if (value == 0) rtc.ctl_d = 0;

			if (((value >> 3) & 1U) == 1) {
				ignoreWeekdayWrite = 1;
			}
			DoNotSetClock = 1;
			break;
		case 0xE: //14
			rtc.ctl_e = value;
			DoNotSetClock = 1;
			break;
		case 0xF: //15
			rtc.ctl_f = value;
			DoNotSetClock = 1;
			break;
		}
		if (DoNotSetClock != 1) {
			HAL_RTC_SetTime(hrtc, &time, RTC_FORMAT_BIN);
			HAL_RTC_SetDate(hrtc, &date, RTC_FORMAT_BIN);
		}
	}

