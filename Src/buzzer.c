#include "global.h"

/*void buzzer(int sound, int length){

	TIM_OC_InitTypeDef ConfigOC;
	ConfigOC.OCMode = TIM_OCMODE_PWM1;
	ConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	ConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	hz = 1000000 / sound;
	TIM3 -> ARR = hz;

    ConfigOC.Pulse = hz / 2;
    HAL_TIM_PWM_ConfigChannel(&htim3, &ConfigOC, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	HAL_Delay(length);

}
*/

void buzzer_init(void){
}
