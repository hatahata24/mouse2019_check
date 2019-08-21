#ifndef INC_GLOBAL_H_
#define INC_GLOBAL_H_


#include "main.h"

#include "buzzer.h"

typedef union {					//
	uint16_t FLAGS;
	struct ms_flags{			//
		uint16_t RSV0:1;		//yobi bit(B0)		(:1は1ビット分の意味，ビットフィールド)
		uint16_t SCND:1;		//niji soukou flag(B1)
		uint16_t RSV2:1;		//yobi bit(B2)
		uint16_t CTRL:1;		//control flag(B3)
		uint16_t DRV:1;			//drive flag(B4)
		uint16_t LOG:1;			//log flag(B5)
		uint16_t GYR:1;			//gyro flag(B6)
		uint16_t RSV7:1;		//スラロームフラグ(B7)
		uint16_t RSV8:1;		//旧走行モードフラグ(B8)
		uint16_t RSV9:1;		//走行開始時加速判断フラグ(B9)
		uint16_t RSV10:1;		//既知区間加速フラグ(B10)
		uint16_t RSV11:1;		//直線優先フラグ(B11)
		uint16_t RSV12:1;		//予備ビット(B12)
		uint16_t RSV13:1;		//予備ビット(B13)
		uint16_t RSV14:1;		//予備ビット(B14)
		uint16_t RSV15:1;		//予備ビット(B15)
	}FLAG;
} mouse_flags;

#ifdef MAIN_C_							//main.cからこのファイルが呼ばれている場合
	/*グローバル変数の定義*/
	volatile mouse_flags MF;			//mouse kyoyou kouzoutai
#else									//main.c以外からこのファイルが呼ばれている場合
	/*グローバル変数の宣言*/
	extern volatile mouse_flags MF;
#endif


#endif /* INC_GLOBAL_H_ */
