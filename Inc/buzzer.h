#ifndef INC_BUZZER_H_
#define INC_DRIVE_H_


#define DO 1046
#define LE 1174
#define MI 1318
#define FA 1396
#define SO 1567
#define LA 1760
#define SI 1975
#define DOO 2093

#ifdef MAIN_C_              //main.cからこのファイルが呼ばれている場合

	  volatile int hz;                  //
#else                       //main.c以外からこのファイルが呼ばれている場合
  /*グローバル変数の宣言*/
	  extern volatile int hz;
#endif



void buzzer(int, int);

#endif /* INC_BUZZER_H_ */
