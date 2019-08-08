#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_


#define DO 1046
#define LE 1174
#define MI 1318
#define FA 1396
#define SO 1567
#define LA 1760
#define SI 1975
#define DOO 2093
#define RST 10

#define pita 11

#ifdef MAIN_C_              //main.cからこのファイルが呼ばれている場合

	  volatile int hz;                  //
	  volatile int pitagola[pita][2]= {{LE, 200},
				{MI, 200},
				{RST, 50},
				{LE, 200},
				{MI, 200},
				{RST, 50},
				{DOO, 200},
				{SI, 200},
				{RST, 200},
				{SO, 250},
				{RST, 500}};
#else                       //main.c以外からこのファイルが呼ばれている場合
  /*グローバル変数の宣言*/
	  extern volatile int hz;
	  extern volatile int pitagola[11][2];
#endif




void buzzer_init(void);
void buzzer(int, int);

#endif /* INC_BUZZER_H_ */
