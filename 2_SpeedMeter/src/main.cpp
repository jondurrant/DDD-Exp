/***
 * Control DDD for a Motor Speed Test
 *
 * Jon Durrant
 * 15-Aug-2022
 */


#include "pico/stdlib.h"
#include "MotorMgr.h"

//Left Motor
#define LEFT_PWR_CW		9
#define LEFT_PWR_CCW	8
#define LEFT_ROTENC_A 	14
#define LEFT_ROTENC_B	15

//Right Motor
#define RIGHT_PWR_CW	6
#define RIGHT_PWR_CCW	7
#define RIGHT_ROTENC_A 	12
#define RIGHT_ROTENC_B	13

char ROBOT_NAME[]="ddd";




/***
 * Main
 * @return
 */
int main( void ){
    stdio_init_all();
    sleep_ms(2000);
    printf("GO\n");

    MotorMgr left(LEFT_PWR_CW,   LEFT_PWR_CCW,  LEFT_ROTENC_A,  LEFT_ROTENC_B);
    MotorMgr right(RIGHT_PWR_CW, RIGHT_PWR_CCW, RIGHT_ROTENC_A, RIGHT_ROTENC_B);

    bool cw = true;
    for (;;){
    	for (float throttle = 0.3; throttle <= 1.0; throttle += 0.1){
    		printf("Throttle set to %.2f\n", throttle);
    		left.setThrottle(throttle, cw);
    		right.setThrottle(throttle, !cw);

    		for (int i=0; i < 20; i++){
    			printf("Left %0.3f Right %0.3f Rads Per Second\n",
    					left.getAvgRadPerSec(),
						right.getAvgRadPerSec());
    			sleep_ms(500);
    		}

    	}
    	cw = !cw;
    }
}
