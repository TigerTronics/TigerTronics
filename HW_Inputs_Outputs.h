
ifndef __HW_INPUTS_OUTPUTS_H__
#define __HW_INPUTS_OUTPUTS_H__



/*  Constants */

#define DI_PORT_1  1    //Left Photo Sensor
#define DI_PORT_2  2    //Center Photo Sensor
#define DI_PORT_3  3	//Right Photo Sensor
#define DI_PORT_4  4	//PressureSwitch
#define DI_PORT_5  5	//Straight line switch (autonomous)
#define DI_PORT_6  6	//GoLeft switch (autonomous)
#define DI_PORT_7  7
#define DI_PORT_8  8

#define DO_PORT_1  1      //Compressor Spike Relay
#define DO_PORT_2  2
#define DO_PORT_3  3
#define DO_PORT_4  4
#define DO_PORT_5  5
#define DO_PORT_6  6
#define DO_PORT_7  7
#define DO_PORT_8  8


#define AI_PORT_1  1	//Gyro
#define AI_PORT_2  2	//Elbow Pot
#define AI_PORT_3  3	//Wrist Pot
#define AI_PORT_4  4	//Shoulder Pot
#define AI_PORT_5  5
#define AI_PORT_6  6
#define AI_PORT_7  7

#define PWM_PORT_1 1     //Hockey Stick Latch
#define PWM_PORT_2 2     //mini bot latch
#define PWM_PORT_3 3     //deploy latch
#define PWM_PORT_4 4     //FR
#define PWM_PORT_5 5     //FL
#define PWM_PORT_6 6     //RR
#define PWM_PORT_7 7     //RL
#define PWM_PORT_8 8	 //Elbow
#define PWM_PORT_9 9	 //Wrist


#define SOLENOID_CHANNEL_1  1  //Shoulder Raise 
#define SOLENOID_CHANNEL_2  2  //Shoulder Lower
#define SOLENOID_CHANNEL_3  3  //Mouth Open  
#define SOLENOID_CHANNEL_4  4  //mouth Close    
#define SOLENOID_CHANNEL_5  5  //deploy minibot
#define SOLENOID_CHANNEL_6  6  //undeploy minibot
#define SOLENOID_CHANNEL_7  7
#define SOLENOID_CHANNEL_8  8

#endif
