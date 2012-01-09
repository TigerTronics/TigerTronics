#ifndef __DDCArm_h__
#define __DDCArm_h__
#define PotMemory 10

#include "WPILib.h"
#include "HW_Inputs_Outputs.h"


/**
* Utility class handling arm controls for the robot.
*/
class DDCArm
{

	
public:
	
	
	
	enum ArmMode
	{
		kStow=0,//arm is stowed
		kRetrieveTube=1,//get tube off ground
		kPickupTube=2,//get tube from person
		kWristBack=3,//move the wrist out of the way (for autonomous)
		kInverseKinematics=4,// moving to position
		kManualOveride=5,
		kStop=6//Emergency Stop
	};

	DDCArm();
	virtual ~DDCArm();
	
	void prepareSignal();
	void OperateArm(float xTweak, float yTweak, int peg, ArmMode mode);
	float GetShoulderAngle();
	float GetElbowAngle();
	float GetWristAngle();
	
	void MoveShoulder(int dir);
	void MoveElbow(int dir);
	void MoveWrist(int dir);
	void MoveClaw(int dir);
	
	bool AtDesiredPosition();
	
	float GetShoulderVoltage();
	float GetElbowVoltage();
	float GetWristVoltage();
	

	

private:
	float avgShoulder, avgElbow, avgWrist;
	static const int kMaxReach=65;//inches
	bool gotToSafeAngle,gotToSafeAngle2,gotToShoulderSafe;
	
	//TODO Untested 'correction".  All 3 instances of PotMemory were previously hardcoded as 20
	//running average arrays
	float runShoulder[PotMemory];
	float runElbow[PotMemory];
	float runWrist[PotMemory];
	
	int arrayPoint;//0 to 19
	
	
	//Motors and Feedback
	Jaguar *ElbowJag;
	Jaguar *WristJag;
	Solenoid *ShoulderUpSolenoid;
	Solenoid *ShoulderDownSolenoid;
	Solenoid *MouthOpenSolenoid;
	Solenoid *MouthCloseSolenoid;
	AnalogChannel *ShoulderPot;
	AnalogChannel *ElbowPot;
	AnalogChannel *WristPot;
		
	int LastPassPeg;
	ArmMode LastPassMode;
	float LastPassXTweak;
	float LastPassYTweak;
	
	
	
	//Attributes
	static const float M_PI=3.14159;	
	float dShoulder, dElbow, dWrist;//the final angles the arm needs to be set to (radians)
	float L1, L2, L3; //length of the arm segments in inches
	float SafeAngle;
	bool FirstPassMove;
	float BeginMoveS;//Shoulder angle at begining of movement
	float BeginMoveE;//Elbow Angle at Begining of Movement
	
	//Functions
	float DDCArm::WavgNumberArray(float array[], int arraylength);
	float DDCArm::avgNumberArray(float array[], int arraylength);
	bool ComputeAngles(float xPos, float yPos);//calculates the angles for Inverse Kinematics
	float GetDistance(float x1, float y1, float x2, float y2);//gets the distance between two points
	bool GetCircleIntersection(float x0, float y0, float r0, float x1,
			float y1, float r1, float &xi, float &yi, float &xi_prime,
			float &yi_prime);//gets the intersection points of two circles
	float GetOptimalAngle1(float x1,float y1,float x2,float y2);//gets the angle closest to segment1 angle
	bool SetShoulderAngle(float angle);//function to set the position of segment1 (its not a jag so we can't call set)
	bool SetElbowAngle(float angle);
	bool SetWristAngle(float angle);
	void MoveArm();//actually moves the arm to the specified angles
	
	
//	void DetermineAnglesBasedOnPeg (int pegnum);
//	int SEgivenW(float Woffset, 
//		     float X, float Y, float POST, float SHOULDER, float ELBOW, float WRIST);
//	float WgivenS(float S, float X, float Y, float POST, float SHOULDER, float ELBOW,
//	              float WRIST);
//	float EgivenWS(float S, float W, float X, float Y, float POST, float SHOULDER, 
//		       float ELBOW, float WRIST);
};

#endif // __DDCArm__
