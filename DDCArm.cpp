#include "DDCArm.h"
#include "HW_Inputs_Outputs.h"
#include "math.h"


/// Trig Macros ///////////////////////////////////////////////////////////////
#define DEGTORAD(A)	((A * M_PI) / 180.0f)
#define RADTODEG(A)	((A * 180.0f) / M_PI)

///////////////////////////////////////////////////////////////////////////////
float PegHeight[7]={0,29.375,64.625,99.875,37.375,72.625,107.875};
//Outside low=1,Outside mid=2, outside top=3, 
//inside bot=4, inside mid=5, insidetop=6

DDCArm::DDCArm(){
	arrayPoint=-1;
	
	ShoulderPot = new AnalogChannel(AI_PORT_4);
	ElbowPot = new AnalogChannel(AI_PORT_2);
	WristPot = new AnalogChannel(AI_PORT_3);
	
	
	ElbowJag = new Jaguar(PWM_PORT_8);
	WristJag = new Jaguar(PWM_PORT_9);
	SafeAngle= DEGTORAD(300);

	
	ShoulderUpSolenoid = new Solenoid(SOLENOID_CHANNEL_1);
	ShoulderDownSolenoid = new Solenoid(SOLENOID_CHANNEL_2);
	MouthOpenSolenoid = new Solenoid(SOLENOID_CHANNEL_3);
	MouthCloseSolenoid = new Solenoid(SOLENOID_CHANNEL_4);

	dShoulder=GetShoulderAngle();
	dElbow=GetElbowAngle();
	dWrist=GetWristAngle();
	
	ShoulderUpSolenoid->Set(false);
	ShoulderDownSolenoid->Set(false);
	MouthOpenSolenoid->Set(false);
	MouthCloseSolenoid->Set(false);

	L1=42.06; //inches 
	L2=23.75; //inches
	L3=14.25; //inches
	
	LastPassMode = kStop;
	LastPassPeg=0;

}

DDCArm::~DDCArm()
{
	delete ElbowJag;
	delete WristJag;
	delete ShoulderUpSolenoid;
	delete ShoulderDownSolenoid;
	delete MouthOpenSolenoid;
	delete MouthCloseSolenoid;
	delete ShoulderPot;
	delete ElbowPot;
	delete WristPot;
}

// Weighted average of a Number Array
float DDCArm::WavgNumberArray(float array[], int arraylength)
{
	int i;
	int weighting;
	float SUM = 0;
	for (i=0; i<arraylength; i++) {
		weighting = (i+arraylength-1-arrayPoint)%arraylength + 1;
		SUM += weighting*array[i];  // Weight and Add 'em up
	}
	return SUM/(arraylength*(arraylength+1)/2); 
}

float DDCArm::avgNumberArray(float array[], int arraylength)
{
	int i;
	float SUM = 0;
	for (i=0; i<arraylength; i++) {
		SUM += array[i];  // Add 'em up
	}
	return SUM/arraylength; 
}

//keeps track of the pointer and fills the history arrays to average.
void DDCArm::prepareSignal()
{
	arrayPoint=(arrayPoint+1)%PotMemory;
	runShoulder[arrayPoint]=ShoulderPot->GetVoltage();
	runElbow[arrayPoint]=ElbowPot->GetVoltage();
	runWrist[arrayPoint]=WristPot->GetVoltage();
}

void DDCArm::OperateArm(float xTweak, float yTweak, int peg, ArmMode mode)
{
	if(peg!=LastPassPeg || mode!=LastPassMode || LastPassXTweak!=xTweak || LastPassYTweak!=yTweak)
	{
		//if it gets to this point its the first time in this state 
		FirstPassMove=true;
		gotToSafeAngle=false;
		gotToSafeAngle2=false;
		gotToShoulderSafe=false;
		switch(mode)
		{
		case kStow:
			dShoulder=DEGTORAD(35);
			dElbow=DEGTORAD(330);
			dWrist=DEGTORAD(177.66);
			break;
		case kRetrieveTube:
			dShoulder=.877;
			dElbow=5.25;
			dWrist=4.41;
			break;
		case kPickupTube:
			dShoulder=.620;
			dElbow=4.13;
			dWrist=2.96;
			break;
		case kWristBack:
			//only for autonomous
			break;
		case kInverseKinematics:
			switch(peg)
			{
			case 1:
				dShoulder=.61;
				dElbow=4.73;
				dWrist=2.17;
				break;
			case 2:
				dShoulder=1.23;
				dElbow=5.18;
				dWrist=2.76;
				break;
			case 3:
				dShoulder=2.06;
				dElbow=4.45;
				dWrist=2.91;
				break;
			case 4:
				dShoulder=.61;
				dElbow=4.62;
				dWrist=1.92;
				break;
			case 5:
				dShoulder=1.34;
				dElbow=5.01;
				dWrist=2.91;
				break;
			case 6:
				dShoulder=2.45;
				dElbow=3.22;
				dWrist=2.15;
				break;
			default:
				break;			
			}
			break;
		case kStop:
			dShoulder=GetShoulderAngle();
			dElbow=GetElbowAngle();
			dWrist=GetWristAngle();
			break;
		case kManualOveride:
			break;
		default:
			break;
	}
}
	
	if(mode !=kManualOveride)
		MoveArm();

	LastPassXTweak=xTweak;
	LastPassYTweak=yTweak;
	LastPassMode=mode;
	LastPassPeg=peg;
}


//Moves the joints to the correct angles and returns true
void DDCArm::MoveArm() 
{
	//TODO stop if the arm has shifted from below to above within the last loopthrough
	//Can do with boolean set joint functions that return false when below and true when above...
	
	/*if (FirstPassMove) 
	{
		BeginMoveS=GetShoulderAngle();
		BeginMoveE=GetElbowAngle();
	}
	FirstPassMove=false;

	//if in safe zone and moving to same safe zone just move
	if ((L1*sin(BeginMoveS)+L2+L3)<kMaxReach && (L1*sin(dShoulder)+L2+L3)<kMaxReach
			&& ((M_PI/2-BeginMoveS)/fabs(M_PI/2-BeginMoveS)*(M_PI/2-dShoulder)/fabs(M_PI/2-dShoulder))>=0) 
	{
		SetShoulderAngle(dShoulder);
		SetElbowAngle(dElbow);
		SetWristAngle(dWrist);
	}
	//else if in safe zone and moving to opposite safe zone, move elbow to safe angle above 180, move shoulder, move elbow and wrist
	else if ((L1*sin(BeginMoveS)+L2+L3)<kMaxReach && (L1*sin(dShoulder)+L2+L3)
			<kMaxReach && ((M_PI/2-BeginMoveS)/fabs(M_PI/2-BeginMoveS)*(M_PI/2-dShoulder)
					/fabs(M_PI/2-dShoulder))<0) 
	{
		if(!gotToSafeAngle)
			gotToSafeAngle=SetElbowAngle(SafeAngle);
		SetWristAngle(M_PI);
		if (gotToSafeAngle) 
		{
			bool b=SetShoulderAngle(dShoulder);
			if (b) 
			{
				SetElbowAngle(dElbow);
				SetWristAngle(dWrist);
			}
		}
	}

	//else if starting in unsafe zone
	else if ((L1*sin(BeginMoveS)+L2+L3)>=kMaxReach) 
	{
		//if e is moving to same side of 180, go to closest safe angle, then move shoulder, then move elbow and wrist
		if (((M_PI-BeginMoveE)/fabs(M_PI-BeginMoveE)*(M_PI-dElbow)/fabs(M_PI-dElbow))>=0) 
		{
			if(!gotToSafeAngle){

				if (BeginMoveE>M_PI) 
				{
					gotToSafeAngle=SetElbowAngle(SafeAngle);
					SetWristAngle(M_PI);
				} 
				else 
				{
					gotToSafeAngle=SetElbowAngle(2*M_PI-SafeAngle);
					SetWristAngle(M_PI);
				}
			}
			if (gotToSafeAngle) 
			{
				bool b=SetShoulderAngle(dShoulder);
				if (b) 
				{
					SetElbowAngle(dElbow);
					SetWristAngle(dWrist);
				}
			}
		}
		//else if e and moving to opposite side of 180 move go to closest safe angle, move to top safe zone, move to opposite safe angle, move shoulder, move wrist and elbow
		else if (((M_PI-BeginMoveE)/fabs(M_PI-BeginMoveE)*(M_PI-dElbow)/fabs(M_PI-dElbow))<0) 
		{
			if(!gotToSafeAngle)
			{		
				if (BeginMoveE>M_PI) 
				{
					gotToSafeAngle=SetElbowAngle(SafeAngle);
					SetWristAngle(M_PI);
				} 
				else 
				{
					gotToSafeAngle=SetElbowAngle(2*M_PI-SafeAngle);
					SetWristAngle(M_PI);
				}
			}
			if (gotToSafeAngle) 
			{
				gotToShoulderSafe=SetShoulderAngle(DEGTORAD(130));//go to top safe zone
				if (gotToShoulderSafe) 
				{ //go to opposite safe angle
					if(!gotToSafeAngle2)
					{
						if (BeginMoveE>M_PI) 
						{
							gotToSafeAngle2=SetElbowAngle(2*M_PI-SafeAngle);
							SetWristAngle(M_PI);
						} 
						else 
						{
							gotToSafeAngle2=SetElbowAngle(SafeAngle);
							SetWristAngle(M_PI);
						}
					}
					if (gotToSafeAngle2)//if in the opposite safe zone
					{
						bool b=SetShoulderAngle(dShoulder);
						if (b)
						{
							SetElbowAngle(dElbow);
							SetWristAngle(dWrist);
						}
					}
				}
			}
		}		
	}
	else
	{
			if(!gotToSafeAngle)
				gotToSafeAngle=SetElbowAngle(SafeAngle);
			SetWristAngle(M_PI);
			if (gotToSafeAngle) 
			{
				bool b=SetShoulderAngle(dShoulder);
				if (b) 
				{
					SetElbowAngle(dElbow);
					SetWristAngle(dWrist);
				}
			}
	}
	//else error move manually
		//printf("Error Move Arm Manually");
*/
	
	SetShoulderAngle(dShoulder);
	SetElbowAngle(dElbow);
	SetWristAngle(dWrist);
}

/*Sets the shoulder to the correct angle
at Param the angle to go to*/
bool DDCArm::SetShoulderAngle(float angle)
{
	float a=GetShoulderAngle();
	if(fabs(angle-a)>DEGTORAD(5))//if the angle isnt equal to where it needs to go
	{
		if(a>angle)
			MoveShoulder(-1);		
		else
			MoveShoulder(1);
		return false;
	}
	else
	{
		MoveShoulder(0);
		return true;
	}
	
}

bool DDCArm::SetElbowAngle(float angle)
{
 	float a=GetElbowAngle();
	if(fabs(angle-a)>DEGTORAD(5))//if the angle isnt equal to where it needs to go
	{
		if(a>angle)
			MoveElbow(-1);//lower the arm
		else
			MoveElbow(1);//raise the arm
		return false;
	}
	else
	{
		MoveElbow(0);//stop
		return true;
	}
}

bool DDCArm::SetWristAngle(float angle)
{
	float a=GetWristAngle();
	if(fabs(angle-a)>DEGTORAD(5))//if the angle isnt equal to where it needs to go
	{
		if(a>angle)
			MoveWrist(-1);//lower the arm
		else
			MoveWrist(1);//raise the arm
		return false;
	}
	else
	{
		MoveWrist(0);//stop
		return true;
	}
}


//Public Function so you can manually move the arm
void DDCArm::MoveShoulder(int dir)
{
	switch(dir)
	{
	case -1:
		ShoulderUpSolenoid->Set(false);
		ShoulderDownSolenoid->Set(true);
		break;
	case 1:
		ShoulderUpSolenoid->Set(true);
		ShoulderDownSolenoid->Set(false);
		break;
	default:
		ShoulderUpSolenoid->Set(false);
		ShoulderDownSolenoid->Set(false);
		break;
	}
	
}

void DDCArm::MoveElbow(int dir)
{
	switch(dir)
		{
		case -1:
			ElbowJag->Set(-1);
			break;
		case 1:
			ElbowJag->Set(1);
			break;
		default:
			ElbowJag->Set(0);
			break;
		}
}

void DDCArm::MoveWrist(int dir)
{
	switch(dir)
		{
		case -1:
			WristJag->Set(.5);
			break;
		case 1:
			WristJag->Set(-.5);
			break;
		default:
			WristJag->Set(0);
			break;
		}
}

void DDCArm::MoveClaw(int dir)
{
	switch(dir)
	{
	case -1:
		MouthOpenSolenoid->Set(false);
		MouthCloseSolenoid->Set(true);
		break;
	case 1:
		MouthOpenSolenoid->Set(true);
		MouthCloseSolenoid->Set(false);
		break;
	default:
		MouthOpenSolenoid->Set(false);
		MouthCloseSolenoid->Set(false);
		break;
	}
}




//get the angle of the shoulder
float DDCArm::GetShoulderAngle()
{
	// float v = ShoulderPot->GetVoltage();//limShoulder;
	float v = WavgNumberArray(runShoulder, PotMemory);
	float d = (v-2.265)/.015;
	float r = DEGTORAD(d);
	return r;
}


//Get the angle from the Elbow jag 
float DDCArm::GetElbowAngle()
{
	// float v=ElbowPot->GetVoltage();//limElbow;
	float v = WavgNumberArray(runElbow, PotMemory);	
	float d = (v+2.229)/.015;
	float r=DEGTORAD(d);
	return r;
}

//Get the angle from the wrist jag
float DDCArm::GetWristAngle()
{
	//Voltage of degree = Voltage of offset- (.015*desired degree)
	// float v=WristPot->GetVoltage();//limWrist;
	float v = WavgNumberArray(runWrist, PotMemory);
	float d = (6.355-v)/.015;
	float r=DEGTORAD(d);
	return r;
}


float DDCArm::GetShoulderVoltage()
{
	return ShoulderPot->GetVoltage();
}

float DDCArm::GetElbowVoltage()
{
	return ElbowPot->GetVoltage();
}

float DDCArm::GetWristVoltage()
{
	return WristPot->GetVoltage();
}

//is the claw in position
//TODO might not need because each set pos checks it self
bool DDCArm::AtDesiredPosition()
{
	float s=GetShoulderAngle();
	float e=GetElbowAngle();
	float w=GetWristAngle();

	if(fabs(s-dShoulder)<=DEGTORAD(5) &&fabs(e-dElbow)<=DEGTORAD(5) &&fabs(w-dWrist)<=DEGTORAD(5))
		return true;
	else
		return false;
}

