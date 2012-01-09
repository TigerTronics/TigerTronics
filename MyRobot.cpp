#include "WPILib.h"
#include "HW_Inputs_Outputs.h"
#include "DashboardDataFormat.h"
#include "DDCArm.h"
#include "Deployment.h"

class Robot : public IterativeRobot
{
	RobotDrive *drive;			// robot drive base object	
	
	DigitalInput *left;			// digital inputs for line tracking sensors
	DigitalInput *middle;
	DigitalInput *right;
	
	
	DDCArm *myarm;                 // This will handle the arm movement and claw movement.
	Gyro *mygyro;
	Deployment *deploy;
	
	Jaguar *FL;					//Drive jaguars
	Jaguar *RL;
	Jaguar *FR;
	Jaguar *RR;
	
	
	Compressor *robot_compressor; // This will handle the compressor
	
	Joystick   *rightStick;       // joystick 1
	Joystick   *leftStick;        // joystick 2
	DriverStation *ds;			// driver station object
	DriverStationEnhancedIO &dseio;	


	
	
	/* Autonomous Variables*/	
	Timer *autotimer;	
	double defaultSteeringGain;
	int binaryValue;
	int previousValue;
	double steeringGain;		
	int oldTimeInSeconds;
	
	double time;
	double speed;
	double turn;
	bool straightLine;
	double stopTime;
	bool goLeft;
	bool atCross;
	DigitalInput *StraightLineSwitch;
	DigitalInput *GoLeftSwitch;
	
	
	/*DS State*/
	int peg;
	DDCArm::ArmMode modeArm;
	float xTweak,yTweak;
	//last pass variables for GetStateForArm() Function
	bool LP1,LP2,LP3,LP4,LP5,LP6,LPStow,LPRetrieve,LPPickUp,LPStopArm;
	bool pull, fire;


public:
	Robot(void) : dseio(ds->GetInstance()->GetEnhancedIO())
	{
		this->SetPeriod(.05);
		
		FL = new Jaguar(PWM_PORT_5);
		RL = new Jaguar(PWM_PORT_7);
		FR = new Jaguar(PWM_PORT_4);
		RR = new Jaguar(PWM_PORT_6);
		
		mygyro = new Gyro(AI_PORT_1);

		drive = new RobotDrive(FL,RL,FR,RR);
	    
		drive->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
		drive->SetInvertedMotor(RobotDrive::kRearRightMotor,true);
	    
		drive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);//inversed because motor physically spins in the rwrong direction
		drive->SetInvertedMotor(RobotDrive::kRearLeftMotor, false);
		
		drive->SetExpiration(15);

		left = new DigitalInput(DI_PORT_1);
		middle = new DigitalInput(DI_PORT_2);
		right = new DigitalInput(DI_PORT_3);
		
		StraightLineSwitch = new DigitalInput(DI_PORT_5);
		GoLeftSwitch = new DigitalInput(DI_PORT_6);
		
		

		robot_compressor = new Compressor(DI_PORT_4,DO_PORT_1);	
		
		
		rightStick = new Joystick(1);
		leftStick = new Joystick(2);

		myarm = new DDCArm();
		deploy = new Deployment();
		autotimer = new Timer();	
	}




	
	void RobotInit(void)
	{	
		//Pegs 1-6
		dseio.SetDigitalConfig(1,DriverStationEnhancedIO::kInputPullDown);
		dseio.SetDigitalConfig(2,DriverStationEnhancedIO::kInputPullDown);
		dseio.SetDigitalConfig(3,DriverStationEnhancedIO::kInputPullDown);
		dseio.SetDigitalConfig(4,DriverStationEnhancedIO::kInputPullDown);
		dseio.SetDigitalConfig(5,DriverStationEnhancedIO::kInputPullDown);
		dseio.SetDigitalConfig(6,DriverStationEnhancedIO::kInputPullDown);
		
		//pickup,retrieve,stow,stop
		dseio.SetDigitalConfig(7,DriverStationEnhancedIO::kInputPullDown);
		dseio.SetDigitalConfig(8,DriverStationEnhancedIO::kInputPullDown);
		dseio.SetDigitalConfig(9,DriverStationEnhancedIO::kInputPullDown);
		dseio.SetDigitalConfig(10,DriverStationEnhancedIO::kInputPullDown);
		
		LP1=false;
		LP2 =false;
		LP3=false;
		LP4 =false;
		LP5=false;
		LP6=false;
		LPStow=false;
		LPRetrieve=false;
		LPPickUp=false;
		LPStopArm=true;
		
		myarm->MoveClaw(-1);
		
		
		//start compressor
		robot_compressor->Start();
		
		mygyro->Reset();
	
	}
	
	
	/********************************** DISABLED MODE ********************************/
	void DisabledInit(void) 
	{
		for(int x=0;x<PotMemory;x++)
		{
			myarm->prepareSignal();
		}

	}	
	void DisabledPeriodic(void) 
	{
		// increment the number of disabled periodic loops completed
		deploy->OperateDeployment(false,true);
	}


	/********************************** AUTONOMOUS MODE ******************************/
	void AutonomousInit(void)
	{		
		defaultSteeringGain = -0.4;	// default value for steering gain
		previousValue = 0;
		
		oldTimeInSeconds = -1;
		
		// set the straight vs forked path variables as read from the DS digital
		// inputs or the I/O Setup panel on the driver station.
		straightLine = StraightLineSwitch->Get();
		stopTime = (straightLine) ? 2.0 : 4.0;
		goLeft = GoLeftSwitch->Get();
		atCross=false;
		
		printf("StraightLine: %d\n", straightLine);
		printf("GoingLeft: %d\n", goLeft);
		
		
		// set up timer for 8 second max driving time and use the timer to
		// pick values from the power profile arrays
		autotimer->Start();
		autotimer->Reset();
		
	}
	/*
	 * Sample line tracking class for FIRST 2011 Competition
	 * Jumpers on driver station digital I/O pins select the operating mode:
	 * The Driver Station digital input 1 select whether the code tracks the straight
	 * line or the forked line. Driver station digital input 2 selects whether the
	 * code takes the left or right fork. You can set these inputs using jumpers on
	 * the USB I/O module or in the driver station I/O Configuration pane (if there
	 * is no Digital I/O module installed.
	 *
	 * Since there is the fork to contend with, the code tracks the edge of the line
	 * using a technique similar to that used with a single-sensor Lego robot.
	 *
	 * The two places to do tuning are:
	 *
	 * defaultSteeringGain - this is the amount of turning correction applied
	 * forkProfile & straightProfile - these are power profiles applied at various
	 *	times (one power setting / second of travel) as the robot moves towards
	 *	the wall.
	 */
	void AutonomousPeriodic(void) 
	{
//		// loop until either we hit the "T" at the end or 8 seconds has
//		// elapsed. The time to the end should be less than 7 seconds
//		// for either path.
//		time = autotimer->Get();
//		//if(time < 8.0 && !atCross) {
//		if(!atCross){
//
//			int timeInSeconds = (int) time;
//			int leftValue = left->Get() ? 0 : 1;  // read the line tracking sensors
//			int middleValue = middle->Get() ? 0 : 1;
//			int rightValue = right->Get() ? 0 : 1;
//
//			// compute the single value from the 3 sensors. Notice that the bits
//			// for the outside sensors are flipped depending on left or right
//			// fork. Also the sign of the steering direction is different for left/right.
//			if (goLeft)//on fork and go left 
//			{
//				binaryValue = leftValue * 4 + middleValue * 2 + rightValue;
//				steeringGain = -defaultSteeringGain;
//			} 
//			else //on straight, or on fork and go right
//			{
//				binaryValue = rightValue * 4 + middleValue * 2 + leftValue;
//				steeringGain = defaultSteeringGain;
//			}
//			speed=-.2;	
//			turn = 0;			// default to no turn
//
//			switch (binaryValue) {
//				case 1:					// just the outside sensor - drive straight
//					turn = 0;
//					break;
//				case 7:					// all sensors - maybe at the "T"
//					if (time > stopTime) {
//						atCross = true;
//						speed = 0;
//					}
//					break;
//				case 0:					// no sensors - apply previous correction
//					if (previousValue == 0 || previousValue == 1) {
//						turn = steeringGain;
//					}
//					else {
//						turn = -steeringGain;
//					}
//					break;
//				default:				// anything else, steer back to the line
//					turn = -steeringGain;
//			}
//
//			
//			// useful debugging output for tuning your power profile and steering gain
//			if(binaryValue != previousValue)
//			{
//				printf("Time: %2.2f sensor: %d speed: %1.2f turn: %1.2f atCross: %d\n", time, binaryValue, speed, turn, atCross);
//			}
//
//			// move the robot forward
//			float angle = mygyro->GetAngle(); // get heading
//			drive->MecanumDrive_Cartesian(turn, speed, 0, angle);
//
//			if (binaryValue != 0) 
//				previousValue = binaryValue;
//			
//			oldTimeInSeconds = timeInSeconds;
//		}
//		// stop driving when finished
//		else
//		    drive->MecanumDrive_Cartesian(0,0,0,mygyro->GetAngle());
//		Wait(.1);
		drive->MecanumDrive_Cartesian(0,0,0,0);
		deploy->OperateDeployment(false,true);
	}

	/********************************** TELEOP MODE **********************************/
	void TeleopInit(void) {}
	void TeleopPeriodic(void) 
	{	
		myarm->prepareSignal();
		// Call the drive routine to drive the robot.
		if(rightStick->GetRawButton(1)|| leftStick->GetRawButton(1))
			drive->MecanumDrive_Cartesian(rightStick->GetX()/2,leftStick->GetY()/2,leftStick->GetX()/2,0.00);
		else
			drive->MecanumDrive_Cartesian(rightStick->GetX(),leftStick->GetY(),leftStick->GetX(),0.00);
		GetStateForArm();
		//Wait(.1);
		if(modeArm==DDCArm::kManualOveride)
		{
			myarm->OperateArm(0,0,0,modeArm);
			//Shoulder movement
			if(leftStick->GetRawButton(3))
				myarm->MoveShoulder(1);
			else if(leftStick->GetRawButton(2))
				myarm->MoveShoulder(-1);
			else
				myarm->MoveShoulder(0);
			
			//Elbow Movement
			if(rightStick->GetRawButton(3))
				myarm->MoveElbow(1);
			else if (rightStick->GetRawButton(2))
				myarm->MoveElbow(-1);
			else
				myarm->MoveElbow(0);
			
			//Wrist Movement
			if(rightStick->GetRawButton(4))
				myarm->MoveWrist(-1);
			else if(rightStick->GetRawButton(5))
				myarm->MoveWrist(1);
			else
				myarm->MoveWrist(0);			
		}	
		
		else
			myarm->OperateArm(0.0,0.0,peg,modeArm);
		
		if(leftStick->GetRawButton(4))
			myarm->MoveClaw(-1);
		else if(leftStick->GetRawButton(5))
			myarm->MoveClaw(1);
		else
			myarm->MoveClaw(0);
		
		if(rightStick->GetRawButton(10))
		{
			printf("S: %f \n E: %f \n W: %f \n \n",myarm->GetShoulderVoltage(),myarm->GetElbowVoltage(), myarm->GetWristVoltage());
		}
		
		deploy->OperateDeployment(fire,pull);
		// Send Data to the Driver Station for Monitoring (w/in .
		//sendIOPortData();
		//Wait(.1);
	}
	
	void GetStateForArm()
	{
		
		//stay in manual overide until another console button is pressed
		if(leftStick->GetRawButton(2)  || leftStick->GetRawButton(3) || 
		   leftStick->GetRawButton(4)  || leftStick->GetRawButton(5) ||
		   rightStick->GetRawButton(2) || rightStick->GetRawButton(3) || 
		   rightStick->GetRawButton(4) || rightStick->GetRawButton(5))
		{
			modeArm=DDCArm::kManualOveride;
			peg=0;
		}
		
		else
		{	
			if(dseio.GetDigital(1))
			{
				modeArm=DDCArm::kInverseKinematics;
				peg=1;
			}
			if(dseio.GetDigital(2))
			{
				modeArm=DDCArm::kInverseKinematics;
				peg=2;
			}
			if(dseio.GetDigital(3))
			{
				modeArm=DDCArm::kInverseKinematics;
				peg=3;
			}
			if(dseio.GetDigital(4))
			{
				modeArm=DDCArm::kInverseKinematics;
				peg=4;
			}
			if(dseio.GetDigital(5))
			{
				modeArm=DDCArm::kInverseKinematics;
				peg=5;
			}
			if(dseio.GetDigital(6))
			{
				modeArm=DDCArm::kInverseKinematics;
				peg=6;
			}
			if(dseio.GetDigital(7))//pickup
			{
				modeArm=DDCArm::kPickupTube;
				peg=0;
			}
			if(dseio.GetDigital(8))//retrieve
			{
				modeArm=DDCArm::kRetrieveTube;
				peg=0;
			}
			if(dseio.GetDigital(9))//stow
			{
				modeArm=DDCArm::kStow;
				peg=0;
			}
			if(dseio.GetDigital(10))//stop
			{
				modeArm = DDCArm::kStop;
				peg=0;
			}
		}
		if(dseio.GetAnalogIn(2)>2.5)
			fire=true;
		else
			fire=false;
		if(dseio.GetAnalogIn(2)<1)
			pull=true;
		else
			pull=false;
	}


};

START_ROBOT_CLASS(Robot)
;




