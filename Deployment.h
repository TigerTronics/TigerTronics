#ifndef __Deployment_h__
#define __Deployment_h__

#include "WPILib.h"
#include "HW_Inputs_Outputs.h"


/**
* Utility class handling arm controls for the robot.
*/
class Deployment
{	
  public:
	  Deployment();
	  virtual ~Deployment();
	  void OperateDeployment(bool fire, bool pull);
  private:
	  Solenoid *DeploySolenoid;
	  Solenoid *UndeploySolenoid;
	  //Deployment
	  Servo *miniBotLatch;

	  Timer *fireTimer;
};

#endif // __Deployment__
