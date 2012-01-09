#include "Deployment.h"
#include "HW_Inputs_Outputs.h"



Deployment::Deployment()
{
	
	DeploySolenoid = new Solenoid(SOLENOID_CHANNEL_5);
	UndeploySolenoid = new Solenoid(SOLENOID_CHANNEL_6);
	miniBotLatch= new Servo(PWM_PORT_2);
	fireTimer= new Timer();
	
	fireTimer->Stop();
	fireTimer->Reset();
	
	
	UndeploySolenoid->Set(true);
	DeploySolenoid->Set(false);
	
}

Deployment::~Deployment()
{
	delete DeploySolenoid;
	delete UndeploySolenoid;
	delete miniBotLatch;

}

void Deployment::OperateDeployment(bool fire, bool pull)
{	
	
	if(fire)
	{
			if(fireTimer->Get()==0)
				fireTimer->Start();
			//unlatch minibot
			miniBotLatch->SetAngle(60);
			//fire minibot
			if(fireTimer->Get()>1.2)
			{
				UndeploySolenoid->Set(false);
				DeploySolenoid->Set(true);
			}		
	}
	else if(pull)
	{
		miniBotLatch->SetAngle(170);
		UndeploySolenoid->Set(true);
		DeploySolenoid->Set(false);
		if(fireTimer->Get()!=0)
		{
			fireTimer->Stop();
			fireTimer->Reset();
		}
	}

	else
	{
		miniBotLatch->SetAngle(170);
		UndeploySolenoid->Set(false);
		DeploySolenoid->Set(false);
		
	}

}
