#include "DashboardDataFormat.h"

void sendVisionData() {
	Dashboard &dash = DriverStation::GetInstance()->GetHighPriorityDashboardPacker();
	dash.AddCluster(); // wire (2 elements)
	{
		dash.AddCluster(); // tracking data
		{
			dash.AddDouble(1.0); // Joystick X
			dash.AddDouble(135.0); // angle
			dash.AddDouble(3.0); // angular rate
			dash.AddDouble(5.0); // other X
		}
		dash.FinalizeCluster();
		dash.AddCluster(); // target Info (2 elements)
		{
			dash.AddCluster(); // targets
			{
				dash.AddDouble(100.0); // target score
				dash.AddCluster(); // Circle Description (5 elements)
				{
					dash.AddCluster(); // Position (2 elements)
					{
						dash.AddDouble(30.0); // X
						dash.AddDouble(50.0); // Y
					}
					dash.FinalizeCluster();
				}
				dash.FinalizeCluster(); // Position
				dash.AddDouble(45.0); // Angle
				dash.AddDouble(21.0); // Major Radius
				dash.AddDouble(15.0); // Minor Radius
				dash.AddDouble(324.0); // Raw score
			}
			dash.FinalizeCluster(); // targets
		}
		dash.FinalizeCluster(); // target Info
	}
	dash.FinalizeCluster(); // wire
	dash.Finalize();
}

void sendIOPortData() {
	Dashboard &dash = DriverStation::GetInstance()->GetLowPriorityDashboardPacker();
	
	// *** ADD ANALOG INPUTS to Driver Station ***
	dash.AddCluster();
	{
		dash.AddCluster();
		{ //analog modules 
			dash.AddCluster();
			{
				for (int i = 1; i <= 8; i++) {
					dash.AddFloat((float) AnalogModule::GetInstance(1)->GetAverageVoltage(i));
					//dash.AddFloat((float) i * 5.0 / 8.0);
				}
			}
			dash.FinalizeCluster();
			dash.AddCluster();
			{
				for (int i = 1; i <= 8; i++) {
					dash.AddFloat((float) AnalogModule::GetInstance(2)->GetAverageVoltage(i));
				}
			}
			dash.FinalizeCluster();
		}
		dash.FinalizeCluster();

		// *** ADD DIGITAL INPUTS to Driver Station ***
		dash.AddCluster();
		{ //digital modules
			dash.AddCluster();
			{
				dash.AddCluster();
				{
					int module = 4;
					dash.AddU8(DigitalModule::GetInstance(module)->GetRelayForward());
					dash.AddU8(DigitalModule::GetInstance(module)->GetRelayReverse());
					dash.AddU16((short)DigitalModule::GetInstance(module)->GetDIO());
					dash.AddU16((short) 0xAAAA);
					dash.AddU16((short)DigitalModule::GetInstance(module)->GetDIODirection());
					dash.AddU16((short) 0x7777);
					dash.AddCluster();
					{
						for (int i = 1; i <= 10; i++) {
							dash.AddU8((unsigned char) DigitalModule::GetInstance(module)->GetPWM(i));
							//dash.AddU8((unsigned char) (i-1) * 255 / 9);
						}
					}
					dash.FinalizeCluster();
				}
				dash.FinalizeCluster();
			}
			dash.FinalizeCluster();

			// *** ADD PWM Inputs to Driver Station ***
			dash.AddCluster();
			{
				dash.AddCluster();
				{
					int module = 6;
					dash.AddU8(DigitalModule::GetInstance(module)->GetRelayForward());
					dash.AddU8(DigitalModule::GetInstance(module)->GetRelayForward());
					dash.AddU16((short)DigitalModule::GetInstance(module)->GetDIO());
					dash.AddU16(DigitalModule::GetInstance(module)->GetDIODirection());
					dash.AddCluster();
					{
						for (int i = 1; i <= 10; i++) {
							dash.AddU8((unsigned char) DigitalModule::GetInstance(module)->GetPWM(i));
							//dash.AddU8((unsigned char) i * 255 / 10);
						}
					}
					dash.FinalizeCluster();
				}
				dash.FinalizeCluster();
			}
			dash.FinalizeCluster();
		}
		dash.FinalizeCluster();

		// Can't read solenoids without an instance of the object
		dash.AddU8((char) 0);
	}
	dash.FinalizeCluster();
	dash.Finalize();
}
