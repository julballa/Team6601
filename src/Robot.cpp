#include <ADXRS450_Gyro.h>
#include <cscore_oo.h>
#include <cscore_oo.inl>
#include <DoubleSolenoid.h>
#include <Encoder.h>
#include <Joystick.h>
#include <RobotBase.h>
#include <SampleRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Spark.h>
#include <SPI.h>
#include <Timer.h>
#include <Values.h>
#include <cmath>
#include <iostream>


using namespace frc;
using namespace std;
class Robot: public frc::SampleRobot
{
	ADXRS450_Gyro* Gyro;
	Encoder* dbEncoders[2];
	Spark* Motors[4];
	Spark* Shooter;
	Joystick* stick[2]; // only joystick
	Joystick* gamepad;
	Spark* Winch;
	Spark* Intake;
	DoubleSolenoid* Pistons[3];
	int Iteration = 1;
	frc::SendableChooser<std::string> chooser;

public:

	Robot() {
		//REVIEW GYRO AND ENCODER PORTS
		Gyro = new ADXRS450_Gyro(SPI::Port::kOnboardCS0);
		dbEncoders[LEFT] = new Encoder(0,1);
		dbEncoders[RIGHT] = new Encoder(2,3);
		Motors[0] = new Spark(0);
		Motors[1] = new Spark(1);
		Motors[2] = new Spark(2);
		Motors[3] = new Spark(3);
		Shooter = new Spark(4);
		stick[LEFT] = new Joystick(1);
		stick[RIGHT] = new Joystick(2);
		gamepad = new Joystick(0);
		Winch = new Spark(7);
		Intake = new Spark(8);
		Pistons[0] = new DoubleSolenoid(6,0);
		Pistons[1] = new DoubleSolenoid(5,1);
		Pistons[2] = new DoubleSolenoid(4,2);
	}

	//BOUNDING RECT BREAKS THE VISION CODE. NEEDS A FIX
	void Default(){
		Motors[0]->Set(0.5);
		Motors[1]->Set(0.5);
		Motors[2]->Set(0.5);
		Motors[3]->Set(0.5);
	}
	void Debug()
	{
		cout << "Iteration: " << endl << Iteration << endl;
		cout << "Left Encoder Inches:" << endl << dbEncoders[LEFT]->GetDistance()/3 << endl;
		cout << "Right Encoder Inches:" << endl << dbEncoders[RIGHT]->GetDistance()/3 << endl;
		cout << "Gyro Output:" << endl << Gyro->GetAngle() << endl;
		Iteration++;
	}
	void PneumaticController(bool buttonController, std::string controlFlow)
	{
		if(controlFlow == "open" && buttonController == true)
		{
			Pistons[1]->Set(DoubleSolenoid::kForward);
			Pistons[2]->Set(DoubleSolenoid::kReverse);
			Wait(0.2);
			Pistons[0]->Set(DoubleSolenoid::kForward);
		}
		else if(controlFlow == "close" && buttonController == true)
		{
			Pistons[0]->Set(DoubleSolenoid::kReverse);
			Wait(0.2);
			Pistons[1]->Set(DoubleSolenoid::kReverse);
			Pistons[2]->Set(DoubleSolenoid::kForward);
		}
	}
	void EncoderReset()
	{
		dbEncoders[LEFT]->Reset();
		dbEncoders[RIGHT]->Reset();
	}
	void GyroReset()
	{
		Gyro->Reset();
	}
	void LeftPeg(){

		PneumaticController(true, "close");

		Motors[0]->Set(-0.5);
		Motors[1]->Set(-0.5);
		Motors[2]->Set(-0.5);
		Motors[3]->Set(-0.5);

		while(dbEncoders[0]->GetDistance()/3 <= (73.5)*(3/4)){}

		Motors[0]->Set(0.0);
		Motors[1]->Set(0.0);
		Motors[2]->Set(0.0);
		Motors[3]->Set(0.0);
		Wait(0.2);
		Motors[0]->Set(-0.35);
		Motors[1]->Set(-0.35);

		Motors[2]->Set(0.35);
		Motors[3]->Set(0.35);
		//CHECK WHICH DIRECTION THIS IS ORIENTED TOWARDS
		while (fabs(Gyro->GetAngle()) < 58){}

		Motors[0]->Set(0.0);
		Motors[1]->Set(0.0);
		Motors[2]->Set(0.0);
		Motors[3]->Set(0.0);
		Wait(0.2);
		Motors[0]->Set(-0.5);
		Motors[1]->Set(-0.5);
		Motors[2]->Set(-0.5);
		Motors[3]->Set(-0.5);
		//If the wheels aren't turning, then encoders won't register
		while(dbEncoders[0]->GetDistance()/3 <= 13.84*0.75){}

		Motors[0]->Set(0.0);
		Motors[1]->Set(0.0);
		Motors[2]->Set(0.0);
		Motors[3]->Set(0.0);
		Wait(0.2);
		PneumaticController(true, "open");
	}
	void RightPeg(){

		PneumaticController(true, "close");

		Motors[0]->Set(-0.5);
		Motors[1]->Set(-0.5);
		Motors[2]->Set(-0.5);
		Motors[3]->Set(-0.5);

		while(dbEncoders[0]->GetDistance()/3 <= (84.5)*(3/4)){}
		Motors[0]->Set(0.0);
		Motors[1]->Set(0.0);
		Motors[2]->Set(0.0);
		Motors[3]->Set(0.0);
		Wait(0.2);
		Motors[0]->Set(0.35);
		Motors[1]->Set(0.35);

		Motors[2]->Set(-0.35);
		Motors[3]->Set(-0.35);
		//CHECK WHICH DIRECTION THIS IS ORIENTED TOWARDS
		while (fabs(Gyro->GetAngle()) < 58){}
		Motors[0]->Set(0.0);
		Motors[1]->Set(0.0);
		Motors[2]->Set(0.0);
		Motors[3]->Set(0.0);
		Wait(0.2);

		Motors[0]->Set(-0.5);
		Motors[1]->Set(-0.5);
		Motors[2]->Set(-0.5);
		Motors[3]->Set(-0.5);

		//If the wheels aren't turning, then encoders won't register
		while(dbEncoders[0]->GetDistance()/3 <= 13.84*0.75){}

		Motors[0]->Set(0.0);
		Motors[1]->Set(0.0);
		Motors[2]->Set(0.0);
		Motors[3]->Set(0.0);

		PneumaticController(true, "open");

	}
	void MiddlePeg()
	{
		EncoderReset();
		GyroReset();
		PneumaticController(true, "close");
		Motors[0]->Set(-0.5);
		Motors[1]->Set(-0.5);
		Motors[2]->Set(-0.5);
		Motors[3]->Set(-0.5);

		while(dbEncoders[0]->GetDistance()/3 <= (79)*(3/4)){}
		//ensure that the gear is on the peg
		Wait(0.35);
		Motors[0]->Set(0.0);
		Motors[1]->Set(0.0);
		Motors[2]->Set(0.0);
		Motors[3]->Set(0.0);
		PneumaticController(true, "close");
	}

	void RobotInit()
	{

		//SDDP = Circumference/(Gear Ratio * encoder value)). When getting multiply by 1/3
		dbEncoders[LEFT]->SetDistancePerPulse((2.0*3.14159*3)/(1440*12.75));
		dbEncoders[RIGHT]->SetDistancePerPulse((2.0*3.14159*3)/(1440*12.75));
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameLeftPeg, autoNameLeftPeg);
		chooser.AddObject(autoNameRightPeg, autoNameRightPeg);
		chooser.AddObject(autoNameMiddlePeg, autoNameMiddlePeg);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		for(int x = 0; x<5; x++)
			ButtonReleased[x] = false;
	}

	void Autonomous()
	{
		while(IsAutonomous()&&IsEnabled())
		{
			/*auto autoSelected = chooser.GetSelected();
			if(autoSelected == autoNameLeftPeg){this->LeftPeg();}
			else if(autoSelected == autoNameRightPeg){this->RightPeg();}
			else if(autoSelected == autoNameMiddlePeg){this->MiddlePeg();}
			else{this->LeftPeg();}*/

		}
	}

	void OperatorControl() override
	{

		while (IsOperatorControl() && IsEnabled()) {
			Motors[0]->Set(stick[LEFT]->GetY()*SpeedConstant);
			Motors[1]->Set(stick[LEFT]->GetY()*SpeedConstant);
			Motors[2]->Set(stick[RIGHT]->GetY()*SpeedConstant);
			Motors[3]->Set(stick[RIGHT]->GetY()*SpeedConstant);
			if(stick[LEFT]->GetRawButton(1)&&stick[RIGHT]->GetRawButton(1))
			{
				SpeedConstant = 0.45;
			}
			else
			{
				SpeedConstant = 1.0;
			}
			//Shooter Button
			if(gamepad->GetRawButton(ABUTTON))//A
			{
				Shooter->Set(-0.8);
			}
			else
			{
				Shooter->Set(0.0);
			}
			if(gamepad->GetRawButton(4))
			{
				Intake->Set(1.0);
			}
			else
			{
				Intake->Set(0.0);
			}
			if(gamepad->GetRawButton(BBUTTON))
			{
				Winch->Set(-1.0);
			}
			else
			{
				Winch->Set(0.0);
			}
			if(stick[RIGHT]->GetRawButton(2))
			{
				Pistons[2]->Set(DoubleSolenoid::kForward);
				Pistons[1]->Set(DoubleSolenoid::kReverse);
			}
			else
			{
				Pistons[2]->Set(DoubleSolenoid::kReverse);
				Pistons[1]->Set(DoubleSolenoid::kForward);
			}
			if(stick[LEFT]->GetRawButton(2))
			{
				Pistons[0]->Set(DoubleSolenoid::kForward);
			}
			else
			{
				Pistons[0]->Set(DoubleSolenoid::kReverse);
			}
			if(gamepad->GetRawButton(5))
			{
				ButtonReleased[0] = true;
			}
			else if(!gamepad->GetRawButton(5)&&ButtonReleased[0])
			{
				this->Debug();
				ButtonReleased[0] = false;
			}
			if(gamepad->GetRawButton(6))
			{
				ButtonReleased[1] = true;
			}
			else if(!gamepad->GetRawButton(6)&&ButtonReleased[1])
			{
				this->EncoderReset();
				this->GyroReset();
				ButtonReleased[1] = false;
			}

			if(stick[RIGHT]->GetRawButton(4))
			{
				ButtonReleased[2] = true;
			}
			else if(!stick[RIGHT]->GetRawButton(4)&&ButtonReleased[2])
			{
				this->PneumaticController(ButtonReleased[2], "open");
				ButtonReleased[2] = false;
			}
			if(stick[LEFT]->GetRawButton(4))
			{
				ButtonReleased[3] = true;
			}
			else if(!stick[LEFT]->GetRawButton(4)&&ButtonReleased[3])
			{
				this->PneumaticController(ButtonReleased[3], "close");
				ButtonReleased[3] = false;
			}

			frc::Wait(0.005);
		}

	}
};

START_ROBOT_CLASS(Robot)
