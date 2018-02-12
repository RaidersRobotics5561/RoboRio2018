/**

 * Example demonstrating the velocity closed-loop servo.

 * Tested with Logitech F350 USB Gamepad inserted into Driver Station]

 *

 * Be sure to select the correct feedback sensor using SetFeedbackDevice() below.

 *

 * After deploying/debugging this to your RIO, first use the left Y-stick

 * to throttle the Talon manually.  This will confirm your hardware setup.

 * Be sure to confirm that when the Talon is driving forward (green) the

 * position sensor is moving in a positive direction.  If this is not the cause

 * flip the boolean input to the SetSensorDirection() call below.

 *

 * Once you've ensured your feedback device is in-phase with the motor,

 * use the button shortcuts to servo to target velocity.

 *

 * Tweak the PID gains accordingly.

 */
using namespace std;
#include <vector>
#include <stdlib.h>// for using for_each
#include <iostream>
#include "WPILib.h"
#include "Joystick.h"
#include "LiveWindow.h"
#include "ctre/Phoenix.h"
#include "const.h"
#include "ADXRS450_Gyro.h"

double DesiredSpeed(double axis, double *DesiredSpeedPrev);
double LagFilter(double FilterGain, double SpeedRaw, double SpeedFiltPrev);
double Errors(double DesiredSpeed, double CurrentSpeed, double *ErrorPrev,
		double *intergal, double kp, double ki, double kd, double upperlimit,
		double lowerlimit);

class Act {
public:

	typedef enum {
		E_StateOff, E_StateForward, E_StateRotate, E_StateEnd
	} T_State;

	T_State Command = E_StateOff;
	bool initialized = false;
	bool complete = false;
	double distanceToTravel = 0;
	double distanceTraveled = 0;
	double gyroStart = 0;

	Act(T_State cmd, double distance) {
		Command = cmd;
		distanceToTravel = distance;
	}
};

class Robot: public IterativeRobot {

	frc::SendableChooser<std::string> V_AutonOption;
	const std::string C_AutonOpt0 = "Controller";
	const std::string C_AutonOpt1 = "GO";
	const std::string C_AutonOpt2 = "Auton";
	std::string V_AutonSelected;

	frc::SendableChooser<std::string> V_StartingPosition;
	const std::string C_StartOpt0 = "Left";
	const std::string C_StartOpt1 = "Middle";
	const std::string C_StartOpt2 = "Right";
	std::string V_StartOptSelected;

private:
	//left Back, SRX:left Front #1
	TalonSRX * _talon0 = new TalonSRX(1);
	//left Front, SRX:left Back #2
	TalonSRX * _talon1 = new TalonSRX(2);
	//right Front, SRX:Right Left #3
	TalonSRX * _talon2 = new TalonSRX(3);
	//right Back, SRX:Right Right #4
	TalonSRX * _talon3 = new TalonSRX(4);

	//intake motor one
	Talon *Talon_PWM0 = new Talon(8);
	//intake motor two
	Talon *Talon_PWM1 = new Talon(9);
	LiveWindow *V_lw;
	//_talon0->SetInverted(true);

	ADXRS450_Gyro Gyro;

	Preferences *Prefs;

	Joystick * _joy = new Joystick(0);

	int kSlotIdx = 0;
	int kPIDLoopIdx = 0;
	int kTimeoutMs = 10;
	double TargetOutput[2];
	double LY_Axis;
	double RX_Axis;
	double GyroAngle;

	double Rt;
	double Lt;

	double IntergalL = 0;
	double IntergalR = 0;
	double SetSpeed = 0;
	double DesiredSpeedPrev = 0;
	double ErrorPrev_L = 0;
	double ErrorPrev_R = 0;
	int rotations = 0;

	std::string gameData;
	//T_State V_DriveState;

	//std::list<Action> ActionList;
	//Action ActionList[2];
	std::vector<Act> ActList;

	double ControlP[2];
	double ControlI[2];

	double SpeedRaw[2], SpeedFilt[2], SpeedFiltPrev[2], desiredSpeed[2],
			output[2], position[2], revolutions[2], distance[2];

	void RobotInit() {

//		Prefs->PutDouble("input1");
		Prefs = Preferences::GetInstance();
		ActList.clear();
		ActList.push_back(Act(Act::T_State::E_StateForward, 10));
		ActList.push_back(Act(Act::T_State::E_StateForward, 100));
		ActList.push_back(Act(Act::T_State::E_StateRotate, 180));

		Gyro.Calibrate();

		V_AutonOption.AddDefault(C_AutonOpt0, C_AutonOpt0);
		V_AutonOption.AddObject(C_AutonOpt1, C_AutonOpt1);
		V_AutonOption.AddObject(C_AutonOpt2, C_AutonOpt2);
		frc::SmartDashboard::PutData("Auto Modes", &V_AutonOption);

		V_StartingPosition.AddDefault(C_StartOpt0, C_StartOpt0);
		V_StartingPosition.AddObject(C_StartOpt1, C_StartOpt1);
		V_StartingPosition.AddObject(C_StartOpt2, C_StartOpt2);
		frc::SmartDashboard::PutData("Starting Position", &V_StartingPosition);

		_talon0->ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

		_talon3->ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

		_talon0->SetSensorPhase(true);

		/* set the peak and nominal outputs, 12V means full */

		_talon0->ConfigNominalOutputForward(0, kTimeoutMs);
		_talon1->ConfigNominalOutputForward(0, kTimeoutMs);
		_talon2->ConfigNominalOutputForward(0, kTimeoutMs);
		_talon3->ConfigNominalOutputForward(0, kTimeoutMs);

		_talon0->ConfigNominalOutputReverse(0, kTimeoutMs);
		_talon1->ConfigNominalOutputReverse(0, kTimeoutMs);
		_talon2->ConfigNominalOutputReverse(0, kTimeoutMs);
		_talon3->ConfigNominalOutputReverse(0, kTimeoutMs);

		_talon0->ConfigPeakOutputForward(1, kTimeoutMs);
		_talon1->ConfigPeakOutputForward(1, kTimeoutMs);
		_talon2->ConfigPeakOutputForward(1, kTimeoutMs);
		_talon3->ConfigPeakOutputForward(1, kTimeoutMs);

		_talon0->ConfigPeakOutputReverse(-1, kTimeoutMs);
		_talon1->ConfigPeakOutputReverse(-1, kTimeoutMs);
		_talon2->ConfigPeakOutputReverse(-1, kTimeoutMs);
		_talon3->ConfigPeakOutputReverse(-1, kTimeoutMs);

	}

	void Act_Forward(Act *a) {
		if (a->initialized == false) {
			distance[0] = 0;
			a->initialized = true;
		}

		//do forward command
		double L_speed = 0.25;
		_talon0->Set(ControlMode::PercentOutput, L_speed);
		_talon1->Set(ControlMode::PercentOutput, L_speed);

		_talon2->Set(ControlMode::PercentOutput, L_speed * -1);
		_talon3->Set(ControlMode::PercentOutput, L_speed * -1);

		//log how much is complete

		a->distanceTraveled = distance[0];
		//a->distanceToTravel = 60;

		//mark movement as complete
		if (a->distanceToTravel <= a->distanceTraveled) {
			a->complete = true;
			L_speed = 0;
			_talon0->Set(ControlMode::PercentOutput, L_speed);
			_talon1->Set(ControlMode::PercentOutput, L_speed);

			_talon2->Set(ControlMode::PercentOutput, L_speed * -1);
			_talon3->Set(ControlMode::PercentOutput, L_speed * -1);
		}
	}

	void Act_Rotate(Act *a) {
		if (a->initialized == false) {
			distance[0] = 0;
			a->gyroStart = GyroAngle;
			a->initialized = true;
		}

		double L_speed = 0.5;


		_talon0->Set(ControlMode::PercentOutput, L_speed);
		_talon1->Set(ControlMode::PercentOutput, L_speed);

		_talon2->Set(ControlMode::PercentOutput, L_speed);
		_talon3->Set(ControlMode::PercentOutput, L_speed);

		double gyroEnd = a->gyroStart + a->distanceToTravel;
		if (gyroEnd > 180){
			gyroEnd -= 360;
		}

		if(a->distanceToTravel > 0) //turn right
		{
			//Start here
			//if anagle < 180
		}
		else //turn left
		{

		}

		if(gyroEnd < 180){
			double L_speed = 0;
			_talon0->Set(ControlMode::PercentOutput, L_speed);
			_talon1->Set(ControlMode::PercentOutput, L_speed);

			_talon2->Set(ControlMode::PercentOutput, L_speed);
			_talon3->Set(ControlMode::PercentOutput, L_speed);
		}
	}

	void TeleopPeriodic() {

		rotations = 0;

//		double input1 = Prefs->GetDouble("input1",0);
		SetSpeed = Prefs->GetDouble("SetSpeed", 1.0);
		ControlP[0] = Prefs->GetDouble("Left P",
				C_WheelSpeedPID_Gain[E_RobotSideLeft][E_PID_Proportional]);
		ControlP[1] = Prefs->GetDouble("Right P",
				C_WheelSpeedPID_Gain[E_RobotSideRight][E_PID_Proportional]);

		ControlI[0] = Prefs->GetDouble("Left I",
				C_WheelSpeedPID_Gain[E_RobotSideLeft][E_PID_Integral]);
		ControlI[1] = Prefs->GetDouble("Right I",
				C_WheelSpeedPID_Gain[E_RobotSideRight][E_PID_Integral]);

		TargetOutput[0] = 0;
		TargetOutput[1] = 0;

		//Reset Sensor Position
		_talon0->SetSelectedSensorPosition(0, kSlotIdx, kTimeoutMs);
		_talon3->SetSelectedSensorPosition(0, kSlotIdx, kTimeoutMs);

		//V_DriveState = E_StateOff;

		while (IsOperatorControl() && IsEnabled()) {
			LY_Axis = _joy->GetRawAxis(1) * -1;
			RX_Axis = _joy->GetRawAxis(5);
			Rt = _joy->GetRawAxis(3);
			Lt = _joy->GetRawAxis(2);

			position[0] = _talon0->GetSelectedSensorPosition(kPIDLoopIdx)
					/ 12.75;
			position[1] = _talon3->GetSelectedSensorPosition(kPIDLoopIdx)
					/ 12.75;

			revolutions[0] = position[0] / C_WheelPulsetoRev[0];
			revolutions[1] = position[1] / C_WheelPulsetoRev[1];

			distance[0] = revolutions[0] * C_PI * C_WheelDiameter[0];
			distance[1] = revolutions[1] * C_PI * C_WheelDiameter[1];

			V_AutonSelected = V_AutonOption.GetSelected();
			V_StartOptSelected = V_StartingPosition.GetSelected();

			if (LY_Axis > 0.01 && LY_Axis < -0.01) {
				LY_Axis = 0;
			}

			if (RX_Axis > 0.01 && RX_Axis < -0.01) {
				RX_Axis = 0;
			}

			gameData =
					frc::DriverStation::GetInstance().GetGameSpecificMessage();

			if (gameData.length() > 0) {
				if (gameData[0] > 'L') {

				}
			}

			GyroAngle = Gyro.GetAngle();

			SpeedRaw[0] = _talon0->GetSelectedSensorVelocity(kPIDLoopIdx)
					/ 12.75;
			SpeedRaw[1] = _talon3->GetSelectedSensorVelocity(kPIDLoopIdx)
					/ 12.75;

			SpeedFilt[0] = LagFilter(C_WheelSpeedLagFilterGain[E_RobotSideLeft],
					SpeedRaw[0], SpeedFiltPrev[0]);
			SpeedFilt[1] = LagFilter(
					C_WheelSpeedLagFilterGain[E_RobotSideRight], SpeedRaw[1],
					SpeedFiltPrev[1]);

			SpeedFiltPrev[0] = SpeedFilt[0];
			SpeedFiltPrev[1] = SpeedFilt[1];

			desiredSpeed[0] = DesiredSpeed(SetSpeed, &DesiredSpeedPrev);
			desiredSpeed[1] = DesiredSpeed(SetSpeed, &DesiredSpeedPrev);

			output[0] =
					Errors(desiredSpeed[0], SpeedFilt[0], &ErrorPrev_L,
							&IntergalL, ControlP[0], ControlI[0],
							C_WheelSpeedPID_Gain[E_RobotSideLeft][E_PID_Derivative],
							C_WheelspeedIntergalLimit[E_RobotSideLeft][E_IntergalUpperLimit],
							C_WheelspeedIntergalLimit[E_RobotSideLeft][E_IntergalLowerLimit]);

			output[1] =
					Errors(desiredSpeed[1], SpeedFilt[1], &ErrorPrev_R,
							&IntergalR, ControlP[1], ControlI[1],
							C_WheelSpeedPID_Gain[E_RobotSideRight][E_PID_Derivative],
							C_WheelspeedIntergalLimit[E_RobotSideRight][E_IntergalUpperLimit],
							C_WheelspeedIntergalLimit[E_RobotSideRight][E_IntergalLowerLimit]);

			if (Rt > 0.25) {
				//Talon_PWM0->Set(ControlMode::PercentOutput, Rt);
				Talon_PWM0->Set(Rt * 0.5);
				//Talon_PWM1->Set(ControlMode::PercentOutput, -1 * Rt);
				Talon_PWM1->Set(Rt * -0.5);
			} else if (Lt > .025) {
				//Talon_PWM0->Set(ControlMode::PercentOutput, -1 * Lt);
				Talon_PWM0->Set(Lt * -0.5);
				//Talon_PWM1->Set(ControlMode::PercentOutput, Lt);
				Talon_PWM1->Set(Lt * 0.5);
			}

			if (V_AutonSelected == C_AutonOpt0) {
				_talon0->Set(ControlMode::PercentOutput, LY_Axis * -1);
				_talon1->Set(ControlMode::PercentOutput, LY_Axis * -1);

				_talon2->Set(ControlMode::PercentOutput, RX_Axis);
				_talon3->Set(ControlMode::PercentOutput, RX_Axis);
			} else if (V_AutonSelected == C_AutonOpt2) {
			}
//				switch(V_DriveState){
//				case E_StateOff:
//					V_DriveState = E_StateForward;
//					break;
//				case E_StateForward:
//					if (distance[0] < 60){
//					TargetOutput[0] = output[0];
//					} else {
//						TargetOutput[0] = 0;
//					}
//
//					if (distance[1] < 60){
//					TargetOutput[1] = output[1];
//					} else {
//						TargetOutput[1] = 0;
//					}
//
//					if (distance[0] >= 60 && distance[1] >= 60){
//						TargetOutput[0] = 0;
//						TargetOutput[1] = 0;
//						V_DriveState = E_StateRotate;
//					}
//
//					break;
//				case E_StateRotate:
//					TargetOutput[0] = output[0] * 1.25;
//					TargetOutput[1] = output[1] * -1.25;
//
//					if (GyroAngle >= 180){
//						TargetOutput[0] = 0;
//						TargetOutput[1] = 0;
//
//						if (rotations >= 1){
//							V_DriveState = E_StateEnd;
//						} else {
//							rotations += 1;
//							V_DriveState = E_StateForward;
//						}
//					}
//					break;
//				case E_StateEnd:
//				default:
//					break;
//				}
//
//			for(Action &a : ActionList)
//			{
//				if(a.complete == false){
//					//call the appropriate action function here
//					if(a.Command == E_StateForward)
//					{
//						Act_Forward(&a);
//					}
//				}
//				else if(a.Command == E_StateEnd){
//					break;
//				}
//			}

			for (int index = 0; (unsigned) index < ActList.size(); ++index) {
				if (ActList[index].complete == false) {
					//call the appropriate action function here
					if (ActList[index].Command
							== Act::T_State::E_StateForward) {
						Act_Forward(&ActList[index]);
					} else if (ActList[index].Command
							== Act::T_State::E_StateRotate) {
						Act_Rotate(&ActList[index]);
					}
					break;
				}
			}
//			_talon0->Set(ControlMode::PercentOutput, 0.25);
//			_talon1->Set(ControlMode::PercentOutput, 0.25);
//
//			_talon2->Set(ControlMode::PercentOutput, 0.25 * -1);
//			_talon3->Set(ControlMode::PercentOutput, 0.25 * -1);

//			_talon0->Set(ControlMode::PercentOutput, TargetOutput[0]);
//			_talon1->Set(ControlMode::PercentOutput, TargetOutput[0]);
//
//			_talon2->Set(ControlMode::PercentOutput, TargetOutput[1] * -1);
//		    _talon3->Set(ControlMode::PercentOutput, TargetOutput[1] * -1);
			Wait(C_ExeTime);

			SmartDashboard::PutNumber("Velocity 0",
					_talon0->GetSelectedSensorVelocity(kPIDLoopIdx) / 12.75);
			SmartDashboard::PutNumber("Velocity 1",
					_talon3->GetSelectedSensorVelocity(kPIDLoopIdx) / 12.75);
			SmartDashboard::PutNumber("Position 0",
					_talon3->GetSelectedSensorPosition(kPIDLoopIdx) / 12.75);
			SmartDashboard::PutNumber("GyroAngle", GyroAngle);
			SmartDashboard::PutNumber("LY_Axis", LY_Axis);
			SmartDashboard::PutNumber("SpeedFilt", SpeedFilt[0]);
			SmartDashboard::PutNumber("SpeedRaw", SpeedRaw[0]);
			SmartDashboard::PutNumber("desiredSpeed", desiredSpeed[0]);
			SmartDashboard::PutNumber("Error", desiredSpeed[0] - SpeedFilt[0]);
			SmartDashboard::PutNumber("Output%", output[0]);
			SmartDashboard::PutNumber("Output%1", output[1]);
			SmartDashboard::PutNumber("Distance Left", distance[0]);
			SmartDashboard::PutNumber("Distance Other Left", distance[1]);
		}

	}
};

double DesiredSpeed(double axis, double *DesiredSpeedPrev) {
	double a = axis * C_speedGain;
	double b = LagFilter(C_SpeedFilterGain, a, *DesiredSpeedPrev);
	*DesiredSpeedPrev = b;
	return b;
}

double Errors(double DesiredSpeed, double CurrentSpeed, double *ErrorPrev,
		double *intergal, double kp, double ki, double kd, double upperlimit,
		double lowerlimit) {
	double Error = DesiredSpeed - CurrentSpeed;
	double der = Error - *ErrorPrev;
	//PID
	double P = Error * kp;
	double I = *intergal + Error * ki;
	double D = kd * der / C_ExeTime;

	*ErrorPrev = Error;

	if (I > upperlimit) {
		I = upperlimit;
	} else if (I < lowerlimit) {
		I = lowerlimit;
	}

	*intergal = I;

	double output = P + I + D;
	if (output > 0.25) {
		output = 0.25;
	} else if (output < -0.25) {
		output = -0.25;
	}

	return output;
}

double LagFilter(double FilterGain, double SpeedRaw, double SpeedFiltPrev) {
	return FilterGain * SpeedRaw + (1 - FilterGain) * SpeedFiltPrev;
}

START_ROBOT_CLASS(Robot)
