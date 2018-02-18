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

#include "const.h"
#include "LookUp.hpp"
#include "Calibrations.hpp"
#include "Control_PID.hpp"
#include "VariableInit.hpp"
#include "Output_SmartDashboard.hpp"
#include "SignalFilter.hpp"

double DesiredSpeed(double axis);
double DesiredSpeedSlow(double axis);


double V_EndGameWinchTime;
bool   V_LED_RainbowLatch;
int    V_AutonState;
bool   V_LED_CmndState[4];
double V_WheelSpeedErrorPrev[E_RobotSideSz];
double V_WheelSpeedErrorIntegral[E_RobotSideSz];
double V_WheelRPM_Raw[E_RobotSideSz];
double V_WheelRPM_Filt[E_RobotSideSz];
double V_WheelRPM_FiltPrev[E_RobotSideSz];
double V_WheelRPM_Desired[E_RobotSideSz];
double V_WheelMotorCmndPct[E_RobotSideSz];
double V_ProportionalGain[E_RobotSideSz];
double V_IntegralGain[E_RobotSideSz];
double V_DerivativeGain[E_RobotSideSz];
double V_Actuators[C_ActuatorsSz];
double V_DistanceTraveled[E_RobotSideSz];
double V_Revolutions[E_RobotSideSz];
double V_WheelSpeedLagFiltGain[E_RobotSideSz];
double LY_Axis;
double RX_Axis;
double GyroAngle;

double Rt;
double Lt;

double input1;
double V_WinchSpeed;
double V_ArmAngleDeg;

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
	double gyroEnd = 0;
	int rotateDirection = 1;
	Act(T_State cmd, double target) {

		Command = cmd;
		if (cmd == Act::T_State::E_StateForward) {
			distanceToTravel = target;
		} else if (cmd == Act::T_State::E_StateRotate) {
			gyroEnd = target;
		}
	}
};

class Robot: public IterativeRobot {

	frc::SendableChooser<std::string> V_AutonOption;
	const std::string C_AutonOpt0 = "Off";
	const std::string C_AutonOpt1 = "On";
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
	//Intake #5
	TalonSRX * _talon4 = new TalonSRX(5);
	//Hook delivery #6
	TalonSRX * _talon5 = new TalonSRX(6);
	// Left articulation
	frc::Spark Spark1 { 0 };
	// Left intake
	frc::Spark Spark2 { 1 };
	//Right intake
	Talon *Talon_PWM0 = new Talon(2);
	//Right articulation
	Talon *Talon_PWM1 = new Talon(3);
	//Winch
	Talon *Talon_PWM2 = new Talon(4);

	AnalogTrigger * mAnalogTrigger = new AnalogTrigger(0); // create an encoder pulse trigger
	Counter* mCounter; // count the encoder pulse triggers in current direction
	float mSpeedPrevious; // to remember previous direction
	int mPosition;

	DigitalOutput *V_LED_State0 = new DigitalOutput(0);
	DigitalOutput *V_LED_State1 = new DigitalOutput(1);
	DigitalOutput *V_LED_State2 = new DigitalOutput(2);
	DigitalOutput *V_LED_State3 = new DigitalOutput(3);

	ADXRS450_Gyro Gyro;

	Preferences *Prefs;

	Joystick * _joy1 = new Joystick(0);
	Joystick * _joy2 = new Joystick(1);

	std::string gameData;

	std::vector<Act> ActList, CurrentList, List_StartLeft, List_StartRight, List_StartMiddle;

	void RobotInit() {

  CameraServer::GetInstance()->StartAutomaticCapture(0);
		Prefs = Preferences::GetInstance();

		VariableInit(Prefs);

		mAnalogTrigger->SetLimitsVoltage(3.5, 3.8); // values higher than the highest minimum (pulse floor), lower than the lowest maximum (pulse ceiling)
		mCounter = new Counter(0);
		mSpeedPrevious = 0.;
		mPosition = 0;

		V_LED_State0->Set(false);
		V_LED_State1->Set(false);
		V_LED_State2->Set(false);
		V_LED_State3->Set(false);

		Gyro.Calibrate();

		V_AutonOption.AddDefault(C_AutonOpt0, C_AutonOpt0);
		V_AutonOption.AddObject(C_AutonOpt1, C_AutonOpt1);
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
		_talon3->SetSensorPhase(true);

		/* set the peak and nominal outputs, 12V means full */

		_talon0->ConfigNominalOutputForward(0, K_TimeoutMs);
		_talon1->ConfigNominalOutputForward(0, K_TimeoutMs);
		_talon2->ConfigNominalOutputForward(0, K_TimeoutMs);
		_talon3->ConfigNominalOutputForward(0, K_TimeoutMs);
		_talon4->ConfigNominalOutputForward(0, K_TimeoutMs);
		_talon5->ConfigNominalOutputForward(0, K_TimeoutMs);

		_talon0->ConfigNominalOutputReverse(0, K_TimeoutMs);
		_talon1->ConfigNominalOutputReverse(0, K_TimeoutMs);
		_talon2->ConfigNominalOutputReverse(0, K_TimeoutMs);
		_talon3->ConfigNominalOutputReverse(0, K_TimeoutMs);
		_talon4->ConfigNominalOutputReverse(0, K_TimeoutMs);
		_talon5->ConfigNominalOutputReverse(0, K_TimeoutMs);

		_talon0->ConfigPeakOutputForward(1, K_TimeoutMs);
		_talon1->ConfigPeakOutputForward(1, K_TimeoutMs);
		_talon2->ConfigPeakOutputForward(1, K_TimeoutMs);
		_talon3->ConfigPeakOutputForward(1, K_TimeoutMs);
		_talon4->ConfigPeakOutputForward(1, K_TimeoutMs);
		_talon5->ConfigPeakOutputForward(1, K_TimeoutMs);

		_talon0->ConfigPeakOutputReverse(-1, K_TimeoutMs);
		_talon1->ConfigPeakOutputReverse(-1, K_TimeoutMs);
		_talon2->ConfigPeakOutputReverse(-1, K_TimeoutMs);
		_talon3->ConfigPeakOutputReverse(-1, K_TimeoutMs);
		_talon4->ConfigPeakOutputReverse(-1, K_TimeoutMs);
		_talon5->ConfigPeakOutputReverse(-1, K_TimeoutMs);

	}

	void Act_Forward(Act *a) {
		if (a->initialized == false) {
			V_DistanceTraveled[E_RobotSideLeft] = 0;
			a->initialized = true;
		}
		//do forward command
		double L_speed = 0.25;

		_talon0->Set(ControlMode::PercentOutput, L_speed);
		_talon1->Set(ControlMode::PercentOutput, L_speed);

		_talon2->Set(ControlMode::PercentOutput, L_speed * -1);
		_talon3->Set(ControlMode::PercentOutput, L_speed * -1);

		//log how much is complete
		a->distanceTraveled = V_DistanceTraveled[E_RobotSideLeft];

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

//	void Act_Rotate(Act *a) {
//				if (a->initialized == false) {
//					a->gyroStart = GyroAngle;
//					a->initialized = true;
//				}
//
//				double L_speed = 0.25;
//
////				a->distanceToTravel =
//				//Starting 12
//				double angleOffset = 15;
//				double angleTarget = 0;
//				bool RotatePositive;
//
//				double angleDiff = 180-abs(abs(GyroAngle - a->gyroEnd)-180); //Find diff and wrap
//				double angleDiffPlus = 180-abs(abs(GyroAngle + L_speed - a->gyroEnd) - 180); //Calc the effect of adding
//				double angleDiffMinus = 180-abs(abs(GyroAngle - L_speed - a->gyroEnd) - 180);  //Calc the effect of subtracting
//
//				if(angleDiffPlus < angleDiff){
//					//Rotate Pos
//					angleTarget = a->gyroEnd + angleOffset;
//					RotatePositive = true;
//				} else if(angleDiffMinus < angleDiff) {
//					//Rotate Negative
//					angleTarget = a->gyroEnd - angleOffset;
//					L_speed *= -1;
//					RotatePositive = false;
//				}
//
//				bool flippedAngle = false;
//				if(angleTarget > 180)
//				{
//					flippedAngle = true;
//					angleTarget -= 360;
//				}
//				else if(angleTarget < -180)
//				{
//					flippedAngle = true;
//					angleTarget += 360;
//				}
//
//				if((RotatePositive and flippedAngle == false) or (RotatePositive == false and flippedAngle == true))
//				{
//					if((GyroAngle >= a->gyroEnd and flippedAngle == false) or (GyroAngle <= angleTarget and flippedAngle == true))
//					{
//						L_speed = 0;
//						a->complete = true;
//					}
//				}
//				else
//				{
//					if((GyroAngle <= a->gyroEnd and flippedAngle == false) or (GyroAngle >= angleTarget and flippedAngle == true))
//					{
//						L_speed = 0;
//						a->complete = true;
//					}
//				}
//
//				_talon0->Set(ControlMode::PercentOutput, L_speed);
//				_talon1->Set(ControlMode::PercentOutput, L_speed);
//
//				_talon2->Set(ControlMode::PercentOutput, L_speed);
//				_talon3->Set(ControlMode::PercentOutput, L_speed);
//
//			}

	void Act_Rotate(Act *a) {
		double modAngle = 360;
		if (a->initialized == false) {
			//a->gyroStart = GyroAngle % 360;
			a->gyroStart = modf(GyroAngle, &modAngle);
			if (a->gyroStart < a->gyroEnd) {
				a->rotateDirection = -1;
			}

			a->initialized = true;
		}

		double L_speed = 0.25 * a->rotateDirection;

		double CurrentAngle = modf(GyroAngle, &modAngle);

		if (a->rotateDirection == 1 && CurrentAngle >= a->gyroEnd) {
			a->complete = true;
			L_speed = 0;
		}

		if (a->rotateDirection == -1 && CurrentAngle <= a->gyroEnd) {
			a->complete = true;
			L_speed = 0;

		}

		_talon0->Set(ControlMode::PercentOutput, L_speed);
		_talon1->Set(ControlMode::PercentOutput, L_speed);

		_talon2->Set(ControlMode::PercentOutput, L_speed);
		_talon3->Set(ControlMode::PercentOutput, L_speed);

	}

	void TeleopPeriodic() {
		T_RobotSide L_RobotSide;
		double L_Forward = 0;
		double L_Rotate = 0;
		bool L_Tank = false;
		bool L_PressBtn = false;
		bool L_Slow = false;
		bool L_PressBtn2 = false;
        int  liftAng = 0.0;

		VariableInit(Prefs);

		//Reset Sensor Position
		_talon0->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);
		_talon2->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);
		_talon3->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);

		while (IsOperatorControl() && IsEnabled()) {
			Rt = _joy1->GetRawAxis(3);
			Lt = _joy1->GetRawAxis(2);

			V_AutonSelected = V_AutonOption.GetSelected();
			V_StartOptSelected = V_StartingPosition.GetSelected();

			gameData =
					frc::DriverStation::GetInstance().GetGameSpecificMessage();

			if (gameData.length() > 0) {
//				if(gameData[0] > 'L')
//				{
//
//				}
			}



			GyroAngle = Gyro.GetAngle();

      if (_joy1->GetRawButtonPressed(1) == true && L_PressBtn == false)
        {
        L_Tank = !L_Tank;
        L_PressBtn = true;
        }
      else if (_joy1->GetRawButtonPressed(1) == false)
        {
        L_PressBtn = false;
        }

      if (_joy1->GetRawButtonPressed(2) == true && L_PressBtn2 == false)
        {
        L_Slow = !L_Slow;
        L_PressBtn2 = true;
        }
      else if (_joy1->GetRawButtonPressed(2) == false)
        {
        L_PressBtn2 = false;
        }

      /* Ok, let's first read the wheel speeds: */
       V_WheelRPM_Raw[E_RobotSideLeft]  = _talon0->GetSelectedSensorVelocity(K_PIDLoopIdx) / 12.75;

       V_WheelRPM_Raw[E_RobotSideRight] = _talon2->GetSelectedSensorVelocity(K_PIDLoopIdx) / 12.75;


       int L_ArmEncoderCount;
       /* Great, now lets read the counts from the arm encoder.  Remember, the encoder will change the rate at which it
        * increases counts base on direction.... Also, this is a non directional counter...*/
       L_ArmEncoderCount = mCounter->Get();

      if (L_Tank == true)
        {
        LY_Axis = _joy1->GetRawAxis(4);
        RX_Axis = (_joy1->GetRawAxis(1)) * -1;
        if (L_Slow == true)
          {
          L_Forward = DesiredSpeedSlow(RX_Axis);
          L_Rotate  = DesiredSpeedSlow(LY_Axis);
          }
        else
          {
          L_Forward = DesiredSpeed(RX_Axis);
          L_Rotate  = DesiredSpeed(LY_Axis);
          }
        V_WheelRPM_Desired[E_RobotSideLeft] = L_Forward + L_Rotate;
        V_WheelRPM_Desired[E_RobotSideRight] = L_Forward - L_Rotate;
        }
      else
        {
        RX_Axis = (_joy1->GetRawAxis(5)) * -1;
        LY_Axis = (_joy1->GetRawAxis(1)) * -1;
        if (L_Slow == true)
          {
          V_WheelRPM_Desired[E_RobotSideLeft] = DesiredSpeedSlow(LY_Axis);
          V_WheelRPM_Desired[E_RobotSideRight] = DesiredSpeedSlow(RX_Axis);
          }
        else
          {
          V_WheelRPM_Desired[E_RobotSideLeft] = DesiredSpeed(LY_Axis);
          V_WheelRPM_Desired[E_RobotSideRight] = DesiredSpeed(RX_Axis);
          }
        }

			V_DistanceTraveled[E_RobotSideLeft] = V_Revolutions[E_RobotSideLeft] * C_PI * C_WheelDiameter[E_RobotSideLeft];
			V_DistanceTraveled[E_RobotSideRight] = V_Revolutions[E_RobotSideRight] * C_PI * C_WheelDiameter[E_RobotSideRight];

	     for (L_RobotSide = E_RobotSideLeft; L_RobotSide < E_RobotSideSz;
	          L_RobotSide = T_RobotSide(int(L_RobotSide) + 1))
	       {
//	      V_WheelRPM_Desired[L_RobotSide] = input1;

         V_WheelRPM_Filt[L_RobotSide] = LagFilter(V_WheelSpeedLagFiltGain[L_RobotSide],
                                                  V_WheelRPM_Raw[L_RobotSide],
                                                  V_WheelRPM_FiltPrev[L_RobotSide]);

         V_WheelRPM_FiltPrev[L_RobotSide] = V_WheelRPM_Filt[L_RobotSide];

         V_Revolutions[L_RobotSide] = V_WheelRPM_Raw[L_RobotSide] / C_WheelPulsetoRev[L_RobotSide];

	        V_WheelMotorCmndPct[L_RobotSide] = Control_PID(V_WheelRPM_Desired[L_RobotSide],
	                                                       V_WheelRPM_Filt[L_RobotSide],
	                                                       &V_WheelSpeedErrorPrev[L_RobotSide],
	                                                       &V_WheelSpeedErrorIntegral[L_RobotSide],
	                                                       V_ProportionalGain[L_RobotSide],
	                                                       V_IntegralGain[L_RobotSide],
	                                                       V_DerivativeGain[L_RobotSide],
	                                                       C_WheelspeedProportionalLimit[L_RobotSide][E_IntergalUpperLimit],
	                                                       C_WheelspeedProportionalLimit[L_RobotSide][E_IntergalLowerLimit],
	                                                       C_WheelspeedIntergalLimit[L_RobotSide][E_IntergalUpperLimit],
	                                                       C_WheelspeedIntergalLimit[L_RobotSide][E_IntergalLowerLimit],
	                                                       C_WheelspeedDerivativeLimit[L_RobotSide][E_IntergalUpperLimit],
	                                                       C_WheelspeedDerivativeLimit[L_RobotSide][E_IntergalLowerLimit],
	                                                       C_WheelspeedCmndLimit[L_RobotSide][E_IntergalUpperLimit],
	                                                       C_WheelspeedCmndLimit[L_RobotSide][E_IntergalLowerLimit]);
	       }

	     liftAng = mCounter->Get();

	      double Lift = 0.0;

	      if (_joy2->GetRawAxis(3) > 0.1)
	        {
	        Lift = _joy2->GetRawAxis(3);
	        }
	      else if (_joy2->GetRawAxis(2) > 0.1)
	        {
	        Lift = -_joy2->GetRawAxis(2);
	        }

	      double winch = 0.0;
	      if (_joy2->GetRawButton(1) == true)
	        {
	        winch = 0.5; // climb direction
	        }
	      else if (_joy2->GetRawButton(2) == true)
	        {
	        winch = -0.5;
	        }

	      double hook = 0.0;
	      if (_joy2->GetRawButton(3) == true)
	        {
	        hook = 0.3; // climb direction
	        }
	      else if (_joy2->GetRawButton(4) == true)
	        {
	        hook = -0.3;
	        }

	      double L_IntakeRoller = 0;
	      double L_IntakeAngle = 0;

	      if (_joy2->GetRawAxis(5) > 0.1 || _joy2->GetRawAxis(5) < -0.1)
	        {
	        L_IntakeAngle = _joy2->GetRawAxis(5);
	        }

	      if (_joy2->GetRawAxis(1) > 0.1 || _joy2->GetRawAxis(1) < -0.1)
	        {
	        L_IntakeRoller = _joy2->GetRawAxis(1);
	        }

			_talon0->Set(ControlMode::PercentOutput,
					V_WheelMotorCmndPct[E_RobotSideLeft]);
			_talon1->Set(ControlMode::PercentOutput,
					V_WheelMotorCmndPct[E_RobotSideLeft]);
			_talon2->Set(ControlMode::PercentOutput,
					(V_WheelMotorCmndPct[E_RobotSideRight] * -1));
			_talon3->Set(ControlMode::PercentOutput,
					(V_WheelMotorCmndPct[E_RobotSideRight] * -1));
      _talon4->Set(ControlMode::PercentOutput, (Lift));
      _talon5->Set(ControlMode::PercentOutput, (hook));
      Spark1.Set(L_IntakeAngle); // Intake angle
      Spark2.Set(L_IntakeAngle); // Intake angle

      Talon_PWM0->Set(L_IntakeRoller); // Intake roller
      Talon_PWM1->Set(-L_IntakeRoller); // Intake roller
			Talon_PWM2->Set(winch);

			SmartDashboard::PutNumber("Count", (double)liftAng);
			UpdateSmartDashboad();

			Wait(C_ExeTime);
		}

	}

	void AutonomousInit() //This method is called once each time the robot enters Autonomous
	{
		ActList.clear();
		ActList.push_back(Act(Act::T_State::E_StateForward, 24));
		ActList.push_back(Act(Act::T_State::E_StateRotate, 180));
		ActList.push_back(Act(Act::T_State::E_StateForward, 24));
		ActList.push_back(Act(Act::T_State::E_StateRotate, 0));

		List_StartLeft.clear();
		List_StartLeft.push_back(Act(Act::T_State::E_StateForward, 24));
		List_StartLeft.push_back(Act(Act::T_State::E_StateRotate, 180));

		List_StartRight.clear();
		List_StartRight.push_back(Act(Act::T_State::E_StateForward, 24));
		List_StartRight.push_back(Act(Act::T_State::E_StateRotate, 180));
	}

	void AutonomousPeriodic() {
		while (IsAutonomous() && IsEnabled()) {

			switch (V_StartingPosition) {
			case C_StartOpt0: //Left
				CurrentList = List_StartLeft;
				break;
			case C_StartOpt1: //Middle
				CurrentList = List_StartMiddle;
				break;
			case C_StartOpt2: //Right
				CurrentList = List_StartRight;
				break;
			default:
				CurrentList = ActList;
		}

		for (int index = 0; (unsigned) index < CurrentList.size(); ++index) {
			if (CurrentList[index].complete == false) {
				//call the appropriate action function here
				if (CurrentList[index].Command == Act::T_State::E_StateForward) {
					Act_Forward(&CurrentList[index]);
				} else if (CurrentList[index].Command
						== Act::T_State::E_StateRotate) {
					Act_Rotate(&CurrentList[index]);
				}
				break;
			}
		}
	}
}
};

double DesiredSpeedSlow(double L_JoystickAxis) {
  double L_DesiredDriveSpeed = 0.0;
  int L_AxisSize = (int)(sizeof(K_DesiredDriveSpeedAxis) / sizeof(K_DesiredDriveSpeedAxis[0]));
  int L_CalArraySize = (int)(sizeof(K_DesiredDriveSpeedSlow) / sizeof(K_DesiredDriveSpeedSlow[0]));

  L_DesiredDriveSpeed = LookUp1D_Table(&K_DesiredDriveSpeedAxis[0],
                                       &K_DesiredDriveSpeedSlow[0],
                                       L_AxisSize,
                                       L_CalArraySize,
                                       L_JoystickAxis);

	return L_DesiredDriveSpeed;
}

double DesiredSpeed(double L_JoystickAxis) {
	double L_DesiredDriveSpeed = 0.0;
	int L_AxisSize = (int)(sizeof(K_DesiredDriveSpeedAxis) / sizeof(K_DesiredDriveSpeedAxis[0]));
	int L_CalArraySize = (int)(sizeof(K_DesiredDriveSpeed) / sizeof(K_DesiredDriveSpeed[0]));

	L_DesiredDriveSpeed = LookUp1D_Table(&K_DesiredDriveSpeedAxis[0],
	                                     &K_DesiredDriveSpeed[0],
	                                     L_AxisSize,
	                                     L_CalArraySize,
	                                     L_JoystickAxis);

	return L_DesiredDriveSpeed;
}

START_ROBOT_CLASS(Robot)
