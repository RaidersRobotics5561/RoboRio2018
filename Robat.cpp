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
double LY_Axis;
double RX_Axis;
double GyroAngle;

double Rt;
double Lt;

double input1;
double V_WinchSpeed;

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


  //Winch, SRX:
//  TalonSRX * _talon4 = new TalonSRX(5);

	// Left articulation
	frc::Spark Spark1{0};
	// Left intake
	frc::Spark Spark2{1};
  //Right intake
  Talon *Talon_PWM0 = new Talon(2);
  //Right articulation
  Talon *Talon_PWM1 = new Talon(3);
  //Winch
  Talon *Talon_PWM2 = new Talon(4);

	DigitalOutput *V_LED_State0 = new DigitalOutput(0);
	DigitalOutput *V_LED_State1 = new DigitalOutput(1);
	DigitalOutput *V_LED_State2 = new DigitalOutput(2);
	DigitalOutput *V_LED_State3 = new DigitalOutput(3);

	ADXRS450_Gyro Gyro;

	Preferences *Prefs;

	Joystick * _joy1 = new Joystick(0);
	Joystick * _joy2 = new Joystick(1);

	std::string gameData;



	void RobotInit() {

    Prefs = Preferences::GetInstance();

    VariableInit(Prefs);

    V_LED_State0->Set(false);
    V_LED_State1->Set(false);
    V_LED_State2->Set(false);
    V_LED_State3->Set(false);

		Gyro.Calibrate();

		V_AutonOption.AddDefault(C_AutonOpt0, C_AutonOpt0);
		V_AutonOption.AddObject(C_AutonOpt1, C_AutonOpt1);
		frc::SmartDashboard::PutData("Auto Modes", &V_AutonOption);

		V_AutonOption.AddDefault(C_StartOpt0, C_StartOpt0);
		V_AutonOption.AddObject(C_StartOpt1, C_StartOpt1);
		V_AutonOption.AddObject(C_StartOpt2, C_StartOpt2);
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
//		_talon4->ConfigNominalOutputForward(0, K_TimeoutMs);

		_talon0->ConfigNominalOutputReverse(0, K_TimeoutMs);
		_talon1->ConfigNominalOutputReverse(0, K_TimeoutMs);
		_talon2->ConfigNominalOutputReverse(0, K_TimeoutMs);
		_talon3->ConfigNominalOutputReverse(0, K_TimeoutMs);
//		_talon4->ConfigNominalOutputReverse(0, K_TimeoutMs);

		_talon0->ConfigPeakOutputForward(1, K_TimeoutMs);
		_talon1->ConfigPeakOutputForward(1, K_TimeoutMs);
		_talon2->ConfigPeakOutputForward(1, K_TimeoutMs);
		_talon3->ConfigPeakOutputForward(1, K_TimeoutMs);
//		_talon4->ConfigPeakOutputForward(1, K_TimeoutMs);

		_talon0->ConfigPeakOutputReverse(-1, K_TimeoutMs);
		_talon1->ConfigPeakOutputReverse(-1, K_TimeoutMs);
		_talon2->ConfigPeakOutputReverse(-1, K_TimeoutMs);
		_talon3->ConfigPeakOutputReverse(-1, K_TimeoutMs);
//		_talon4->ConfigPeakOutputReverse(-1, K_TimeoutMs);
  }

	void TeleopPeriodic() {
	  T_RobotSide L_RobotSide;

	  VariableInit(Prefs);

    //Reset Sensor Position
    _talon0->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);
    _talon3->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);

		while (IsOperatorControl() && IsEnabled()) {
			LY_Axis = _joy1->GetRawAxis(1);
			RX_Axis = _joy1->GetRawAxis(5);
			Rt = _joy1->GetRawAxis(3);
			Lt = _joy1->GetRawAxis(2);

			V_AutonSelected = V_AutonOption.GetSelected();
			V_StartOptSelected = V_StartingPosition.GetSelected();

			gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

			if(gameData.length() > 0){
//				if(gameData[0] > 'L')
//				{
//
//				}
			}

			GyroAngle = Gyro.GetAngle();



//			V_WinchSpeed = _talon4->GetSelectedSensorVelocity(kPIDLoopIdx);
      V_WheelRPM_Raw[E_RobotSideLeft]  = _talon0->GetSelectedSensorVelocity(K_PIDLoopIdx) / 12.75;
      V_WheelRPM_Raw[E_RobotSideRight] = _talon3->GetSelectedSensorVelocity(K_PIDLoopIdx) / 12.75;

			V_WheelRPM_Desired[E_RobotSideLeft]  = DesiredSpeed(LY_Axis);
			V_WheelRPM_Desired[E_RobotSideRight] = DesiredSpeed(RX_Axis);

      for (L_RobotSide = E_RobotSideLeft;
           L_RobotSide < E_RobotSideSz;
           L_RobotSide = T_RobotSide(int(L_RobotSide) + 1))
        {
        V_WheelRPM_Filt[L_RobotSide] = LagFilter(C_WheelSpeedLagFilterGain[L_RobotSide],
                                                 V_WheelRPM_Raw[L_RobotSide],
                                                 V_WheelRPM_FiltPrev[L_RobotSide]);

        V_WheelRPM_FiltPrev[L_RobotSide]  = V_WheelRPM_Filt[L_RobotSide];

        V_WheelRPM_Desired[L_RobotSide] = 20;

        V_WheelMotorCmndPct[L_RobotSide] =  Control_PID(V_WheelRPM_Desired[L_RobotSide],
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

//			if (Rt > 0.25) {
//				//Talon_PWM0->Set(ControlMode::PercentOutput, Rt);
//				Talon_PWM0->Set(Rt);
//				//Talon_PWM1->Set(ControlMode::PercentOutput, -1 * Rt);
//				Talon_PWM1->Set(Rt * -1);
//			} else if (Lt > .025) {
//				//Talon_PWM0->Set(ControlMode::PercentOutput, -1 * Lt);
//				Talon_PWM0->Set(Lt * -1);
//				//Talon_PWM1->Set(ControlMode::PercentOutput, Lt);
//				Talon_PWM1->Set(Lt);
//			}

//			if (V_AutonSelected == "On") {
//				_talon0->Set(ControlMode::PercentOutput, LY_Axis * -1);
//				_talon1->Set(ControlMode::PercentOutput, LY_Axis * -1);
//
//				_talon2->Set(ControlMode::PercentOutput, RX_Axis);
//				_talon3->Set(ControlMode::PercentOutput, RX_Axis);
//			} else {
//				_talon0->Set(ControlMode::PercentOutput, V_WheelMotorCmndPct[0]);
//				_talon1->Set(ControlMode::PercentOutput, V_WheelMotorCmndPct[0]);
//
//				_talon2->Set(ControlMode::PercentOutput, V_WheelMotorCmndPct[1] * -1);
//				_talon3->Set(ControlMode::PercentOutput, V_WheelMotorCmndPct[1] * -1);
//			}
      _talon0->Set(ControlMode::PercentOutput, V_WheelMotorCmndPct[E_RobotSideLeft]);
      _talon1->Set(ControlMode::PercentOutput, V_WheelMotorCmndPct[E_RobotSideLeft]);
      _talon2->Set(ControlMode::PercentOutput, (V_WheelMotorCmndPct[E_RobotSideRight] * -1));
      _talon3->Set(ControlMode::PercentOutput, (V_WheelMotorCmndPct[E_RobotSideRight] * -1));

//      _talon0->Set(ControlMode::PercentOutput, 0.0);
//      _talon1->Set(ControlMode::PercentOutput, 0.0);
//      _talon2->Set(ControlMode::PercentOutput, 0.0);
//      _talon3->Set(ControlMode::PercentOutput, 0.0);

//      _talon0->Set(ControlMode::PercentOutput, LY_Axis * -1);
//      _talon1->Set(ControlMode::PercentOutput, LY_Axis * -1);
//      _talon2->Set(ControlMode::PercentOutput, RX_Axis);
//      _talon3->Set(ControlMode::PercentOutput, RX_Axis);

//      _talon4->Set(ControlMode::PercentOutput, RX_Axis);

      UpdateSmartDashboad();

			Wait(C_ExeTime);
		}

	}
};

double DesiredSpeed(double L_JoystickAxis)
  {
  double L_DesiredDriveSpeed = 0.0;
  int    L_AxisSize = sizeof(K_DesiredDriveSpeedAxis) / sizeof(K_DesiredDriveSpeedAxis[0]);
  int    L_CalArraySize = sizeof(K_DesiredDriveSpeed) / sizeof(K_DesiredDriveSpeed[0]);

  L_DesiredDriveSpeed = LookUp1D_Table(&K_DesiredDriveSpeedAxis[0],
                                       &K_DesiredDriveSpeed[0],
                                       L_AxisSize,
                                       L_CalArraySize,
                                       L_JoystickAxis);

	return L_DesiredDriveSpeed;
  }





START_ROBOT_CLASS(Robot)
