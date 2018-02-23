#include "const.h"
#include "LookUp.hpp"
#include "Calibrations.hpp"
#include "Control_PID.hpp"
#include "VariableInit.hpp"
#include "Output_SmartDashboard.hpp"
#include "SignalFilter.hpp"
#include "Vars.hpp"
#include "AnalogTriggerType.h"
#include "Output_Actuators.hpp"
#include "Input_Controller.hpp"
#include "Control_LED.h"

double DesiredSpeed(double axis);

double DesiredLiftHeight(double L_JoystickAxis, double L_DesiredLiftHeightPrev,
		double L_MaxHeight);

double LukeStoppers(double L_DesiredSpeed, double L_CurrentSpeed,
		double L_RampRate);

double LiftCmdDisable(double LiftHight, double CommandedHight, double MinHight,
		double L_MotorCmnd);

double V_EndGameWinchTime;
bool V_LED_RainbowLatch;

double V_HookPosition;
double V_HookPositionErrorPrev;
double V_HookPositionErrorIntegral;
double V_HookLiftHeightDesired;
double V_HookPID_Gain[E_PID_Sz];

double V_IntakePosition;
double V_IntakePositionErrorPrev;
double V_IntakePositionErrorIntegral;
double V_IntakeLiftHeightDesired;
double V_IntakePID_Gain[E_PID_Sz];
double V_IntakePositionPrev;

double V_IntakeArmPulseToRev[E_ArmCmndSz];

double V_WheelRPM_Filt[E_RobotSideSz];
double V_WheelRPM_FiltPrev[E_RobotSideSz];
double V_WheelRPM_Desired[E_RobotSideSz];
double V_WheelSpeedErrorPrev[E_RobotSideSz];
double V_WheelSpeedErrorIntegral[E_RobotSideSz];
double V_WheelRPM_Raw[E_RobotSideSz];
double V_WheelProportionalGain[E_RobotSideSz];
double V_WheelIntegralGain[E_RobotSideSz];
double V_WheelDerivativeGain[E_RobotSideSz];
double V_WheelSpeedLagFiltGain[E_RobotSideSz];
double V_DistanceTraveled[E_RobotSideSz];
double V_Revolutions[E_RobotSideSz];
double V_LukeStopperRamp;
double V_RotateGain;
double Timer;

RoboState RobatState;
LED_Mode V_LED_Mode;
T_DriveMode DriveMode;

double GyroAngle;
double input1;
double V_ArmAngleDeg;

double V_RobotUserCmndPct[E_RobotUserCmndSz]; // This is the requested value from the driver for the various motors/functions
double V_RobotMotorCmndPct[E_RobotMotorSz];

class Act {
public:
	typedef enum {
		E_StateOff, E_StateForward, E_StateRotate, E_StateLiftArm, E_StateEnd
	} T_State;

	T_State Command = E_StateOff;
	bool initialized = false;
	bool complete = false;
	bool motorComplete = false;
	double distanceStarting = 0;
	double distanceToTravel = 0;
	double distanceTraveled = 0;
	double gyroStart = 0;
	double gyroEnd = 0;
	double liftamount = 0;
	double StartTime = 0;
	int rotateDirection = 1;
	int gyroStartRot = 0;

	Act(T_State cmd, double target) {

		Command = cmd;
		if (cmd == Act::T_State::E_StateForward) {
			distanceToTravel = target;
		} else if (cmd == Act::T_State::E_StateRotate) {
			gyroEnd = target;
		} else if (cmd == Act::T_State::E_StateLiftArm) {
			liftamount = target;
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

	frc::SendableChooser<std::string> V_AutonSwitchSelect;
	const std::string C_SwitchOpt0 = "Tall";
	const std::string C_SwitchOpt1 = "Short";
	std::string V_AutonSwitchOptSelected;

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
	Spark *Spark1 = new Spark(0);
	// Left intake
	Spark *Spark2 = new Spark(1);
	//Right intake
	Talon *Talon_PWM0 = new Talon(2);
	//Right articulation
	Talon *Talon_PWM1 = new Talon(3);
	//Winch
	Talon *Talon_PWM2 = new Talon(4);

//	AnalogTrigger *mAnalogTrigger(4); // create an encoder pulse trigger
	AnalogTrigger *mAnalogTrigger = new AnalogTrigger(4); // create an encoder pulse trigger
//	mAnalogTrigger = new AnalogTrigger(4); // create an encoder pulse trigger
	Counter* mCounter; // count the encoder pulse triggers in current direction
//	Counter *mCounter = new Counter(4);

	DigitalOutput *V_LED_State0 = new DigitalOutput(0);
	DigitalOutput *V_LED_State1 = new DigitalOutput(1);
	DigitalOutput *V_LED_State2 = new DigitalOutput(2);
	DigitalOutput *V_LED_State3 = new DigitalOutput(3);

	ADXRS450_Gyro Gyro;

	Preferences *Prefs;

	Joystick *_joy1 = new Joystick(0);
	Joystick *_joy2 = new Joystick(1);

	std::string gameData;

	bool setActList = false;
	std::vector<Act> ActList, CurrentList;

	// Start, End, Switch Pos
	// Left,Small,Left (LSL)
	std::vector<Act> LSL, LSR, LLL, LLR, RSL, RSR, RLL, RLR, MSL, MSR, MLL, MLR;

	/******************************************************************************
	 * Function:     RobotInit
	 *
	 * Description:
	 *
	 ******************************************************************************/
	void RobotInit() {

		CameraServer::GetInstance()->StartAutomaticCapture(0);
		Prefs = Preferences::GetInstance();

		mAnalogTrigger->SetLimitsVoltage(3.5, 3.8); // values higher than the highest minimum (pulse floor), lower than the lowest maximum (pulse ceiling)
		mCounter = new Counter(4);
		RobatState = C_Disabled;

		VariableInit(Prefs, mCounter);

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

		_talon2->ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

		_talon3->ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

		_talon4->ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

		_talon5->ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

		_talon0->SetSensorPhase(true);
		_talon2->SetSensorPhase(true);
		_talon3->SetSensorPhase(true);
		_talon4->SetSensorPhase(true);
		_talon5->SetSensorPhase(true);

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
//			_talon0->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);
//			_talon3->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);
//			V_DistanceTraveled[E_RobotSideLeft] = 0;
			a->distanceStarting = (V_DistanceTraveled[E_RobotSideLeft]
					+ V_DistanceTraveled[E_RobotSideRight]) / 2;
			a->initialized = true;
		}
		//do forward command
		double L_rSpeed = 0.35;
		double L_lSpeed = 0.42;
		//log how much is complete
		a->distanceTraveled = (V_DistanceTraveled[E_RobotSideLeft]
				+ V_DistanceTraveled[E_RobotSideRight]) / 2
				- a->distanceStarting;

		//SmartDashboard::PutNumber("distanceTraveled", a->distanceTraveled);

		//a->distanceToTravel = 60;
		//mark movement as complete

		if (a->distanceToTravel <= a->distanceTraveled) {
			a->complete = true;
			L_rSpeed = 0;
			L_lSpeed = 0;
		}

		_talon0->Set(ControlMode::PercentOutput, L_lSpeed);
		_talon1->Set(ControlMode::PercentOutput, L_lSpeed);

		_talon2->Set(ControlMode::PercentOutput, L_rSpeed * -1);
		_talon3->Set(ControlMode::PercentOutput, L_rSpeed * -1);
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

	void Act_OutputCube(Act *a) {
		if(!a->initialized){
			a->StartTime = Timer;
			a->initialized = true;
		}
		T_ArmCmnd L_ArmAnglePrev = E_ArmCmndOff;
		T_ArmCmnd L_ArmAnglePrevPrev = E_ArmCmndOff;

		Read_Sensors(_talon0, _talon2, _talon4, _talon5, mCounter, &Gyro,
				&V_ArmAngleDeg, L_ArmAnglePrev, L_ArmAnglePrevPrev);

		if (a->motorComplete) {
			V_RobotMotorCmndPct[E_RobotMotorLift] = Control_PID(
					0, V_IntakePosition,
					&V_IntakePositionErrorPrev, &V_IntakePositionErrorIntegral,
					V_IntakePID_Gain[E_PID_Proportional],
					V_IntakePID_Gain[E_PID_Integral],
					V_IntakePID_Gain[E_PID_Derivative],
					K_Intake_PID_Limit[E_PID_Proportional],
					-K_Intake_PID_Limit[E_PID_Proportional],
					K_Intake_PID_Limit[E_PID_Integral],
					-K_Intake_PID_Limit[E_PID_Integral],
					K_Intake_PID_Limit[E_PID_Derivative],
					-K_Intake_PID_Limit[E_PID_Derivative], K_IntakeCmndLimit,
					-K_IntakeCmndLimit);

			V_RobotMotorCmndPct[E_RobotMotorLift] = LiftCmdDisable(
					V_IntakePosition, V_IntakeLiftHeightDesired,
					K_IntakeMinCmndHeight,
					V_RobotMotorCmndPct[E_RobotMotorLift]);
		} else {

			V_RobotMotorCmndPct[E_RobotMotorLift] = Control_PID(
					V_IntakeLiftHeightDesired, V_IntakePosition,
					&V_IntakePositionErrorPrev, &V_IntakePositionErrorIntegral,
					V_IntakePID_Gain[E_PID_Proportional],
					V_IntakePID_Gain[E_PID_Integral],
					V_IntakePID_Gain[E_PID_Derivative],
					K_Intake_PID_Limit[E_PID_Proportional],
					-K_Intake_PID_Limit[E_PID_Proportional],
					K_Intake_PID_Limit[E_PID_Integral],
					-K_Intake_PID_Limit[E_PID_Integral],
					K_Intake_PID_Limit[E_PID_Derivative],
					-K_Intake_PID_Limit[E_PID_Derivative], K_IntakeCmndLimit,
					-K_IntakeCmndLimit);

			V_RobotMotorCmndPct[E_RobotMotorLift] = LiftCmdDisable(
					V_IntakePosition, V_IntakeLiftHeightDesired,
					K_IntakeMinCmndHeight,
					V_RobotMotorCmndPct[E_RobotMotorLift]);
		}

		if (V_IntakePosition < a->liftamount + 1
				&& V_IntakePosition > a->liftamount - 1) {
			double currentTime = Timer - a->StartTime;

				//Run Rollers to output Cube
				if(currentTime > 1){
					a->motorComplete = true;
				}
		}
	}

	void Act_Rotate(Act *a) {

		double modAngle = 360;
		//double L_GAngle = GyroAngle;
		double L_posGAngle = GyroAngle;
		while (L_posGAngle < 0) {
			L_posGAngle += 360;
		}

		double L_modA = fmod(L_posGAngle, modAngle);
		int L_rotA = floor(GyroAngle / 360);
		double angleDiff = 180 - abs(abs(L_modA - a->gyroEnd) - 180); //Find diff and wrap
		double angleDiffPlus = 180 - abs(abs(L_modA + 1 - a->gyroEnd) - 180); //Calc the effect of adding
		double angleDiffMinus = 180 - abs(abs(L_modA - 1 - a->gyroEnd) - 180); //Calc the effect of subtracting
		int currentBestDirection = 1;
		if (angleDiffMinus < angleDiff) {
			currentBestDirection = -1;
		}
		if (a->initialized == false) {
			a->gyroStart = L_modA;
			a->gyroStartRot = L_rotA;

			if (angleDiffPlus < angleDiff) {
				//Rotate Pos
			} else if (angleDiffMinus < angleDiff) {
				//Rotate Negative
				a->rotateDirection = -1;
			}

			a->initialized = true;
		}

		double L_rSpeed = 0.4 * a->rotateDirection;
		double L_lSpeed = 0.45 * a->rotateDirection;

		double CurrentAngle = L_modA;

//		if (a->rotateDirection == 1 && (CurrentAngle >= a->gyroEnd || L_rotA > a->gyroStartRot)) {
//			a->complete = true;
//			L_lSpeed = 0;
//			L_rSpeed = 0;
//		}
//
//		if (a->rotateDirection == -1 && (CurrentAngle <= a->gyroEnd || L_rotA < a->gyroStartRot)) {
//			a->complete = true;
//			L_lSpeed = 0;
//			L_rSpeed = 0;
//		}

		if (currentBestDirection != a->rotateDirection) {
			a->complete = true;
			L_lSpeed = 0;
			L_rSpeed = 0;
		}

		_talon0->Set(ControlMode::PercentOutput, L_lSpeed);
		_talon1->Set(ControlMode::PercentOutput, L_lSpeed);

		_talon2->Set(ControlMode::PercentOutput, L_rSpeed);
		_talon3->Set(ControlMode::PercentOutput, L_rSpeed);

	}

	/******************************************************************************
	 * Function:     TeleopPeriodic
	 *
	 * Description:  RoboRio call during the teleop periodic period.
	 ******************************************************************************/
	void TeleopPeriodic() {
		T_ArmCmnd L_ArmAnglePrev = E_ArmCmndOff;
		T_ArmCmnd L_ArmAnglePrevPrev = E_ArmCmndOff;

		VariableInit(Prefs, mCounter);

		//Reset Sensor Position
		_talon0->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);
		_talon2->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);
		_talon3->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);
		_talon4->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);
		_talon5->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);

		RobatState = C_Teleop;

		while (IsOperatorControl() && IsEnabled()) {
			V_AutonSelected = V_AutonOption.GetSelected();
			V_StartOptSelected = V_StartingPosition.GetSelected();

			DtrmnControllerMapping(_joy1, _joy2);

			/* Read all of the available sensors */
			Read_Sensors(_talon0, _talon2, _talon4, _talon5, mCounter, &Gyro,
					&V_ArmAngleDeg, L_ArmAnglePrev, L_ArmAnglePrevPrev);

			V_WheelRPM_Desired[E_RobotSideLeft] = DesiredSpeed(
					V_RobotUserCmndPct[E_RobotUserCmndLeftWheel]);

			V_WheelRPM_Desired[E_RobotSideLeft] = LukeStoppers(
					V_WheelRPM_Desired[E_RobotSideLeft],
					V_WheelRPM_Filt[E_RobotSideLeft], V_LukeStopperRamp);

			V_WheelRPM_Desired[E_RobotSideRight] = DesiredSpeed(
					V_RobotUserCmndPct[E_RobotUserCmndRightWheel]);

			V_WheelRPM_Desired[E_RobotSideRight] = LukeStoppers(
					V_WheelRPM_Desired[E_RobotSideRight],
					V_WheelRPM_Filt[E_RobotSideRight], V_LukeStopperRamp);

			V_RobotMotorCmndPct[E_RobotMotorLeftWheel] =
					Control_PID(V_WheelRPM_Desired[E_RobotSideLeft],
							V_WheelRPM_Filt[E_RobotSideLeft],
							&V_WheelSpeedErrorPrev[E_RobotSideLeft],
							&V_WheelSpeedErrorIntegral[E_RobotSideLeft],
							V_WheelProportionalGain[E_RobotSideLeft],
							V_WheelIntegralGain[E_RobotSideLeft],
							V_WheelDerivativeGain[E_RobotSideLeft],
							C_WheelspeedProportionalLimit[E_RobotSideLeft][E_IntergalUpperLimit],
							C_WheelspeedProportionalLimit[E_RobotSideLeft][E_IntergalLowerLimit],
							C_WheelspeedIntergalLimit[E_RobotSideLeft][E_IntergalUpperLimit],
							C_WheelspeedIntergalLimit[E_RobotSideLeft][E_IntergalLowerLimit],
							C_WheelspeedDerivativeLimit[E_RobotSideLeft][E_IntergalUpperLimit],
							C_WheelspeedDerivativeLimit[E_RobotSideLeft][E_IntergalLowerLimit],
							C_WheelspeedCmndLimit[E_RobotSideLeft][E_IntergalUpperLimit],
							C_WheelspeedCmndLimit[E_RobotSideLeft][E_IntergalLowerLimit]);

			V_RobotMotorCmndPct[E_RobotMotorRightWheel] =
					Control_PID(V_WheelRPM_Desired[E_RobotSideRight],
							V_WheelRPM_Filt[E_RobotSideRight],
							&V_WheelSpeedErrorPrev[E_RobotSideRight],
							&V_WheelSpeedErrorIntegral[E_RobotSideRight],
							V_WheelProportionalGain[E_RobotSideRight],
							V_WheelIntegralGain[E_RobotSideRight],
							V_WheelDerivativeGain[E_RobotSideRight],
							C_WheelspeedProportionalLimit[E_RobotSideRight][E_IntergalUpperLimit],
							C_WheelspeedProportionalLimit[E_RobotSideRight][E_IntergalLowerLimit],
							C_WheelspeedIntergalLimit[E_RobotSideRight][E_IntergalUpperLimit],
							C_WheelspeedIntergalLimit[E_RobotSideRight][E_IntergalLowerLimit],
							C_WheelspeedDerivativeLimit[E_RobotSideRight][E_IntergalUpperLimit],
							C_WheelspeedDerivativeLimit[E_RobotSideRight][E_IntergalLowerLimit],
							C_WheelspeedCmndLimit[E_RobotSideRight][E_IntergalUpperLimit],
							C_WheelspeedCmndLimit[E_RobotSideRight][E_IntergalLowerLimit]);

			V_IntakeLiftHeightDesired = DesiredLiftHeight(
					V_RobotUserCmndPct[E_RobotUserCmndLift],
					V_IntakeLiftHeightDesired, K_MaxIntakeLiftHeight);

			V_RobotMotorCmndPct[E_RobotMotorLift] = Control_PID(
					V_IntakeLiftHeightDesired, V_IntakePosition,
					&V_IntakePositionErrorPrev, &V_IntakePositionErrorIntegral,
					V_IntakePID_Gain[E_PID_Proportional],
					V_IntakePID_Gain[E_PID_Integral],
					V_IntakePID_Gain[E_PID_Derivative],
					K_Intake_PID_Limit[E_PID_Proportional],
					-K_Intake_PID_Limit[E_PID_Proportional],
					K_Intake_PID_Limit[E_PID_Integral],
					-K_Intake_PID_Limit[E_PID_Integral],
					K_Intake_PID_Limit[E_PID_Derivative],
					-K_Intake_PID_Limit[E_PID_Derivative], K_IntakeCmndLimit,
					-K_IntakeCmndLimit);

			V_RobotMotorCmndPct[E_RobotMotorLift] = LiftCmdDisable(
					V_IntakePosition, V_IntakeLiftHeightDesired,
					K_IntakeMinCmndHeight,
					V_RobotMotorCmndPct[E_RobotMotorLift]);

			V_RobotMotorCmndPct[E_RobotMotorWinch] =
					V_RobotUserCmndPct[E_RobotUserCmndWinch]; // climb direction

			V_HookLiftHeightDesired = DesiredLiftHeight(
					V_RobotUserCmndPct[E_RobotUserCmndHook],
					V_HookLiftHeightDesired, K_MaxHookLiftHeight);

			V_RobotMotorCmndPct[E_RobotMotorHook] = Control_PID(
					V_HookLiftHeightDesired, V_HookPosition,
					&V_HookPositionErrorPrev, &V_HookPositionErrorIntegral,
					V_HookPID_Gain[E_PID_Proportional],
					V_HookPID_Gain[E_PID_Integral],
					V_HookPID_Gain[E_PID_Derivative],
					K_Hook_PID_Limit[E_PID_Proportional],
					-K_Hook_PID_Limit[E_PID_Proportional],
					K_Hook_PID_Limit[E_PID_Integral],
					-K_Hook_PID_Limit[E_PID_Integral],
					K_Hook_PID_Limit[E_PID_Derivative],
					-K_Hook_PID_Limit[E_PID_Derivative], K_HookCmndLimit,
					-K_HookCmndLimit);

			V_RobotMotorCmndPct[E_RobotMotorIntakeArmAng] =
					V_RobotUserCmndPct[E_RobotUserCmndIntakeArmAng];

			V_RobotMotorCmndPct[E_RobotMotorIntakeRoller] =
					V_RobotUserCmndPct[E_RobotUserCmndIntakeRoller];

			L_ArmAnglePrevPrev = L_ArmAnglePrev;
			L_ArmAnglePrev = E_ArmCmndOff;

			if (V_RobotMotorCmndPct[E_RobotMotorIntakeArmAng] > 0) {
				L_ArmAnglePrev = E_ArmCmndUp;
			} else if (V_RobotMotorCmndPct[E_RobotMotorIntakeArmAng] < 0) {
				L_ArmAnglePrev = E_ArmCmndDwn;
			}

			V_LED_Mode = UpdateLED_Output(RobatState, false,
					V_RobotMotorCmndPct[E_RobotMotorWinch], V_LED_State0,
					V_LED_State1, V_LED_State2, V_LED_State3);

			UpdateActuatorCmnds(_talon0, _talon1, _talon2, _talon3, _talon4,
					_talon5, Spark1, Spark2, Talon_PWM0, Talon_PWM1, Talon_PWM2,
					V_RobotMotorCmndPct);

			UpdateSmartDashboad();

			Wait(C_ExeTime);
		}
	}

	/******************************************************************************
	 * Function:     AutonomousInit
	 *
	 * Description: This method is called once each time the robot enters Autonomous
	 *
	 ******************************************************************************/
	void AutonomousInit() {

		V_StartOptSelected = V_StartingPosition.GetSelected();

		_talon0->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);
		_talon3->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);

//		List_StartLeft.clear();
//		List_StartLeft.push_back(Act(Act::T_State::E_StateForward, 24));
//		List_StartLeft.push_back(Act(Act::T_State::E_StateRotate, 180));

		ActList.clear();
		ActList.push_back(Act(Act::T_State::E_StateForward, 24));
		ActList.push_back(Act(Act::T_State::E_StateRotate, 180));
//		ActList.push_back(Act(Act::T_State::E_StateForward, 24));
//		ActList.push_back(Act(Act::T_State::E_StateRotate, 10));
	}

	/******************************************************************************
	 * Function:     AutonomousPeriodic
	 *
	 * Description:
	 *
	 ******************************************************************************/
	void AutonomousPeriodic() {
		while (IsAutonomous() && IsEnabled()) {

			V_WheelRPM_Raw[E_RobotSideLeft] =
					_talon0->GetSelectedSensorPosition(K_PIDLoopIdx) / 12.75;
			V_WheelRPM_Raw[E_RobotSideRight] =
					(_talon3->GetSelectedSensorPosition(K_PIDLoopIdx) / 12.75)
							* -1;

			V_Revolutions[E_RobotSideLeft] = V_WheelRPM_Raw[E_RobotSideLeft]
					/ C_WheelPulsetoRev[0];

			V_Revolutions[E_RobotSideRight] = V_WheelRPM_Raw[E_RobotSideRight]
					/ C_WheelPulsetoRev[1];

			V_DistanceTraveled[E_RobotSideLeft] = V_Revolutions[E_RobotSideLeft]
					* C_PI * C_WheelDiameter[E_RobotSideLeft];
			V_DistanceTraveled[E_RobotSideRight] =
					V_Revolutions[E_RobotSideRight] * C_PI
							* C_WheelDiameter[E_RobotSideRight];

			GyroAngle = Gyro.GetAngle();

			gameData =
					frc::DriverStation::GetInstance().GetGameSpecificMessage();

			if (V_AutonSwitchOptSelected == C_SwitchOpt0) {
				if (gameData[1] == 'R') {
					if (V_StartOptSelected == C_StartOpt0) {
						CurrentList = LLR;
					} else if (V_StartOptSelected == C_StartOpt1) {
						CurrentList = MLR;
					} else if (V_StartOptSelected == C_StartOpt2) {
						CurrentList = RLR;
					}
				} else {
					if (V_StartOptSelected == C_StartOpt0) {
						CurrentList = LLL;
					} else if (V_StartOptSelected == C_StartOpt1) {
						CurrentList = MLL;
					} else if (V_StartOptSelected == C_StartOpt2) {
						CurrentList = RLL;
					}
				}
			} else {
				if (gameData[0] == 'R') {
					if (V_StartOptSelected == C_StartOpt0) {
						CurrentList = LSR;
					} else if (V_StartOptSelected == C_StartOpt1) {
						CurrentList = MSR;
					} else if (V_StartOptSelected == C_StartOpt2) {
						CurrentList = RSR;
					}
				} else {
					if (V_StartOptSelected == C_StartOpt0) {
						CurrentList = LSL;
					} else if (V_StartOptSelected == C_StartOpt1) {
						CurrentList = MSL;
					} else if (V_StartOptSelected == C_StartOpt2) {
						CurrentList = RSL;
					}
				}
			}
			for (int index = 0; (unsigned) index < CurrentList.size();
					++index) {
				if (CurrentList[index].complete == false) {
					//call the appropriate action function here
					if (CurrentList[index].Command
							== Act::T_State::E_StateForward) {
						Act_Forward(&CurrentList[index]);
					} else if (CurrentList[index].Command
							== Act::T_State::E_StateRotate) {
						Act_Rotate(&CurrentList[index]);
					} else if (CurrentList[index].Command
							== Act::T_State::E_StateLiftArm) {
						Act_OutputCube(&CurrentList[index]);
					}
					break;
				}
			}
			SmartDashboard::PutBoolean("CurrentList[0].initialized",
					CurrentList[0].initialized);
			SmartDashboard::PutBoolean("CurrentList[0].Complete",
					CurrentList[0].complete);
			SmartDashboard::PutNumber("CurrentList[0].distanceToTravel",
					CurrentList[0].distanceToTravel);
			SmartDashboard::PutNumber("CurrentList[0].distanceTraveled",
					CurrentList[0].distanceTraveled);
			SmartDashboard::PutBoolean("CurrentList[1].initialized",
					CurrentList[1].initialized);
			SmartDashboard::PutBoolean("CurrentList[1].Complete",
					CurrentList[1].complete);
			SmartDashboard::PutNumber("CurrentList[1].gyroStart",
					CurrentList[1].gyroStart);
			SmartDashboard::PutNumber("CurrentList[1].gyroEnd",
					CurrentList[1].gyroEnd);
			SmartDashboard::PutNumber("CurrentList[1].gyroStartRot",
					CurrentList[1].gyroStartRot);
			SmartDashboard::PutNumber("CurrentList[1].rotateDirection",
					ActList[1].rotateDirection);
			SmartDashboard::PutBoolean("CurrentList[2].initialized",
					CurrentList[2].initialized);
			SmartDashboard::PutBoolean("CurrentList[2].Complete",
					CurrentList[2].complete);
			SmartDashboard::PutNumber("CurrentList[2].distanceToTravel",
					CurrentList[2].distanceToTravel);
			SmartDashboard::PutNumber("CurrentList[2].distanceTraveled",
					CurrentList[2].distanceTraveled);
//			SmartDashboard::PutBoolean("CurrentList[3].initialized", CurrentList[3].initialized);
//			SmartDashboard::PutBoolean("CurrentList[3].Complete", CurrentList[3].complete);
//			SmartDashboard::PutNumber("CurrentList[3].gyroStart", CurrentList[3].gyroStart);
//			SmartDashboard::PutNumber("CurrentList[3].gyroEnd", CurrentList[3].gyroEnd);
//			SmartDashboard::PutNumber("CurrentList[3].gyroStartRot", CurrentList[3].gyroStartRot);
//			SmartDashboard::PutNumber("CurrentList[3].rotateDirection", CurrentList[3].rotateDirection);

			double modAngle = 360;
			double L_posGAngle = GyroAngle;
			while (L_posGAngle < 0) {
				L_posGAngle += 360;
			}

			double L_modA = fmod(L_posGAngle, modAngle);
			int L_rotA = floor(GyroAngle / 360);

			SmartDashboard::PutNumber("GyroAngle2", GyroAngle);
			SmartDashboard::PutNumber("V_DistanceTraveled[E_RobotSideLeft]",
					V_DistanceTraveled[E_RobotSideLeft]);
			SmartDashboard::PutNumber("L_posGAngle", L_posGAngle);
			SmartDashboard::PutNumber("L_modA", L_modA);
			SmartDashboard::PutNumber("L_rotA", L_rotA);
			Wait(C_ExeTime);
			Timer += C_ExeTime;
		}

	}
};

START_ROBOT_CLASS(Robot)
