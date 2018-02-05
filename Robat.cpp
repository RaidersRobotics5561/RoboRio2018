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

#include "WPILib.h"
#include "Joystick.h"
//#include "LiveWindow.h"
#include "ctre/Phoenix.h"
#include "const.h"
#include "ADXRS450_Gyro.h"

double DesiredSpeed(double axis,double *DesiredSpeedPrev);
double LagFilter(double FilterGain,double SpeedRaw, double SpeedFiltPrev);
double Errors(double DesiredSpeed, double CurrentSpeed, double *intergal,
              double kp, double ki, double kd, double upperlimit, double lowerlimit);

class Robot: public IterativeRobot {

	frc::SendableChooser<std::string> V_AutonOption;
	const std::string C_AutonOpt0 = "Off";
	const std::string C_AutonOpt1 = "On";
	std::string V_AutonSelected;
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
	double TargetSpeed = 0;
	double LY_Axis;
	double RX_Axis;
	double GyroAngle;

	double Rt;
	double Lt;

	double IntergalL = 0;
	double IntergalR = 0;
	double input1 = 0;
	double DesiredSpeedPrev = 0;

	double SpeedRaw[2], SpeedFilt[2], SpeedFiltPrev[2], desiredSpeed[2],
			output[2];

	void RobotInit() {

//		Prefs->PutDouble("input1");
		Prefs = Preferences::GetInstance();

		Gyro.Calibrate();

		V_AutonOption.AddDefault(C_AutonOpt0, C_AutonOpt0);
		V_AutonOption.AddObject(C_AutonOpt1, C_AutonOpt1);
		frc::SmartDashboard::PutData("Auto Modes", &V_AutonOption);

		_talon0->ConfigSelectedFeedbackSensor(
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

	void TeleopPeriodic() {

//		double input1 = Prefs->GetDouble("input1",0);
		input1 = Prefs->GetDouble("input1", 1.0);

		_talon0->SetSelectedSensorPosition(0,kSlotIdx,kTimeoutMs);


		while (IsOperatorControl() && IsEnabled()) {
			LY_Axis = _joy->GetRawAxis(1) * -1;
			RX_Axis = _joy->GetRawAxis(5);
			Rt = _joy->GetRawAxis(3);
			Lt = _joy->GetRawAxis(2);

			V_AutonSelected = V_AutonOption.GetSelected();

			if (LY_Axis > 0.01 && LY_Axis < -0.01) {
				LY_Axis = 0;
			}

			if (RX_Axis > 0.01 && RX_Axis < -0.01) {
				RX_Axis = 0;
			}

			GyroAngle = Gyro.GetAngle();

			SpeedRaw[0] = _talon0->GetSelectedSensorVelocity(kPIDLoopIdx)
					/ 12.75;
			SpeedRaw[1] = _talon3->GetSelectedSensorVelocity(kPIDLoopIdx)
					/ 12.75;

			SpeedFilt[0] = LagFilter(C_FiltGain_L,SpeedRaw[0],SpeedFiltPrev[0]);
			SpeedFilt[1] = LagFilter(C_FiltGain_R,SpeedRaw[1],SpeedFiltPrev[1]);

			SpeedFiltPrev[0] = SpeedFilt[0];
			SpeedFiltPrev[1] = SpeedFilt[1];

			desiredSpeed[0] = DesiredSpeed(input1,&DesiredSpeedPrev);
			desiredSpeed[1] = DesiredSpeed(input1,&DesiredSpeedPrev);

			output[0] = Errors(desiredSpeed[0], SpeedFilt[0], &IntergalL,
					C_ErrorP_L, C_ErrorI_L, C_ErrorD_L, C_IntergalUpperLimit_L,
					C_IntergalLowerLimit_L);

			output[1] = Errors(desiredSpeed[1], SpeedFilt[1], &IntergalR,
					C_ErrorP_R, C_ErrorI_R, C_ErrorD_R, C_IntergalUpperLimit_R,
					C_IntergalLowerLimit_R);

	if(Rt > 0.25){
		//Talon_PWM0->Set(ControlMode::PercentOutput, Rt);
		Talon_PWM0->Set(Rt);
		//Talon_PWM1->Set(ControlMode::PercentOutput, -1 * Rt);
		Talon_PWM1->Set(Rt * -1);
	} else if(Lt > .025){
		//Talon_PWM0->Set(ControlMode::PercentOutput, -1 * Lt);
		Talon_PWM0->Set(Lt * -1);
		//Talon_PWM1->Set(ControlMode::PercentOutput, Lt);
		Talon_PWM1->Set(Lt);
	}

			if (V_AutonSelected == "On") {
				_talon0->Set(ControlMode::PercentOutput, LY_Axis * -1);
				_talon1->Set(ControlMode::PercentOutput, LY_Axis * -1);

				_talon2->Set(ControlMode::PercentOutput, RX_Axis);
				_talon3->Set(ControlMode::PercentOutput, RX_Axis);
			} else {
				_talon0->Set(ControlMode::PercentOutput, output[0]);
				_talon1->Set(ControlMode::PercentOutput, output[0]);

				_talon2->Set(ControlMode::PercentOutput, output[1] * -1);
				_talon3->Set(ControlMode::PercentOutput, output[1] * -1);
			}
			Wait(0.01);

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
		}

	}
};

double DesiredSpeed(double axis,double *DesiredSpeedPrev) {
	double a = axis * C_speedGain;
	double b = LagFilter(C_SpeedFilterGain,a,*DesiredSpeedPrev);
    *DesiredSpeedPrev = b;
	return b;
}

double Errors(double DesiredSpeed, double CurrentSpeed, double *intergal,
		double kp, double ki, double kd, double upperlimit, double lowerlimit) {
	double Error = DesiredSpeed - CurrentSpeed;
	double P = Error * kp;
	double I = *intergal + Error * ki;

	if (I > upperlimit) {
		I = upperlimit;
	} else if (I < lowerlimit) {
		I = lowerlimit;
	}

	*intergal = I;

	double output = P + I;
	if (output > 0.5) {
		output = 0.5;
	} else if (output < -0.5) {
		output = -0.5;
	}

	return output;
}

double LagFilter(double FilterGain,double SpeedRaw, double SpeedFiltPrev){
	return FilterGain * SpeedRaw + (1 - FilterGain) * SpeedFiltPrev;
}

START_ROBOT_CLASS(Robot)
