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
#include "LiveWindow.h"
#include "ctre/Phoenix.h"

double DesiredSpeed(int axis, std::string go);
double Errors(double DesiredSpeed, double CurrentSpeed, double c);

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

	LiveWindow *V_lw;
	//_talon0->SetInverted(true);

	Joystick * _joy = new Joystick(0);

	int kSlotIdx = 0;
	int kPIDLoopIdx = 0;
	int kTimeoutMs = 10;
	double TargetSpeed = 0;
	double LY_Axis;
	double RX_Axis;
	void RobotInit() {

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

		SmartDashboard::PutNumber("Velocity 0",
				_talon0->GetSelectedSensorVelocity(kPIDLoopIdx) / 12.75);
		SmartDashboard::PutNumber("Velocity 1",
				_talon3->GetSelectedSensorVelocity(kPIDLoopIdx) / 12.75);
		SmartDashboard::PutNumber("Position 0",
				_talon3->GetSelectedSensorPosition(kPIDLoopIdx) / 12.75);

		V_AutonSelected = V_AutonOption.GetSelected();

		if (V_AutonSelected == C_AutonOpt1) {
			LY_Axis = 0.15;
			RX_Axis = 0.15;
		} else {
			LY_Axis = _joy->GetRawAxis(1);
			RX_Axis = _joy->GetRawAxis(5);
		}
		SmartDashboard::PutNumber("LY_Axis", LY_Axis);

		double speed = DesiredSpeed(LY_Axis, V_AutonSelected);
		double output = Errors(speed,
				_talon0->GetSelectedSensorVelocity(kPIDLoopIdx) / 12.75,0.1);

		double speed1 = DesiredSpeed(RX_Axis, V_AutonSelected);
				double output1 = Errors(speed1,
						_talon3->GetSelectedSensorVelocity(kPIDLoopIdx) / 12.75,0.1);

		//		_talon0->Set(ControlMode::Velocity, TargetSpeed);
		_talon0->Set(ControlMode::PercentOutput, output * -1);
		_talon1->Set(ControlMode::PercentOutput, output * -1);

		_talon2->Set(ControlMode::PercentOutput, RX_Axis);
		_talon3->Set(ControlMode::PercentOutput, RX_Axis);

	}
};

double DesiredSpeed(int axis, std::string go) {
	double gain = 10;

	if (go == "On") {
		return gain * 0.15;
	} else {
		return axis * gain;
	}
}

double Errors(double DesiredSpeed, double CurrentSpeed, double c) {
	return (DesiredSpeed - CurrentSpeed) * c;
}

START_ROBOT_CLASS(Robot)
