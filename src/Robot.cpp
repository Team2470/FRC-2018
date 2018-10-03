//CPP INCLUDES//
#include <iostream>
#include <string>
#include <cmath>
#include <algorithm>

//FIRST INCLUDES//
#include <Drive/DifferentialDrive.h>
#include <Joystick.h>
#include <SampleRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Spark.h>
#include <DriverStation.h>
#include <Timer.h>
#include <DigitalInput.h>
#include <DigitalOutput.h>
#include <Encoder.h>
#include <PowerDistributionPanel.h>

//OUR INCLUDES//
#include "sonar_helper.hpp"
#include "controller_helper.h"
#include "driver_helper.hpp"
#include "actuator_helper.hpp"
#include "motor_helper.hpp"
#include "ir_helper.hpp"

class Robot2018 : public frc::SampleRobot
{
public:
	Robot2018()
	{
		// Note SmartDashboard is not initialized here, wait until
		// RobotInit to make SmartDashboard calls
		//m_robotDrive.SetExpiration(0.1);
	}

	~Robot2018()
	{
		delete m_leftMotor;
		delete m_rightMotor;
		delete m_winchMotor;
		delete m_placeholderMotor;

		delete m_driveSystem;
		delete m_winchSystem;

		delete m_ultrasonicFrontLeft;
		delete m_ultrasonicFrontRight;
		delete m_ultrasonicBackLeft;
		delete m_ultrasonicBackRight;
		delete m_ultrasonicLift;

		delete m_liftShortRange;
		delete m_liftLongRange;

		//delete m_actuatorMotor;
		delete m_liftMotor;

		delete m_robotLimitSwitch;
		delete m_blockLeftLimitSwitch;
		delete m_blockRightLimitSwitch;

		delete m_plexiglassLightControl;
	}

	void RobotInit()
	{
		//Sendable chooser stuff that we did not use
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

		m_plexiglassLightControl->Set(true);
		robotClock.Start();
		liftEncoder->Reset();
	}

	void Autonomous()
	{
		std::string gameData;
		std::string placement;

		//placement = SmartDashboard::GetString("DB/String 3", "");

		bool autoLeft = SmartDashboard::GetBoolean("DB/Button 0", false);
		bool autoRight = SmartDashboard::GetBoolean("DB/Button 1", false);
		bool switchLeft = SmartDashboard::GetBoolean("DB/Button 2", false);
		bool switchRight = SmartDashboard::GetBoolean("DB/Button 3", false);

		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

		if (gameData.length() > 0)
		{
			if (gameData[0] == 'L')
			{
				//Our color is left on our switch
				if (autoLeft == true)
				{
					//autoPlaceSwitch(Left);
					autoCrossLine();
				}
				else if (autoRight == true)
				{
					autoCrossLine();
				}
				else if (switchLeft == true)
				{
					autoPlaceSwitchStraight(true);
				}
				else if (switchRight == true)
				{
					autoPlaceSwitchStraight(false);
				}
				else
				{
					autoCrossLine();
				}
			}
			else if (gameData[0] == 'R')
			{
				//Our color is right on our switch
				if (autoLeft == true)
				{
					autoCrossLine();
				}
				else if (autoRight == true)
				{
					//autoPlaceSwitch(Right);
					autoCrossLine();
				}
				else if (switchLeft == true)
				{
					autoPlaceSwitchStraight(false);
				}
				else if (switchRight == true)
				{
					autoPlaceSwitchStraight(true);
				}
				else
				{
					autoCrossLine();
				}
			}
		}
	}

	void OperatorControl() override
	{
		//variables yeah
		int liftRevolutions = 0;

		//Dashboard Variables
		float dashboardSlider0;				//Numeric Slider from the SmartDashboard (#0)
		float dashboardSlider1;				//Numeric Slider from the SmartDashboard (#1)
		float dashboardSlider2;				//Numeric Slider from the SmartDashboard (#2)
		float dashboardSlider3;				//Numeric Slider from the SmartDashboard (#3)

		//Sensor Variables
		float rangeInchesFL = 0.0;			//Ultrasonic Sensor (Front Left)
		float rangeInchesFR = 0.0;			//Ultrasonic Sensor (Front Right)
		float rangeInchesBR = 0.0;			//Ultrasonic Sensor (Back Right)
		float rangeInchesBL = 0.0;			//Ultrasonic Sensor (Back Left)
		float rangeInchesLIFT = 0.0;
		double liftHeight = 0.0;

		//Sets the channels for the joysticks
		LeftDriveJoystick.SetXChannel(joystick_x_channel);
		LeftDriveJoystick.SetYChannel(joystick_y_channel);
		RightDriveJoystick.SetXChannel(joystick_x_channel);
		RightDriveJoystick.SetYChannel(joystick_y_channel);

		//First three commands for the xbox controller
		//m_driveSystem->moveCtrl = Axis_XBOX::XBOX_RIGHT_JOYSTICK_Y;
		//m_driveSystem->rotateCtrl = Axis_XBOX::XBOX_LEFT_JOYSTICK_X;
		//m_driveSystem->reverseDrive = -1;
		//Last three commands for the driver station
		m_driveSystem->moveCtrl = RightDriveJoystick.GetYChannel();
		m_driveSystem->rotateCtrl = LeftDriveJoystick.GetXChannel();
		m_driveSystem->reverseDrive = 1;
		/////////////1 for driver station and -1 for xbox

		//the stuffs for the winch system
		m_winchSystem->fwdDrive = Axis_XBOX::XBOX_RIGHT_TRIGGER;
		m_winchSystem->bckDrive = Axis_XBOX::XBOX_LEFT_TRIGGER;
		m_winchSystem->multiMove = true;
		m_winchSystem->multiRotate = false;
		m_winchSystem->rotateEnable = false;

		while (IsOperatorControl() && IsEnabled())
		{
			liftRevolutions = liftEncoder->Get();

			//Xbox Start button turns on and off the plexiglass lights
			if(XboxController.GetRawButton(Button_XBOX::XBOX_START))
			{
				m_plexiglassLightControl->Set(!m_plexiglassLightControl->Get());
			}

			//Dashboard variables
			dashboardSlider0 = SmartDashboard::GetNumber("DB/Slider 0", 0.0);
			dashboardSlider1 = SmartDashboard::GetNumber("DB/Slider 1", 0.0);
			dashboardSlider2 = SmartDashboard::GetNumber("DB/Slider 2", 0.0);
			dashboardSlider3 = SmartDashboard::GetNumber("DB/Slider 3", 0.0);

			//Sensor variables
			rangeInchesFL = m_ultrasonicFrontLeft->sonarRange();
			rangeInchesFR = m_ultrasonicFrontRight->sonarRange();
			rangeInchesBR = m_ultrasonicBackRight->sonarRange();
			rangeInchesBL = m_ultrasonicBackLeft->sonarRange();
			rangeInchesLIFT = m_ultrasonicLift->sonarRange();

			//Drive system
			m_driveSystem->arcadeDrive();

			//Winch system
			m_winchSystem->arcadeDrive();


			//If the A button is pressed the motor speeds are halved?
			if (LogitechController.GetRawButton(Button_LOGITECH::LOGITECH_BACK))
			{
				m_driveSystem->motorMultiplier = 0.0;
				SmartDashboard::PutString("DB/String 0", "Motors STOPPED");
			}
			else if (XboxController.GetRawButton(Button_XBOX::XBOX_RIGHT_BUMPER))
			{
				m_driveSystem->motorMultiplier = 1.0;
				SmartDashboard::PutString("DB/String 0", "Motors at FULL speed");
			}
			else
			{
				m_driveSystem->motorMultiplier = 0.5;
				SmartDashboard::PutString("DB/String 0", "Motors at HALF speed");
			}

			/*
			if (LogitechController.GetRawButton(Button_LOGITECH::LOGITECH_START))
			{
				//m_actuatorMotor->ExtendVariable(0.74);
				//SmartDashboard::PutString("DB/String 2", "MID POSITION");
				m_grabberMotor->InstantaneousStop();
			}
			*/

			if (RightButtonHub.GetRawButton(Generic_Controller_Right::BUTTON_RED_LEFT) && !RightButtonHub.GetRawButton(Generic_Controller_Right::BUTTON_RED_RIGHT))
			{
				//Extends the actuator to pinch a block
				//m_actuatorMotor->ExtendBlockMax();

				//while block limit switch is off
				//closeGrabber(grabber_speed);
				grabberCloseTimeLast = (robotClock.Get());
				CLOSEPRESS = true;
			}
			else if (!RightButtonHub.GetRawButton(Generic_Controller_Right::BUTTON_RED_LEFT) && RightButtonHub.GetRawButton(Generic_Controller_Right::BUTTON_RED_RIGHT))
			{
				//Retracts the actuator to release a block
				//m_actuatorMotor->ExtendBlockMin();

				//while robot limit swtich is off
				//openGrabber(grabber_speed);
				grabberOpenTimeLast = (robotClock.Get());
				OPENPRESS = true;
			}
			else if (XboxController.GetRawButton(Button_XBOX::XBOX_Y) && !XboxController.GetRawButton(Button_XBOX::XBOX_A))
			{
				//Extends the actuator to pinch a block
				//m_actuatorMotor->ExtendBlockMax();

				//while block limit switch is off
				//closeGrabber(grabber_speed);
				grabberCloseTimeLast = (robotClock.Get());
				CLOSEPRESS = true;
			}
			else if (!XboxController.GetRawButton(Button_XBOX::XBOX_Y) && XboxController.GetRawButton(Button_XBOX::XBOX_A))
			{
				//Retracts the actuator to release a block
				//m_actuatorMotor->ExtendBlockMin();

				//while robot limit swtich is off
				//openGrabber(grabber_speed);
				grabberOpenTimeLast = (robotClock.Get());
				OPENPRESS = true;
			}

			/*
			else if (ButtonJoystick.GetRawButton(Button_Generic::BUTTON_RED_BOTTOM_RIGHT) && !ButtonJoystick.GetRawButton(Button_Generic::BUTTON_RED_BOTTOM_LEFT))
			{
				//Extends the actuator to pinch a block
				//m_actuatorMotor->ExtendBlockMax();

				//while block limit switch is off
				//closeGrabber(grabber_speed);
				grabberCloseTimeLast = (robotClock.Get());
				CLOSEPRESS = true;
			}
			else if (!ButtonJoystick.GetRawButton(Button_Generic::BUTTON_RED_BOTTOM_RIGHT) && ButtonJoystick.GetRawButton(Button_Generic::BUTTON_RED_BOTTOM_LEFT))
			{
				//Retracts the actuator to release a block
				//m_actuatorMotor->ExtendBlockMin();

				//while robot limit swtich is off
				//openGrabber(grabber_speed);
				grabberOpenTimeLast = (robotClock.Get());
				OPENPRESS = true;
			}
			*/

			//grabber opening and closing
			if (OPENPRESS)
			{
				openGrabber(grabber_speed);
			}
			else if (CLOSEPRESS)
			{
				closeGrabber(grabber_speed);
			}

			//linear actuator that keeps the hook from falling off
			if (LogitechController.GetRawButton(Button_LOGITECH::LOGITECH_LEFT_BUMPER))
			{
				//Extends the locking actuator
				m_winchLockMotor->ExtendMax();
			}
			if (LogitechController.GetRawButton(Button_LOGITECH::LOGITECH_RIGHT_BUMPER))
			{
				//Retracts the locking actuator
				m_winchLockMotor->ExtendMin();
			}

			//testing functions
			if (LogitechController.GetRawButton(Button_LOGITECH::LOGITECH_X))
			{
				//m_liftMotor->Brake();
				//m_liftMotor->InstantaneousRotation(0.01);
				XPRESS = true;

			}
			if (LogitechController.GetRawButton(Button_LOGITECH::LOGITECH_B))
			{
				//m_liftMotor->SlowSpeed(0.3, -0.13);
			}

			liftHeight = getLiftHeight();
			SmartDashboard::PutString("DB/String 1", "lift height: " + std::to_string(liftHeight));

			//m_actuatorMotor->ExtendVariable(dashboardSlider0);
			DpadLift();

			XPRESS = liftPortal(XPRESS);

			//Gets the values of the x and y axis for each of the driver joysticks
			LeftDriveJoystickX = LeftDriveJoystick.GetX();
			LeftDriveJoystickY = LeftDriveJoystick.GetY();
			RightDriveJoystickX = RightDriveJoystick.GetX();
			RightDriveJoystickY = RightDriveJoystick.GetY();

			//A bunch of variables and things that gets outputted to the SmartDashboard so we can see whats happening
			SmartDashboard::PutString("DB/String 2", "bL: " + std::to_string(m_blockLeftLimitSwitch->Get()) + " bR: " + std::to_string(m_blockRightLimitSwitch->Get()) + " r: " + std::to_string(m_robotLimitSwitch->Get()));
			//SmartDashboard::PutString("DB/String 3", "current: " + std::to_string(pdp->GetCurrent(14)));
			SmartDashboard::PutString("DB/String 3", "leftjoyY: " + std::to_string(LeftDriveJoystickY));
			SmartDashboard::PutString("DB/String 4", "ultraLIFT " + std::to_string(rangeInchesLIFT) + " inches");
			SmartDashboard::PutString("DB/String 5", "ultraFL: " + std::to_string(rangeInchesFL) + " inches");
			SmartDashboard::PutString("DB/String 6", "ultraFR: " + std::to_string(rangeInchesFR) + " inches");
			SmartDashboard::PutString("DB/String 7", "ultraBR: " + std::to_string(rangeInchesBR) + " inches");
			SmartDashboard::PutString("DB/String 8", "ultraBL: " + std::to_string(rangeInchesBL) + " inches");
			SmartDashboard::PutString("DB/String 9", std::to_string(m_liftMotor->GetSpeed()) + "state" + std::to_string(DPAD_State) + "dpad" + std::to_string(LogitechController.GetPOV()));

			frc::Wait(0.005);
		}
	}

	void Test() override
	{
		//We never used this, but it would let us test things out with the driver station test mode (I think)
	}

private:
	enum Channel_Controller
	{
		//The number correlates to the 'USB Order' number on the driver station
		XBOX_CONTROLLER = 0,				//When we are using the logitech and xbox controllers
		//The logitech controller and the trigger joystick share the same channel because the FRC driver station only allows for 6 (0-5) channels
		LOGITECH_CONTROLLER = 1,			//When we are using the logitech and xbox controllers
		//TRIGGER_JOYSTICK = 1,				//When we are using the driver station
		LEFT_DRIVE_JOYSTICK = 2,			//When we are using the driver station
		LEFT_BUTTON_HUB = 3,				//When we are using the driver station
		RIGHT_DRIVE_JOYSTICK = 4,			//When we are using the driver station
		RIGHT_BUTTON_HUB = 5				//When we are using the driver station
	};

	enum Channel_Analog
	{
		//Channels for the analog sensors on the roboRIO
		ULTRASONIC_SENSOR_FRONT_LEFT = 0,
		ULTRASONIC_SENSOR_FRONT_RIGHT = 1,
		ULTRASONIC_SENSOR_BACK_RIGHT = 2,
		ULTRASONIC_SENSOR_BACK_LEFT = 3,
		INFRARED_SENSOR_LIFT_LONG = 4,
		INFRARED_SENSOR_LIFT_SHORT = 5,
		ULTRASONIC_SENSOR_LIFT = 6,
		PLACEHOLDER = 10
	};

	enum Channel_PWM
	{
		//Channels for the PWM motors on the roboRIO
		LEFT_MOTOR = 0,
		RIGHT_MOTOR = 1,
		LIFT_MOTOR = 2,
		WINCH_MOTOR = 3,
		//ACTUATOR_MOTOR = 5,
		GRABBER_MOTOR = 5,
		WINCH_LOCK_MOTOR = 6,
		PLACEHOLDER_MOTOR = 10
	};

	enum Channel_Digital
	{
		//Channels for the digital sensors on the roboRIO
		PLEXIGLASS_LIGHT_CONTROL = 0,
		LIMITSWITCH_ROBOT = 1,
		LIMITSWITCH_BLOCK_LEFT = 2,
		LIMITSWITCH_BLOCK_RIGHT = 3,
		ENCODER_LIFT_A = 4,
		ENCODER_LIFT_B = 5
	};

	frc::PowerDistributionPanel* pdp = new frc::PowerDistributionPanel();

	DPad_LOGITECH DPAD_State = DPad_LOGITECH::LOGITECH_STOP_BRAKE;

	//Our joysticks (includes the xbox and logitech controllers, the arcadee joysticks, and the button hubs)
	frc::Joystick XboxController { Channel_Controller::XBOX_CONTROLLER };
	frc::Joystick LogitechController { Channel_Controller::LOGITECH_CONTROLLER };
	//frc::Joystick TriggerJoystick { Channel_Controller::TRIGGER_JOYSTICK };
	frc::Joystick LeftDriveJoystick { Channel_Controller::LEFT_DRIVE_JOYSTICK };
	frc::Joystick LeftButtonHub { Channel_Controller::LEFT_BUTTON_HUB };
	frc::Joystick RightDriveJoystick { Channel_Controller::RIGHT_DRIVE_JOYSTICK };
	frc::Joystick RightButtonHub { Channel_Controller::RIGHT_BUTTON_HUB };

	//Our standalone spark motors
	frc::Spark* m_leftMotor = new frc::Spark(Channel_PWM::LEFT_MOTOR);
	frc::Spark* m_rightMotor = new frc::Spark(Channel_PWM::RIGHT_MOTOR);
	frc::Spark* m_winchMotor = new frc::Spark(Channel_PWM::WINCH_MOTOR);
	frc::Spark* m_placeholderMotor = new frc::Spark(Channel_PWM::PLACEHOLDER_MOTOR);

	//Our BjorgDrive systems for driving the robot and for using the winch, the function takes in two motors and two joysticks from above
	//&RightDriveJoystick and &LeftDriveJoystick for Driver Station
	//&XboxController and &XboxController for XBOX Controller
	BjorgDrive* m_driveSystem = new BjorgDrive(m_leftMotor, m_rightMotor, &RightDriveJoystick, &LeftDriveJoystick);
	BjorgDrive* m_winchSystem = new BjorgDrive(m_winchMotor, m_placeholderMotor, &XboxController, &XboxController);

	//Our MaxSonar (ultrasonic) sensors, takes the sensor channel and the specific type of MaxSonar sensor
	MaxSonar* m_ultrasonicFrontLeft = new MaxSonar(Channel_Analog::ULTRASONIC_SENSOR_FRONT_LEFT, Ultrasonic_Sensor_Type::LV);
	MaxSonar* m_ultrasonicFrontRight = new MaxSonar(Channel_Analog::ULTRASONIC_SENSOR_FRONT_RIGHT, Ultrasonic_Sensor_Type::LV);
	MaxSonar* m_ultrasonicBackRight = new MaxSonar(Channel_Analog::ULTRASONIC_SENSOR_BACK_RIGHT, Ultrasonic_Sensor_Type::LV);
	MaxSonar* m_ultrasonicBackLeft = new MaxSonar(Channel_Analog::ULTRASONIC_SENSOR_BACK_LEFT, Ultrasonic_Sensor_Type::LV);
	MaxSonar* m_ultrasonicLift = new MaxSonar(Channel_Analog::ULTRASONIC_SENSOR_LIFT, Ultrasonic_Sensor_Type::HRLV);

	//Our Infrared sensors, takes the sensor channel and the specific type of infrared sensor
	Infrared* m_liftLongRange = new Infrared(Channel_Analog::INFRARED_SENSOR_LIFT_LONG, Infrared_Sensor_Type::GP2Y0A710K0F);
	Infrared* m_liftShortRange = new Infrared(Channel_Analog::INFRARED_SENSOR_LIFT_SHORT, Infrared_Sensor_Type::GP2Y0A02YK0F);
//	Infrared* m_irFrontLeft = new Infrared(Channel_Analog::INFRARED_SENSOR_FRONT_LEFT, Infrared_Sensor_Type::OPB732WZ);
//	Infrared* m_irFrontRight = new Infrared(Channel_Analog::INFRARED_SENSOR_FRONT_RIGHT, Infrared_Sensor_Type::OPB732WZ);
//	Infrared* m_irBackLeft = new Infrared(Channel_Analog::INFRARED_SENSOR_BACK_LEFT, Infrared_Sensor_Type::OPB732WZ);
//	Infrared* m_irBackRight = new Infrared(Channel_Analog::INFRARED_SENSOR_BACK_RIGHT, Infrared_Sensor_Type::OPB732WZ);

	//Our Encoder, takes in two channels, ?, and ?
	frc::Encoder* liftEncoder = new Encoder(Channel_Digital::ENCODER_LIFT_A, Channel_Digital::ENCODER_LIFT_B, false, Encoder::k4X);

	//Our linear actuators, takes the PWM channel for the motor
	//Actuator* m_actuatorMotor = new Actuator(Channel_PWM::ACTUATOR_MOTOR);
	Actuator* m_winchLockMotor = new Actuator(Channel_PWM::WINCH_LOCK_MOTOR);

	//Our generic motors, take the PWM channel and the motor tpype
	Motor* m_liftMotor = new Motor(Channel_PWM::LIFT_MOTOR, Motor_Type::SPARK);
	Motor* m_grabberMotor = new Motor(Channel_PWM::GRABBER_MOTOR, Motor_Type::SPARK);

	//Our digital sensors, takes the channel for the sensor
	frc::DigitalInput* m_robotLimitSwitch = new frc::DigitalInput(Channel_Digital::LIMITSWITCH_ROBOT);
	frc::DigitalInput* m_blockLeftLimitSwitch = new frc::DigitalInput(Channel_Digital::LIMITSWITCH_BLOCK_LEFT);
	frc::DigitalInput* m_blockRightLimitSwitch = new frc::DigitalInput(Channel_Digital::LIMITSWITCH_BLOCK_RIGHT);

	//Light control for the lights on the plexiglass
	frc::DigitalOutput* m_plexiglassLightControl = new frc::DigitalOutput(Channel_Digital::PLEXIGLASS_LIGHT_CONTROL);

	//Sendable chooser for autonomous that we never used
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";

	//////////////////there be a lot of stuff here
	//constant variables
	static constexpr double lift_up_dur = .5;
	static constexpr double lift_down_dur = .5;
	static constexpr double lift_up_speed = 0.5;
	static constexpr double lift_down_speed = -0.2;
	static constexpr double grabber_speed = 0.6;
	static constexpr double switch_height = 18;
	static constexpr double portal_height = 24;
	static constexpr double max_current_grabber = 25;
	static constexpr double max_current_lift = 40;					//placeholder value
	static constexpr double grabber_hold = .3;
	static const int max_lift_encoder = 2000;						//placeholder value
	static const int max_auto_encoder = 1000;						//placeholder value
	static const int joystick_x_channel = 1;
	static const int joystick_y_channel = 0;

	//Other variables
	bool longRange = false;
	bool XPRESS = false;
	bool OPENPRESS = false;
	bool CLOSEPRESS = false;
	double speedTest = 0;
	int auto_lift_revolutions = 0;
	double LeftDriveJoystickX = 0;
	double LeftDriveJoystickY = 0;
	double RightDriveJoystickX = 0;
	double RightDriveJoystickY = 0;
	double TriggerJoystickX = 0;
	double TriggerJoystickY = 0;

	//(Probably) all of the timer related stuff
	frc::Timer robotClock;
	double liftTimeLast = 0;
	double liftTimeCurrent = 0;
	double autoTimeLast = 0;
	double autoTimeCurrent = 0;
	double grabberOpenTimeLast = 0;
	double grabberOpenTimeCurrent = 0;
	double grabberCloseTimeLast = 0;
	double grabberCloseTimeCurrent = 0;

	//Custom Functions
	//Custom Functions
	//Custom Functions
	//Custom Functions

	void openGrabber(double speed)
	{
		if (pdp->GetCurrent(14) <= max_current_grabber || m_blockRightLimitSwitch->Get() == 1)
		{
			if (m_blockLeftLimitSwitch->Get() == 0)
			{
				SmartDashboard::PutString("DB/String 2", "STOPPED RELEASING");
				m_grabberMotor->InstantaneousStop();
				OPENPRESS = false;
			}
			else
			{
				grabberOpenTimeCurrent = (robotClock.Get());
				if ((grabberOpenTimeCurrent - grabberOpenTimeLast) <= 2)
				{
					SmartDashboard::PutString("DB/String 2", "RELEASING");
					m_grabberMotor->InstantaneousRotation(speed);
				}
				else
				{
					m_grabberMotor->InstantaneousStop();
					OPENPRESS = false;
					CLOSEPRESS = false;
				}
			}
		}
		else
		{
			m_grabberMotor->InstantaneousStop();
			OPENPRESS = false;
		}
		CLOSEPRESS = false;
	}

	void closeGrabber(double speed)
	{
		if (pdp->GetCurrent(14) <= max_current_grabber || m_blockRightLimitSwitch->Get() == 1)
		{
			if (m_robotLimitSwitch->Get() == 0 || pdp->GetCurrent(14) >= 20)
			{
				SmartDashboard::PutString("DB/String 2", "STOPPED PINCHING");
				m_grabberMotor->InstantaneousRotation(grabber_hold);
				if (pdp->GetCurrent(14) >= 20)
				{
					m_grabberMotor->InstantaneousStop();
					OPENPRESS = false;
					CLOSEPRESS = false;
				}
			}
			else
			{
				grabberCloseTimeCurrent = (robotClock.Get());
				//if ((grabberCloseTimeCurrent - grabberCloseTimeLast) <= 2)
				{
					SmartDashboard::PutString("DB/String 2", "PINCHING");
					m_grabberMotor->InstantaneousRotation(-speed);
				}
				//else
				{
				//	m_grabberMotor->InstantaneousStop();
				//	OPENPRESS = false;
				//	CLOSEPRESS = false;
				}
			}
		}
		else
		{
			m_grabberMotor->InstantaneousStop();
			CLOSEPRESS = false;
		}
		OPENPRESS = false;
	}

	bool liftPortal(bool pressed)
	{
		bool atHeight;

		if (pressed)
		{
			atHeight = liftVariableHeight(portal_height, 1.5);
		}

		if (atHeight)
		{
			pressed = false;
		}

		return pressed;
	}

	double getLiftHeight()
	{
		double height;

		//gets the range depending on the longRange boolean
		if (longRange)
		{
			height = m_liftLongRange->irRange();
		}
		else if (!longRange)
		{
			height = m_liftShortRange->irRange();
		}

		if (!longRange && height > 55)
		{
			//sets the longRange bool to true (long) if the height is greater than 55 and is currently on short range
			longRange = true;
		}
		else if (longRange && height < 50)
		{
			//sets the longRange bool to false (short) if the height is less than 50 and is currently on long range
			longRange = false;
		}

		return height;
	}

	bool liftVariableHeight(double h, double t)
	{
		liftTimeCurrent = (robotClock.Get());

		bool liftState = false;

		int lift_encoder = liftEncoder->Get();

		if (getLiftHeight() < h && (liftTimeCurrent - liftTimeLast) <= t && lift_encoder < max_lift_encoder)
		{
			m_liftMotor->InstantaneousRotation(lift_up_speed);
			std::cout << "height: " << getLiftHeight() << std::endl;
			std::cout << "liftTimeCurrent:" << liftTimeCurrent << "liftTimeLast: " << liftTimeLast << "time " << (liftTimeCurrent - liftTimeLast) << std::endl;
		}
		else
		{
			liftState = true;
			std::cout << "-----------------------------------------------------" << std::endl;
			m_liftMotor->InstantaneousStop();
		}

		return liftState;
	}

	void autoCrossLine()
	{

		double dist = 0;

//		std::cout << "open grabber" << std::endl;
		//m_actuatorMotor->ExtendBlockMax();
		//closeGrabber(grabber_speed);										//need to test

		autoTimeLast = (robotClock.Get());
		autoTimeCurrent = autoTimeLast;

		//Drives past the auto-line
		while (dist <= 120 && (autoTimeCurrent - autoTimeLast) <= 1.8)	//SLIGHTLY GOES PAST 120 INCHES//
		{
			m_driveSystem->arcadeDrive(0.75, 0.0);
			dist = ( ( m_ultrasonicBackLeft->sonarRange() + m_ultrasonicBackRight->sonarRange() ) / 2 );
			autoTimeCurrent = (robotClock.Get());

//			std::cout << std::to_string(dist) << std::endl;
		}
		m_driveSystem->arcadeDrive(-1.0, 0.0);
		frc::Wait(0.05);		m_driveSystem->arcadeDrive(0.0, 0.0);
	}

	void autoPlaceSwitchStraight(bool sameSide)
	{
		double distBack = 0;
		double distFront = 0;
		bool liftH;

		//while (autoTimeCurrent - autoTimeLast < 1)

		//m_actuatorMotor->ExtendBlockMax();
		closeGrabber(grabber_speed);										//need to test
		//frc::Wait(1);

		liftTimeLast = (robotClock.Get());
		liftTimeCurrent = liftTimeLast;
		//make while so it lifts the lift while the distance is under a foot or the time is less than two seconds
/*
		do
		{
			liftH = liftVariableHeight(switch_height, 2);
			auto_lift_revolutions = liftEncoder->Get();
			std::cout << "lift encoder " << liftEncoder->Get() << std::endl;
		}
		while (!liftH && auto_lift_revolutions < max_auto_encoder);

*/
		autoTimeLast = (robotClock.Get());
		autoTimeCurrent = autoTimeLast;

		while (distBack <= 120 && (autoTimeCurrent - autoTimeLast) <= 0.8 && distFront <= 8)
		{
			m_driveSystem->arcadeDrive(0.75, 0.0);
			distBack = (( m_ultrasonicBackLeft->sonarRange() + m_ultrasonicBackRight->sonarRange()) / 2);
			distFront = ((m_ultrasonicFrontLeft->sonarRange() + m_ultrasonicFrontRight->sonarRange()) / 2);
			autoTimeCurrent = (robotClock.Get());
		}
		m_driveSystem->arcadeDrive(0.0, 0.0);

		if (sameSide)
		{
			//m_actuatorMotor->ExtendBlockMin();
			openGrabber(grabber_speed);										//need to test
		}
	}

	void autoPlaceSwitchTurn(std::string pos)
	{
		double dist = 0;

		autoCrossLine();
		frc::Wait(0.05);

		if (pos == "Left")
		{
			autoTimeLast = (robotClock.Get());
			autoTimeCurrent = autoTimeLast;
			while ((autoTimeCurrent - autoTimeLast) <= 0.6)
			{
				m_driveSystem->arcadeDrive(0.0, 0.75);
				autoTimeCurrent = (robotClock.Get());
			}
		}
		else if (pos == "Right")
		{
			autoTimeLast = (robotClock.Get());
			autoTimeCurrent = autoTimeLast;
			while ((autoTimeCurrent - autoTimeLast) <= 0.6)
			{
				m_driveSystem->arcadeDrive(0.0, -0.75);
				autoTimeCurrent = (robotClock.Get());
			}
		}

		autoTimeLast = (robotClock.Get());
		autoTimeCurrent = autoTimeLast;
		//make while so it lifts the lift while the distance is under a foot or the time is less than two seconds
		while (getLiftHeight() < 12 || (autoTimeCurrent - autoTimeLast) <= 2)
		{
			m_liftMotor->GradualRotation(0.6, 0.5);
			autoTimeCurrent = (robotClock.Get());
		}
		frc::Wait(0.5);			///////////////////////////lift sensor?//////////////////////
		m_liftMotor->InstantaneousRotation(0.15);

		while (dist >= 8)
		{
			m_driveSystem->arcadeDrive(0.5, 0.0);
			dist = ( ( m_ultrasonicFrontLeft->sonarRange() + m_ultrasonicFrontRight->sonarRange() ) / 2 );
		}
		frc::Wait(0.1);
		m_driveSystem->arcadeDrive(0.0, 0.0);

//		std::cout << "release grabber" << std::endl;
		//m_actuatorMotor->ExtendBlockMin();
		openGrabber(grabber_speed);										//need to test

		frc::Wait(0.1);
		m_driveSystem->arcadeDrive(-0.5, 0.0);
		frc::Wait(0.1);
		m_driveSystem->arcadeDrive(0.0, 0.0);

		////////////////////////////////////////////////////LOWER LIFT/////////////////////////////////////
		////////////////////////////////////////////////////LOWER LIFT/////////////////////////////////////
	}

	//I don't remember how this works but it does work
	void DpadLift()
	{
		int DPAD_Value = LOGITECH_RELEASED;

		//Old system that used the actual logitech controller dpad
		//DPAD_Value = LogitechController.GetPOV();

		//System that should emulate the dpad using two arcade buttons and a safety switch
		if (RightButtonHub.GetRawButton(Generic_Controller_Right::SWITCH_SAFE1))
		{
			if (RightButtonHub.GetRawButton(Generic_Controller_Right::BUTTON_RED_TOP) && !RightButtonHub.GetRawButton(Generic_Controller_Right::BUTTON_RED_BOTTOM))
			{
				DPAD_Value = DPad_LOGITECH::LOGITECH_TOP;			//0 degrees
			}
			else if (!RightButtonHub.GetRawButton(Generic_Controller_Right::BUTTON_RED_TOP) && RightButtonHub.GetRawButton(Generic_Controller_Right::BUTTON_RED_BOTTOM))
			{
				DPAD_Value = DPad_LOGITECH::LOGITECH_BOTTOM;		//180 degrees
			}
			else
			{
				DPAD_Value = DPad_LOGITECH::LOGITECH_RELEASED;		//-1 'degrees', the value for the figurative dpad being released
			}
		}
		else
		{
			DPAD_Value = DPad_LOGITECH::LOGITECH_RELEASED;		//-1 'degrees', the value for the figurative dpad being released
		}


		m_liftMotor->ResetAccelerate();

		switch(DPAD_Value)
		{
			case DPad_LOGITECH::LOGITECH_TOP:
				//Move lift up if prior State is TOP or RELEASE
				if (DPAD_State == LOGITECH_RELEASED || DPAD_State == LOGITECH_TOP || DPAD_State == LOGITECH_STOP_BRAKE)
				{
					//m_liftMotor->GradualRotation(lift_up_speed, lift_up_dur);
					m_liftMotor->InstantaneousRotation(lift_up_speed);
				}
				//Stop lift if prior State is not TOP or RELEASE
				else
				{
					//m_liftMotor->GradualRotation(0.0, lift_up_dur);
					m_liftMotor->InstantaneousRotation(0.0);
				}
				DPAD_State = LOGITECH_TOP;
				break;

			case LOGITECH_BOTTOM:
				//Move lift down if prior State is BOTTOM or RELEASE
				if (DPAD_State == LOGITECH_RELEASED || DPAD_State == LOGITECH_BOTTOM || DPAD_State == LOGITECH_STOP_BRAKE)
				{
					m_liftMotor->InstantaneousRotation(lift_down_speed);
				}
				//Stop lift if prior State is not BOTTOM or RELEASE
				else
				{
					m_liftMotor->GradualRotation(0.0, lift_down_dur);
				}
				DPAD_State = LOGITECH_BOTTOM;
				break;

			case LOGITECH_RELEASED:
				//Stop lift if DPad Released
				if (DPAD_State == LOGITECH_TOP)
				{
					m_liftMotor->ResetAccelerate(true);
					DPAD_State = LOGITECH_TRANSITION;
				}

				if (DPAD_State == LOGITECH_BOTTOM)
				{
					m_liftMotor->ResetAccelerate(true);
					DPAD_State = LOGITECH_STOP_BRAKE;
				}

				if (DPAD_State == LOGITECH_STOP_BRAKE)
				{
					m_liftMotor->GradualRotation(0.0, lift_down_dur, Stop);
				}


				if (DPAD_State == LOGITECH_TRANSITION)
				{
					m_liftMotor->GradualRotation(0.0, lift_down_dur, Stop);
				}

				if (m_liftMotor->GetSpeed() == 0.0 && DPAD_State != LOGITECH_STOP_BRAKE)
				{
					DPAD_State = LOGITECH_RELEASED;
				}

				if (DPAD_State == LOGITECH_RELEASED)
				{
					//m_liftMotor->Brake();
					m_liftMotor->InstantaneousRotation(0.15);
				}
				//Shouldn't need \/ \/
				//if (m_liftMotor->GetSpeed() <= 0.01 && m_liftMotor->GetSpeed() >= -0.01)
				//{
				//	std::cout << "6" << std::endl;
				//	DPAD_State = DPad_LOGITECH::LOGITECH_RELEASED;
				//	m_liftMotor->InstaneousStop();
				//}
				break;

			default:
				break;
		}
	}
};

void DisabledActions()
{
	int pwmActuator = 4;
	int pwmWinch = 3;

	int openActuatorArms = 1;
	int windWinch = 2;
	int unwindWinch = 3;

	long timeout = 0;

	Actuator* actuatorArm = new Actuator(pwmActuator);
	Motor* winchMotor = new Motor(pwmWinch, Motor_Type::SPARK);

	// Open Actuator
	if(openActuatorArms)
	{
		actuatorArm->ExtendMax();
	}
	// Wind Winch (climb)
	while(windWinch && timeout < 1600)
	{
		winchMotor->InstantaneousRotation(0.5);
		frc::Wait(0.005);
		timeout++;
	}
	// Unwind Winch (lower)
	while(unwindWinch && timeout < 1600)
	{
		winchMotor->InstantaneousRotation(-0.5);
		frc::Wait(0.005);
		timeout++;
	}
	winchMotor->InstantaneousStop();
	timeout = 0;
	frc::Wait(0.005);
}

START_ROBOT_CLASS(Robot2018)
