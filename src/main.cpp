/*

AUTHOR: Shawn M. Rhoads
https://github.com/shawn-rhoads/

*/


#include "main.h"
#include "okapi/api.hpp"
#include "api.hpp"
#include "auto/pathGenerator.hpp"
#include "auto/purePursuit.hpp"
#include "auto/backwardsPurePursuit.hpp"

using namespace okapi;

Controller controller;

std::shared_ptr<OdomChassisController> drive;
std::string test("test");

autolib::PathGenerator path1( {1.0, 2.0, 4.0} );
autolib::PathGenerator path2( {1.0, 2.0, 4.0} );
autolib::PathGenerator path3( {1.0, 2.0, 4.0} );
autolib::PathGenerator path4( {1.0, 2.0, 4.0} );

autolib::PathGenerator startTopNeutralDropOtherSidePath( {1.0, 2.0, 4.0} );


pros::Imu inertial(12);
double inertialYawBase = 0;
double inertialYawStart = 0;

Motor liftMotor(13);
Motor clutchMotor(-14);

pros::ADIDigitalOut pistonF ('H');
pros::ADIDigitalOut pistonB ('G');//
pros::ADIDigitalOut pistonC ('F');

pros::Gps gps(11,0.02,0.2);
pros::c::gps_status_s_t status;

Motor left1(15);
Motor left2(16);
Motor left3(17);

Motor right1(-18);
Motor right2(-19);
Motor right3(-20);

ControllerButton armUpButton(ControllerDigital::R1);
ControllerButton armDownButton(ControllerDigital::R2);
ControllerButton grab1Button(ControllerDigital::L1);
ControllerButton grab2Button(ControllerDigital::L2);
ControllerButton swapButton(ControllerDigital::X);
ControllerButton clutchUpButton(ControllerDigital::up);
ControllerButton clutchDownButton(ControllerDigital::down);
ControllerButton backPistonButton(ControllerDigital::left);
ControllerButton neutralBackButton(ControllerDigital::right);
ControllerButton forceRaisedButton(ControllerDigital::Y);
ControllerButton brakeModeButton(ControllerDigital::A);
ControllerButton SwapperButton (ControllerDigital::A);
pros::ADIDigitalOut pistonS ('D');

void lowerBack() {
	clutchMotor.moveVelocity(0);
	while (clutchMotor.getEfficiency() > 20) {
		if (clutchMotor.getEfficiency() > 20)
			clutchMotor.moveVelocity(-200);
		else {
			clutchMotor.moveVoltage(0);
			std::cout<<"CLUTCH STOPPING"<<"\n"<<std::flush;
		}
		pros::delay(5);
	}
	clutchMotor.moveVoltage(0);
	clutchMotor.tarePosition();
}

void initialize() {

	//pros::lcd::register_btn1_cb(on_center_button);

  startTopNeutralDropOtherSidePath.generatePath( {
			autolib::Pose{-12_in, -49_in, 89_deg },
			autolib::Pose{-23_in, -35_in, 60_deg },
			autolib::Pose{1_in, -18.5_in, 30_deg},
		}, test
	);

	pistonF.set_value(0);
	pistonB.set_value(0);
	pistonC.set_value(1);
	pistonS.set_value(0);
	pros::delay(400);
	clutchMotor.moveVelocity(-200);
	liftMotor.moveVelocity(-100);
	pros::delay(100);
	bool clutchEnded = false, liftEnded = false;
	while (clutchMotor.getEfficiency() > 35 || liftMotor.getEfficiency() > 35) {
		if (!clutchEnded) {
			if (clutchMotor.getEfficiency() > 35)
				clutchMotor.moveVelocity(-200);
			else {
				clutchMotor.moveVoltage(0);
				clutchEnded = true;
				std::cout<<"CLUTCH STOPPING"<<"\n"<<std::flush;
			}
		}
		if (!liftEnded) {
			if (liftMotor.getEfficiency() > 35)
				liftMotor.moveVelocity(-80);
			else {
				liftMotor.moveVoltage(0);
				liftEnded = true;
				std::cout<<"LIFT STOPPING"<<"\n"<<std::flush;
			}
		}
		pros::delay(5);
	}
	clutchMotor.moveVoltage(0);
	liftMotor.moveVoltage(0);
	clutchMotor.tarePosition();
	liftMotor.tarePosition();
	std::cout<<"CLUTCH & LIFT STOPPING"<<"\n"<<std::flush;
	pros::delay(500);
}

void disabled() {}

void competition_initialize() {}

double degreesToRadians(double degrees) {
	double radians = degrees * M_PI / 180;
	return radians;
}

double radiansToDegrees(double radians) {
	double degrees = radians / M_PI * 180;
	return degrees;
}

void setOrientation(QLength x, QLength y, QAngle yaw) {
	drive->setState(OdomState{x, y, yaw});
	double value = yaw.convert(okapi::degree);
	std::cout << "convert: " << value << "\n" << std::flush;
	while (value < 0) value = value + 360;
	while (value > 360) value = value - 360;
	inertial.set_heading(value);
}

double getDriveX() {
	OdomState ds = drive->getState();
	return ds.x.getValue();
}

double getDriveY() {
	OdomState ds = drive->getState();
	return ds.y.getValue();
}

double getDriveYaw() {
	OdomState ds = drive->getState();
	return ds.theta.getValue();
}

void setAtGps() {
	pros::delay(50);
	pros::c::gps_status_s_t status;
	status = gps.get_status();
	pros::delay(50);
	setOrientation(okapi::QLength(status.x),okapi::QLength(status.y),okapi::QAngle(getDriveYaw()));
	std::cout<<"GPS READING (x,y): " << status.x << "," << status.y <<"\n"<<std::flush;
	pros::delay(50);
}

void outputLocationDetails() {
	double yaw = degreesToRadians(inertial.get_yaw());
	std::cout << "odom says: " << drive->getState().x.getValue() << "," << drive->getState().y.getValue() << ", yaw: " << drive->getState().theta.getValue() << "  inertial says: " << inertial.get_yaw() << " (" << yaw << ")\n" << std::flush;

}

void autonomous() {
	drive =
		ChassisControllerBuilder()
			.withMotors(
				{15,16,17},
				{-18,-19,-20}
			)
			.withSensors(
				IntegratedEncoder{16},
				IntegratedEncoder{19, true},
				RotationSensor{10, true}
			)
			// Green gearset, 4 in wheel diam, 11.5 in wheel track
			.withDimensions({AbstractMotor::gearset::green, (36.0 / 60.0)}, {{3.25_in, 13.485_in}, imev5GreenTPR})
			.withOdometry({{3.25_in * 60 / 36, 13.485_in, 2.165_in, 2.75_in}, imev5GreenTPR})
	// Use the same scales as the chassis (above)
		.buildOdometry();

	//setOrientation(0_in, 0_in, 90_deg);
	//drive->setMaxVelocity(50);
	//drive->turnAngle(90_deg);
	std::cout << "test" << std::flush;
	// while(true) {
	// 	outputLocationDetails();
	// 	pros::delay(100);
	// }
	setOrientation(-58_in, -57_in, 180_deg);
  outputLocationDetails();

	autolib::PurePursuit StartSegment( startTopNeutralDropOtherSidePath.getPaths(), /* lookahead distance */ 9_in, false);

	outputLocationDetails();

	left1.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	left2.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	left3.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	right1.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	right2.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	right3.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

	pistonF.set_value(0);

	left1.moveVelocity(-200);
	left2.moveVelocity(-200);
	left3.moveVelocity(-200);
	right1.moveVelocity(-200);
	right2.moveVelocity(-200);
	right3.moveVelocity(-200);

	while(getDriveX() < -0.275) {
		pros::delay(20);
	}

	pistonB.set_value(1);

	left1.moveVelocity(0);
	left2.moveVelocity(0);
	left3.moveVelocity(0);
	right1.moveVelocity(0);
	right2.moveVelocity(0);
	right3.moveVelocity(0);
	pistonC.set_value(1);
	pros::delay(100);
	clutchMotor.moveAbsolute(1500, 200);
	bool ppactive = true;
	while (ppactive) {
		if (ppactive) {
		ppactive = StartSegment.run(getDriveX(), getDriveY(), getDriveYaw(), test, drive, 0.5);
		}
		if (getDriveX() > -0.1 && getDriveY() > 0.05) {
			ppactive = false;
		}
		pros::delay(5);
	}
	pistonF.set_value(1);
	left1.moveVelocity(0);
	left2.moveVelocity(0);
	left3.moveVelocity(0);
	right1.moveVelocity(0);
	right2.moveVelocity(0);
	right3.moveVelocity(0);
	pros::delay(100);
	liftMotor.moveAbsolute(400,100);
	drive->turnAngle(40_deg);
	pros::delay(175);
	clutchMotor.moveAbsolute(10,200);
	drive->moveDistance(-60_in);
	/*pistonB.set_value(0);
	drive->setMaxVelocity(90);
	drive->moveDistance(15_in);
	drive->turnAngle(80_deg);
	drive->setMaxVelocity(40);
	setOrientation(0_in,0_in,0_deg);
	left1.moveVelocity(-100);
	left2.moveVelocity(-100);
	left3.moveVelocity(-100);
	right1.moveVelocity(-100);
	right2.moveVelocity(-100);
	right3.moveVelocity(-100);
	while(getDriveX() > -0.8) {
		pros::delay(20);
	}
	left1.moveVelocity(0);
	left2.moveVelocity(0);
	left3.moveVelocity(0);
	right1.moveVelocity(0);
	right2.moveVelocity(0);
	right3.moveVelocity(0);
	pistonB.set_value(1);
	pistonC.set_value(1);
	pros::delay(100);
	clutchMotor.moveVelocity(200);
	while (clutchMotor.getPosition() < 3000) {
		pros::delay(20);
	}
	clutchMotor.moveVelocity(0);
	pistonC.set_value(0);
	pros::delay(100);
	clutchMotor.moveVelocity(-200);
	drive->setMaxVelocity(100);
	drive->moveDistance(21_in);
	clutchMotor.tarePosition();
	pistonC.set_value(1);
	clutchMotor.moveVelocity(-200);
	while (clutchMotor.getPosition() > -2500) {
		pros::delay(10);
	}
	clutchMotor.moveVelocity(0);
	pistonB.set_value(0);
	drive->moveDistance(6_in);*/
}

void startTopNeutralDropOtherSide() {

}

void opcontrol() {

	 std::shared_ptr<OdomChassisController> driver =
	 ChassisControllerBuilder()
	 	.withMotors(
	 		{-18,-19,-20},
	 		{15,16,17}
	 	)
	 	// Green gearset, 4 in wheel diam, 11.5 in wheel track
	 	.withDimensions({AbstractMotor::gearset::green, (36.0 / 60.0)}, {{3.25_in, 12.75_in}, imev5GreenTPR})
	 	.withOdometry() // Use the same scales as the chassis (above)

	 .buildOdometry();

	liftMotor.setBrakeMode(AbstractMotor::brakeMode::hold);
	int grab1 = 1;
	int grab2 = 1;
	int swapper = 0;
	int swap = 0;
	int db = 0;
	int db2 = 0;
	int db3 = 0;
	int db4 = 0;
	int db5 = 0;
	int db6 = 0;
	int db7 = 0;
	pistonC.set_value(0);
	int clutchPos = 0;
	bool raised = false;
	bool lowering = false;
	int intaking = 0;
	bool backPistonOverride = false;
	bool clutchOverride = false;
	int backPiston = 1;
  int reversing = 0;
	double target = 0;
	int brakeMode = 0;

	while(1) {
		driver->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY),
														controller.getAnalog(ControllerAnalog::rightY));
		if (armUpButton.isPressed()) {
    	liftMotor.moveVoltage(12000);
			std::cout << "Arm Moving Up" << "\n" << std::flush;

    } else if (armDownButton.isPressed()) {
      liftMotor.moveVoltage(-12000);
			std::cout << "Arm Moving Down" << "\n" << std::flush;
    } else {
      liftMotor.moveVoltage(0);
    }
		if (grab1Button.isPressed() && db <= 0) {
			swapper = 0;
			if (grab1) {
				grab1 = 0;
				std::cout << "Front Piston Releasing" << "\n" << std::flush;
			}
			else {
				grab1 = 1;
				std::cout << "Front Piston Grabbing" << "\n" << std::flush;
			}
			db = 16;
		}
		pistonF.set_value(grab1);
		if (neutralBackButton.isPressed() && db2 <= 0) {
			backPistonOverride = false;
			clutchOverride = false;
			if (grab2) {
				raised = false;
				intaking = 0;
				std::cout << "Lowering" << "\n" << std::flush;
				grab2 = 0;
				pistonC.set_value(1);
				clutchPos = 1;
				lowering = true;
				clutchMotor.tarePosition();
				clutchMotor.moveVelocity(-150);
				target = 0;
			}
				else {
					raised = false;
					intaking = 0;
					std::cout << "Grabbing and Raising" << "\n" << std::flush;
					clutchMotor.moveVelocity(0);
					grab2 = 1;
					pistonC.set_value(1);
					clutchPos = 1;
					pistonB.set_value(1);
					pros::delay(100);
					clutchMotor.tarePosition();
					clutchMotor.moveRelative(1000,200);
					pros::delay(300);
					target = 1000;
					}
					db2 = 16;
		}
		if (grab2Button.isPressed() && db2 <= 0) {
			backPistonOverride = false;
			clutchOverride = false;
			if (grab2) {
				raised = false;
				intaking = 0;
				std::cout << "Lowering" << "\n" << std::flush;
				grab2 = 0;
				pistonC.set_value(1);
				clutchPos = 1;
				lowering = true;
				clutchMotor.tarePosition();
				clutchMotor.moveVelocity(-150);
				target = 0;
			}
			else {
				raised = false;
				intaking = 0;
				std::cout << "Grabbing and Raising" << "\n" << std::flush;
				clutchMotor.moveVelocity(0);
				grab2 = 1;
				pistonC.set_value(1);
				clutchPos = 1;
				pistonB.set_value(1);
				pros::delay(100);
				clutchMotor.tarePosition();
				clutchMotor.moveRelative(3100,200);
				target = 3100;
				pros::delay(75);
				}
				db2 = 16;
		}

		if (grab2 && clutchMotor.isStopped() && clutchPos == 1) {
			if (clutchMotor.getPosition() > target - 100 || raised) {
				std::cout << "Raise Finished, Swapping To Intake, position: " << clutchMotor.getPosition() << " target: " << target<<  "\n" << std::flush;
				raised = true;
				pistonC.set_value(0);
				clutchPos = 0;
				intaking = 0;
			}
		} else if (clutchPos == 0 && liftMotor.getPosition() > 300) {
			std::cout << "Intake Free, Running" << "\n" << std::flush;
			clutchMotor.moveVelocity(-200);
			intaking = intaking + 1;
			if ((clutchMotor.getEfficiency() < 30 && intaking > 10) || reversing > 0) {
				clutchMotor.moveVelocity(200);
				int reversing = reversing + 1;
				if (reversing == 60)
					reversing = -1;
			}
		} else if (clutchPos == 0) {
			intaking = 0;
			clutchMotor.moveVelocity(0);
		}

		if (lowering) {
			if (clutchMotor.getEfficiency() > 20 || clutchMotor.getPosition() > -50)
				clutchMotor.moveVelocity(-200);
			else {
				clutchMotor.moveVoltage(0);
				std::cout<<"Lowered"<<"\n"<<std::flush;
				lowering = false;
				clutchMotor.tarePosition();
				pistonB.set_value(0);
			}
		}

		if (clutchMotor.getEfficiency() < 20 && clutchMotor.getCurrentDraw() < -10) {
			std::cout << "Low getEfficiency, stopping" << "\n" << std::flush;
			clutchMotor.moveVelocity(0);
		}
		if (swapButton.isPressed() && db5 <= 0) {
			if (clutchPos == 1) {
				clutchPos = 0;
				pistonC.set_value(0);
				grab2 = true;
			} else {
				clutchPos = 1;
				pistonC.set_value(1);
				grab2 = false;
			}
			db5 = 16;
		}
		if (forceRaisedButton.isPressed() && db6 <= 0) {
			if (!raised) {
				raised = true;
				clutchPos = 0;
				pistonC.set_value(0);
				grab2=true;
			} else {
				raised = false;
				clutchPos = 1;
				pistonC.set_value(1);
				grab2=false;
			}
			db6 = 16;
		}
		if (backPistonButton.isPressed() && db4 <= 0) {
			if (backPiston == 0) {
				backPiston = 1;
			} else {
				backPiston = 0;
			}
			backPistonOverride = true;
			db4 = 16;
		}
		if (SwapperButton.isPressed() && db7 <= 0) {
			swapper = 1;
			db7 = 16;
		}
		pistonS.set_value(swapper);
		if (backPistonOverride) {
			pistonB.set_value(backPiston);
		}
		if (clutchUpButton.isPressed()) {
			clutchOverride = true;
    	clutchMotor.moveVoltage(12000);
			std::cout << "Clutch Moving Up" << "\n" << std::flush;
    } else if (clutchDownButton.isPressed()) {
			clutchOverride = true;
     	clutchMotor.moveVoltage(-12000);
			std::cout << "Clutch Moving Down" << "\n" << std::flush;
    } else if (clutchOverride){
      clutchMotor.moveVoltage(0);
    }
		pros::delay(10);
		db--;
		db2--;
		db3--;
		db4--;
		db5--;
		db6--;
		db7--;
	}


}
