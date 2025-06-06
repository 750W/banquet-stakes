#include <fstream>
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

using namespace pros;

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-16, -15, -14},
                            pros::MotorGearset::blue); // front, middle, back
pros::MotorGroup rightMotors({7, 8, 9}, pros::MotorGearset::blue);
inline pros::MotorGroup driveleft({-16, -15, -14});
pros::Imu imu(11);

// // tracking wheels
// // horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
// pros::Rotation horizontalEnc(20);
// // vertical tracking wheel encoder. Rotation sensor, port 11, reversed
// pros::Rotation verticalEnc(-11);
// // horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
// lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// // vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
// lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12, // 12 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(6, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            4, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(0.9, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             0, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// intake
Motor intake (12, MotorGearset::blue, MotorEncoderUnits::degrees);
int intakeControl = 1;

// ladybrown
Motor lb (-19, MotorGearset::blue, MotorEncoderUnits::degrees);
Rotation rotSensor(5);
const int numStates = 4;
int states[numStates] = {0, 84, 300, 450};
int currentState = 0;
int target = 0;

// pneumatics
ADIDigitalOut clamp ('A');
ADIDigitalOut rush('H');

// color sensor
Optical colorSensor(13);
double minHue;
double maxHue;
bool sorting = true;

/* LADY BROWN */

void nextState() {
	++currentState;
	target = states[currentState %= numStates];
}

void lbControl() {
	double kP = 1;
	int pos = rotSensor.get_position()/100;
	lb.move(kP*(target - pos));
}

/* COLOR SENSOR */

int readAuton() {
	std::ifstream autonFile("/usd/auton.txt");
	int auton;
	autonFile >> auton;
	autonFile.close();
	return auton;
}

void readColor() {
	int auton = readAuton();
	if (auton % 2 == 0) { // blue
		minHue = 7; // rejects red
		maxHue = 25;
	} else { // red
		minHue = 190; // rejects blue
		maxHue = 230;
	}
}

void colorSort() {
	if (sorting && colorSensor.get_hue() >= minHue && colorSensor.get_hue() <= maxHue) {
        pros::lcd::print(7, "WE KILL RAM");
        pros::delay(175);
        // momentarily reverse intake
        intakeControl = -1;
        pros::delay(50);
        intakeControl = 1;
    } else {
      pros::lcd::print(7, "we love ram");
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
	target = 0;
  rotSensor.set_reversed(true);
	rotSensor.reset_position();
	colorSensor.set_led_pwm(100);
	readColor();

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
			            // Print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // Log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::lcd::print(3, "Left Total: %f", leftMotors.get_power_all());
            pros::lcd::print(4, "Right Total: %f", rightMotors.get_power_all());
            pros::lcd::print(5, "Rotation: %d", rotSensor.get_position()/100);
            pros::lcd::print(6, "Hue: %f", colorSensor.get_hue());
            int pos = rotSensor.get_position()/100;
	          int vel = 1*(target - pos);
            //pros::lcd::print(6, "LB VELOCITY: %d", vel);

            // Log color sensor status
            if (sorting) {
              if (maxHue == 25) {
                controller.print(0, 0, "KILL RED       ");
              } else {
                controller.print(0, 0, "KILL BLUE      ");
              }
            } else {
              controller.print(0, 0, "COLOR SORT OFF");
            }
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);

        }
    });

	 // Ladybrown velocity task
    pros::Task liftControlTask([]{
      while(true) {
        lbControl();
        pros::delay(20);
      }
    });

    // Color sensor task
    pros::Task colorSensorTask([]{
      while(true) {
        colorSort();
        pros::delay(10);
      }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}


void blue_goal_rush() {};
void blue_ring_rush() {
  chassis.setPose(0, 0, 0);
  /*
  chassis.moveToPose(-96.2, 59.7, 120, 2000, {.forwards=true, .maxSpeed = 120});
  chassis.moveToPose(-60.2, 59.5, 120, 2000, {.forwards=true, .maxSpeed = 120});
  chassis.moveToPose(-59.2, 60.8, 120, 1000, {.forwards=true, .maxSpeed = 100});
  chassis.moveToPose(-30.2, 91.1, 120, 3000, {.forwards=true, .maxSpeed = 120});
  chassis.moveToPose(-9.8, 110.2, 120, 2000, {.forwards=true, .maxSpeed = 120});
  chassis.moveToPose(-9.3, 112.0, 120, 500, {.forwards=true, .maxSpeed = 80});
  chassis.moveToPose(-10.9, 127.5, 120, 1500, {.forwards=true, .maxSpeed = 100});
  chassis.moveToPose(-30.7, 124.8, 120, 2000, {.forwards=true, .maxSpeed = 120});
  chassis.moveToPose(-50.4, 121.4, 120, 2000, {.forwards=true, .maxSpeed = 120});
  chassis.moveToPose(-59.9, 119.8, 120, 1500, {.forwards=true, .maxSpeed = 100});
  chassis.moveToPose(-80.2, 130.7, 120, 2500, {.forwards=true, .maxSpeed = 120});
  chassis.moveToPose(-120.1, 146.9, 120, 3000, {.forwards=true, .maxSpeed = 120});
  chassis.moveToPose(-160.4, 164.5, 120, 2500, {.forwards=true, .maxSpeed = 120});
  chassis.moveToPose(-169.3, 168.7, 120, 1500, {.forwards=true, .maxSpeed = 100});
  chassis.moveToPose(-155.1, 120.8, 120, 4000, {.forwards=false, .maxSpeed = 120});
  chassis.moveToPose(-140.5, 70.9, 120, 4000, {.forwards=false, .maxSpeed = 120});
  chassis.moveToPose(-125.4, 19.0, 120, 4000, {.forwards=false, .maxSpeed = 120});
  chassis.moveToPose(-120.0, 0, 120, 2000, {.forwards=false, .maxSpeed = 100});
  chassis.setPose(-120.0, 0, 0);*/
};
void red_goal_rush() {};
void red_ring_rush() {
  chassis.setPose(0, 0, 0);
/*
  chassis.moveToPose(96.2, 59.7, 120, 2000, {.forwards=true, .maxSpeed = 120});
  chassis.moveToPose(60.2, 59.5, 120, 2000, {.forwards=true, .maxSpeed = 120});
  chassis.moveToPose(59.2, 60.8, 120, 1000, {.forwards=true, .maxSpeed = 100});
  chassis.moveToPose(30.2, 91.1, 120, 3000, {.forwards=true, .maxSpeed = 120});
  chassis.moveToPose(9.8, 110.2, 120, 2000, {.forwards=true, .maxSpeed = 120});
  chassis.moveToPose(9.3, 112.0, 120, 500, {.forwards=true, .maxSpeed = 80});
  chassis.moveToPose(10.9, 127.5, 120, 1500, {.forwards=true, .maxSpeed = 100});
  chassis.moveToPose(30.7, 124.8, 120, 2000, {.forwards=true, .maxSpeed = 120});
  chassis.moveToPose(50.4, 121.4, 120, 2000, {.forwards=true, .maxSpeed = 120});
  chassis.moveToPose(59.9, 119.8, 120, 1500, {.forwards=true, .maxSpeed = 100});
  chassis.moveToPose(80.2, 130.7, 120, 2500, {.forwards=true, .maxSpeed = 120});
  chassis.moveToPose(120.1, 146.9, 120, 3000, {.forwards=true, .maxSpeed = 120});
  chassis.moveToPose(160.4, 164.5, 120, 2500, {.forwards=true, .maxSpeed = 120});
  chassis.moveToPose(169.3, 168.7, 120, 1500, {.forwards=true, .maxSpeed = 100});
  chassis.moveToPose(155.1, 120.8, 120, 4000, {.forwards=false, .maxSpeed = 120});
  chassis.moveToPose(140.5, 70.9, 120, 4000, {.forwards=false, .maxSpeed = 120});
  chassis.moveToPose(125.4, 19.0, 120, 4000, {.forwards=false, .maxSpeed = 120});
  chassis.moveToPose(120.0, 0, 120, 2000, {.forwards=false, .maxSpeed = 100});
  chassis.setPose(120.0, 0, 0);*/
};

// need to update values!!!
void solo_awp() {
  chassis.setPose(0, 0, 0);
  // lb
  target = 500;
  pros::delay(750);

  // first mogo
  chassis.moveToPoint(0, -37, 1000, {.forwards=false}, false); // first mogo
  target = -70;
  clamp.set_value(true);
  pros::delay(1000);

  // turn to mogo ring 1
  chassis.turnToHeading(162,1000,{.direction=AngularDirection::CW_CLOCKWISE},false); // turn to square
  // pros::delay(1000);
  chassis.setPose(0, 0, 0);
  pros::delay(500);

  //intake square
  intake.move(-INTAKE_SPEED);
  chassis.moveToPoint(0, 18, 1000, {}, false);
  pros::delay(500);
  /*
  chassis.turnToHeading(-40,1000,{},false); // turn to second ring
  chassis.setPose(0, 0, 0);
  pros::delay(1000);
  chassis.moveToPoint(0, 12, 1000, {}, false); //go to second ring
  pros::delay(500);
  */
  chassis.setPose(0, 0, 0);
  chassis.turnToHeading(-107,1000,{},false); // turn to third ring
  /*
  pros::delay(500);
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, 30, 1000, {}, false); //go to third ring
  pros::delay(500);
  chassis.turnToHeading(-180,1000,{},false); // turn to third ring
  pros::delay(500);
*/
  //
}

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
  int autonNum = readAuton();
	switch (autonNum) {
		case 0: // start: red left
		  solo_awp();
			break;
    case 1: // start: blue right
			solo_awp();
			break;
    case 2: // start: red left
			red_ring_rush();
			break;
		case 3: // start: blue right
			blue_ring_rush();
			break;
		case 4: // start: red right
			red_goal_rush();
			break;
    case 5: // start: blue left
		  blue_goal_rush();
			break;
	}
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // variables
	  bool clamp_extended = false;
    bool pusher_extended = false;
    bool lb_pull = false;

    chassis.setBrakeMode(E_MOTOR_BRAKE_COAST);
    // loop to continuously update motors
    while (true) {

		/* DRIVE */
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.curvature(leftY, rightX, 0.2);

		/* INTAKE */
		if (controller.get_digital(E_CONTROLLER_DIGITAL_R2)) { // Run intake
		intake.move(INTAKE_SPEED * intakeControl);
		} else if (controller.get_digital(E_CONTROLLER_DIGITAL_R1)) {
		intake.move(-INTAKE_SPEED * intakeControl);
		} else {
		intake.move(0);
		}

		/* LADY BROWN */
		if(controller.get_digital_new_press(E_CONTROLLER_DIGITAL_L2)) // Move lady brown to next position
			nextState();

		if(controller.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) { // Bring lady brown to pull/pick up mobile goals
			if (lb_pull)
				target = 700;
			else
				target = 0;
			currentState = 0; // reset current state back to 0
			lb_pull = !lb_pull;
		}

		/* PNEUMATICS */
		if(controller.get_digital_new_press(E_CONTROLLER_DIGITAL_L1))
			pusher_extended = !pusher_extended;

		if((controller.get_digital_new_press(E_CONTROLLER_DIGITAL_A)))
			clamp_extended = !clamp_extended;

    if((controller.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)))
			sorting = !sorting;

		// Update pneumatics states
		clamp.set_value(clamp_extended);
		rush.set_value(pusher_extended);


        // delay to save resources
        pros::delay(10);
    }
}
