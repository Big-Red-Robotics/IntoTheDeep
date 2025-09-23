package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Isha Update Real Practice TeleOp", group = "TeleOp") //if telemetry says second, this is running
public class firstTeleop extends OpMode {

    // Declare hardware variables for motors and servo
    private DcMotor leftFrontMotor = null;      // Left front motor for driving
    private DcMotor rightFrontMotor = null; // Right front motor for driving
    private DcMotor leftBackMotor = null;      // Left back motor for driving
    private DcMotor rightBackMotor = null; // Right back motor for driving
    //private DcMotor moveArmUpMotor1SimultaneousWithMotor2 = null;         // Servo for controlling a mechanism (e.g., arm)
    //private DcMotor moveArmUpMotor2SimultaneousWithMotor1 = null;

    private ElapsedTime runtime = new ElapsedTime();  // Timer for runtime (optional, for autonomous or timing)

    // This method is called when the OpMode is initialized
    @Override
    public void init() {
        // Initialize motors and servo using the hardware map
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front_motor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front_motor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back_motor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back_motor");
       // moveArmUpMotor1SimultaneousWithMotor2 = hardwareMap.get(Servo.class, "up_arm_motor1");
        //moveArmUpMotor2SimultaneousWithMotor1 = hardwareMap.get(Servo.class, "up_arm_motor2");
        //(I don't know what this line if for or when I might need it "optional")private ElapsedTime runtime = new ElapsedTime();  // Timer for runtime (optional, for autonomous or timing)

        // Set motor directions (if needed)
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);  // If the right front motor needs to spin in reverse
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE); // If the right front motor needs to spin in reverse
        //leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        //if the arm moving up doesn't work properly, then add the reverse command (copy above) thing for one, or both arm motors
        // Initialize runtime (optional)
        runtime.reset();

        // Display a message in the Driver Station
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // This method is called repeatedly while the OpMode is running
    @Override
    public void loop() {
       // Get the joystick values to drive the robot
        double leftPower = -gamepad1.left_stick_y;  // Invert to match typical control behavior (forward is negative)
        double rightPower = -gamepad1.right_stick_y;

        // Set power to motors for movement
        leftFrontMotor.setPower(leftPower);
        rightFrontMotor.setPower(rightPower);
        leftBackMotor.setPower(leftPower);
        rightBackMotor.setPower(rightPower);

        // Control the arm servo using the d-pad on the gamepad
      /*  if (gamepad1.dpad_up) {    /*CAUTION: the way the code is right now, the arm will fully fully extend with dpad up,
      not an extend in two parts, need to code this (easy, just have a different position, or maybe not even dpad)
            moveArmUpMotor1SimultaneousWithMotor2.setPosition(1.0);  // Move the arm motor to the up position (fully extended)
            moveArmUpMotor2SimultaneousWithMotor1.setPosition(1.0);
        } else if (gamepad1.dpad_down) {
            moveArmUpMotor1SimultaneousWithMotor2.setPosition(0.0);  // Move the arm motor to the down position (fully retracted)
            moveArmUpMotor2SimultaneousWithMotor1.setPosition(0.0);
        } */

        // Display telemetry data to the Driver Station (for debugging and feedback)
        telemetry.addData("Left Motor Power", leftPower);
        telemetry.addData("Right Motor Power", rightPower);
       // telemetry.addData("Arm Servo Position", moveArmUpMotor1SimultaneousWithMotor2.getPosition());
        telemetry.update();
    }

    // This method is called when the OpMode is stopped
    @Override
    public void stop() {
        // Stop motors when the program ends (to ensure they don't continue running)
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
}

/* This is the motor ports for the old robot that I am working on- the one with the control hub
wifi " FTC- Xam3 "

leftFrontMotor - Front left motor - Port 1 on Control Hub
rightFrontMotor - Front right motor - Port 0 on Expansion Hub
leftBackMotor - Back left motor- Port 3 on Control Hub
rightBackMotor - Back right motor - Port 2 on Expansion Hub
moveArmUpMotor1SimultaneousWithMotor2-   (Control hub port 2?)
moveArmUpMotor2SimultaneousWithMotor1 -
 */

/* Update December 17th Tuesday 2024, added the back motors + arm motors. If something goes
wrong, edit out the arm thing code and run/test with only the drive train motors
 */

/* Code for if I am using and servo (ex: for the arm)
 private Servo armServo = null;         // Servo for controlling a mechanism (e.g., arm)
  armServo = hardwareMap.get(Servo.class, "arm_servo");

  // Control the arm servo using the d-pad on the gamepad
        if (gamepad1.dpad_up) {
            armServo.setPosition(1.0);  // Move the arm servo to the up position (fully extended)
        } else if (gamepad1.dpad_down) {
            armServo.setPosition(0.0);  // Move the arm servo to the down position (fully retracted)
        }

        telemetry.addData("Arm Servo Position", armServo.getPosition());
 */