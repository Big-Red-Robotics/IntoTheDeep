//This is the code that you can use as an example for other teleops
//Has drivetrain and arm (arm up and down & arm extend out)
//Isha First commit check

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Isha Now Basic: Omni Linear OpMode", group= "Linear OpMode")
public class IshaWorkingOpmodeTemplateForOpmodes extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors and arm motor.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor arm = null;
    private DcMotor armExtend = null;

    // Deadzone value //Chatgpt add for the robot to stop moving after stop touching joystick
    private static final double DEADZONE = 0.05;

   private final double ARM_SPEED = 0.5;

    @Override
    public void runOpMode() {
        // Initialize hardware variables
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_motor");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_motor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_motor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_motor");
        arm = hardwareMap.get(DcMotor.class, "up_arm_motor");
        armExtend = hardwareMap.get(DcMotor.class, "extend_arm_motor");

        // Set motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

       //To limit the drifting/the power of the motor keeping the wheels keep moving even after joystick stop
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Provide feedback that initialization is complete
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Get joystick values for drive control
            double axial = -gamepad1.left_stick_y;  // Forward/Backward
            double lateral = gamepad1.left_stick_x; // Left/Right (Strafing)
            double yaw = gamepad1.right_stick_x;    // Rotational control

           /* // Apply deadzone to joystick inputs- Deadzone just restricts the joysticks so that it won't move to much even if the driver accidently moves the joystick slightly
            if (Math.abs(axial) < DEADZONE) axial = 0;
            if (Math.abs(lateral) < DEADZONE) lateral = 0;
            if (Math.abs(yaw) < DEADZONE) yaw = 0; */

            // Calculate individual wheel powers for Omni Drive
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize wheel powers to prevent exceeding motor power limits
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Set power to the motors
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

           // Arm control with D-pad /use gamepad2 to switch it
            if (gamepad1.dpad_up) {
                arm.setPower(ARM_SPEED); // Move arm up
            } else if (gamepad1.dpad_down) {
                arm.setPower(-ARM_SPEED); // Move arm down
            } else {
                arm.setPower(0); // Stop arm when no button is pressed
            }

             // Arm extend or retract with D-pad
            if (gamepad1.dpad_right) {
                armExtend.setPower(ARM_SPEED); // extend arm
            } else if (gamepad1.dpad_left) {
                armExtend.setPower(-ARM_SPEED); // retract arm
            } else {
                armExtend.setPower(0); // Stop arm when no button is pressed
            }

            // Display telemetry data
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Arm Motor Power", arm.getPower());
            telemetry.addData("Arm Motor Power", armExtend.getPower());
            telemetry.update();
        }
    }
}


