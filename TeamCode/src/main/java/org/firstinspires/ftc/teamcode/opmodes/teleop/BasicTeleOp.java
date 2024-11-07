package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.opmodes.drivetuning.MecanumDrive;

@TeleOp(name="2024-2025 INTOTHEDEEP")
public class BasicTeleOp extends LinearOpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    @Override
    public void runOpMode() {
        //initialize components
       // MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        Arm arm = new Arm(hardwareMap);
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "dFL");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "dBL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "dFR");
        rightBackDrive = hardwareMap.get(DcMotor.class, "dBR");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("waiting to start!");
        telemetry.addLine("test!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            //chassis
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);


            //claw intake & outtake
            if(gamepad2.left_bumper)
                arm.intake();
            else if (gamepad2.right_bumper)
                arm.outtake();
            else arm.stopIntake();

            //claw rotator (wrist 1)
            if(gamepad2.left_trigger > 0.2)
                arm.swingWristRight();
            else if (gamepad2.right_trigger > 0.2)
                arm.swingWristLeft();
            else arm.stopWrist();

            //lift
            if (gamepad2.a) {
                //ground default
                arm.setArmPosition(Arm.GROUND);
                arm.setArmExtensionPosition(0);

                arm.hang = false;
            } else if (gamepad2.x) {
                //low basket
                arm.setArmPosition(Arm.LOW);
                arm.setArmExtensionPosition(0);

                arm.hang = false;
            }else if (gamepad2.y) {
               // if()
                //high basket
                arm.setArmPosition(Arm.HIGH);
                arm.setArmExtensionPosition(Arm.EXTEND);

                arm.hang = false;
            } else if (gamepad2.b) {
                //intake
                arm.setArmPosition(200);
                arm.setArmExtensionPosition(700);

                arm.hang = false;
            }

            if(gamepad1.y) {
                arm.setArmPosition(2200);
                arm.hang = true;
            } else if(gamepad1.x){
                arm.setArmPosition(2550);
                arm.hang = true;
            } else if(gamepad1.b && arm.hang){
                arm.setArmPosition(-500);
            }


            if (gamepad2.dpad_down) arm.setArmPosition(200);
            else if (gamepad2.dpad_left) arm.setArmExtensionPosition(500);
            else if (gamepad2.dpad_up) arm.setArmPosition(Arm.GROUND);
            else if (gamepad2.dpad_right) arm.setArmExtensionPosition(1550);


            arm.update();

            //telemetry
            telemetry.addData("arm position", arm.getArmPosition());
            telemetry.addData("arm target", arm.getArmTargetPosition());
            telemetry.addLine();
//            telemetry.addData("wrist",arm.getWristPosition());
            telemetry.addLine();
            telemetry.addData("ArmEx power", arm.getArmExPower());
            telemetry.addData("ArmEx position", arm.getArmExPosition());
            telemetry.addData("ArmEx target position", arm.getArmExTargetPosition());
            telemetry.addLine();
//            telemetry.addData("limit switch", arm.slideZeroReset.getValue());

            telemetry.update();
        }
    }
}