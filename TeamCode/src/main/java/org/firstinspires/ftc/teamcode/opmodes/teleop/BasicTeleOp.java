package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.drive.MecanumDrive;

@TeleOp(name="2024-2025 INTOTHEDEEP")
public class BasicTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        //initialize components
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        Arm arm = new Arm(hardwareMap);

        telemetry.addLine("waiting to start!");
        telemetry.addLine("test!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            //chassis
            double speed = 1.0;
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * speed,
                            -gamepad1.left_stick_x * speed
                    ),
                    -gamepad1.right_stick_x * speed
            ));

            drive.updatePoseEstimate();

            //claw intake & outtake
            if(gamepad2.left_bumper)
                arm.intake();
            else if (gamepad2.right_bumper)
                arm.outtake();
            else arm.stopIntake();

            //claw rotator (wrist 1)
            if(gamepad2.left_trigger > 0.2)
                arm.swingWristLeft();
            else if (gamepad2.right_trigger > 0.2)
                arm.swingWristRight();
            else arm.stopWrist();

            //lift
            if (gamepad2.a) {
                //ground default
                arm.setArmPosition(Arm.GROUND);
                arm.setArmExtensionPosition(0);
            } else if (gamepad2.x) {
                //low basket
                arm.setArmPosition(Arm.LOW);
                arm.setArmExtensionPosition(0);
            }else if (gamepad2.y) {
                //high basket
                arm.setArmPosition(Arm.HIGH);
                arm.setArmExtensionPosition(Arm.EXTEND);
            } else if (gamepad2.b) {
                //intake
                arm.setArmPosition(Arm.VERY_LOW);
                arm.setArmExtensionPosition(700);
            }

            if(gamepad1.y) {
                arm.setArmExtensionPosition(0);
                arm.setArmPosition(5000);
                arm.hang = true;
            } else if(gamepad1.x){
                arm.setArmExtensionPosition(0);
                arm.setArmPosition(5500);
                arm.hang = true;
            } else if(gamepad1.b && arm.hang){
                arm.setArmExtensionPosition(0);
                arm.setArmPosition(-500);
            }

            if (gamepad2.dpad_down) arm.setArmPosition(200);
            if (gamepad2.dpad_down) arm.setArmPosition(Arm.VERY_LOW);
            else if (gamepad2.dpad_left) arm.setArmExtensionPosition(500);
            else if (gamepad2.dpad_up) arm.setArmPosition(Arm.GROUND);
            else if (gamepad2.dpad_right) arm.setArmExtensionPosition(1550);

            arm.update();

            //telemetry
            telemetry.addData("arm power", arm.getArmPower());
            telemetry.addData("arm position", arm.getArmPosition());
            telemetry.addData("arm target", arm.getArmTargetPosition());
            telemetry.addLine();
//            telemetry.addData("wrist",arm.getWristPosition());
            telemetry.addLine();
            telemetry.addData("ArmEx power", arm.getArmExPower());
            telemetry.addData("ArmEx position", arm.getArmExPosition());
            telemetry.addData("ArmEx target position", arm.getArmExTargetPosition());
//            telemetry.addData("ArmEx runmode", arm.getRunMode());
            telemetry.addLine();
//            telemetry.addData("limit switch", arm.slideZeroReset.getValue());

            telemetry.update();
        }
    }
}