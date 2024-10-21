package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.opmodes.drivetuning.MecanumDrive;

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
            if(gamepad2.left_trigger > 0.2){
            } else if (gamepad2.right_trigger > 0.2) {
            }

            //lift
            if (gamepad2.a) {
                //ground default
                arm.setArmPosition(Arm.GROUND);
                arm.setArmExtensionPosition(0);
            } else if (gamepad2.x) {
                //low basket
                arm.setArmPosition(Arm.HIGH);
                arm.setArmExtensionPosition(0);
            }else if (gamepad2.y) {
                //high basket
                arm.setArmPosition(Arm.HIGH);
                arm.setArmExtensionPosition(Arm.EXTEND);
            } else if (gamepad2.b) {
                //intake
                arm.setArmPosition(200);
                arm.setArmExtensionPosition(700);
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
            telemetry.addData("wrist",arm.getWristPosition());
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