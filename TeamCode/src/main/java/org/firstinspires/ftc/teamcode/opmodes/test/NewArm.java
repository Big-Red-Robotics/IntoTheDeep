package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class NewArm extends LinearOpMode {

    @Override
    public void runOpMode() {
        //initialize components
        CRServo leftArm = hardwareMap.get(CRServo.class, "lA");
        CRServo rightArm = hardwareMap.get(CRServo.class, "rA");
        CRServo leftDServo = hardwareMap.get(CRServo.class, "lD");
        CRServo rightDServo = hardwareMap.get(CRServo.class, "rD");
        ServoImplEx claw = hardwareMap.get(ServoImplEx.class, "claw");

        double speed = 0.5;

        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            //move components and log data

            if(gamepad1.a){
                leftArm.setPower(0.5);
                rightArm.setPower(0.5);
            } else if(gamepad1.y){
                leftArm.setPower(-0.5);
                rightArm.setPower(-0.5);
            } else {
                leftArm.setPower(0);
                rightArm.setPower(0);
            }

            if(gamepad1.b){
                claw.setPosition(0.5);
            } else if (gamepad1.x) {
                claw.setPosition(0);
            }

            if (gamepad1.dpad_right){
                rightDServo.setPower(speed);
                leftDServo.setPower(-speed);
            } else if (gamepad1.dpad_left) {
                rightDServo.setPower(-speed);
                leftDServo.setPower(speed);
            } else if (gamepad1.dpad_up) {
                rightDServo.setPower(speed);
                leftDServo.setPower(speed);
            } else if (gamepad1.dpad_down) {
                rightDServo.setPower(-speed);
                leftDServo.setPower(-speed);
            } else {
                rightDServo.setPower(0);
                leftDServo.setPower(0);
            }

            telemetry.addData("data", null);
            telemetry.update();
        }
    }
}