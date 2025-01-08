package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Arm;

@TeleOp
public class EncoderTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        //initialize components

        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        Arm arm = new Arm(hardwareMap);


        while(opModeIsActive()) {
            //move components and log data

            telemetry.addData("Lift Encoder Value:", arm.getArmPosition());
            telemetry.addData("Arm Extension Encoder: ", arm.getArmExPosition());
            telemetry.addData("data", null);
            telemetry.update();
        }
    }
}