package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Arm;

@TeleOp
public class DifferentialTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        //initialize components

        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        Arm arm = new Arm(hardwareMap);


        while(opModeIsActive()) {
            //move components and log data
            //gamepad1 dpad
            if(gamepad1.dpad_up)
                arm.dUp();
            else if(gamepad1.dpad_down)
                arm.dDown();
            else if(gamepad1.dpad_right)
                arm.dRight();
            else if (gamepad1.dpad_left)
                arm.dLeft();
            else
                arm.stopDs();


            if(gamepad1.a)
                arm.claw.setPosition(0.2);
            else if (gamepad1.b)
                arm.claw.setPosition(0.6);



            telemetry.addData("Encoder",arm.claw.getPosition());
            telemetry.addData("Lift Encoder Value:", arm.getArmPosition());
            telemetry.addData("Arm Extension Encoder: ", arm.getArmExPosition());
            telemetry.addData("data", null);
            telemetry.update();
        }
    }
}