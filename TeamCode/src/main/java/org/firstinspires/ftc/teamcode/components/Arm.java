package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;

public class Arm {
    public final DcMotorEx arm;
    private final ServoImplEx wrist;
    private final DcMotor armExtension;
    public final TouchSensor slideZeroReset;

    public Arm(HardwareMap hardwareMap){
        //TODO: adjust values
        this.slideZeroReset = hardwareMap.get(TouchSensor.class,"touch");
        this.arm = hardwareMap.get(DcMotorEx.class, RobotConfig.liftL);
        this.wrist = hardwareMap.get(ServoImplEx.class, RobotConfig.clawPivot);
        this.armExtension = hardwareMap.get(DcMotor.class, RobotConfig.armExtension);

        arm.setDirection(DcMotor.Direction.REVERSE);

        armExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtension.setTargetPosition(0);
        armExtension.setPower(0.0);

        wrist.setPwmRange(new PwmControl.PwmRange(1200, 1800));
        wrist.setPosition(0); //somehow this doesn't work
    }

    public void resetArm(){
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setTargetPosition(arm.getTargetPosition());
    }

    public void resetArmExtension(){
        armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtension.setTargetPosition(armExtension.getTargetPosition());
    }

    public void setArmExtensionPower(double power){
        armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtension.setPower(power);
        armExtension.setTargetPosition(armExtension.getCurrentPosition());
    }

    public void setLiftPosition(int armPosition){
        arm.setTargetPosition(armPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //armEx
    public void setArmExtensionPosition(int position){
        armExtension.setTargetPosition(position);
    }


    //claw rotator
    public void moveClawRotator(int pos){
        wrist.setPosition(pos);
    }

    public void setLiftPower(double power) {
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setPower(power);
        arm.setTargetPosition(arm.getCurrentPosition());
    }

    public void toPosition(int position, int rotator, boolean pivot, Telemetry t){
        //TODO
        //claw pivot
//        if (pivot) wrist.setPosition(CP_DEFAULT);
//        else wrist.setPosition(CP_FLIP);
//
////        clawPivot.setPosition(clawPivotPosition);
//
//        //set lift target
//        setLiftPosition(position);
//        arm.setPower(0.1);
//        rightLift.setPower(0.1);
//
//        //if retrack back. Default retract at -0.6 unless
//        if (armExtension.getTargetPosition() == 0){
//            armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            //the touch sensor is flipped
//            while(slideZeroReset.isPressed()) {
//                if (Math.abs(armExtension.getCurrentPosition()) < 50) armExtension.setPower(-0.1);
//                else armExtension.setPower(-0.6);
//            }
//            armExtension.setPower(0.0);
//        }
//        while (armExtension.isBusy()) {
//            // when it has a target.
//            armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            armExtension.setPower(0.6);
//        }
//        armExtension.setPower(0.0);
//
//        //set lift power
//        while (arm.isBusy() || rightLift.isBusy()) {
//            t.addData("target position", getLiftTargetPosition());
//            t.addData("current position", getLiftPosition());
//            t.update();
//            for(DcMotor lift: lifts){
//                if (lift.getCurrentPosition() > lift.getTargetPosition()) {
//                    if(hang) lift.setPower(1.0);
//                    else if (lift.getCurrentPosition() < 700 || lift.getCurrentPosition() > 1300) {
//                        //can depend on gravity
//                        if (lift.getCurrentPosition() < MIDDLE) lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                        else lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//                        lift.setPower(0.0);
//                    }
//                    else lift.setPower(0.3);
//                } else if(lift.getCurrentPosition() > LOW && lift.getTargetPosition() == MIDDLE) lift.setPower(0.3);
//                else lift.setPower(0.8);
//            }
//
//            if(getLiftTargetPosition() == 0 && getLiftPosition() < 15) break;
//        }
//        arm.setPower(0.5);
//        rightLift.setPower(0.5);
//
//        //claw rotator
//        moveClawRotator(rotator);
    }

    public void update(boolean rotateClaw) {
        //TODO
        //this touch sensor is flipped
//        if(!slideZeroReset.isPressed()) armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        //claw rotator
//        if(rotateClaw) moveClawRotator(rotatorLevel);
//
//        wrist.setPosition(clawPivotPosition);
//
//        if (armExtension.getTargetPosition() == 0){
//            armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            //the touch sensor is flipped
//            if(slideZeroReset.isPressed()) {
//                if (Math.abs(armExtension.getCurrentPosition()) < 50) armExtension.setPower(-0.1);
//                else armExtension.setPower(-0.6);
//            }
//            else armExtension.setPower(0.0);
//        } else if(armExtension.isBusy()) {
//            armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            if(armExtension.getTargetPosition() == 100) armExtension.setPower(1.0);
//            else if(getLiftTargetPosition() == LOW || getLiftTargetPosition() == MIDDLE || getLiftTargetPosition() == VERY_LOW || getLiftTargetPosition() == AUTON)
//                armExtension.setPower(0.6);
//            else armExtension.setPower(1.0);
//        } else armExtension.setPower(0.0);
//
//        //the actual lift part
//        for (DcMotor lift : lifts) {
//            if (lift.isBusy()) {
//                if(hang) lift.setPower(1.0);
//                else if(lift.getTargetPosition() == HIGH && lift.getCurrentPosition() > HANG + 20){
//                    //can depend on gravity
//                    lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                    lift.setPower(0.0);
//                } else if (lift.getCurrentPosition() > lift.getTargetPosition()) {
//                    if (lift.getCurrentPosition() < 700) {
//                        //can depend on gravity
//                        if ((lift.getCurrentPosition() < MIDDLE && lift.getTargetPosition() == 0) || (lift.getCurrentPosition() < MIDDLE + 100)) lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                        else lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//                        lift.setPower(0.0);
//                    } else lift.setPower(0.7);
//                } else if(lift.getCurrentPosition() > LOW && lift.getTargetPosition() == MIDDLE) lift.setPower(0.3);
//                else lift.setPower(0.8);
//            } else if (lift.getTargetPosition() == HIGH || lift.getTargetPosition() == GROUND) {
//                lift.setPower(0.0);
//            }
//        }
    }

    public int getArmPosition() {return arm.getCurrentPosition();}
    public int getArmTargetPosition() {return arm.getTargetPosition();}

    public double getWristPosition() {return wrist.getPosition();}

    public double getArmExPower() {return armExtension.getPower();}
    public int getArmExPosition() {return armExtension.getCurrentPosition();}
    public int getArmExTargetPosition() {return armExtension.getTargetPosition();}
}