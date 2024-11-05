package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;

public class Arm {
    public final DcMotorEx arm;
    private final CRServo wrist;
    private final CRServo intake;
    private final DcMotor armExtension;
//    public final TouchSensor slideZeroReset;

    public static final int LOW = 600;
    public static final int HIGH = 1600;
    public static final int GROUND = 0;

    public static final int EXTEND = 1800;

    public Arm(HardwareMap hardwareMap){
        //TODO: adjust values
//        this.slideZeroReset = hardwareMap.get(TouchSensor.class,"touch");
        this.arm = hardwareMap.get(DcMotorEx.class, RobotConfig.arm);
        this.wrist = hardwareMap.get(CRServo.class, RobotConfig.wrist);
        this.armExtension = hardwareMap.get(DcMotor.class, RobotConfig.armExtension);
        this.intake = hardwareMap.get(CRServo.class, RobotConfig.intake);

        armExtension.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        armExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtension.setTargetPosition(0);
        armExtension.setPower(0.0);

        arm.setTargetPosition(0);

        resetArm();
        resetArmExtension();
    }

    //wrist
    public void swingWristRight(){
        wrist.setPower(0.5);
    }

    public void swingWristLeft(){
        wrist.setPower(-0.5);
    }

    public void stopWrist(){
        wrist.setPower(0);
    }

    //wrist as Servo (not CRServo)
//    public void setWristPosition(int pos){
//        wrist.setPosition(pos);
//    }

    //intake
    public void intake(){
        intake.setPower(1.0);
    }

    public void outtake(){
        intake.setPower(-1.0);
    }

    public void stopIntake(){
        intake.setPower(0);
    }

    //arm
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

    public void setArmPosition(int armPosition){
        arm.setTargetPosition(armPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //armEx
    public void setArmExtensionPosition(int position){
        armExtension.setTargetPosition(position);
    }

    //general
    public void toPosition(int position, int rotator, boolean pivot, Telemetry t){
        //TODO
    }

    public void update() {
        //TODO
        //this touch sensor is flipped
//        if(!slideZeroReset.isPressed()) armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //claw rotator
//        if(rotateClaw) moveClawRotator(rotatorLevel);
//
//        wrist.setPosition(clawPivotPosition);

        if (armExtension.getTargetPosition() == 0){
//            armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            //the touch sensor is flipped
//            if(slideZeroReset.isPressed()) {
//                if (Math.abs(armExtension.getCurrentPosition()) < 50) armExtension.setPower(-0.1);
//                else armExtension.setPower(-0.6);
//            }
//            else armExtension.setPower(0.0);
            armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(armExtension.isBusy()) armExtension.setPower(1);
            else armExtension.setPower(0.0);
        } else if(armExtension.isBusy()) {
            armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(armExtension.getTargetPosition() == EXTEND) armExtension.setPower(1.0);
            else armExtension.setPower(1);
        } else armExtension.setPower(0.0);

        //arm (lift)
        if (arm.isBusy()) {
            if(arm.getTargetPosition() != GROUND) arm.setPower(0.8);
            else if (arm.getCurrentPosition() > arm.getTargetPosition()) {
                if (arm.getCurrentPosition() > 800) arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                else arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                arm.setPower(0.0);
            }
        } else arm.setPower(0.0);
    }

    public int getArmPosition() {return arm.getCurrentPosition();}
    public int getArmTargetPosition() {return arm.getTargetPosition();}

//    public double getWristPosition() {return wrist.getPosition();} //wrist as Servo (not CRServo)

    public double getArmExPower() {return armExtension.getPower();}
    public int getArmExPosition() {return armExtension.getCurrentPosition();}
    public int getArmExTargetPosition() {return armExtension.getTargetPosition();}
}