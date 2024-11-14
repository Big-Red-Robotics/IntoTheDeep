package org.firstinspires.ftc.teamcode.components;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

    public void update() {
        //armExtension
        if (armExtension.getTargetPosition() == 0){
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

    public Action toPosition(int pos) {
        return new LiftToPosition(pos);
    }

    public Action outtakeAction() {
        return new PowerIntake(-1.0);
    }
    public Action intakeAction() {
        return new PowerIntake(1.0);
    }
    public Action stopIntakeAction() {
        return new PowerIntake(0.0);
    }

    public int getArmPosition() {return arm.getCurrentPosition();}
    public int getArmTargetPosition() {return arm.getTargetPosition();}
//    public double getWristPosition() {return wrist.getPosition();} //wrist as Servo (not CRServo)
    public double getArmExPower() {return armExtension.getPower();}
    public int getArmExPosition() {return armExtension.getCurrentPosition();}
    public int getArmExTargetPosition() {return armExtension.getTargetPosition();}

    public class PowerIntake implements Action {
        private final double power;

        public PowerIntake(double power){
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(power);
            return false;
        }
    }

    public class LiftToPosition implements Action {
        private boolean initialized = false;
        private final int targetPosition;

        public LiftToPosition(int pos){
            this.targetPosition = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                setArmPosition(targetPosition);
                arm.setPower(1.0);
                initialized = true;
            }

            // checks lift's current position
            packet.put("current Arm position", getArmPosition());
            packet.put("target Arm position", getArmTargetPosition());
            if (arm.isBusy()) {
                // true causes the action to rerun
                return true;
            } else {
                // false stops action rerun
                arm.setPower(0);
                return false;
            }
        }
    }
}