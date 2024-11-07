package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.teaminfo.TeamColor;

@Disabled
@Autonomous(name="Basic Autonomous - LEFT")
public class BasicRightAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() {
        //initialize components
        Arm arm = new Arm(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("waiting to start!");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            if(gamepad1.b || gamepad2.b) RobotConfig.teamColor = TeamColor.RED;
            if(gamepad1.x || gamepad2.x) RobotConfig.teamColor = TeamColor.BLUE;

            arm.setArmExtensionPosition(0);

            if(gamepad1.right_bumper) arm.resetArmExtension();


            telemetry.addData("Team color", RobotConfig.teamColor);
            telemetry.addData("Initial side", RobotConfig.initialSide);
            telemetry.update();

            sleep(100);
        }

        //TODO: set the write coordinates for initial point
        //also could do this when instantiating MecanumDrive, but...
        boolean isRed = RobotConfig.teamColor == TeamColor.RED;
        Pose2d initialPose;
        if (isRed) initialPose = new Pose2d(60, -24, Math.toRadians(-90));
        else initialPose = new Pose2d(-60, 24, Math.toRadians(90));

        drive.pose = initialPose;

        Action tab1 = drive.actionBuilder(initialPose)
                .lineToX(isRed ? 55 : -55)
                .lineToY(isRed ? 60 : -60)
                .build();

        while(opModeIsActive()) {
            //autonomous code
            Actions.runBlocking(tab1);


        }
    }
}
