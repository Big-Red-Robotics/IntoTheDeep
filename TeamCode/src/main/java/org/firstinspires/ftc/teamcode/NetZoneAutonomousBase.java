package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;

public abstract class NetZoneAutonomousBase extends LinearOpMode {
    protected enum Alliance {
        BLUE,
        RED
    }

    private final Alliance alliance;

    protected NetZoneAutonomousBase(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void runOpMode() {
        telemetry.addLine("NetZone autonomous ready.");
        telemetry.addData("Alliance", alliance);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        runAutonomousSequence();

        telemetry.addLine("NetZone autonomous complete.");
        telemetry.update();
        sleep(250);
    }

    private void runAutonomousSequence() {
        List<Step> steps = buildSteps();
        ElapsedTime stepTimer = new ElapsedTime();

        for (Step step : steps) {
            if (!opModeIsActive()) {
                return;
            }

            telemetry.addData("Step", step.description);
            telemetry.addData("Target", "%.1f sec", step.durationSeconds);
            telemetry.update();

            stepTimer.reset();
            while (opModeIsActive() && stepTimer.seconds() < step.durationSeconds) {
                telemetry.addData("Elapsed", "%.1f sec", stepTimer.seconds());
                telemetry.update();
                idle();
            }
        }
    }

    private List<Step> buildSteps() {
        List<Step> steps = new ArrayList<>();
        String strafeDirection = alliance == Alliance.BLUE ? "strafe left" : "strafe right";

        steps.add(new Step("Leave start tile", 1.0));
        steps.add(new Step("Drive toward NetZone", 1.5));
        steps.add(new Step("Align with target (" + strafeDirection + ")", 1.0));
        steps.add(new Step("Score preload", 1.0));
        steps.add(new Step("Park in NetZone", 1.0));

        return steps;
    }

    protected static class Step {
        private final String description;
        private final double durationSeconds;

        private Step(String description, double durationSeconds) {
            this.description = description;
            this.durationSeconds = durationSeconds;
        }
    }
}
