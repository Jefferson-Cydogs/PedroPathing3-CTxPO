package org.firstinspires.ftc.teamcode.cydogs.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.cydogs.components.Turret;
import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;

/**
 * Autonomous turret tracking for the blue alliance.
 * Only uses targets whose name contains "blue" and that look upright
 * (up cell, not the upside-down down cell).
 * Camera is mounted on the turret. If no target is found, the turret
 * sweeps across its range. When a target is found, the turret moves
 * until the tag is centered left/right in the camera view.
 */
@Autonomous(name = "Turret Track BLUE", group = "cydogs")
public class TurretTrackBlueAuto extends LinearOpMode {

    private static final String SERVO_NAME = "turret_servo";
    private static final String WEBCAM_NAME = "Webcam 1";

    private static final double LOCK_TOLERANCE_DEG = 3.0;
    private static final double TRACK_GAIN = 0.15;
    private static final double SEARCH_STEP_DEG = 1.0;

    private TagSight eyes;
    private Turret turret;
    private double searchDirection = 1.0;

    @Override
    public void runOpMode() {
        eyes = new TagSight(hardwareMap, WEBCAM_NAME, "blue");
        turret = new Turret(this, SERVO_NAME);
        turret.initialize();

        telemetry.addLine("Turret Track BLUE ready");
        telemetry.addData("turret max deg", "%.1f", Turret.MAX_TURRET_DEGREES);
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            eyes.update();

            if (eyes.hasTarget) {
                trackTag();
            } else {
                searchForTag();
            }

            telemetry.addData("filter", "blue + upright");
            telemetry.addData("hasTarget", eyes.hasTarget);
            telemetry.addData("target", eyes.targetName);
            telemetry.addData("bearing", "%.1f", eyes.bearingDegrees);
            telemetry.addData("roll", "%.1f", eyes.rollDegrees);
            telemetry.addData("upright", eyes.isUpright);
            telemetry.addData("turret deg", "%.1f", turret.getTurretDegrees());
            telemetry.addData("mode", eyes.hasTarget ? "TRACK" : "SEARCH");
            telemetry.update();

            sleep(20);
        }

        eyes.close();
    }

    private void trackTag() {
        if (Math.abs(eyes.bearingDegrees) <= LOCK_TOLERANCE_DEG) {
            return;
        }
        double nudge = eyes.bearingDegrees * TRACK_GAIN;
        turret.nudgeByDegrees(nudge);
    }

    private void searchForTag() {
        if (turret.isAtMax()) {
            searchDirection = -1.0;
        } else if (turret.isAtMin()) {
            searchDirection = 1.0;
        }
        turret.nudgeByDegrees(searchDirection * SEARCH_STEP_DEG);
    }
}