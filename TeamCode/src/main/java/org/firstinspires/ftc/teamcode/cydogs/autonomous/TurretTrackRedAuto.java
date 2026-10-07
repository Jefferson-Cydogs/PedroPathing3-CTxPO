package org.firstinspires.ftc.teamcode.cydogs.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.cydogs.components.Turret;
import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;

/**
 * Autonomous turret tracking for the red alliance.
 * Only uses targets whose name contains "red" and that look upright
 * (up tag, not the upside-down down tag).
 * Camera is mounted on the turret. If no target is found, the turret
 * sweeps across its range. When a target is found, the turret moves
 * until the tag is centered left/right in the camera view.
 */
@Autonomous(name = "Turret Track RED", group = "cydogs")
public class TurretTrackRedAuto extends LinearOpMode {

    private static final String SERVO_NAME = "turret_servo";
    private static final String WEBCAM_NAME = "Webcam 1";

    // How close to center counts as locked (degrees of bearing)
    private static final double LOCK_TOLERANCE_DEG = 3.0;

    // How strongly to correct toward the tag
    private static final double TRACK_GAIN = 0.15;

    // Turret degrees to move each loop while searching
    private static final double SEARCH_STEP_DEG = 1.0;

    private TagSight eyes;
    private Turret turret;

    // +1 toward max angle, -1 toward min angle
    private double searchDirection = 1.0;

    @Override
    public void runOpMode() {
        eyes = new TagSight(hardwareMap, WEBCAM_NAME, "red");
        turret = new Turret(this, SERVO_NAME);
        turret.initialize();

        telemetry.addLine("Turret Track RED ready");
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

            telemetry.addData("filter", "red + upright");
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

    /**
     * Move until the tag is near the horizontal center of the camera.
     * Does not use up/down aiming.
     */
    private void trackTag() {
        if (Math.abs(eyes.bearingDegrees) <= LOCK_TOLERANCE_DEG) {
            return;
        }
        // If motion goes the wrong way, use: -eyes.bearingDegrees * TRACK_GAIN
        double nudge = eyes.bearingDegrees * TRACK_GAIN;
        turret.nudgeByDegrees(nudge);
    }

    /** Sweep back and forth until a matching tag is found. */
    private void searchForTag() {
        if (turret.isAtMax()) {
            searchDirection = -1.0;
        } else if (turret.isAtMin()) {
            searchDirection = 1.0;
        }
        turret.nudgeByDegrees(searchDirection * SEARCH_STEP_DEG);
    }
}