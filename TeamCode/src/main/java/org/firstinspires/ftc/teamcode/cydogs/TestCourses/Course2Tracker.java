package org.firstinspires.ftc.teamcode.cydogs.TestCourses;

import com.pedropathing.api.PoseFactory;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseLED;
import org.firstinspires.ftc.teamcode.cydogs.vision.CameraPan;
import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Course2 with continuous camera tracking in the main loop.
 * Ivy sequence only waits for lock before each shoot pause.
 *
 * LED:
 *   red    = locked on target (from main loop when allowed)
 *   green  = shoot pause (same idea as Course2)
 *   purple = collect pause (same idea as Course2)
 */
@Autonomous(name = "Course2 Tracker", group = "Autonomous")
public class Course2Tracker extends LinearOpMode {

    private BaseLED statusLED;
    private Follower follower;
    private TagSight eyes;
    private CameraPan pan;

    // Match robot config
    private static final String WEBCAM_NAME = "Webcam 1";
    private static final String PAN_SERVO_NAME = "cameraPan";
    private static final String LED_NAME = "backLED";

    // Aiming
    private static final double LOCK_TOL_DEG = 3.0;
    private static final double TRACK_GAIN = 0.15;
    private static final double SEARCH_STEP_DEG = 1.0;
    private static final long LOCK_WAIT_TIMEOUT_MS = 3000;

    private double searchDirection = 1.0;

    // When false, green/purple steps own the LED (loop won't force red)
    private boolean statusLedFromLock = true;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses (same as Course2)
    private final Pose start = poseFactory.of(58.3239, 2.0611, 90);
    private final Pose shoot = poseFactory.of(58.2267, 15.7509, 90);
    private final Pose colectfromGardenStart = poseFactory.of(58.2267, 15.7509, 90);
    private final Pose colectfromGarden = poseFactory.of(6.2363, 8.5693, 90);
    private final Pose colectfromGardenControl1 = poseFactory.of(25.6113, 21.2454, 0);
    private final Pose shoot2Start = poseFactory.of(6.2363, 8.5693, 90);
    private final Pose shoot2 = poseFactory.of(30.7938, 14.0484, 90);
    private final Pose flowerStart = poseFactory.of(30.7938, 14.0484, 90);
    private final Pose flower = poseFactory.of(11.0529, 45.4562, 0);
    private final Pose shoot3Start = poseFactory.of(11.0529, 45.4562, 0);
    private final Pose shoot3 = poseFactory.of(26.6907, 21.6305, 90);
    private final Pose parkStart = poseFactory.of(26.6907, 21.6305, 90);
    private final Pose park = poseFactory.of(5.4051, 91.6807, 0);

    // Path methods (same as Course2)
    public Path shoot() {
        return line(start, shoot).linear(start, shoot);
    }

    public Path collectFromGarden() {
        return curve(colectfromGardenStart, colectfromGardenControl1, colectfromGarden)
                .linear(colectfromGardenStart, colectfromGarden);
    }

    public Path shoot2() {
        return line(shoot2Start, shoot2).linear(shoot2Start, shoot2);
    }

    public Path collectFromflower() {
        return line(flowerStart, flower).linear(flowerStart, flower);
    }

    public Path shoot3() {
        return line(shoot3Start, shoot3).linear(shoot3Start, shoot3);
    }

    public Path park() {
        return line(parkStart, park).linear(parkStart, park);
    }

    /** One frame of search/track. Called every loop (background aiming). */
    private void updateAim() {
        eyes.update();
        if (eyes.hasTarget) {
            if (Math.abs(eyes.bearingDegrees) > LOCK_TOL_DEG) {
                // If it tracks the wrong way, use: -eyes.bearingDegrees * TRACK_GAIN
                pan.nudgeByDegrees(eyes.bearingDegrees * TRACK_GAIN);
            }
        } else {
            if (pan.isAtMax()) {
                searchDirection = -1.0;
            } else if (pan.isAtMin()) {
                searchDirection = 1.0;
            }
            pan.nudgeByDegrees(searchDirection * SEARCH_STEP_DEG);
        }
    }

    private boolean isLocked() {
        return eyes.hasTarget && Math.abs(eyes.bearingDegrees) <= LOCK_TOL_DEG;
    }

    /**
     * Does not aim (loop already aims). Only waits until locked or timeout
     * so the sequence can continue to the green shoot pause.
     */
    private Command waitUntilLockedOrTimeout(long timeoutMs) {
        final long[] startMs = {0L};
        return Command.build()
                .setStart(() -> startMs[0] = System.currentTimeMillis())
                .setDone(() -> isLocked()
                        || System.currentTimeMillis() - startMs[0] > timeoutMs);
    }

    /** Shoot pause: green for 1s (same meaning as Course2). */
    private Command shootPause() {
        return sequential(
                instant(() -> {
                    statusLedFromLock = false;
                    statusLED.setGreen();
                }),
                waitMs(1000),
                instant(() -> {
                    statusLED.setOff();
                    statusLedFromLock = true;
                })
        );
    }

    /** Collect pause: purple for 1.5s (same meaning as Course2). */
    private Command collectPause() {
        return sequential(
                instant(() -> {
                    statusLedFromLock = false;
                    statusLED.setPurple();
                }),
                waitMs(1500),
                instant(() -> {
                    statusLED.setOff();
                    statusLedFromLock = true;
                })
        );
    }

    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot()),
                waitUntilLockedOrTimeout(LOCK_WAIT_TIMEOUT_MS),
                shootPause(),

                follow(follower, collectFromGarden()),
                collectPause(),

                follow(follower, shoot2()),
                waitUntilLockedOrTimeout(LOCK_WAIT_TIMEOUT_MS),
                shootPause(),

                follow(follower, collectFromflower()),
                collectPause(),

                follow(follower, shoot3()),
                waitUntilLockedOrTimeout(LOCK_WAIT_TIMEOUT_MS),
                shootPause(),

                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        statusLED = new BaseLED(hardwareMap, LED_NAME);
        eyes = new TagSight(hardwareMap, WEBCAM_NAME, null); // null = any tag; or "red" / "blue"
        pan = new CameraPan(this, PAN_SERVO_NAME);
        pan.initialize();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            updateAim(); // continuous tracking

            // Red only means locked, and only when green/purple are not showing
            if (statusLedFromLock) {
                if (isLocked()) {
                    statusLED.setRed();
                } else {
                    statusLED.setOff();
                }
            }

            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            telemetry.addData("locked", isLocked());
            telemetry.addData("hasTarget", eyes.hasTarget);
            telemetry.addData("bearing", "%.1f", eyes.bearingDegrees);
            telemetry.addData("pan deg", "%.1f", pan.getDegrees());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }
}