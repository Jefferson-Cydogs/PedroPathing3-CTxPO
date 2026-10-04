package org.firstinspires.ftc.teamcode.cydogs;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
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

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Base Autonomous", group = "Autonomous")
public class BaseAutonomous extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses
    private final Pose start = poseFactory.of(54.275, 7.875, 90);
    private final Pose initialScore = poseFactory.of(60, 12, 90);
    private final Pose collectfromgarden = poseFactory.of(12, 11, 90);
    private final Pose collectfromgardenControl1 = poseFactory.of(24, 18, 90);
    private final Pose scoreoppositehive = poseFactory.of(60, 125, 270);
    private final Pose scoreoppositehiveControl1 = poseFactory.of(24, 96, 225);

    // Path methods
    public Path initialScore() {
        return Paths.line(start, initialScore).linear(start, initialScore);
    }
    public Path collectfromgarden() {
        return Paths.curve(initialScore, collectfromgardenControl1, collectfromgarden).linear(initialScore, collectfromgarden);
    }
    public Path scoreoppositehive() {
        return Paths.curve(collectfromgarden, scoreoppositehiveControl1, scoreoppositehive).linear(collectfromgarden, scoreoppositehive);
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, initialScore()),
                follow(follower, collectfromgarden()),
                follow(follower, scoreoppositehive())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }
}
