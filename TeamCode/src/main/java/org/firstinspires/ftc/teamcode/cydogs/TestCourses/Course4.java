package org.firstinspires.ftc.teamcode.cydogs.TestCourses;

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

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseLED;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Course 4", group = "Autonomous")
public class Course4 extends LinearOpMode {
    private BaseLED statusLED;
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58.5821, 140.2044, -90);
    private final Pose waitthenshoot = poseFactory.of(58.7432, 129.1058, -90);
    private final Pose collectfromclosestflowerStart = poseFactory.of(58.7432, 129.1058, -90);
    private final Pose collectfromclosestflower = poseFactory.of(46.6072, 131.1715, -90);
    private final Pose gotoferthestflowerStart = poseFactory.of(46.6072, 131.1715, -90);
    private final Pose gotoferthestflower = poseFactory.of(9.6588, 47.2637, 0);
    private final Pose shootcolectformgardenStart = poseFactory.of(9.6588, 47.2637, 0);
    private final Pose shootcolectformgarden = poseFactory.of(9.8832, 9.8175, 90);
    private final Pose parkStart = poseFactory.of(9.8832, 9.8175, 90);
    private final Pose park = poseFactory.of(7.7473, 117.1077, 0);
    private final Pose parkControl1 = poseFactory.of(47.2263, 116.7929, 0);

    // Autonomous routine
    public Command autoRoutine() {
        statusLED = new BaseLED(hardwareMap, "backLED");
        return sequential(
                follow(follower, waitthenshoot()),
                instant(() -> statusLED.setGreen()),
                waitMs(3500),
                instant(() -> statusLED.setOff()),
                follow(follower, collectfromclosestflower()),
                instant(() -> statusLED.setPurple()),
                waitMs(1500),
                instant(() -> statusLED.setOff()),
                follow(follower, gotoferthestflower()),
                instant(() -> statusLED.setPurple()),
                waitMs(1500),
                instant(() -> statusLED.setOff()),
                follow(follower, shootcolectformgarden()),
                instant(() -> statusLED.setGreen()),
                waitMs(8000),
                instant(() -> statusLED.setOff()),
                follow(follower, park())
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

    public Path waitthenshoot() {
        return Paths.line(start, waitthenshoot).linear(start, waitthenshoot);
    }

    public Path collectfromclosestflower() {
        return Paths.line(collectfromclosestflowerStart, collectfromclosestflower).linear(collectfromclosestflowerStart, collectfromclosestflower);
    }

    public Path gotoferthestflower() {
        return Paths.line(gotoferthestflowerStart, gotoferthestflower).linear(gotoferthestflowerStart, gotoferthestflower);
    }

    public Path shootcolectformgarden() {
        return Paths.line(shootcolectformgardenStart, shootcolectformgarden).linear(shootcolectformgardenStart, shootcolectformgarden);
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
}
