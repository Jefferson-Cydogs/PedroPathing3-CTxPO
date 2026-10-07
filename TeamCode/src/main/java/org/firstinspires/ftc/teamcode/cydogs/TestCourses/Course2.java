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
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Course2", group = "Autonomous")
public class Course2 extends LinearOpMode {
    private BaseLED statusLED;
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses
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

    // Path methods
    public Path shoot() {
        return line(start, shoot).linear(start, shoot);
    }
    public Path collectFromGarden() {
        return curve(colectfromGardenStart, colectfromGardenControl1, colectfromGarden).linear(colectfromGardenStart, colectfromGarden);
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

    // Autonomous routine
    public Command autoRoutine() {
        statusLED = new BaseLED(hardwareMap, "backLED");
        return sequential(
                follow(follower, shoot()),
                instant(() -> statusLED.setGreen()),
                waitMs(1000),
                instant(() -> statusLED.setOff()),
                follow(follower, collectFromGarden()),
                instant(() -> statusLED.setPurple()),
                waitMs(1500),
                instant(() -> statusLED.setOff()),
                follow(follower, shoot2()),
                instant(() -> statusLED.setGreen()),
                waitMs(1000),
                instant(() -> statusLED.setOff()),
                follow(follower, collectFromflower()),
                instant(() -> statusLED.setPurple()),
                waitMs(1500),
                instant(() -> statusLED.setOff()),
                follow(follower, shoot3()),
                instant(() -> statusLED.setGreen()),
                waitMs(1000),
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

}
