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
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseLED;

@Autonomous(name = "Course 1", group = "Autonomous")
public class Course1 extends LinearOpMode {

    private BaseLED statusLED;
    //statusLED = new BaseLED(hardwareMap, "backLED");
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses
    private final Pose start = poseFactory.of(56, 8.875, 90);
    private final Pose shoot = poseFactory.of(56.3926, 13.9434, 90);
    private final Pose gardenCollectStart = poseFactory.of(56.3926, 13.9434, 90);
    private final Pose gardenCollect = poseFactory.of(10.5195, 11.6263, 90);
    private final Pose gardenCollectControl1 = poseFactory.of(27.7305, 22.4141, 0);
    private final Pose shoot2Start = poseFactory.of(10.5195, 11.6263, 90);
    private final Pose shoot2 = poseFactory.of(59, 126.7, -90);
    private final Pose shoot2Control1 = poseFactory.of(24.043, 109.375, 0);
    private final Pose flowerCollectStart = poseFactory.of(59, 126.7, -90);
    private final Pose flowerCollect = poseFactory.of(50.1287, 124.7524, -90);
    private final Pose flowerCollectControl1 = poseFactory.of(47.7961, 126.7883, 0);
    private final Pose shoot3Start = poseFactory.of(50.1287, 124.7524, -90);
    private final Pose shoot3 = poseFactory.of(59, 126.6643, -90);
    private final Pose parkStart = poseFactory.of(59, 126.6643, -90);
    private final Pose park = poseFactory.of(11.4, 94.5, 0);

    // Path methods
    public Path shoot() {
        return line(start, shoot).linear(start, shoot);
    }
    public Path gardenCollect() {
        return curve(gardenCollectStart, gardenCollectControl1, gardenCollect).linear(gardenCollectStart, gardenCollect);
    }
    public Path shoot2() {
        return curve(shoot2Start, shoot2Control1, shoot2).linear(shoot2Start, shoot2);
    }
    public Path flowerCollect() {
        return curve(flowerCollectStart, flowerCollectControl1, flowerCollect).linear(flowerCollectStart, flowerCollect);
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
                follow(follower, gardenCollect()),
                instant(() -> statusLED.setPurple()),
                waitMs(1500),
                instant(() -> statusLED.setOff()),
                follow(follower, shoot2()),
                instant(() -> statusLED.setGreen()),
                waitMs(1000),
                instant(() -> statusLED.setOff()),
                follow(follower, flowerCollect()),
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
