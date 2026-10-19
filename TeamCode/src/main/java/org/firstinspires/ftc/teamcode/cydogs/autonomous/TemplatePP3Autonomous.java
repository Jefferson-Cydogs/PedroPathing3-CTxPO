package org.firstinspires.ftc.teamcode.cydogs.autonomous;

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

@Autonomous(name = "Template PP3 AUtonomous", group = "Autonomous")
public class TemplatePP3Autonomous extends LinearOpMode {
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses
    private final Pose start = poseFactory.of(1, 1, 90);
    private final Pose shoot = poseFactory.of(56.3926, 13.9434, 90);
    private final Pose gardenCollect = poseFactory.of(10.5195, 11.6263, 90);
    private final Pose gardenCollectControl1 = poseFactory.of(27.7305, 22.4141, 0);
    private final Pose park = poseFactory.of(11.4, 94.5, 0);

    // Path methods
    public Path shoot() {
        return line(start, shoot).linear(start, shoot);
    }
    public Path gardenCollect() {
        return curve(shoot, gardenCollectControl1, gardenCollect).linear(shoot, gardenCollect);
    }
    public Path park() {
        return line(gardenCollect, park).linear(gardenCollect, park);
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot()),
                waitMs(1000),
                follow(follower, gardenCollect()),
                waitMs(1500),
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
