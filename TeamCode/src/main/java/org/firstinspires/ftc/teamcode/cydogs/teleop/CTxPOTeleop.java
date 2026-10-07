package org.firstinspires.ftc.teamcode.cydogs.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.cydogs.chassis.CTxPORobotChassis;
import org.firstinspires.ftc.teamcode.cydogs.core.EventTracker;


@TeleOp(name="CThrouxPO TeleOp", group= "TeleOp")
public class CTxPOTeleop extends LinearOpMode {

    /** declare variables here */
    private CTxPORobotChassis Wheels;

    private ElapsedTime currentTimer;
    private ElapsedTime matchTimer;
    private EventTracker eventTracker;

    @Override
    public void runOpMode()
    {
        /** Execute initialization actions here */
        Wheels = new CTxPORobotChassis(this);
        Wheels.InitializeChassisTeleop(.6,.3,.5);
        initializeDevices();
        initializePositions();
        currentTimer = new ElapsedTime();
        matchTimer = new ElapsedTime();
        eventTracker = new EventTracker();

        waitForStart();
        //tagReader.initAprilTag();
        matchTimer.reset();

        while (opModeIsActive()) {
            /** Execute OpMode actions here */
            Wheels.OptimizedTeleopDrive();

            //tagReader.displayDetections(tagReader.GetDetections());
            manageDriverControls();
            manageManipulatorControls();

            if (eventTracker.doEvent("Telemetry",currentTimer.seconds(),0.5)) {
                telemetry.update();
            }
        }
    }

    private void manageDriverControls()
    {

    }

    private void manageManipulatorControls()
    {

    }

    private void initializeDevices()
    {

    }

    private void initializePositions()
    {

    }
}

