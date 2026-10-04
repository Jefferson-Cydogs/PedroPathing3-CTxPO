package org.firstinspires.ftc.teamcode.pedro;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("leftFrontWheel");
                c.backLeftName.set("leftBackWheel");
                c.frontRightName.set("rightFrontWheel");
                c.backRightName.set("rightBackWheel");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

                c.manualBrakeMode.set(true);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpointOdometry");
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                c.xPodOffset.set(-1.85);
                c.yPodOffset.set(-6.22);
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
                c.globalDistanceUnit.set(DistanceUnit.INCH);
                c.offsetUnits.set(DistanceUnit.INCH);
            }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                /* // End Constraints, with default values. Only enable if follower is taking too long to finish paths.
                c.parametricTConstraint.set(0.025);
                c.velocityConstraint.set(0.1);
                c.translationalConstraint.set(0.1);
                c.headingConstraint.set(0.007);
                c.timeoutConstraint.set(100.0);*/
                Controller primaryTranslationalForward = Controller.proportional(0.25124910091264396);
                Controller secondaryTranslationalForward = Controller.proportional(0.09282982928760126);
                Controller primaryTranslationalLateral = Controller.proportional(0.3711983132376161);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1371478581396959);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012620118871428228));
                c.brake.set(Controller.proportionalFeedforward(0.010727101040713994));

                c.headingFeedback.set(Controller.proportional(3.8824796145671927));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03469433847849049, 0.008141347617940373));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07224852110582221, 0.025888522337115583));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0017841325335874938, 0.002393756881700669));

                c.maxAchievableForwardVelocity.set(73.13426502952187);
                c.maxAchievableStrafeVelocity.set(59.81550013418003);
                c.naturalForwardDeceleration.set(36.914908004329064);
                c.naturalStrafeDeceleration.set(59.53920329854236);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
