package org.firstinspires.ftc.teamcode.cydogs.vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseServo;

/**
 * Turns the camera left/right with a positional servo (no gear reduction).
 *
 * Servo position 0.0 = one end of travel, 1.0 = the other end.
 * Angles are in degrees across that full travel (default 0 to 270).
 *
 * Use with TagSight: nudge by bearing until the tag is centered.
 */
public class CameraPan extends BaseServo {

    // Full pan range in degrees (match your servo's real travel if needed)
    public static final double MIN_DEGREES = 0.0;
    public static final double MAX_DEGREES = 270.0;

    // Last commanded pan angle (degrees)
    private double currentDegrees;

    /**
     * @param opMode       the running OpMode
     * @param hardwareName name of the pan servo in the robot config
     */
    public CameraPan(OpMode opMode, String hardwareName) {
        super(
                opMode,
                hardwareName,
                Servo.Direction.FORWARD,
                0.0,   // min servo position
                1.0,   // max servo position
                0.5    // start centered
        );
        currentDegrees = MAX_DEGREES / 2.0;
    }

    /** Connect hardware and move to the starting angle. Call once before use. */
    public void initialize() {
        Initialize();
        setDegrees(currentDegrees);
    }

    /**
     * Aim to an angle in degrees within min/max.
     * Maps directly to servo position (1:1, no gear ratio).
     */
    public void setDegrees(double degrees) {
        currentDegrees = clamp(degrees);
        double position = currentDegrees / MAX_DEGREES;
        SetPosition(position);
    }

    /** Change aim by a small amount from the current angle. */
    public void nudgeByDegrees(double deltaDegrees) {
        setDegrees(currentDegrees + deltaDegrees);
    }

    public double getDegrees() {
        return currentDegrees;
    }

    public boolean isAtMin() {
        return currentDegrees <= MIN_DEGREES + 0.5;
    }

    public boolean isAtMax() {
        return currentDegrees >= MAX_DEGREES - 0.5;
    }

    private double clamp(double degrees) {
        if (degrees < MIN_DEGREES) {
            return MIN_DEGREES;
        }
        if (degrees > MAX_DEGREES) {
            return MAX_DEGREES;
        }
        return degrees;
    }
}