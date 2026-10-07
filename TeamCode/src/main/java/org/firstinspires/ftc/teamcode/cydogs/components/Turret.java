package org.firstinspires.ftc.teamcode.cydogs.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseServo;

/**
 * Turret control built on BaseServo.
 * Mechanical setup:
 *   The servo drives a small gear that turns a larger turret gear (about 5:1).
 *   Five degrees of servo motion ≈ one degree of turret motion.
 * If the servo can travel about 270 degrees, the turret range is about
 * 270 / 5 = 54 degrees. This class works in turret degrees and converts
 * to a servo position from 0.0 to 1.0.
 */
public class Turret extends BaseServo {

    // Servo degrees moved for each 1 degree of turret motion
    public static final double GEAR_RATIO = 5.0;

    // Approximate full travel of the servo horn (degrees)
    public static final double SERVO_TRAVEL_DEGREES = 270.0;

    // Turret angle limits after the gear reduction
    public static final double MIN_TURRET_DEGREES = 0.0;
    public static final double MAX_TURRET_DEGREES = SERVO_TRAVEL_DEGREES / GEAR_RATIO;

    // Last commanded turret angle (degrees)
    private double currentTurretDegrees;

    /**
     * @param opMode       the running OpMode
     * @param hardwareName name of the servo in the robot configuration
     */
    public Turret(OpMode opMode, String hardwareName) {
        super(
                opMode,
                hardwareName,
                Servo.Direction.FORWARD,
                0.0,   // min servo position
                1.0,   // max servo position
                0.5    // start in the middle of servo travel
        );

        currentTurretDegrees = MAX_TURRET_DEGREES / 2.0;
    }

    /** Connect to hardware and move to the starting angle. Call once before use. */
    public void initialize() {
        Initialize();
        setTurretDegrees(currentTurretDegrees);
    }

    /**
     * Aim the turret to an angle in turret degrees (within min/max).
     * Converts through the gear ratio into a servo position.
     */
    public void setTurretDegrees(double turretDegrees) {
        currentTurretDegrees = clampTurret(turretDegrees);

        // 0.0 = one end of travel, 1.0 = the other end
        double fraction = currentTurretDegrees / MAX_TURRET_DEGREES;
        SetPosition(fraction);
    }

    /** Change aim by a small amount from the current turret angle. */
    public void nudgeByDegrees(double deltaTurretDegrees) {
        setTurretDegrees(currentTurretDegrees + deltaTurretDegrees);
    }

    public double getTurretDegrees() {
        return currentTurretDegrees;
    }

    public boolean isAtMin() {
        return currentTurretDegrees <= MIN_TURRET_DEGREES + 0.5;
    }

    public boolean isAtMax() {
        return currentTurretDegrees >= MAX_TURRET_DEGREES - 0.5;
    }

    private double clampTurret(double degrees) {
        if (degrees < MIN_TURRET_DEGREES) {
            return MIN_TURRET_DEGREES;
        }
        if (degrees > MAX_TURRET_DEGREES) {
            return MAX_TURRET_DEGREES;
        }
        return degrees;
    }
}