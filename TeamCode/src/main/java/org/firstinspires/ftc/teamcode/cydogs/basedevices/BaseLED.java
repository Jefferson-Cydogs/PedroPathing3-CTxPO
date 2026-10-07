package org.firstinspires.ftc.teamcode.cydogs.basedevices;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A simple wrapper for controlling a servo-based LED indicator
 * (e.g. a REV Blinkin LED Driver).
 *
 * These LED drivers don't use "position" the way a real servo does -- instead,
 * each position value (0.0 to 1.0) corresponds to a specific color or pattern.
 * This class exposes a few simple named methods so the rest of the code doesn't
 * need to remember magic numbers for colors.
 */
public class BaseLED {

    // The underlying servo object used to control the LED driver.
    private Servo led;

    // Position values that correspond to specific colors on the LED driver.
    // NOTE: these are just placeholder examples -- check your LED driver's
    // datasheet (e.g. the REV Blinkin color chart) for the actual values
    // that correspond to the colors you want.
    private final double WHITE_POSITION = 1.000;
    private final double PURPLE_POSITION = 0.720;
    private final double BLUE_POSITION = 0.611;
    private final double GREEN_POSITION = 0.500;
    private final double YELLOW_POSITION = 0.377;
    private final double RED_POSITION = 0.280;
    private final double OFF_POSITION = 0.000;

    /**
     * Constructor. Grabs the LED's servo port from the hardware map by its configured name.
     *
     * @param hardwareMap the robot's hardware map (passed in from your OpMode)
     * @param deviceName  the name of the LED servo port as configured in the Driver Station
     */
    public BaseLED(HardwareMap hardwareMap, String deviceName) {
        this.led = hardwareMap.get(Servo.class, deviceName);
    }

    public void setPurple() {
        led.setPosition(PURPLE_POSITION);
    }

    public void setBlue() {
        led.setPosition(BLUE_POSITION);
    }

    public void setGreen() {
        led.setPosition(GREEN_POSITION);
    }

    public void setYellow() {
        led.setPosition(YELLOW_POSITION);
    }

    public void setRed() {
        led.setPosition(RED_POSITION);
    }

    public void setOff() {
        led.setPosition(OFF_POSITION);
    }
}