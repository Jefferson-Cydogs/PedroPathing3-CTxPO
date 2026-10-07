package org.firstinspires.ftc.teamcode.cydogs.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/**
 * Finds AprilTags with the webcam (single tags or BIOBUZZ clusters).
 * Filters (all optional / configurable):
 *   - Color word in the tag name, e.g. "red" or "blue" (case-insensitive)
 *   - Upright orientation using ftcPose.roll so the down side of a hive is ignored
 * Each loop, call update(), then read hasTarget, bearingDegrees, etc.
 */
public class TagSight {

    // |roll| below this (degrees) counts as right-side-up / scorable cell
    private static final double UPRIGHT_ROLL_LIMIT_DEG = 90.0;

    private final AprilTagProcessor processor;
    private final VisionPortal portal;

    // If set, the target name must contain this word (example: "red")
    private final String requiredColorWord;

    // True when a usable target is in view
    public boolean hasTarget = false;

    // Left/right angle to the target (degrees). Negative = left, positive = right
    public double bearingDegrees = 0;

    // Tag id for a single tag; -1 for a cluster
    public int tagId = -1;

    // Name from the SDK (used for color filtering and telemetry)
    public String targetName = "";

    // True when the locked target is upright (up cell)
    public boolean isUpright = false;

    // Roll of the locked target (degrees); useful for debugging
    public double rollDegrees = 0;

    /**
     * @param hardwareMap        from the OpMode
     * @param webcamName         configuration name (example: "Webcam 1")
     * @param requiredColorWord  "red", "blue", or null to accept any color
     */
    public TagSight(HardwareMap hardwareMap, String webcamName, String requiredColorWord) {
        this.requiredColorWord = requiredColorWord;

        processor = new AprilTagProcessor.Builder().build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .addProcessor(processor)
                .build();
    }

    /** Accept any alliance color. */
    public TagSight(HardwareMap hardwareMap, String webcamName) {
        this(hardwareMap, webcamName, null);
    }

    /**
     * Refresh detection fields from the latest camera frame.
     * Call once every loop.
     */
    public void update() {
        hasTarget = false;
        tagId = -1;
        bearingDegrees = 0;
        targetName = "";
        isUpright = false;
        rollDegrees = 0;

        List<AprilTagDetection> detections = processor.getDetections();
        if (detections == null || detections.isEmpty()) {
            return;
        }

        for (AprilTagDetection detection : detections) {

            // ----- Single tag -----
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                if (single.ftcPose == null) {
                    continue;
                }

                String name = (single.metadata != null)
                        ? single.metadata.name
                        : ("id " + single.id);

                if (!nameMatchesColor(name)) {
                    continue;
                }
                if (!poseIsUpright(single.ftcPose.roll)) {
                    continue;
                }

                hasTarget = true;
                bearingDegrees = single.ftcPose.bearing;
                rollDegrees = single.ftcPose.roll;
                isUpright = true;
                tagId = single.id;
                targetName = name;
                return;
            }

            // ----- Cluster (hive cell) -----
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                if (cluster.ftcPose == null) {
                    continue;
                }

                String name = "cluster";
                if (cluster.metadata != null) {
                    if (cluster.metadata.shortName != null && !cluster.metadata.shortName.isEmpty()) {
                        name = cluster.metadata.shortName;
                    } else if (cluster.metadata.name != null) {
                        name = cluster.metadata.name;
                    }
                }

                if (!nameMatchesColor(name)) {
                    continue;
                }
                if (!poseIsUpright(cluster.ftcPose.roll)) {
                    continue;
                }

                hasTarget = true;
                bearingDegrees = cluster.ftcPose.bearing;
                rollDegrees = cluster.ftcPose.roll;
                isUpright = true;
                tagId = -1;
                targetName = name;
                return;
            }
        }
    }

    /**
     * True if no color filter is set, or the name contains the required word
     * (case-insensitive).
     */
    private boolean nameMatchesColor(String name) {
        if (requiredColorWord == null || requiredColorWord.isEmpty()) {
            return true;
        }
        if (name == null) {
            return false;
        }
        return name.toLowerCase().contains(requiredColorWord.toLowerCase());
    }

    /**
     * True if roll looks right-side-up (up cell), not upside-down (down cell).
     * BIOBUZZ guidance: |roll| less than 90 degrees ≈ scorable / upright.
     */
    private boolean poseIsUpright(double roll) {
        return Math.abs(roll) < UPRIGHT_ROLL_LIMIT_DEG;
    }

    public void close() {
        if (portal != null) {
            portal.close();
        }
    }
}