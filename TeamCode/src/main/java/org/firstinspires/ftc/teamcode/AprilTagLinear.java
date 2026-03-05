package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagHelper {

    private static final int TAG_BLUE_GOAL = 21;
    private static final int TAG_RED_GOAL  = 22;
    private static final double IN_TO_CM   = 2.54;

    public static class FieldPose {
        public double x, y, heading;
        public FieldPose(double x, double y, double heading) {
            this.x = x; this.y = y; this.heading = heading;
        }
    }

    /**
     * Initialises the AprilTag pipeline, scans for goal tags, logs telemetry,
     * and returns the robot's field pose. Closes the vision portal when done.
     *
     * @param hwmap     the OpMode's hardwareMap
     * @param telemetry the OpMode's telemetry
     * @return          FieldPose (x cm, y cm, heading deg), or null if no tag found
     */
    public static FieldPose getRobotPoseFromAprilTag(HardwareMap hwmap, Telemetry telemetry) {

        // --- Build processor and portal ---
        AprilTagProcessor processor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hwmap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(processor)
                .build();

        // --- Detect tags ---
        FieldPose robotPose     = null;
        AprilTagDetection usedTag = null;

        for (AprilTagDetection tag : processor.getDetections()) {
            if ((tag.id == TAG_BLUE_GOAL || tag.id == TAG_RED_GOAL) && tag.ftcPose != null) {
                robotPose = computeFieldPose(tag);
                if (robotPose != null) { usedTag = tag; break; }
            }
        }

        // --- Telemetry ---
        if (usedTag == null) {
            telemetry.addLine("No goal tag detected.");
        } else {
            String goalName = (usedTag.id == TAG_BLUE_GOAL) ? "BLUE GOAL" : "RED GOAL";

            if (usedTag.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s  [%s]",
                        usedTag.id, usedTag.metadata.name, goalName));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown  [%s]",
                        usedTag.id, goalName));
            }

            telemetry.addLine(String.format("XYZ: %.1f, %.1f, %.1f (cm)",
                    usedTag.ftcPose.x, usedTag.ftcPose.y, usedTag.ftcPose.z));
            telemetry.addLine(String.format("PRY: %.1f, %.1f, %.1f (deg)",
                    usedTag.ftcPose.pitch, usedTag.ftcPose.roll, usedTag.ftcPose.yaw));
            telemetry.addLine(String.format("RBE: %.1f (cm), %.1f (deg), %.1f (deg)",
                    usedTag.ftcPose.range, usedTag.ftcPose.bearing, usedTag.ftcPose.elevation));

            if (robotPose != null) {
                telemetry.addLine("---- ROBOT FIELD POSE ----");
                telemetry.addLine(String.format("Field X: %.1f cm  (%.1f in)", robotPose.x, robotPose.x / IN_TO_CM));
                telemetry.addLine(String.format("Field Y: %.1f cm  (%.1f in)", robotPose.y, robotPose.y / IN_TO_CM));
                telemetry.addLine(String.format("Heading: %.1f deg", robotPose.heading));
            }
        }

        telemetry.update();
        portal.close();

        return robotPose;
    }

    // --- Private helpers ---

    private static FieldPose getTagFieldPose(int tagId) {
        switch (tagId) {
            case TAG_BLUE_GOAL: return new FieldPose(-72 * IN_TO_CM,  72 * IN_TO_CM, -90.0);
            case TAG_RED_GOAL:  return new FieldPose( 72 * IN_TO_CM,  72 * IN_TO_CM,  90.0);
            default:            return null;
        }
    }

    private static FieldPose computeFieldPose(AprilTagDetection tag) {
        FieldPose tagPose = getTagFieldPose(tag.id);
        if (tagPose == null || tag.ftcPose == null) return null;

        double headingRad = Math.toRadians(tagPose.heading);
        double rotX = tag.ftcPose.x * Math.cos(headingRad) - tag.ftcPose.y * Math.sin(headingRad);
        double rotY = tag.ftcPose.x * Math.sin(headingRad) + tag.ftcPose.y * Math.cos(headingRad);

        return new FieldPose(
                tagPose.x - rotX,
                tagPose.y - rotY,
                tagPose.heading - tag.ftcPose.yaw
        );
    }
}
