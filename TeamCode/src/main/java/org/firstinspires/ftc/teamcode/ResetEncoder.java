package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@Autonomous(name = "ResetEncoder")
public class ResetEncoder extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;




    private static final double TICKS_PER_INCH = 45.0;
    private static final double DRIVE_SPEED = 0.5;
    private static final double MIN_TAG_DECISION_MARGIN = 75.0;
    private static final long TAG_SCAN_TIMEOUT_MS = 1000;



    @Override
    public void runOpMode() {

        initDriveMotors();
        initAprilTagVision();

        telemetry.addLine("Autonomous ready. Waiting for START...");
        telemetry.update();
        waitForStart();

        if (!opModeIsActive()) return;

       
       
        driveInches(24);

        
        scanAndResetIfTagFound("Checkpoint 1");

        
        strafeInches(12);

       
        scanAndResetIfTagFound("Checkpoint 2");

        
        driveInches(18);

        
        scanAndResetIfTagFound("Checkpoint 3");

       

        telemetry.addLine("Autonomous complete.");
        telemetry.update();
        visionPortal.close();
    }

    
     */
    private void scanAndResetIfTagFound(String label) {
        telemetry.addLine("[" + label + "] Scanning for AprilTag...");
        telemetry.update();

        long startTime = System.currentTimeMillis();
        boolean resetPerformed = false;

        while (opModeIsActive()
                && !resetPerformed
                && (System.currentTimeMillis() - startTime) < TAG_SCAN_TIMEOUT_MS) {

            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null
                        && detection.decisionMargin >= MIN_TAG_DECISION_MARGIN) {

                    // Confident detection — zero the encoders now
                    resetEncoders();

                    telemetry.addLine("[" + label + "] Tag ID " + detection.id
                            + " found — encoders reset.");
                    telemetry.update();

                    resetPerformed = true;
                    break;
                }
            }

            sleep(50); // short poll interval to avoid hammering the processor
        }

        if (!resetPerformed) {
            telemetry.addLine("[" + label + "] No reliable tag found — continuing with current encoder values.");
            telemetry.update();
        }
    }

    // ── Encoder reset

    private void resetEncoders() {
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // ── Movement helpers

    private void driveInches(double inches) {
        int ticks = (int) (inches * TICKS_PER_INCH);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + ticks);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + ticks);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + ticks);
        backRight.setTargetPosition(backRight.getCurrentPosition() + ticks);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPower(DRIVE_SPEED);
        waitForMotors();
        setMotorPower(0);
    }


    private void strafeInches(double inches) {
        int ticks = (int) (inches * TICKS_PER_INCH);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + ticks);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - ticks);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - ticks);
        backRight.setTargetPosition(backRight.getCurrentPosition() + ticks);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPower(DRIVE_SPEED);
        waitForMotors();
        setMotorPower(0);
    }


    private void waitForMotors() {
        while (opModeIsActive()
                && (frontLeft.isBusy() || frontRight.isBusy()
                ||  backLeft.isBusy()  || backRight.isBusy())) {

            telemetry.addData("FL / FR", "%d / %d",
                    frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
            telemetry.addData("BL / BR", "%d / %d",
                    backLeft.getCurrentPosition(), backRight.getCurrentPosition());
            telemetry.update();
        }
    }

    //  Initialisation

    private void initDriveMotors() {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void initAprilTagVision() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();
    }



    private void setMotorMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    private void setMotorPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }
}
