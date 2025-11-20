package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.lang.Math;


@TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")
public class autonomous extends LinearOpMode {

    // Declare OpMode members. alr yo its dhir here
    private DcMotorEx motorFL = null;
    private DcMotorEx motorFR = null;
    private DcMotorEx motorBL = null;
    private DcMotorEx motorBR = null;
    private DcMotorEx motorST = null;
    //    private IMU imu; hi im here

    // MATH Setup
    float x = 0;
    float y = 0;
    float orientation = 0;
    int side = -1;

    public void moveTo(float nx, float ny, float nr){
        int hi = 0;
    }

    public void setOrientation(float nx){
        int hi = 0;
    }

    public void goShootingArea(){
        // go to closest shooting area place
        // y = x m = -1
        // y = -x m = 1
        // y = x-48 m = -1
        // y = -x-48 m = 1
        if ((y > -x - 48) && (y > x - 48) && (y < -x) && (y < x)) {
            moveTo(0, 20, 0);
        } else {
            //a = m*b - x + y;
            //m*b - x + y = b;
            //bm*b - b = (x-y)/(m-1);
            if (x > 0){
                // m = -1
                // y = x
                float x1 = (x-y)/(-2);
                float y1 = x1;
                // y = x - 48
                float x2 = (x+y+48)/(-2);
                float y2 = x2 - 48;

                double r1 = Math.pow( Math.pow(x - x1, 2) + Math.pow(y - y1, 2), 0.5);
                double r2 = Math.pow( Math.pow(x - x2, 2) + Math.pow(y - y2, 2), 0.5);

                if (r1 > r2) {
                    moveTo(x1, y1, 0);
                } else {
                    moveTo(x2, y2, 0);
                }
            } else {
                // m = 1
                // y = -x
                float x1 = (x+y)/(2);
                float y1 = -x1;
                // y = -x - 48
                float x2 = (-x+y+48)/(-2);
                float y2 = -x2 - 48;

                double r1 = Math.pow( Math.pow(x - x1, 2) + Math.pow(y - y1, 2), 0.5);
                double r2 = Math.pow( Math.pow(x - x2, 2) + Math.pow(y - y2, 2), 0.5);

                if (r1 > r2) {
                    moveTo(x1, y1, 0);
                } else {
                    moveTo(x2, y2, 0);
                }
            }

        }
    }

    public void autoShoot(){
        // check if it is in the shooting area
        if ( !(((x < y) && (-x < y)) || ((y < x - 48) && (y < -x - 48))) ) {
            goShootingArea();
        }

        float goalx = 72 * side;
        float goaly = 72;

        double deg = (360 + side * Math.toDegrees(Math.acos((72 - y)/(Math.pow( Math.pow(goalx - x, 2) + Math.pow(goaly - y, 2), 0.5))))) % 360;

        setOrientation((float) deg);
        motorST.setPower(1);
    }

    @Override
    public void runOpMode() {
        // INITIALIZE
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR");
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR");
        motorST = hardwareMap.get(DcMotorEx.class, "motorST");
        // imu = hardwareMap.get(IMU.class, "imu");

        motorFR.setDirection(DcMotorEx.Direction.REVERSE);
        motorBR.setDirection(DcMotorEx.Direction.REVERSE);

        // MATH Setup
        x = 0;
        y = 0;
        orientation = 0;

        // start positions
        // startPos1 = (-15, -27, 0) or (15, -27, 0)
        // startPos2 = (-15, -27, 135) or (15, -27, 225)

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {





            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double powerFL = (y + x + rx) / denominator;
            double powerBL = (y - x + rx) / denominator;
            double powerFR = (y - x - rx) / denominator;
            double powerBR = (y + x - rx) / denominator;

            motorFL.setPower(powerFL);
            motorBL.setPower(powerBL);
            motorFR.setPower(powerFR);
            motorBR.setPower(powerBR);
            // Setup a variable for each drive wheel to save power level for telemetry
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            // Show the elapsed game time and wheel power.
            telemetry.addData("Motors", "power (%.2f)", y);
            telemetry.addData("Motors", "rpm (%.2f)", motorFR.getVelocity());
            telemetry.addData("Motors", "rpm (%.2f)", motorFL.getVelocity());
            telemetry.addData("Motors", "rpm (%.2f)", motorBR.getVelocity());
            telemetry.addData("Motors", "rpm (%.2f)", motorBL.getVelocity());
            telemetry.update();
        }
    }
}
