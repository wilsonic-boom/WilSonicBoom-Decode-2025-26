package org.firstinspires.ftc.teamcode;

// test this is sriram :)
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterTrigger;

@TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")
public class shooter extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Create necessary objects to operate shooter

        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "ShooterMotor");
        Servo triggerServo = hardwareMap.get(Servo.class, "trigger");

        // Create positions to refer servo to

        double rest = 0.1;
        double shoot = 0.5;

        // Let the servo rest initially

        triggerServo.setPosition(rest);

        waitForStart();

        while (opModeIsActive()) {
            // Use A to start up motor and B to cool down

            while (opModeIsActive()) {
                if (gamepad1.a) {
                    shooterMotor.setPower(1.0);
                } else {
                    shooterMotor.setPower(0.0);
                }
                if (gamepad1.b) {
                    shooterMotor.setPower(0.0);
                }
                // Upon pressing the right bumper fire the shooter

                if (gamepad1.right_bumper) {
                    triggerServo.setPosition(shoot);
                } else {
                    triggerServo.setPosition(rest);
                }
                telemetry.addData("Shooter Power is ", shooterMotor.getPower());
                telemetry.addData("Servo Position is ", triggerServo.getPosition());
                telemetry.update();

            }
        }

    }
}
