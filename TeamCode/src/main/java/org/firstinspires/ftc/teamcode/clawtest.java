package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.HardwareMapper;

@TeleOp(name="ClawTest", group="Linear OpMode")

public class clawtest extends LinearOpMode {
    private Servo claw;

    public void runOpMode() {
        double servoPos = 0;
        claw = hardwareMap.get(Servo.class,"claw");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.cross) {
                servoPos+=0.0001;
                //Telemetry.addData("A", gamepad1.a);
            }
            else if (gamepad1.square) {
                servoPos-=.0001;

            }

            servoPos = servoPos>1 ? 1:servoPos;
            servoPos = servoPos<0 ? 0:servoPos;

            claw.setPosition(servoPos);
            telemetry.addData("Status", "Initialized");
            telemetry.addData("servopos", servoPos);
            telemetry.update();
        }
    }

}
