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
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                servoPos+=0.01;
                //Telemetry.addData("A", gamepad1.a);
            }
            else if (gamepad1.b) {
                servoPos-=.01;
                System.out.println("B Working");
            }
            claw.setPosition(servoPos);
        }
    }

}
