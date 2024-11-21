
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.control.Movement;
import org.firstinspires.ftc.teamcode.datatypes.Pose;
import org.firstinspires.ftc.teamcode.util.HardwareMapper;

@TeleOp(name="Test_Op_Mode", group="Linear OpMode")


public class Test_Op_Mode extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor[] motors;
    private Servo myServo, intake;//andy lau add :)
    private Movement movement;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        motors = HardwareMapper.getMotors();
        myServo = hardwareMap.get(Servo.class, "deposit1");//andy lau add :)
        intake = hardwareMap.get(Servo.class, "intake");

        movement = new Movement(motors);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) { // THIS IS THE MAIN LOOP
            Pose input = new Pose(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            movement.move(input);

            telemetry.update();

        }
    }}
