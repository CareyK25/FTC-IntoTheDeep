
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.control.Movement;
import org.firstinspires.ftc.teamcode.datatypes.Pose;
import org.firstinspires.ftc.teamcode.util.HardwareMapper;

@TeleOp(name="TeLeOp_Odometry", group="Linear OpMode")
public class TeLeOp_Odometry extends LinearOpMode {
    private DcMotor[] motors = HardwareMapper.getMotors(hardwareMap);
    private Movement movement = new Movement(motors);

    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) { // THIS IS THE MAIN LOOP
            Pose delta = new Pose(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            movement.move(delta);



        }
    }}
