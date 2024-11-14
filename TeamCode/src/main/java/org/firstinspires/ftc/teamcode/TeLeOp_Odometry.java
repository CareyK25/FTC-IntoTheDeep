
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.Constants.BACK_LEFT;
import static org.firstinspires.ftc.teamcode.util.Constants.BACK_RIGHT;
import static org.firstinspires.ftc.teamcode.util.Constants.FRONT_LEFT;
import static org.firstinspires.ftc.teamcode.util.Constants.FRONT_RIGHT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;


@TeleOp(name="Andys_Op_Mode", group="Linear OpMode")


public class TeLeOp_Odometry extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor[] motors = new DcMotor[4];

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        motors[FRONT_LEFT]  = hardwareMap.get(DcMotor.class, "frontLeft");
        motors[BACK_LEFT]  = hardwareMap.get(DcMotor.class, "backLeft");
        motors[FRONT_RIGHT] = hardwareMap.get(DcMotor.class, "frontRight");
        motors[BACK_RIGHT] = hardwareMap.get(DcMotor.class, "backRight");

        motors[FRONT_LEFT].setDirection(DcMotor.Direction.FORWARD);
        motors[BACK_LEFT].setDirection(DcMotor.Direction.REVERSE);
        motors[FRONT_RIGHT].setDirection(DcMotor.Direction.REVERSE);
        motors[BACK_RIGHT].setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) { // THIS IS THE MAIN LOOP
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y + (-gamepad2.left_stick_y);  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x + gamepad2.left_stick_x;
            double yaw     =  (-gamepad1.right_stick_x) + (-gamepad2.right_stick_x);

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Send calculated power to wheels
            motors[FRONT_LEFT].setPower(leftFrontPower);
            motors[FRONT_RIGHT].setPower(rightFrontPower);
            motors[BACK_LEFT].setPower(leftBackPower);
            motors[BACK_RIGHT].setPower(rightBackPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();

        }
    }}
