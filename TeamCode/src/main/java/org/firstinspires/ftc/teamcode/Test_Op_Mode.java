
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.control.InputDevice;
import org.firstinspires.ftc.teamcode.control.Movement;
import org.firstinspires.ftc.teamcode.control.Odometry;
import org.firstinspires.ftc.teamcode.datatypes.InputState;
import org.firstinspires.ftc.teamcode.datatypes.Pair;
import org.firstinspires.ftc.teamcode.datatypes.Pose;
import org.firstinspires.ftc.teamcode.util.HardwareMapper;

@TeleOp(name="Test_Op_Mode", group="Linear OpMode")


public class Test_Op_Mode extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor[] motors;
    private Movement movement;
    private DcMotor piston;
    private Servo testservo;
    private Servo intake;

    //send encoders to odometry in order              [leftDeadwheel,  rightDeadwheel, backDeadwheel]
    private Odometry otto;
    //send encoders to odometry in order              [leftDeadwheel,  rightDeadwheel, backDeadwheel]


    @Override
    public void runOpMode() {
        motors = HardwareMapper.getMotors(hardwareMap);
        piston = hardwareMap.get(DcMotor.class, "piston");
        testservo = hardwareMap.get(Servo.class, "clawservomonkey");
        intake = hardwareMap.get(Servo.class, "intake");

        movement = new Movement(motors);
        otto = new Odometry(new DcMotor[] {motors[0], motors[1], motors[2]});


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        double s = 0;
        Pair prev_finger1 = new Pair(gamepad1.touchpad_finger_1_x, gamepad1.touchpad_finger_1_y);
        Pair prev_finger2 = new Pair(gamepad1.touchpad_finger_2_x, gamepad1.touchpad_finger_2_y);

        InputState pgp1 = new InputState(gamepad1); // previous gamepad1, used to save past input states to test for button changes

        boolean canDrive = false;

        Gamepad.RumbleEffect rumbler = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)
                .addStep(0.0, 0.0, 300)
                .addStep(1.0, 0.0, 250)
                .addStep(0.0, 0.0, 250)
                .addStep(1.0, 0.0, 250).build();

        Gamepad.RumbleEffect.Builder rippleBuilder = new Gamepad.RumbleEffect.Builder();

        for (int i = 0; i < 100; i++) {
            // Example logic for step values; you can modify this based on your needs.

            // Add step to the builder
            rippleBuilder.addStep(i/100.0, 1-(i/100.0), 30);
        }
        for (int i = 0; i < 100; i++) {
            // Example logic for step values; you can modify this based on your needs.
            // Add step to the builder
            rippleBuilder.addStep(1-(i/100.0), (i/100.0), 30);
        }

// Build the final RumbleEffect
        Gamepad.RumbleEffect ripple = rippleBuilder.build();

        double gowthams_speed_hehe = 0.1;

        otto.resetEncoders();
        //todo OP MODE CODEEEEEEEEEs
    while (opModeIsActive()) { // THIS IS THE MAIN LOOP

        InputState gp1 = new InputState(gamepad1);

//        Pose input = new Pose(gp1.getLeft_stick().getX()*0, gp1.getLeft_stick().getY(), gp1.getRight_stick().getX());
//        if (canDrive) {
//            movement.move(input, gowthams_speed_hehe);
//        }
        if (canDrive) {
            temp_old_drive(gamepad1, gamepad2, gowthams_speed_hehe, motors);
        }

        if (gp1.isOptions() && !pgp1.isOptions()) {
            if (canDrive) {
                gamepad1.rumbleBlips(1);
            } else {
                gamepad1.rumbleBlips(2);
            }
            canDrive = !canDrive;
        }

        if (gp1.isTriangle() && !pgp1.isTriangle()) {
            gamepad1.runRumbleEffect(ripple);
        }

        if (gp1.getLeft_trigger() > 0.5 && !(pgp1.getLeft_trigger() > 0.5)) {
            gowthams_speed_hehe -= 0.1;
        }
        if (gp1.getRight_trigger() > 0.5 && !(pgp1.getRight_trigger() > 0.5)) {
            gowthams_speed_hehe += 0.1;
        }


        if (gp1.isSquare()) {
            piston.setPower(gowthams_speed_hehe);
            gamepad1.rumble(100);
        } else if (gp1.isCross()) {
            piston.setPower(-gowthams_speed_hehe);
            gamepad1.rumbleBlips(2);
        } else {
            piston.setPower(0);

        }
        s += gamepad1.left_trigger/1000;
        s -= gamepad1.right_trigger/1000;
        if (s > 1) {
            s = 1;
        } else if (s <0) {
            s=0;
        }


        if ((gp1.isTouchpad_1() && gp1.isTouchpad_2()) && (pgp1.isTouchpad_1() && pgp1.isTouchpad_2())) {
            double distance_delta = gp1.getTouchpad_finger_1().distance(gp1.getTouchpad_finger_2()) -
                    pgp1.getTouchpad_finger_1().distance(pgp1.getTouchpad_finger_2());;//distance between fingers
            telemetry.addData("distance_delta", distance_delta);
            gowthams_speed_hehe += distance_delta/4;// the 4 is just to scale down the speed of change
            piston.setPower(distance_delta);
        } else if (gp1.isTouchpad_1()) {
            if (canDrive) {
                test_touchpad_drive(new Pose(gp1.getTouchpad_finger_1(), 0), gowthams_speed_hehe, motors);
            }
        }

//        if (gamepad1.a) {
//            // Move servo to one position,added limits in order to stop servo from over extending, setting a range
//            intake.setPosition(Range.clip(intake.getPosition()+.01, .465, .70));
//        } else if (gamepad1.b) {
//            // Move servo to another position, added limits in order to stop servo from over extending
//            intake.setPosition(Range.clip(intake.getPosition()-.01, .465, .70));
//        }


        if (gowthams_speed_hehe > 1) {
            gowthams_speed_hehe = 1;
        } else if (gowthams_speed_hehe < 0) {
            gowthams_speed_hehe = 0;
        }

        telemetry.addData("Driving is ", ((canDrive) ? "enabled" : "disabled"));
        telemetry.addData("s ", s);
        telemetry.addData("servo pose", testservo.getPosition());
        telemetry.addData("touch x", gamepad1.touchpad_finger_1_x);
        telemetry.addData("touch y", gamepad1.touchpad_finger_1_y);
        telemetry.addData("touch2 x", gamepad1.touchpad_finger_2_x);
        telemetry.addData("touch2 y", gamepad1.touchpad_finger_2_y);
        telemetry.addData("motorpos", piston.getCurrentPosition());
        telemetry.addData("gowthams Speed", gowthams_speed_hehe);
        telemetry.addData("odometry:", otto.getPose());
        telemetry.addData("left", gp1.getTouchpad_finger_1());
        telemetry.update();

        otto.updateOdometry();
        pgp1 = gp1; // saving previous input state for gamepad1
        }
    }

    private static void test_touchpad_drive(Pose diff, double gowthams_speed_hehe, DcMotor[] motors) {
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = diff.getY();  // Note: pushing stick forward gives negative value
        double lateral = diff.getX();
        double yaw     =  diff.getY();

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
        motors[3].setPower(leftFrontPower*gowthams_speed_hehe);
        motors[2].setPower(rightFrontPower*gowthams_speed_hehe);
        motors[1].setPower(leftBackPower*gowthams_speed_hehe);
        motors[0].setPower(rightBackPower*gowthams_speed_hehe);

    }

    private static void temp_old_drive(Gamepad gamepad1, Gamepad gamepad2, double gowthams_speed_hehe, DcMotor[] motors) {
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = gamepad1.left_stick_y + (gamepad2.left_stick_y);  // Note: pushing stick forward gives negative value
        double lateral = (gamepad1.right_stick_x) + (gamepad2.right_stick_x);
        double yaw     = -gamepad1.left_stick_x + (-gamepad2.left_stick_x);

        if (axial * 0.1 > lateral) {
            lateral = 0;
        }
        if (lateral * 0.1 > axial) {
            axial = 0;
        }
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
        motors[3].setPower(leftFrontPower*gowthams_speed_hehe);
        motors[2].setPower(rightFrontPower*gowthams_speed_hehe);
        motors[1].setPower(leftBackPower*gowthams_speed_hehe);
        motors[0].setPower(rightBackPower*gowthams_speed_hehe);

    }
}
