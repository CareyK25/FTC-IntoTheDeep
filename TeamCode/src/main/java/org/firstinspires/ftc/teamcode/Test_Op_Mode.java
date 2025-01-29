
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.Constants.BACK_LEFT;
import static org.firstinspires.ftc.teamcode.util.Constants.BACK_RIGHT;
import static org.firstinspires.ftc.teamcode.util.Constants.FRONT_LEFT;
import static org.firstinspires.ftc.teamcode.util.Constants.FRONT_RIGHT;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
import org.firstinspires.ftc.teamcode.rendertypes.BoundingBox;
import org.firstinspires.ftc.teamcode.rendertypes.Display;
import org.firstinspires.ftc.teamcode.util.HardwareMapper;

@TeleOp(name="Test_Op_Mode", group="Linear OpMode")


public class Test_Op_Mode extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor[] motors;
    private Movement movement;
    private DcMotor slide1;
    private Servo testservo;
    private Servo intaketest;


    //send encoders to odometry in order              [leftDeadwheel,  rightDeadwheel, backDeadwheel]
    private Odometry otto;
    private Display disp;

    private BoundingBox bbTest = new BoundingBox(new Pair[]{
            new Pair(-3, -6),
            new Pair(3, -6),
            new Pair(3, 6),
            new Pair(-3, 6),


    });

    @Override
    public void runOpMode() {
        motors = HardwareMapper.getMotors(hardwareMap);
        slide1 = hardwareMap.get(DcMotor.class, "slide1");
        testservo = hardwareMap.get(Servo.class, "clawservomonkey");
        intaketest = hardwareMap.get(Servo.class, "intakeservo");

        intaketest.setDirection(Servo.Direction.REVERSE);

        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        slide1.setTargetPosition(0);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        movement = new Movement(motors);
        otto = new Odometry(new DcMotor[] {motors[0], motors[1], motors[2]});
        disp = new Display(20, 20, telemetry);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        InputState pgp1 = new InputState(gamepad1); // previous gamepad1, used to save past input states to test for button changes

        boolean canDrive = false;
        boolean runDisplay = false;

//        Gamepad.RumbleEffect rumbler = new Gamepad.RumbleEffect.Builder()
//                .addStep(0.0, 1.0, 500)
//                .addStep(0.0, 0.0, 300)
//                .addStep(1.0, 0.0, 250)
//                .addStep(0.0, 0.0, 250)
//                .addStep(1.0, 0.0, 250).build();
//
//        Gamepad.RumbleEffect.Builder rippleBuilder = new Gamepad.RumbleEffect.Builder();
//
//        for (int i = 0; i < 100; i++) {
//            // Example logic for step values; you can modify this based on your needs.
//
//            // Add step to the builder
//            rippleBuilder.addStep(i/100.0, 1-(i/100.0), 30);
//        }
//        for (int i = 0; i < 100; i++) {
//            // Example logic for step values; you can modify this based on your needs.
//            // Add step to the builder
//            rippleBuilder.addStep(1-(i/100.0), (i/100.0), 30);
//        }
//
//// Build the final RumbleEffect
//        Gamepad.RumbleEffect ripple = rippleBuilder.build();

        double gowthams = 0.1;

        double testintakepos = 0;
        otto.resetEncoders();
        disp.fill('.');

    while (opModeIsActive()) { // todo THIS IS THE MAIN LOOP

        // grab input state at beginning of loop and put it into an object
        InputState gp1 = new InputState(gamepad1);

        // todo THUMBSTICKS to drive
        if (canDrive) {
            temp_old_drive(gamepad1, gamepad2, gowthams, motors);
        }

        // todo OPTIONS to toggle driving GUIDE to toggle display
        if (gp1.isOptions() && !pgp1.isOptions()) {
            if (canDrive) {
                gamepad1.rumbleBlips(1);
            } else {
                gamepad1.rumbleBlips(2);
            }
            canDrive = !canDrive;
        }
        if (gp1.isGuide() && !pgp1.isGuide()) {
            if (runDisplay) {
                gamepad1.rumbleBlips(1);
            } else {
                gamepad1.rumbleBlips(2);
            }
            runDisplay = !runDisplay;
        }

        // todo REGULAR BUTTONS
        if (gp1.isTriangle() && !pgp1.isTriangle()) {
            gamepad1.rumble(1, 1, 200);
        }

        slide1.setPower(gowthams);
        if (gp1.isSquare()) {
            slide1.setTargetPosition(0);
        } else if (gp1.isCross()) {
            slide1.setTargetPosition(1850);
        }

        // todo TRIGGERS
        if (gp1.getLeft_trigger() > 0.5 && !(pgp1.getLeft_trigger() > 0.5)) {
            gowthams -= 0.1;
        }
        if (gp1.getRight_trigger() > 0.5 && !(pgp1.getRight_trigger() > 0.5)) {
            gowthams += 0.1;
        }

        // todo BUMPERS
        if (gp1.isLeft_bumper()) {
            testintakepos+= 0.01;
            intaketest.setPosition(testintakepos);
        } else if (gp1.isRight_bumper()) {
            testintakepos-=0.01;
            intaketest.setPosition(testintakepos);
        }

        // todo TOUCHPAD
        if ((gp1.isTouchpad_1() && gp1.isTouchpad_2()) && (pgp1.isTouchpad_1() && pgp1.isTouchpad_2())) {
            double distance_delta = gp1.getTouchpad_finger_1().distance(gp1.getTouchpad_finger_2()) -
                    pgp1.getTouchpad_finger_1().distance(pgp1.getTouchpad_finger_2());;//distance between fingers
            gowthams += distance_delta/4;// the 4 is just to scale down the speed of change
            slide1.setPower(distance_delta);
        }


        // todo TELEMETRY AND DISPLAY
        if (runDisplay) {
            disp.fill(' ');
            bbTest.rotate(otto.getPose().getR());
            disp.addPixels(bbTest.render());
            disp.update();
        } else {
            telemetry.addData("Driving is ", ((canDrive) ? "enabled" : "disabled"));
            telemetry.addData("servo pose", testservo.getPosition());
            telemetry.addData("touch x", gamepad1.touchpad_finger_1_x);
            telemetry.addData("touch y", gamepad1.touchpad_finger_1_y);
            telemetry.addData("touch2 x", gamepad1.touchpad_finger_2_x);
            telemetry.addData("touch2 y", gamepad1.touchpad_finger_2_y);
            telemetry.addData("motorpos", slide1.getCurrentPosition());
            telemetry.addData("gowthams Speed", gowthams);
            telemetry.addData("odometry:", otto.getPose());
            telemetry.addData("left", gp1.getTouchpad_finger_1());
            telemetry.addData("intakeservopos", testintakepos);
            telemetry.update();
        }

        testintakepos = bound(testintakepos, 0, 1);
        gowthams = bound(gowthams, 0, 1);
        pgp1 = gp1; // saving previous input state for gamepad1
        otto.updateOdometry();
        }
    }


    public static double bound(double i, double l, double u) {
        return (i < l) ? l : (i > u) ? u : i;
    }


    private static void temp_old_drive(Gamepad gamepad1, Gamepad gamepad2, double gowthams_speed_hehe, DcMotor[] motors) {
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        if (Math.abs(axial)*0.8 >Math.abs(lateral)) {
            lateral=0;
        } else if (Math.abs(lateral)*0.8 > Math.abs(axial)) {
            axial=0;
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
