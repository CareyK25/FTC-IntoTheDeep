
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.Movement;
import org.firstinspires.ftc.teamcode.control.Odometry;
import org.firstinspires.ftc.teamcode.datatypes.InputState;
import org.firstinspires.ftc.teamcode.datatypes.Pair;
import org.firstinspires.ftc.teamcode.rendertypes.BoundingBox;
import org.firstinspires.ftc.teamcode.rendertypes.Display;
import org.firstinspires.ftc.teamcode.rendertypes.GameMap;
import org.firstinspires.ftc.teamcode.util.HardwareMapper;

@TeleOp(name="Test_Op_Mode", group="Linear OpMode")


public class Test_Op_Mode extends LinearOpMode {


    private GameMap field;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor[] motors;
    private Movement movement;
    private DcMotor slideMotor;
    private Servo testservo;
    private Servo axleServo;



    private Odometry otto;
    private Display disp;
    private GameMap map;

    private BoundingBox bbTest = new BoundingBox(new Pair[]{
            new Pair(-3, -6),
            new Pair(3, -6),
            new Pair(3, 6),
            new Pair(-3, 6),


    });

    @Override
    public void runOpMode() {
        motors = HardwareMapper.getMotors(hardwareMap);
        slideMotor = hardwareMap.get(DcMotor.class, "slide");
        axleServo = hardwareMap.get(Servo.class, "axle");

        axleServo.setDirection(Servo.Direction.REVERSE);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        movement = new Movement(motors);
        otto = new Odometry(new DcMotor[] {motors[0], motors[1], motors[2]});
        disp = new Display(30, 30, telemetry);
        map = new GameMap(new Pair(144, 144));

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
        double axleAngle = 0;
        double slidePos = 0;

        otto.resetEncoders();
        disp.fill('.');

    while (opModeIsActive()) { // todo THIS IS THE MAIN LOOP

        // grab input state at beginning of loop and put it into an object
        InputState gp1 = new InputState(gamepad1);

        // todo THUMBSTICKS to drive
        if (canDrive) {
            temp_old_drive(gamepad1, gamepad2, gowthams, motors, telemetry);
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


        if (gp1.isSquare()) {
            slidePos = slideMotor.getCurrentPosition();
            slideMotor.setPower(gowthams);
            slideMotor.setTargetPosition(0);
        } else if (gp1.isCross()) {
            slidePos = slideMotor.getCurrentPosition();
            slideMotor.setPower(gowthams);
            slideMotor.setTargetPosition(-1900);

        } else {
            slideMotor.setPower(1);
            slideMotor.setTargetPosition((int)slidePos);
        }
        slidePos = bound(slidePos, -1800, 0);

        // todo TRIGGERS
        if (gp1.getLeft_trigger() > 0.5 && !(pgp1.getLeft_trigger() > 0.5)) {
            gowthams -= 0.1;
        }
        if (gp1.getRight_trigger() > 0.5 && !(pgp1.getRight_trigger() > 0.5)) {
            gowthams += 0.1;
        }

        // todo BUMPERS
        if (gp1.isLeft_bumper()) {
            axleAngle+= 0.003;
            axleServo.setPosition(axleAngle);
        } else if (gp1.isRight_bumper()) {
            axleAngle-=0.003;
            axleServo.setPosition(axleAngle);
        } else {
            axleAngle = axleServo.getPosition();
            axleServo.setPosition(axleAngle);
        }
        axleAngle = bound(axleAngle, 0.45, 0.80);


        // todo TOUCHPAD
        if ((gp1.isTouchpad_1() && gp1.isTouchpad_2()) && (pgp1.isTouchpad_1() && pgp1.isTouchpad_2())) {
            double distance_delta = gp1.getTouchpad_finger_1().distance(gp1.getTouchpad_finger_2()) -
                    pgp1.getTouchpad_finger_1().distance(pgp1.getTouchpad_finger_2());;//distance between fingers
            gowthams += distance_delta/4;// the 4 is just to scale down the speed of change
            slideMotor.setPower(distance_delta);
        }


        double[] encoder_delta = otto.getLocalDelta();
        // todo TELEMETRY AND DISPLAY
        if (runDisplay) {
            map.renderBuffer();
            disp.fill(' ');
            disp.fill(map.sampleImage(new Pair(otto.getPose().getX(), otto.getPose().getY()), disp.getHeight(), disp.getWidth())); // SAMPLES FROM TOP LEFT CORNER BASED ON CENTER INPUT
            bbTest.rotate(otto.getPose().getR());
            disp.addPixels(bbTest.render());
            disp.update();
        } else {
            telemetry.addData("Driving is ", ((canDrive) ? "enabled" : "disabled"));
            telemetry.addData("motorpos", slideMotor.getCurrentPosition());
            telemetry.addData("gowthams Speed", gowthams);
            telemetry.addData("odometry:", otto.getPose());
            telemetry.addData("intakeservopos", axleAngle);
            telemetry.addData("raw left thumbstick", gamepad1.left_stick_x + ", " + gamepad1.left_stick_y);
            telemetry.addData("raw right thumbstick", gamepad1.right_stick_x + ", " + gamepad1.right_stick_y);
            telemetry.addData("encoder deltas", encoder_delta[0]+ ", " + encoder_delta[1] + ", " + encoder_delta[2]);
            telemetry.addData("left odo", motors[0].getCurrentPosition());
            telemetry.addData("back odo", motors[1].getCurrentPosition());
            telemetry.addData("right odo", motors[2].getCurrentPosition());
            telemetry.addData("right odo", motors[3].getCurrentPosition());
            telemetry.update();
        }


        gowthams = bound(gowthams, 0, 1);
        pgp1 = gp1; // saving previous input state for gamepad1
        otto.updateOdometry();
        }
    }


    public static double bound(double i, double l, double u) {
        return (i < l) ? l : (i > u) ? u : i;
    }


    private void temp_old_drive(Gamepad gamepad1, Gamepad gamepad2, double gowthams_speed_hehe, DcMotor[] motors, Telemetry telemetry) {
        double max;

        //gamepad1.rumble(gowthams_speed_hehe * Math.abs(gamepad1.left_stick_x), gowthams_speed_hehe * Math.abs(gamepad1.left_stick_y), 30);

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x * (1-(gowthams_speed_hehe/2));

        double[] encoder_delta = otto.getLocalDelta();




        if (Math.abs(axial)*0.8 >Math.abs(lateral)) {
            lateral=encoder_delta[1]/2;
            //lateral=0.1;
        } else if (Math.abs(lateral)*0.8 > Math.abs(axial)) {
            axial=lateral=(encoder_delta[0] + encoder_delta[2])/4;
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
