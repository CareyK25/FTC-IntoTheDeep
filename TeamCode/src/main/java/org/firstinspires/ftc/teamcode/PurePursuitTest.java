package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.datatypes.Pose;

@Autonomous(name="Pure Pursuit Test", group="Linear OpMode")

public class PurePursuitTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    Pose targetPose = new Pose(new double[]{25, 25, 0});

    private IMU imu = null;

    //send encoders to odometry in order              [leftDeadwheel,  rightDeadwheel, backDeadwheel]
    private Odometry otto = new Odometry(new DcMotor[]{leftBackDrive, rightFrontDrive, leftFrontDrive});


    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");
        imu = hardwareMap.get(IMU.class, "imu");
        otto.setEncoders(new DcMotor[]{leftBackDrive, rightFrontDrive, leftFrontDrive});

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        otto.resetEncoders();
        imu.resetYaw();

        while (opModeIsActive()) {
            double max;

            double [] deltas = RobotMovement.goToPosition(otto.getPose().getX(), otto.getPose().getY(), .5, 0, targetPose, .5);
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = deltas[1];  // Note: pushing stick forward gives negative value
            double lateral =  deltas[0];
            double yaw     =  deltas[2];

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
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power and odometry is in cm
            telemetry.addData("IMURot", Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw()));
            telemetry.addData("PoseRot" , otto.getPose().getR());

            telemetry.addData("Pose" , otto.getPose());
            telemetry.addData("leftEncoder", otto.ticksToCm(otto.getLeftEncoder().getCurrentPosition()));
            telemetry.addData("rightEncoder", otto.ticksToCm(otto.getRightEncoder().getCurrentPosition()));
            telemetry.addData("backEncoder", otto.ticksToCm(otto.getBackEncoder().getCurrentPosition()));
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();

            otto.updateOdometry();
        }




    }
}
