package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.Movement;
import org.firstinspires.ftc.teamcode.control.Odometry;
import org.firstinspires.ftc.teamcode.datatypes.Pose;
import org.firstinspires.ftc.teamcode.util.HardwareMapper;

@Autonomous(name="Pure Pursuit Test", group="Linear OpMode")

public class PurePursuitTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    Pose targetPose = new Pose(new double[]{20, 0, 0});



    //send encoders to odometry in order
    // j[leftDeadwheel,  rightDeadwheel, backDeadwheel]
    private DcMotor[] motors;
    private Odometry otto;
    private RobotMovement percy = new RobotMovement();


    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        motors = HardwareMapper.getMotors(hardwareMap);
        otto = new Odometry(new DcMotor[] {motors[0], motors[1], motors[2]});

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        otto.resetEncoders();
        //percy.setTargetPose(new Pose(new double[]{25, 25, 0}));

        while (opModeIsActive()) {
            double max;

            double [] deltas = percy.goToPosition(targetPose, otto.getPose(), 0.2);

            telemetry.addData("directions", "X:" + deltas[0] + " Y:"+deltas[1]+" R:" +deltas[2]);
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = deltas[0];  // Note: pushing stick forward gives negative value
            double lateral =  deltas[1];
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
            motors[3].setPower(leftFrontPower);
            motors[2].setPower(rightFrontPower);
            motors[1].setPower(leftBackPower);
            motors[0].setPower(rightBackPower);

            // Show the elapsed game time and wheel power and odometry is in cm

            otto.updateOdometry();
            telemetry.addData("PoseRot" , otto.getPose().getR());
            telemetry.addData("Pose" , otto.getPose());
            telemetry.addData("leftEncoder", Odometry.ticksToIn(otto.getLeftEncoder().getCurrentPosition()));
            telemetry.addData("rightEncoder", Odometry.ticksToIn(otto.getRightEncoder().getCurrentPosition()));
            telemetry.addData("backEncoder", Odometry.ticksToIn(otto.getBackEncoder().getCurrentPosition()));
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();



        }




    }
}
