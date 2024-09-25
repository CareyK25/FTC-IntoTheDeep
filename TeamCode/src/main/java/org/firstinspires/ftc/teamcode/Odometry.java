package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.datatypes.Matrix;
import org.firstinspires.ftc.teamcode.datatypes.ThreadSafePose;
import org.opencv.core.Mat;

public class Odometry extends Thread {
    private final ThreadSafePose pose;


    // unit conversion constants
    public static final int TPR = 2000; // ticks per revolution
    public static final double DEADWHEEL_RADIUS = 2.4; // in CM
    public static final double DEADWHEEL_CIRCUMFERENCE = 2*Math.PI*DEADWHEEL_RADIUS; // in CM

    // odometry calculation constants
    private static final double TRACKWIDTH = 28.33; //todo (do it in CM)
    private static final double FORWARD_OFFSET = 14.5; //todo (do it in CM)

    // used to grab encoders from the array with more readablility
    public static final int LEFT = 0;
    public static final int RIGHT = 1;
    public static final int BACK = 2;
    private DcMotor[] encoders;

    private double[] encoder_pos; // stores previous encoder positions

    private boolean isRunning;

    public Odometry(DcMotor[] encoders) {
        super();
        this.setDaemon(true);
        this.encoders = encoders;
        this.encoder_pos = new double[3];
        this.pose = new ThreadSafePose(new double[] {0, 0, 0}); //starts at (0, 0) with heading 0
        this.isRunning = false;
    }

    public void setEncoders(DcMotor[] encoders) {
        this.encoders = encoders;
    }

    public ThreadSafePose getPose() {
        return pose;
    }

    public DcMotor[] getEncoders() {return encoders;}
    public DcMotor getLeftEncoder() {return encoders[LEFT];}
    public DcMotor getRightEncoder() {return encoders[RIGHT];}
    public DcMotor getBackEncoder() {return encoders[BACK];}

    public void resetEncoders() {
        for (DcMotor enc : encoders) {
            enc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            enc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public double ticksToCm(double ticks) {
        return (ticks/TPR)*DEADWHEEL_CIRCUMFERENCE;
    }


    @Override
    public void start() {
        isRunning = true;
        resetEncoders();
    }


    @Override
    public void run() {
        while(isRunning) {
            double[] encoder_delta = new double[encoders.length];

            // calculate delta for all encoder positions
            for (int i = 0; i<encoders.length; i++) {
                double current_pos = ticksToCm(encoders[i].getCurrentPosition()); // use CM as units
                encoder_delta[i] = current_pos - encoder_pos[i];
                encoder_pos[i] = current_pos;
            }

            double phi = (encoder_delta[LEFT] - encoder_delta[RIGHT]) / TRACKWIDTH;
            double delta_middle = (encoder_delta[LEFT] + encoder_delta[RIGHT])/2;
            double delta_perp = encoder_delta[BACK] - FORWARD_OFFSET * phi;

            double heading = pose.getR();
            Matrix rotation = new Matrix(new double[][]{
                    {Math.cos(heading), -Math.sin(heading), 0},
                    {Math.sin(heading), Math.cos(heading),  0},
                    {0                , 0                ,  1}}
            );
            Matrix curvature = new Matrix(new double[][]{
                    {(Math.sin(phi)) / phi  , (Math.cos(phi)-1)/phi, 0},
                    {(1-Math.cos(phi)) / phi, (Math.sin(phi))/phi  , 0},
                    {0                      , 0                    , 1}}
            );
            Matrix local_delta = new Matrix(new double[][]{
                    {delta_middle},
                    {delta_perp},
                    {phi}
            });
            Matrix pose_delta = (phi != 0) ?
                    rotation.multiply(curvature).multiply(local_delta) :
                    rotation.multiply(local_delta);
            telemetry.addData("pose delta", pose_delta.toString());
            pose.add(pose_delta);
        }
        try {
            Thread.sleep(10);
        } catch (Exception e) {

        }
    }

    public void updateOdometry() {
        double[] encoder_delta = new double[encoders.length];

        // calculate delta for all encoder positions
        for (int i = 0; i<encoders.length; i++) {
            double current_pos = ticksToCm(encoders[i].getCurrentPosition()); // use CM as units
            encoder_delta[i] = current_pos - encoder_pos[i];
            encoder_pos[i] = current_pos;
        }

        double phi = (encoder_delta[LEFT] - encoder_delta[RIGHT]) / TRACKWIDTH;
        double delta_middle = (encoder_delta[LEFT] + encoder_delta[RIGHT])/2;
        double delta_perp = encoder_delta[BACK] - FORWARD_OFFSET * phi;

        double heading = pose.getR();
        Matrix rotation = new Matrix(new double[][]{
                {Math.cos(heading), -Math.sin(heading), 0},
                {Math.sin(heading), Math.cos(heading),  0},
                {0                , 0                ,  1}}
        );

        Matrix curvature = new Matrix(new double[][]{
                {(Math.sin(phi)) / phi  , (Math.cos(phi)-1)/phi, 0},
                {(1-Math.cos(phi)) / phi, (Math.sin(phi))/phi  , 0},
                {0                      , 0                    , 1}}
        );

        Matrix local_delta = new Matrix(new double[][]{
                {delta_middle},
                {delta_perp},
                {phi}
        });


        Matrix pose_delta = (phi != 0) ?
                rotation.multiply(curvature).multiply(local_delta) :
                rotation.multiply(local_delta);

        pose.add(pose_delta);
    }
}
