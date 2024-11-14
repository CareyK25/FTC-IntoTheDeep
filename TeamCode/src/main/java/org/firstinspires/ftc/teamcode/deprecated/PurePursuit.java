package org.firstinspires.ftc.teamcode.deprecated;

import org.firstinspires.ftc.teamcode.datatypes.Pair;
import org.firstinspires.ftc.teamcode.datatypes.Pose;

public class PurePursuit {
    public static final double RIGHT_ANGLE = Math.PI/2;
    private Pose currentPose;
    private Pose targetPose;
    private  boolean isPathing;

    public PurePursuit(Pose odometry_pose) {
        this.currentPose = odometry_pose;
        this.targetPose = new Pose(0, 0, 0);
        this.isPathing = false;
    }

    public Pose getCurrentPose() {
        return currentPose;
    }

    public void setCurrentPose(Pose currentPose) {
        this.currentPose = currentPose;
    }

    public Pose getTargetPose() {
        return targetPose;
    }

    public void setTargetPose(Pose targetPose) {
        this.targetPose = targetPose;
    }

    public boolean isPathing() {
        return isPathing;
    }

    public void setPathing(boolean pathing) {
        isPathing = pathing;
    }

    public double[] goToPosition() {
        double deltaX = targetPose.getX()- currentPose.getX();
        double deltaY = targetPose.getY()- currentPose.getY();
        double absoluteAngleToTarget = Math.atan2(deltaY, deltaX);// Global angle
        double displacement_magnitude = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

        Pair local = new Pair(deltaX/displacement_magnitude, deltaY/displacement_magnitude);
        local.rotate(absoluteAngleToTarget-currentPose.getR()+RIGHT_ANGLE); //turns local axis into global axis
        return new double[]{local.getX(), local.getY(), 0};
    }
}
