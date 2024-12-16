package org.firstinspires.ftc.teamcode.deprecated;

import org.firstinspires.ftc.teamcode.datatypes.Pair;
import org.firstinspires.ftc.teamcode.datatypes.Pose;

public class PurePursuit {
    public static final double RIGHT_ANGLE = Math.PI/2;
    private Pose robotPose;
    private Pose targetPose;

    public PurePursuit(Pose odometry_pose) {
        this.robotPose = odometry_pose;
        this.targetPose = new Pose(0, 0, 0);
    }

    public Pose getRobotPose() {
        return robotPose;
    }

    public void setRobotPose(Pose robotPose) {
        this.robotPose = robotPose;
    }

    public Pose getTargetPose() {
        return targetPose;
    }

    public void setTargetPose(Pose targetPose) {
        this.targetPose = targetPose;
    }


}
