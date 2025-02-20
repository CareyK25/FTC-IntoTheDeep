package org.firstinspires.ftc.teamcode.PurePursuit;

import org.firstinspires.ftc.teamcode.datatypes.Pose;
import org.firstinspires.ftc.teamcode.util.MathFunctions;

import java.util.ArrayList;

public class Route {

    private ArrayList<Pose> pathPoses;
    private Pose robotPose;

    public Route(ArrayList<Pose> p) {
        pathPoses = p;
    }

    public void run(double movementSpeed, double turnSpeed) {
        for (int i = 0; i<pathPoses.size();i++) {
            while (MathFunctions.distance(robotPose.getPoint(), pathPoses.get(i).getPoint()) > 0.6 /*Don't forget angle part*/) {
                double [] move = RobotMovement.goToPosition(pathPoses.get(i), robotPose, movementSpeed, turnSpeed);

            }
        }
    }

}
