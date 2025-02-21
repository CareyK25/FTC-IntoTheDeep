package org.firstinspires.ftc.teamcode.PurePursuit;

import static org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement.initPos;
import static org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement.robotPose;
import static org.firstinspires.ftc.teamcode.util.Actuation.otto;

import org.firstinspires.ftc.teamcode.datatypes.Pose;
import org.firstinspires.ftc.teamcode.util.Actuation;
import org.firstinspires.ftc.teamcode.util.MathFunctions;

import java.util.ArrayList;

public class Route {

    private ArrayList<Pose> pathPoses;

    public Route(ArrayList<Pose> p) {
        pathPoses = p;
    }

    public void run(double movementSpeed, double turnSpeed) {
        for (int i = 0; i<pathPoses.size();i++) {
            while (MathFunctions.distance(robotPose.getPoint(), pathPoses.get(i).getPoint()) > 0.6 || Math.abs(MathFunctions.angleWrap(pathPoses.get(i).getR()-robotPose.getR())) > Math.toRadians(3)) {
                otto.updateOdometry();
                robotPose = otto.getPose();
                robotPose = new Pose(robotPose.getX()+initPos.getX(), robotPose.getY()+initPos.getY(), robotPose.getR());
                RobotMovement.goToPosition(pathPoses.get(i), movementSpeed, turnSpeed);
            }
            Actuation.drive(0,0,0);
        }
    }

}
