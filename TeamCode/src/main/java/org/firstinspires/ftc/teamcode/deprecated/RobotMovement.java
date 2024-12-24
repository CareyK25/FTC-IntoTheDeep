package org.firstinspires.ftc.teamcode.deprecated;

import org.firstinspires.ftc.teamcode.datatypes.CurvePoint;
import org.firstinspires.ftc.teamcode.datatypes.Pair;
import org.firstinspires.ftc.teamcode.datatypes.Point;
import org.firstinspires.ftc.teamcode.datatypes.Pose;
import org.firstinspires.ftc.teamcode.util.MathFunctions;

import java.util.ArrayList;

public class RobotMovement {

    private static int lastFoundIndex = 0;

    public static double[] goToPosition(Pose targetPose, Pose robotPose, double movementSpeed, double turnSpeed) {
        // Global angle
//        double relativeAngleToTarget = MathFunctions.angleWrap(absoluteAngleToTarget-(Math.toRadians(robotPos.getR())-Math.toRadians(90)));
        double deltaX = targetPose.getX() - robotPose.getX();
        double deltaY = targetPose.getY() - robotPose.getY();
        double dist = Math.sqrt(Math.pow(deltaY, 2) + Math.pow(deltaX, 2));
        double absoluteAngleToTarget = Math.atan2(deltaY, deltaX);
        Pair local = new Pair(deltaX, deltaY);
        local.rotate(absoluteAngleToTarget-robotPose.getR()+(Math.PI/2)); //turns local axis into global axis
        double movementXPower = local.getX()/dist; // Normalizes the movement Power
        double movementYPower = local.getY()/dist;
        double dX = movementXPower*movementSpeed;
        double dY = movementYPower*movementSpeed;
        System.out.println("target angle: " + targetPose.getR() + " World Heading: " + robotPose.getR());
        double deltaAngle = MathFunctions.angleWrap(targetPose.getR()-robotPose.getR());
        System.out.println("Delta angle: " + deltaAngle);
        //GraphicsLineCircle.deltaHeading = ((deltaAngle/Math.PI)*turnSpeed*180)/((Math.abs(deltaX) + Math.abs(deltaY)));
        double deltaHeading = ((deltaAngle/(Math.PI))*turnSpeed);
        if (deltaAngle > 0) {
            deltaHeading = Math.max(deltaHeading, 1);
        }
        else if (deltaHeading<0){
            deltaHeading = Math.min(deltaHeading, -1);
        }
        System.out.println("DeltaX: "+deltaX + " DeltaY:" + deltaY);
        return new double[]{dX, dY, deltaHeading};
    }

    public static double[] goToPosition(Pose targetPose, Pose robotPose, double movementSpeed) {
        double deltaX = targetPose.getX() - robotPose.getX();
        double deltaY = targetPose.getY() - robotPose.getY();
        double absoluteAngleToTarget = Math.atan2(deltaY, deltaX);// Global angle
        //double relativeAngleToTarget = MathFunctions.angleWrap(absoluteAngleToTarget-(Math.toRadians(robotPos.getR())-Math.toRadians(90)));
        double distance = MathFunctions.distance(new Point(targetPose.getX(), targetPose.getY()), new Point(robotPose.getX(), robotPose.getY()));
        Pair local = new Pair(deltaX, deltaY);
        //local.rotate(-robotPose.getR()); //turns local axis into global axis
        local.rotate(absoluteAngleToTarget-robotPose.getR()+(Math.PI/2));
        double movementXPower = local.getX()/distance; // Normalizes the movement Power
        double movementYPower = local.getY()/distance;
        double dX = movementXPower*movementSpeed;
        double dY = movementYPower*movementSpeed;
        //System.out.println("DeltaX: "+deltaX + " DeltaY:" + deltaY);
        return new double[]{dX, dY, 0};
    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Pose robotPose, double followRadius) {
        CurvePoint followMe = pathPoints.get(lastFoundIndex);
        for (int i = lastFoundIndex; i<pathPoints.size()-1;i++) {
            CurvePoint startPoint = pathPoints.get(i);
            CurvePoint endPoint = pathPoints.get(i+1);

            Point [] intersections = getIntersections(followRadius, robotPose.getPoint(), startPoint.toPoint(), endPoint.toPoint());

            /*double minAngle = Integer.MAX_VALUE;
            for (int j = 0; j<intersections.length;j++) {
                double absoluteAngle = Math.atan2(intersections[j].getY()-GraphicsLineCircle.WorldPosY, intersections[j].getX()-GraphicsLineCircle.WorldPosX);
                double deltaAngle = Math.abs(MathFunctions.angleWrap(absoluteAngle-Math.toRadians(GraphicsLineCircle.WorldHeading)));
                System.out.println("Point: "+intersections[j] + " dAngle: " + deltaAngle);
                if (minAngle > deltaAngle) {
                    minAngle = deltaAngle;
                    followMe.setPoint(intersections[j]);
                }
            }*/
            double minCurveDist = Integer.MAX_VALUE;
            for (int j = 0; j<intersections.length;j++) {
                double intersectCurveDist = MathFunctions.distance(intersections[j], pathPoints.get(lastFoundIndex+1).toPoint());
                if (minCurveDist>intersectCurveDist) {
                    minCurveDist = intersectCurveDist;
                    followMe.setPoint(intersections[j]);
                }
            }
            System.out.println("Last Index: " + lastFoundIndex);
            System.out.println("Len: " + intersections.length);
            double centerCurveDist = MathFunctions.distance(robotPose.getPoint(), pathPoints.get(lastFoundIndex+1).toPoint());
            if (centerCurveDist>minCurveDist) {
                lastFoundIndex = i;
                break;
            }
            else {
                if (lastFoundIndex == pathPoints.size()-2) {
                    followMe = pathPoints.get(pathPoints.size()-1);
                }
                lastFoundIndex++;
            }
        }
        return followMe;
    }
    public static Point[] getIntersections(double r, Point center, Point p1, Point p2) {
        double h = center.getX();
        double k = center.getY();
        if (p1.getX() == p2.getX()) {
            p2 = new Point(p2.getX()+.0001, p2.getY());
            System.out.println(p2);
        }
        double m = (p2.getY()-p1.getY())/(p2.getX()-p1.getX());
        double b = p1.getY()-m*p1.getX();
        double xpoint1;
        double xpoint2;
        double ypoint1;
        double ypoint2;
        double minX = Math.min(p1.getX(), p2.getX());
        double maxX = Math.max(p1.getX(), p2.getX());

        if (-1*b*b-2*b*h*m+2*b*k-h*h*m*m+2*h*k*m-k*k+m*m*r*r+r*r < 0) {
            Point [] intersections = {};
            return intersections;
        }
        else if (Math.sqrt(-1*b*b-2*b*h*m+2*b*k-h*h*m*m+2*h*k*m-k*k+m*m*r*r+r*r) == 0) {
            xpoint1 = (Math.sqrt(-1*b*b-2*b*h*m+2*b*k-h*h*m*m+2*h*k*m-k*k+m*m*r*r+r*r) - b*m+h+k*m)/(m*m+1);
            ypoint1 = (m*xpoint1+b);
            if (xpoint1<=maxX && xpoint1 >= minX) {
                Point [] intersections = new Point[1];
                intersections[0] = new Point(xpoint1, ypoint1);
                return intersections;
            }
            Point [] intersections = {};
            return intersections;
        }
        else {
            xpoint1 = (Math.sqrt(-1*b*b-2*b*h*m+2*b*k-h*h*m*m+2*h*k*m-k*k+m*m*r*r+r*r) - b*m+h+k*m)/(m*m+1);
            ypoint1 = (m*xpoint1+b);
            xpoint2 = (-1*Math.sqrt(-1*b*b-2*b*h*m+2*b*k-h*h*m*m+2*h*k*m-k*k+m*m*r*r+r*r) - b*m+h+k*m)/(m*m+1);
            ypoint2 = (m*xpoint2+b);
            if (xpoint1<=maxX && xpoint1 >= minX && xpoint2<=maxX && xpoint2 >= minX) {
                Point [] intersections = new Point[2];
                intersections[0] = new Point(xpoint1, ypoint1);
                intersections[1] = new Point(xpoint2, ypoint2);
                return intersections;
            }
            if (xpoint1<=maxX && xpoint1 >= minX) {
                Point [] intersections = new Point[1];
                intersections[0] = new Point(xpoint1, ypoint1);
                return intersections;
            }
            if (xpoint2<=maxX && xpoint2 >= minX) {
                Point [] intersections = new Point[1];
                intersections[0] = new Point(xpoint2, ypoint2);
                return intersections;
            }
            Point [] intersections = {};
            return intersections;
        }
    }
    public static CurvePoint followCurve(ArrayList<CurvePoint> allPoints, Pose robotPose) { // can be void return type
        CurvePoint followMe = getFollowPointPath(allPoints, robotPose, allPoints.get(0).getFollowDistance());
        if (lastFoundIndex != allPoints.size()-1 || MathFunctions.distance(robotPose.getPoint(), allPoints.get(allPoints.size()-1).toPoint()) >.6 || Math.abs(MathFunctions.angleWrap(allPoints.get(lastFoundIndex == allPoints.size()-1? allPoints.size()-1:lastFoundIndex+1).getTargetHeading()-robotPose.getR()))>Math.toRadians(2)) {
            goToPosition(followMe.toPose(), robotPose, allPoints.get(lastFoundIndex+1).getMoveSpeed());
        }
        return followMe;
    }
    public static void move(ArrayList<CurvePoint> allPoints, Pose robotPose) {
        if (MathFunctions.distance(new Point(robotPose.getX(), robotPose.getY()), allPoints.get(lastFoundIndex+1).toPoint()) >.6 || Math.abs(MathFunctions.angleWrap(allPoints.get(lastFoundIndex+1).getTargetHeading()-robotPose.getR()))>Math.toRadians(2)) {
            goToPosition(allPoints.get(lastFoundIndex+1).toPose(), robotPose, allPoints.get(lastFoundIndex+1).getMoveSpeed(), allPoints.get(lastFoundIndex+1).getTurnSpeed());
            //System.out.println("Last Index: " + lastFoundIndex);
        }
        else if (lastFoundIndex != allPoints.size()-2) {
            lastFoundIndex++;
        }
    }

}
