package org.firstinspires.ftc.teamcode.deprecated;

import org.firstinspires.ftc.teamcode.datatypes.Pair;
import org.firstinspires.ftc.teamcode.datatypes.Pose;
import org.firstinspires.ftc.teamcode.util.MathFunctions;

public class RobotMovement {

    public static double[] goToPosition(double x, double y, double movementSpeed, double targetHeading, Pose robotPos, double turnSpeed) {
        // Global angle
//        double relativeAngleToTarget = MathFunctions.angleWrap(absoluteAngleToTarget-(Math.toRadians(robotPos.getR())-Math.toRadians(90)));
//        double dist = Math.sqrt(Math.pow(y- robotPos.getY(), 2) + Math.pow(x- robotPos.getX(), 2));//Math.hypot(y-GraphicsLineCircle.WorldPosY, x-GraphicsLineCircle.WorldPosX);
        double deltaX = x- robotPos.getX();
        double deltaY = y- robotPos.getY();
        double globalAngleToTarget = Math.atan2(deltaY, deltaX);
        Pair local = new Pair(deltaX, deltaY);
        local.rotate(-robotPos.getR()); //turns local axis into global axis
        double movementXPower = deltaX/(Math.abs(deltaX) + Math.abs(deltaY)); // Normalizes the movement Power
        double movementYPower = deltaY/(Math.abs(deltaX) + Math.abs(deltaY));
        double dX = movementXPower*movementSpeed;
        double dY = movementYPower*movementSpeed;
        System.out.println("target angle: " + targetHeading + " World Heading: " + Math.toRadians(robotPos.getR()));
        double deltaAngle = MathFunctions.angleWrap(targetHeading-Math.toRadians(robotPos.getR()));
        System.out.println("Delta angle: " + deltaAngle);
        //GraphicsLineCircle.deltaHeading = ((deltaAngle/Math.PI)*turnSpeed*180)/((Math.abs(deltaX) + Math.abs(deltaY)));
        double deltaHeading = ((deltaAngle/(Math.PI))*turnSpeed);
        if (deltaAngle > 0) {
            deltaHeading = Math.max(deltaHeading, 0.4);
        }
        else {
            deltaHeading = Math.min(deltaHeading, 0);
        }
        System.out.println("DeltaX: "+deltaX + " DeltaY:" + deltaY);
        return new double[]{dX, dY, deltaHeading};
    }

    public static double[] goToPosition(Pose targetPose, double movementSpeed, Pose robotPos) {
        double absoluteAngleToTarget = Math.atan2(targetPose.getY()- robotPos.getY(), targetPose.getX()- robotPos.getX());// Global angle
        double relativeAngleToTarget = MathFunctions.angleWrap(absoluteAngleToTarget-(Math.toRadians(robotPos.getR())-Math.toRadians(90)));
        double dist = Math.sqrt(Math.pow(targetPose.getY()- robotPos.getY(), 2) + Math.pow(targetPose.getX()- robotPos.getX(), 2));//Math.hypot(y-GraphicsLineCircle.WorldPosY, x-GraphicsLineCircle.WorldPosX);
        double deltaX = targetPose.getX()- robotPos.getX();
        double deltaY = targetPose.getY()- robotPos.getY();
        Pair local = new Pair(deltaX, deltaY);
        local.rotate(-robotPos.getR()); //turns local axis into global axis
        double movementXPower = deltaX/(Math.abs(deltaX) + Math.abs(deltaY)); // Normalizes the movement Power
        double movementYPower = deltaY/(Math.abs(deltaX) + Math.abs(deltaY));
        double dX = movementXPower*movementSpeed;
        double dY = movementYPower*movementSpeed;
        //System.out.println("DeltaX: "+deltaX + " DeltaY:" + deltaY);
        return new double[]{dX, dY, 0};
    }


}
