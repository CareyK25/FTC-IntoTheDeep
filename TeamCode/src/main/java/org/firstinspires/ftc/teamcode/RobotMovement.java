package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.datatypes.Pose;
import org.firstinspires.ftc.teamcode.util.MathFunctions;

public class RobotMovement {
    public static void goToPosition(double x, double y, double movementSpeed, double targetHeading, Pose robotPos, double turnSpeed) {
        double absoluteAngleToTarget = Math.atan2(y- robotPos.getY(), x- robotPos.getX());
        double relativeAngleToTarget = MathFunctions.angleWrap(absoluteAngleToTarget-(Math.toRadians(robotPos.getR())-Math.toRadians(90)));
        double dist = Math.sqrt(Math.pow(y- robotPos.getY(), 2) + Math.pow(x- robotPos.getX(), 2));//Math.hypot(y-GraphicsLineCircle.WorldPosY, x-GraphicsLineCircle.WorldPosX);
        double deltaX = x- robotPos.getX();
        double deltaY = y- robotPos.getY();
        double movementXPower = deltaX/(Math.abs(deltaX) + Math.abs(deltaY));
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

    }
}
