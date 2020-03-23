package org.firstinspires.ftc.teamcode.skystone2;

import org.firstinspires.ftc.teamcode.gearmasters.MovementVars;

import static org.firstinspires.ftc.teamcode.gearmasters.Robot.*;

public class RobotMovement {
    public static void goToPosition(double x, double y, double movementSpeed) {

        double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);

        double absoluteAngleToTarget = Math.atan2(y-worldYPosition, x-worldXPosition);

        double relativeAngleToPoint = MathFunctions.angleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;

        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        MovementVars.movement_x = movementXPower;
        MovementVars.movement_y = movementYPower;
    }
}
