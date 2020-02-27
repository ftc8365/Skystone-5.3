package org.firstinspires.ftc.teamcode.skystone2;

public class PIDController {

    final double RAMP_UP_RATE_DRIVE     = 0.01;
    final double RAMP_UP_RATE_TURN      = 0.01;
    final double RAMP_UP_RATE_STRAFE    = 0.05;

    final double RAMP_DOWN_DRIVE_TICKS  = 150;
    final double RAMP_DOWN_DRIVE_RANGE  = 30;
    final double RAMP_DOWN_TURN_DEGREES = 30;
    final double RAMP_DOWN_STRAFE_TICKS = 150;

    final double MIN_DRIVE_POWER        = 0.10;
    final double MIN_TURN_POWER         = 0.35;
    final double MIN_STRAFE_POWER       = 0.20;
    final double TURN_POWER             = 0.70;
    final double TURN_TOLERANCE         = 5.0;


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // getDrivePower
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private double getDrivePower( double curPower, double ticksToGo, double targetPower, boolean rampDown ) {
        double power = curPower;

        // Ramp down power
        if ( rampDown && (ticksToGo <= RAMP_DOWN_DRIVE_TICKS) ) {
            power = ( ticksToGo / RAMP_DOWN_DRIVE_TICKS)  * curPower;

            if (power < this.MIN_DRIVE_POWER)
                power = MIN_DRIVE_POWER;
        }
        else if (targetPower - power > 0.001) {
            power += this.RAMP_UP_RATE_DRIVE;
        }

        return power;
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // getDrivePower
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private double getDriveRangePower( double curPower, double range, double targetPower ) {
        double power = curPower;

        // Ramp down power
        if ( range <= RAMP_DOWN_DRIVE_RANGE ) {
            power = ( range / RAMP_DOWN_DRIVE_RANGE)  * curPower;

            if (power < this.MIN_DRIVE_POWER)
                power = MIN_DRIVE_POWER;
        }
        else if (targetPower - power > 0.001) {
            power += this.RAMP_UP_RATE_DRIVE;
        }

        return power;
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // getTurnPower
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private double getTurnPower( double curPower, double degreesToGo, double targetPower, boolean rampDown ) {
        double power = curPower;
        double RAMP_DOWN_DISTANCE = 30;

        // Ramp down power
        if ( rampDown && (degreesToGo <= RAMP_DOWN_DISTANCE) ) {
            power = ( degreesToGo / RAMP_DOWN_DISTANCE)  * curPower;

            if (power < MIN_TURN_POWER)
                power = MIN_TURN_POWER;
        }
        else if (targetPower - power > 0.001) {
            power += this.RAMP_UP_RATE_TURN;
        }
        return power;
    }





}