/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Skystone - DriverControl", group="TeleOp")

public class SkystoneDriverControl extends LinearOpMode {

    SkystoneRobot robot = new SkystoneRobot();

    double DRIVE_POWER_RATIO = 0.60;
    double TURN_POWER_RATIO = 0.50;

    double liftGrabberPos = 0.0;
    double liftRotatorPos = 0.0;
    boolean useDirectionAware = false;

    enum LiftState {
        LIFT_UP,
        LIFT_DOWN
    }


    LiftState liftState = LiftState.LIFT_DOWN;

    @Override
    public void runOpMode()
    {
        robot.setOpMode(this);
        robot.initMotors();
        robot.initAttachmentServos();
        robot.initGyroSensor();

        robot.servoLiftGrabber.setPosition(this.liftGrabberPos);
        robot.servoLiftRotator.setPosition(liftRotatorPos);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("",  "------------------------------");
            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() ) {

            operateIntake();
            operateDriveTrain();
//            operateFoundation();

            telemetry.update();
        }

        robot.stopAllMotors();
    }

    void operateFoundation(){
        if (gamepad2.dpad_down) {
            robot.servoFoundation.setPosition(0.3);
        }
        if (gamepad2.dpad_up){
            robot.servoFoundation.setPosition(0.8);
        }
    }

    void operateIntake() {

        if (gamepad2.right_trigger > 0) {
            robot.servoLiftGrabber.setPosition(0.30);
        }

        if (gamepad2.left_trigger > 0) {
            robot.servoLiftGrabber.setPosition(0.00);
        }

        if (gamepad2.left_bumper) {
            robot.servoLiftRotator.setPosition(1.00);
        }

        if (gamepad2.right_bumper) {
            robot.servoLiftRotator.setPosition(0.00);
        }

        robot.motorGrabber.setPower(0.0);

        if (Math.abs(gamepad2.left_stick_y) > 0.1) {

            if (gamepad2.left_stick_y < 0.01) {
                robot.motorLift.setPower(-0.40);
                this.liftState = LiftState.LIFT_UP;
            }
            else if (gamepad2.left_stick_y > 0.01) {
                robot.motorLift.setPower(0.15);
                this.liftState = LiftState.LIFT_DOWN;
            }
        }
        else {
            if (this.liftState == LiftState.LIFT_UP)
                robot.motorLift.setPower(-0.10);
            else
                robot.motorLift.setPower( 0.10);
        }

        if (Math.abs(gamepad2.right_stick_y) > 0.1) {

            if (gamepad2.right_stick_y > 0.01)
                robot.motorGrabber.setPower( 0.35 );
            else if (gamepad2.right_stick_y < 0.01)
                robot.motorGrabber.setPower( -0.35 );

        }

        //////////////////////////////////////////////////////
        // RIGHT TRIGGER ON         => Intake In
        // LEFT  TRIGGER ON         => Intake Out
        //////////////////////////////////////////////////////
        if ( gamepad1.right_trigger > 0 ) {
            robot.turnIntakeOn( SkystoneRobot.IntakeDirection.INTAKE_DIRECTION_IN );
        } else if ( gamepad1.left_trigger > 0 ) {
            robot.turnIntakeOn( SkystoneRobot.IntakeDirection.INTAKE_DIRECTION_OUT );
        } else {
            robot.turnIntakeoff();
        }

//        telemetry.addData("Grabber Position", this.liftGrabberPos);
//        telemetry.addData("Rotator Position", this.liftRotatorPos);

    }

    void operateDriveTrain() {

        ///////////////////////////////////////////////////////////
        // Gamepad1 Button X - turns on/off Direction Aware Drive
        ///////////////////////////////////////////////////////////

        if (gamepad1.x == true)
        {
            this.useDirectionAware = !this.useDirectionAware;
        }

        ///////////////////////////////////////////////////////////
        // Gamepad1 Button Y - for Direction Aware Drive - reset direction
        ///////////////////////////////////////////////////////////
//        if (gamepad1.y == true)
//        {
//            this.initGyroPosition = gyroSensor.getHeading();
//        }

        double motorFRPower = 0;
        double motorFLPower = 0;
        double motorBRPower = 0;
        double motorBLPower = 0;

        double driveStickXValue = gamepad1.left_stick_x;
        double driveStickYValue = gamepad1.left_stick_y;

        double turnStickXValue = gamepad1.right_stick_x;
        double turnStickYValue = gamepad1.right_stick_y;

        int joystickPosition = 0;

        if (this.useDirectionAware)
            joystickPosition = getDirectionAwareJoystickPosition( driveStickXValue, driveStickYValue);
        else
            joystickPosition = getJoystickPosition(driveStickXValue, driveStickYValue);

        if (Math.abs(turnStickXValue) > 0.1) {
            motorFRPower = -1 * turnStickXValue * TURN_POWER_RATIO;
            motorFLPower =  1 * turnStickXValue * TURN_POWER_RATIO;
            motorBRPower = -1 * turnStickXValue * TURN_POWER_RATIO;
            motorBLPower =  1 * turnStickXValue * TURN_POWER_RATIO;
        }
        else {
            double value = Math.max( Math.abs(driveStickYValue), Math.abs(driveStickXValue));

            switch (joystickPosition) {
                case 1:
                    motorFRPower = 1 * value;
                    motorFLPower = 1 * value;
                    motorBRPower = 1 * value;
                    motorBLPower = 1 * value;
                    break;

                case 2:
                    motorFRPower = 0 * value;
                    motorFLPower = 1 * value;
                    motorBRPower = 1 * value;
                    motorBLPower = 0 * value;
                    break;

                case 3:
                    motorFRPower = -1 * value;
                    motorFLPower =  1 * value;
                    motorBRPower =  1 * value;
                    motorBLPower = -1 * value;
                    break;

                case 4:
                    motorFRPower = -1 * value;
                    motorFLPower = 0 * value;
                    motorBRPower = 0 * value;
                    motorBLPower = -1 * value;
                    break;
                case 5:
                    motorFRPower = -1 * value;
                    motorFLPower = -1 * value;
                    motorBRPower = -1 * value;
                    motorBLPower = -1 * value;
                    break;
                case 6:
                    motorFRPower =  0 * value;
                    motorFLPower = -1 * value;
                    motorBRPower = -1 * value;
                    motorBLPower =  0 * value;
                    break;
                case 7:
                    motorFRPower =  1 * value;
                    motorFLPower = -1 * value;
                    motorBRPower = -1 * value;
                    motorBLPower =  1 * value;
                    break;
                case 8:
                    motorFRPower = 1 * value;
                    motorFLPower = 0 * value;
                    motorBRPower = 0 * value;
                    motorBLPower = 1 * value;
                    break;
                case 0:
                    motorFRPower = 0;
                    motorFLPower = 0;
                    motorBRPower = 0;
                    motorBLPower = 0;
                    break;
            }

            motorFRPower = motorFRPower * DRIVE_POWER_RATIO;
            motorFLPower = motorFLPower * DRIVE_POWER_RATIO;
            motorBRPower = motorBRPower * DRIVE_POWER_RATIO;
            motorBLPower = motorBLPower * DRIVE_POWER_RATIO;
        }

        robot.motorFR.setPower(motorFRPower);
        robot.motorFL.setPower(motorFLPower);
        robot.motorBR.setPower(motorBRPower);
        robot.motorBL.setPower(motorBLPower);

        telemetry.addData("x", driveStickXValue);
        telemetry.addData("y", driveStickYValue);
        telemetry.addData("heading", robot.getCurrentHeading());
        telemetry.addData("joystick-pos", joystickPosition );
        telemetry.addData("Direction Aware", this.useDirectionAware);
    }

    int getJoystickPosition(double x, double y) {

        if (Math.abs(x) < 0.1 && Math.abs(y) < 0.1)
            return 0;

        int quadrant = 0;
        double angle = Math.atan( Math.abs(x) / Math.abs(y)) * 180 / Math.PI;

        if (x >= 0 & y <= 0)
            quadrant = 1;
        else if (x >= 0 & y >= 0) {
            angle = 180 - angle;
            quadrant = 2;
        }
        else if (x <= 0 & y > 0) {
            angle = 180 + angle;
            quadrant = 3;
        }
        else if (x <= 0 & y <= 0) {
            angle = 360 - angle;
            quadrant = 4;
        }

        return (int)(((angle + 22.5)%360) / 45) + 1;
    }


    int getDirectionAwareJoystickPosition(double x, double y)
    {
        int position = getJoystickPosition(x,y);

        if (position == 0)
            return  0;

        double degrees = robot.getCurrentPositionInDegrees();

        int offset = 0;

        if (degrees >= 0 && degrees <= 22.5)
            offset = 0;
        else if (degrees > 22.5 && degrees <= 67.5)
            offset = -1;
        else if (degrees > 67.5 && degrees <= 112.5)
            offset = -2;
        else if (degrees > 112.5 && degrees <= 157.5)
            offset = -3;
        else if (degrees > 157.5 && degrees <= 202.5)
            offset = -4;
        else if (degrees > 202.5 && degrees <= 247.5)
            offset = -5;
        else if (degrees > 247.5 && degrees <= 292.5)
            offset = -6;
        else if (degrees > 337.5 && degrees <= 337.5)
            offset = -7;
        else
            offset = 0;


        position += offset;
        if (position <= 0)
            position += 8;

        return position;
    }

}


