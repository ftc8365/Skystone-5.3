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

package org.firstinspires.ftc.teamcode.skystone2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
@Autonomous(name="Auto Blue Foundation", group="Autonomous")
//@Disabled
public class AutonomousBlueFoundation extends LinearOpMode {

    //////////////////////////////////////////////////////////////////////
    // Declare OpMode members
    //////////////////////////////////////////////////////////////////////

    ElapsedTime runtime = new ElapsedTime();

    ElapsedTime autonomusTimer = new ElapsedTime();

    SkystoneRobot robot = new SkystoneRobot();

    SkystoneRobot.SkystonePosition skystonePosition = SkystoneRobot.SkystonePosition.SKYSTONE_POSITION_UNKNOWN;

    @Override
    public void runOpMode() {

        robot.setAllianceMode(SkystoneRobot.AllianceMode.ALLIANCE_BLUE);
        robot.setOpMode(this);
        robot.initGyroSensor();
        robot.initDriveMotors();
        robot.initIntakeMotors();
        robot.initFoundationServos();
        robot.initIntakeServos();
        robot.initLiftServos();
        robot.initRangeSensors();
        robot.initColorSensors();
        robot.initTensorFlowObjectDetectionWebcam();
        robot.initCameraServo();
        robot.raiseFoundationServos();

        robot.initV4BLState(SkystoneRobot.V4BLState.V4BL_STATE_INTAKE);
        robot.raiseGrabber();

        while (!opModeIsActive() && !isStopRequested()) {

            skystonePosition = robot.scanSkystone(skystonePosition);
            telemetry.addData("Skystone", skystonePosition);

            telemetry.addData("distance", robot.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Gyro Pos", robot.getCurrentPositionInDegrees());
            telemetry.addData("MotorFR Pos", robot.motorFR.getCurrentPosition());
            telemetry.addData("MotorFL Pos", robot.motorFL.getCurrentPosition());
            telemetry.addData("range_sensor", robot.rangeSensorBack.getDistance(DistanceUnit.INCH));

            telemetry.addData("", "------------------------------");
            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }


        ///////////////////////////////////////
        // Start of program
        ///////////////////////////////////////

        startAuto();

        ////////////////
        // STEP 1
        ////////////////
        grabSkystone();

        ////////////////
        // STEP 2
        ////////////////
        grabFoundation();

        ////////////////
        // STEP 3
        ////////////////
        turnFoundation();

        ////////////////
        // STEP 4
        ////////////////
        dropSkystone();

        ////////////////
        // STEP 5
        ////////////////
        moveUnderAllianceBridge();

        robot.stopDriveMotors();

        telemetry.update();


        // REMOVE LATER
        while (opModeIsActive() ) {
            sleep(100);
        }

    }

    void startAuto() {
        robot.resetAutonomousTimer();

        telemetry.addData("startAuto Begin", robot.autonomusTimer.milliseconds());

        robot.shutdownTensorFlow();
        robot.setLatchPosition(SkystoneRobot.LatchPosition.LATCH_POSITION_1);
        robot.servoCamera.setPosition(0.3);

        telemetry.addData("startAuto End", robot.autonomusTimer.milliseconds());
    }

    /*
    void grabSkystoneMotion() {
        if (!opModeIsActive())
            return;

        robot.turnIntakeOn(SkystoneRobot.IntakeDirection.INTAKE_DIRECTION_IN);

        robot.setCoordiantes(0,0);
        robot.driveToCoordinate(0,1120);
        robot.driveToCoordinate(560,940);

    }
    */


    void grabSkystone() {

        telemetry.addData("grabSkyStone Begin", robot.autonomusTimer.milliseconds());

        if (!opModeIsActive())
            return;

        robot.turnIntakeOn(SkystoneRobot.IntakeDirection.INTAKE_DIRECTION_IN);

        robot.driveForwardTillRotation(1.50,0.10,0.50,0, true, false);

        switch (skystonePosition) {
            case SKYSTONE_POSITION_1:
                robot.driveForwardTillRotation(1.75, 0.30, 0.30, 0, false, true);

                runtime.reset();

                while (runtime.seconds() < 3) {
                    if (robot.stoneDetected())  {
                        robot.grabStone();
                        break;
                    }
                }

                robot.turnIntakeoff();

                robot.curveBackwardTillRotation(1.0, 0.5, 90, false, true);

                break;

            case SKYSTONE_POSITION_2:
                robot.turnRightTillDegrees(90, true, true);
                robot.driveBackwardTillRotation(0.25, 0.20,0.50, 90, false, false);
                robot.driveLeftTillRotation(0.50, 0.50,0.50, 90, false, false);
                robot.driveForwardTillRotation(0.45, 0.30,0.30, 90, false, true);

                runtime.reset();

                while (runtime.milliseconds() < 750) {
                    if (robot.stoneDetected())  {
                        break;
                    }
                }

                robot.driveRightTillRotation(0.40, 0.50,0.50, 90, false, true);

                robot.turnIntakeoff();

                if (robot.stoneDetected())  {
                    telemetry.addData("grabStone Begin", robot.autonomusTimer.milliseconds());
                    robot.grabStone();
                    telemetry.addData("grabStone End", robot.autonomusTimer.milliseconds());
                    break;
                }

                break;

            case SKYSTONE_POSITION_3:
                robot.turnRightTillDegrees(85, true, true);
                robot.driveForwardTillRotation(0.25, 0.20, 0.50, 90, false, false);
                robot.driveLeftTillRotation(0.50, 0.50,0.50, 90, false, false);
                robot.driveForwardTillRotation(0.45, 0.50,0.30, 90, false, true);
                sleep(500);
                robot.driveRightTillRotation(0.40, 0.50,0.50, 90, false, true);

                robot.turnIntakeoff();

                if (robot.stoneDetected()) {
                    robot.grabStone();
                }

 //               robot.turnRightTillDegrees(80, false, true);

                break;
        }

        telemetry.addData("grabSkyStone End", robot.autonomusTimer.milliseconds());

    }

    void grabFoundation() {
        telemetry.addData("grabFoundation Begin", robot.autonomusTimer.milliseconds());

        if (!opModeIsActive())
            return;

        double distanceToGo = 3.75;

        switch (skystonePosition) {
            case SKYSTONE_POSITION_1:
                distanceToGo = 1.5;
                break;

            case SKYSTONE_POSITION_2:
                distanceToGo += 0.35;
                break;

            case SKYSTONE_POSITION_3:
                distanceToGo += 0.70;
                break;
        }

        robot.setFoundationServos(SkystoneRobot.FoundationServoPosition.FOUNDATION_SERVO_MIDDLE);

        robot.driveBackwardTillRotation(distanceToGo, 0.70,0.70, 90, false, false);

        robot.driveBackwardTillRange(22,0.70,0.35, 90, true);

        robot.turnRightTillDegrees(170, false, true);

        long startBackupPos = robot.motorFR.getCurrentPosition();
        robot.driveBackwardTillTime(500,0.25,true);
        long endBackupPos = robot.motorFR.getCurrentPosition();

        robot.lowerFoundationServos();
        sleep(250);

        long distanceMoved = startBackupPos - endBackupPos;

        robot.driveForwardTillTicks(distanceMoved,0.1,0.5,180,false,true);

        telemetry.addData("grabFoundation End", robot.autonomusTimer.milliseconds());
    }

    void dropSkystone() {
        telemetry.addData("dropSkystone Begin", robot.autonomusTimer.milliseconds());

        if (opModeIsActive() && robot.stoneDetected())
            robot.dropStone();

        telemetry.addData("dropSkystone End", robot.autonomusTimer.milliseconds());
    }

    void turnFoundation() {
        telemetry.addData("turnFoundation Begin", robot.autonomusTimer.milliseconds());

        if (opModeIsActive()) {
            robot.turnLeftTillDegrees(100, 1.0, false, true);

            robot.raiseFoundationServos();

            robot.driveBackwardTillTime(1, 0.35, false);
            if (robot.stoneDetected())
                robot.dropStone();

        }

        telemetry.addData("turnFoundation End", robot.autonomusTimer.milliseconds());
    }

    void moveUnderAllianceBridge() {
        telemetry.addData("moveUnderAllianceBridge Begin", robot.autonomusTimer.milliseconds());

        if (opModeIsActive()) {
            robot.driveForwardTillRotation(3.0, 0.60, 0.60, 90, false, true);
        }
        telemetry.addData("moveUnderAllianceBridge End", robot.autonomusTimer.milliseconds());
    }

}
