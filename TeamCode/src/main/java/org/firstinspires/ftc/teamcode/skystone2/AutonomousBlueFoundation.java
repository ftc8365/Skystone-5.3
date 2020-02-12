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

    enum AutoStep {
        AUTO_STEP_0,
        AUTO_STEP_1,
        AUTO_STEP_2,
        AUTO_STEP_3,
        AUTO_STEP_4,
        AUTO_STEP_5,
        AUTO_STEP_6,
        AUTO_STEP_7
    }

    AutoStep nextAutoStep = AutoStep.AUTO_STEP_0;
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
        robot.initTensorFlowObjectDetectionWebcam();
        robot.initCameraServo();
        robot.raiseFoundationServos();

        robot.initV4BLState(SkystoneRobot.V4BLState.V4BL_STATE_INTAKE, 0);
        robot.raiseGrabber();


        while (!opModeIsActive() && !isStopRequested()) {

            skystonePosition = robot.scanSkystone(skystonePosition);
            telemetry.addData("Skystone", skystonePosition);


            telemetry.addData("distance", robot.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Gyro Pos", robot.getCurrentPositionInDegrees());
            telemetry.addData("MotorFR Pos", robot.motorFR.getCurrentPosition());
            telemetry.addData("MotorFL Pos", robot.motorFL.getCurrentPosition());
            telemetry.addData("range_sensorFR", robot.rangeSensorBR.rawUltrasonic());
            telemetry.addData("range_sensorFL", robot.rangeSensorBL.rawUltrasonic());

            telemetry.addData("", "------------------------------");
            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }

        double sidewayRotation;
        double siteLocationDistanceOffset = 0;

        switch (skystonePosition) {
            case SKYSTONE_POSITION_1:
            case SKYSTONE_POSITION_UNKNOWN:
                sidewayRotation = 1.6;
                siteLocationDistanceOffset = 0.0;
                break;
            case SKYSTONE_POSITION_2:
                sidewayRotation = 2.8;
                siteLocationDistanceOffset = 0.75;
                break;
            case SKYSTONE_POSITION_3:
                sidewayRotation = 3.6;
                siteLocationDistanceOffset = 1.50;
                break;
            default:
                sidewayRotation = 2.0;
                siteLocationDistanceOffset = 0.0;
                break;
        }

        ///////////////////////////////////////
        // Start of program
        ///////////////////////////////////////

        boolean autoDone = false;

        while (!autoDone && opModeIsActive()) {

            switch (nextAutoStep) {
                case AUTO_STEP_0:
                    startAuto();
                    nextAutoStep = AutoStep.AUTO_STEP_1;
                    break;

                case AUTO_STEP_1:
                    grabSkystone();
                    nextAutoStep = AutoStep.AUTO_STEP_2;
                    autoDone = true;
                    break;

                case AUTO_STEP_2:
//                    grabFoundation();
                    nextAutoStep = AutoStep.AUTO_STEP_3;
                    break;

                case AUTO_STEP_3:
//                    dropSkystone();
                    nextAutoStep = AutoStep.AUTO_STEP_4;
                    break;

                case AUTO_STEP_4:
//                    turnFoundation();
                    nextAutoStep = AutoStep.AUTO_STEP_5;
                    break;

                case AUTO_STEP_5:
//                    moveUnderAllianceBridge();
                    nextAutoStep = AutoStep.AUTO_STEP_6;
                    break;

                case AUTO_STEP_6:
                    nextAutoStep = AutoStep.AUTO_STEP_7;
                    break;

                case AUTO_STEP_7:
                    autoDone = true;
                    break;
            }
        }

        robot.stopDriveMotors();
    }

    void startAuto() {
        robot.shutdownTensorFlow();
        robot.resetAutonomousTimer();
        robot.setLatchPosition(SkystoneRobot.LatchPosition.LATCH_POSITION_1);
        robot.servoCamera.setPosition(0);
    }

    void grabSkystone() {

        robot.driveForwardTillRotation(1.50,0.50,0, true, false);

        robot.turnIntakeOn(SkystoneRobot.IntakeDirection.INTAKE_DIRECTION_IN);

        switch (skystonePosition) {
            case SKYSTONE_POSITION_2:
                robot.driveForwardTillRotation(1.50, 0.30, 0, true, true);
                robot.driveBackwardTillRotation(0.75, 0.50, 0, false, false);
                robot.turnRightTillDegrees(80, false, true);
                break;

            case SKYSTONE_POSITION_1:
                robot.turnRightTillDegrees(85, false, false);
                robot.driveBackwardTillRotation(0.25, 0.50, 90, true, true);
                break;
        }



/*

        robot.driveForwardTillRotation(0.5,0.40,90,true,false);

        robot.driveLeftTillRotation(1.3,.50, 90, false, false);
        robot.driveForwardTillRotation(0.50,0.30,90, false, true);
        robot.driveRightTillRotation(1.2,0.50,90, false, true);
*/
        robot.turnIntakeoff();

        if (robot.stoneDetected()) {
            robot.grabStone();
        }

        sleep(1000);

    }

    void grabFoundation(){
        robot.turnRightTillDegrees(90,true, true);
//        robot.driveBackwardTillRotation(4.75,0.5,90,true,true);
        robot.driveBackwardTillRange(30, 0.50, 90, true);

        robot.turnRightTillDegrees(180,true,true);
        robot.driveBackwardTillTime(1000,0.3,true);
        robot.lowerFoundationServos();
    }

    void dropSkystone(){
        robot.dropStone();
    }

    void turnFoundation(){
        robot.driveForwardTillRotation(1.75,0.6,180,true,true);
        robot.turnLeftTillDegrees(90,true,true);
        robot.raiseFoundationServos();
    }

    void moveUnderAllianceBridge() {
        robot.driveForwardTillRotation(2,0.4,90,true,true);
    }

}
