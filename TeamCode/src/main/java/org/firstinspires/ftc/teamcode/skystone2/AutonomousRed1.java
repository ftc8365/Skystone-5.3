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
@Autonomous(name="Autonomous Red 1", group="Autonomous")
//@Disabled
public class AutonomousRed1 extends LinearOpMode {

    //////////////////////////////////////////////////////////////////////
    // Declare OpMode members
    //////////////////////////////////////////////////////////////////////

    ElapsedTime runtime = new ElapsedTime();

    ElapsedTime autonomusTimer = new ElapsedTime();

    SkystoneRobot robot = new SkystoneRobot();


    @Override
    public void runOpMode() {

        robot.setAllianceMode(SkystoneRobot.AllianceMode.ALLIANCE_RED );
        robot.setOpMode( this );
        robot.initGyroSensor();
        robot.initDriveMotors();
        robot.initFoundationServos();
        robot.initIntakeServos();
        robot.initRangeSensors();
        robot.initTensorFlowObjectDetectionWebcam();
        robot.initCameraServo();
        robot.raiseFoundationServos();

        SkystoneRobot.SkystonePosition skystonePosition = SkystoneRobot.SkystonePosition.SKYSTONE_POSITION_UNKNOWN;

        while (!opModeIsActive() && !isStopRequested()) {

            skystonePosition = robot.scanSkystone( skystonePosition);
            telemetry.addData( "Skystone", skystonePosition);


            telemetry.addData("distance",  robot.distanceSensor.getDistance(DistanceUnit.CM));

            telemetry.addData( "Gyro Pos", robot.getCurrentPositionInDegrees());
            telemetry.addData( "MotorFR Pos", robot.motorFR.getCurrentPosition());
            telemetry.addData( "MotorFL Pos", robot.motorFL.getCurrentPosition());
            telemetry.addData( "range_sensorFR", robot.rangeSensorFR.rawUltrasonic());
            telemetry.addData( "range_sensorFL", robot.rangeSensorFL.rawUltrasonic());

            telemetry.addData("",  "------------------------------");
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

        robot.resetAutonomousTimer();

        robot.shutdownTensorFlow();

        robot.setLatchPosition( SkystoneRobot.LatchPosition.LATCH_POSITION_1 );
        robot.servoCamera.setPosition(0);

        //        robot.driveForwardTillRotation(1.35, 0.50, 0,true,false);
//        robot.driveLeftTillRotation(sidewayRotation, 0.5,0, true, true);

        while (opModeIsActive() ) {

            telemetry.addData( "MotorFR Pos", robot.motorFR.getCurrentPosition());
            telemetry.addData( "MotorFL Pos", robot.motorFL.getCurrentPosition());
            telemetry.addData( "Gyro Pos", robot.getCurrentPositionInDegrees());
            telemetry.update();
        }


    }

}
