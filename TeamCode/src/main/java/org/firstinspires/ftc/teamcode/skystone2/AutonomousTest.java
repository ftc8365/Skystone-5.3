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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


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
@Autonomous(name="Autonomous Test", group="Autonomous")
@Disabled
public class AutonomousTest extends LinearOpMode {

    //////////////////////////////////////////////////////////////////////
    // Declare OpMode members
    //////////////////////////////////////////////////////////////////////

    ElapsedTime runtime = new ElapsedTime();

    ElapsedTime autonomusTimer = new ElapsedTime();

    SkystoneRobot robot = new SkystoneRobot();


    @Override
    public void runOpMode() {

        robot.setAllianceMode(SkystoneRobot.AllianceMode.ALLIANCE_BLUE );
        robot.setOpMode( this );
        robot.initGyroSensor();
        robot.initDriveMotors();
        robot.initFoundationServos();
        robot.initRangeSensors();
//        robot.initTensorFlowObjectDetectionWebcam();
        robot.initCameraServo();
        robot.raiseFoundationServos();

        org.firstinspires.ftc.teamcode.skystone2.SkystoneRobot.SkystonePosition skystonePosition = org.firstinspires.ftc.teamcode.skystone2.SkystoneRobot.SkystonePosition.SKYSTONE_POSITION_UNKNOWN;

        while (!opModeIsActive() && !isStopRequested()) {

//            skystonePosition = robot.scanSkystone( skystonePosition);
            telemetry.addData( "Skystone", skystonePosition);

            telemetry.addData( "Gyro Pos", robot.getCurrentPositionInDegrees());
            telemetry.addData( "MotorFR Pos", robot.motorFR.getCurrentPosition());
            telemetry.addData( "MotorFL Pos", robot.motorFL.getCurrentPosition());
            telemetry.addData( "range_sensorFR", robot.rangeSensorFR.rawUltrasonic());
            telemetry.addData( "range_sensorFL", robot.rangeSensorFL.rawUltrasonic());

            telemetry.addData("",  "------------------------------");
            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }

//        robot.shutdownTensorFlow();


        robot.driveForwardTillRotation(1.0, 0.40, 0, true, false);
        robot.turnRightTillDegrees(85, true, false);
        robot.driveBackwardTillRotation(1.0, 0.40, 90,true, true);

        robot.turnRightTillDegrees(180, true, false);

        robot.driveBackwardTillRotation(1, 0.35, 180,true, true);

        robot.lowerFoundationServos();
        robot.driveBackwardTillTime(250,0.30,false);
        robot.driveForwardTillRotation(1, 0.50,180,false, true);

        robot.turnLeftTillDegrees(90, false, true);

        robot.raiseFoundationServos();
        sleep(100);
        robot.driveForwardTillRotation(1, 0.40,90, true, true);

        /*
        robot.driveBackwardTillTime(200, 0.50, false);
        robot.lowerFoundationServos();
        robot.driveBackwardTillTime(100, 0.30, false);
        robot.driveForwardTillRotation(1, 0.50, 0, false, true);
        robot.turnLeftTillDegrees(270, false,true);
        robot.raiseFoundationServos();
*/
//        robot.driveDiagionalTillRotation( 2, 0.60, true);
//        robot.turnRightTillDegrees(90, false);
            //       robot.driveForwardTillRotation(1, 0.30, 90, true);


        while (opModeIsActive() ) {

            telemetry.addData( "MotorFR Pos", robot.motorFR.getCurrentPosition());
            telemetry.addData( "MotorFL Pos", robot.motorFL.getCurrentPosition());
            telemetry.addData( "Gyro Pos", robot.getCurrentPositionInDegrees());
            telemetry.update();
        }

    }

}