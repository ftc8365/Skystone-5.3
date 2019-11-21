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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
//@Disabled
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
        robot.initMotors();
        robot.initRangeSensors();
        robot.initGyroSensor();
        robot.initColorSensors();
        robot.initIntakeServos();
        robot.initAttachmentServos();

        robot.initTensorFlowObjectDetectionWebcam();

        robot.initCameraServo();
        robot.servoCamera.setPosition(0.55);
//        robot.setLatchPosition( SkystoneRobot.LatchPosition.LATCH_POSITION_INITIAL );

        double lastSecond = -1;
        SkystoneRobot.SkystonePosition skystonePosition = SkystoneRobot.SkystonePosition.SKYSTONE_POSITION_UNKNOWN;

        while (!opModeIsActive() && !isStopRequested()) {

            if ( runtime.seconds() > lastSecond + 2 ) {

//                skystonePosition = robot.scanSkystone( skystonePosition);
                telemetry.addData( "Skystone", skystonePosition);
                telemetry.addData( "Gyro Pos", robot.getCurrentPositionInDegrees());
                telemetry.addData( "RangeFR", robot.rangeSensorFR.rawUltrasonic());
                telemetry.addData( "RangeFL", robot.rangeSensorFL.rawUltrasonic());


                telemetry.addData("",  "------------------------------");
                telemetry.addData(">", "Press Play to start");
                telemetry.update();
            }
        }

        double sidewayRotation;
        double siteLocationDistanceOffset = 0;

        switch (skystonePosition) {
            case SKYSTONE_POSITION_1:
            case SKYSTONE_POSITION_UNKNOWN:
                sidewayRotation = 0.9;
                siteLocationDistanceOffset = 0.0;
                break;
            case SKYSTONE_POSITION_2:
                sidewayRotation = 1.7;
                siteLocationDistanceOffset = 0.1;
                break;
            case SKYSTONE_POSITION_3:
                sidewayRotation = 2.4;
                siteLocationDistanceOffset = 0.2;
                break;
            default:
                sidewayRotation = 0.9;
                break;
        }


        ///////////////////////////////////////
        // Start of program
        ///////////////////////////////////////

        robot.resetAutonomousTimer();

        robot.shutdownTensorFlow();

//        robot.driveForwardTillRange(26, 0.3, -1,true);

        robot.turnIntakeOn( SkystoneRobot.IntakeDirection.INTAKE_DIRECTION_IN);

        robot.curveLeftTillDegrees(300, true);




        ////////////////////////////
        //
        // 2. Pick up first stone
        //
        /////////////////////////////
/*
        robot.turnIntakeOn( SkystoneRobot.IntakeDirection.INTAKE_DIRECTION_IN);

        robot.driveForwardTillRotation(1.5, 0.25, -1,false );

        robot.turnIntakeoff();

        robot.driveBackwardTillRotation(0.75, 0.5, true);


        ////////////////////////////
        //
        // 3. Deliver first stone to building site
        //
        /////////////////////////////


        robot.turnLeftTillDegrees(270, true);

        robot.servoPoker.setPosition(0.50);

        robot.driveForwardTillRotation(0.5, 0.5, 270,true);

        robot.turnIntakeOn( SkystoneRobot.IntakeDirection.INTAKE_DIRECTION_OUT);

        robot.turnLeftTillDegrees(90,true);

        robot.turnIntakeoff();

        robot.setLatchPosition( SkystoneRobot.LatchPosition.LATCH_POSITION_2 );

        robot.driveForwardTillRotation(2, 0.50, 90, true);

*/
        while (opModeIsActive() ) {

//            telemetry.addData("color",  "red %d green %d blue %d", robot.colorSensor.red(), robot.colorSensor.green(), robot.colorSensor.blue());
//            telemetry.addData("blue",  robot.isColorBlue());
            telemetry.addData("skystone", skystonePosition);
            telemetry.addData("pos", robot.getCurrentPositionInDegrees());

            telemetry.addData( "RangeFR", robot.rangeSensorFR.rawUltrasonic());
            telemetry.addData( "RangeFL", robot.rangeSensorFL.rawUltrasonic());
//            telemetry.addData("range", robot.rangeSensorFR.rawUltrasonic());
            telemetry.update();
        }

    }

}
