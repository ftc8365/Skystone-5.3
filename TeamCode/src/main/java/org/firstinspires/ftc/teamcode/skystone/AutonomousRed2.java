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
@Autonomous(name="Autonomous Red 2", group="Autonomous")
//@Disabled
public class AutonomousRed2 extends LinearOpMode {

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
        robot.initMotors();
        robot.initRangeSensors();
        robot.initGyroSensor();
        robot.initColorSensors();
        robot.initIntakeServos();
        robot.initAttachmentServos();

        robot.initTensorFlowObjectDetectionWebcam();

        robot.initCameraServo();
        robot.servoCamera.setPosition(0.52);
        robot.setLatchPosition( SkystoneRobot.LatchPosition.LATCH_POSITION_INITIAL );

        double lastSecond = -1;
        SkystoneRobot.SkystonePosition skystonePosition = SkystoneRobot.SkystonePosition.SKYSTONE_POSITION_UNKNOWN;

        robot.servoLiftGrabber.setPosition(0.00);


        while (!opModeIsActive() && !isStopRequested()) {

            if ( runtime.seconds() > lastSecond + 2 ) {

                skystonePosition = robot.scanSkystone( skystonePosition );
                telemetry.addData( "Skystone", skystonePosition);
                telemetry.addData( "Gyro Pos", robot.getCurrentPositionInDegrees());

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
                sidewayRotation = 1.4;
                siteLocationDistanceOffset = 0.0;
                break;
            case SKYSTONE_POSITION_2:
                sidewayRotation = 2.0;
                siteLocationDistanceOffset = 0.75;
                break;
            case SKYSTONE_POSITION_3:
                sidewayRotation = 2.6;
                siteLocationDistanceOffset = 1.50;
                break;
            default:
                sidewayRotation = 1.4;
                siteLocationDistanceOffset = 0.0;
                break;
        }


        ///////////////////////////////////////
        // Start of program
        ///////////////////////////////////////

        robot.resetAutonomousTimer();

        robot.shutdownTensorFlow();

        robot.setLatchPosition( SkystoneRobot.LatchPosition.LATCH_POSITION_1 );

        robot.lockLiftMotor();

        ////////////////////////////
        //
        // 1. Drive towards first stone
        //
        /////////////////////////////

        robot.driveForwardTillRotation(1.35, 0.50, 0,true);

        robot.setLatchPosition( SkystoneRobot.LatchPosition.LATCH_POSITION_2 );

        robot.driveLeftTillRotation(sidewayRotation, 0.50, true);

        ////////////////////////////
        //
        // 2. Pick up first stone
        //
        /////////////////////////////

        robot.turnIntakeOn( SkystoneRobot.IntakeDirection.INTAKE_DIRECTION_IN);

        robot.driveForwardTillRotation(1.70, 0.25, 0,false );

        robot.turnIntakeoff();

        robot.driveBackwardTillRotation(1.2, 0.5, true);


        ////////////////////////////
        //
        // 3. Deliver first stone to building site
        //
        /////////////////////////////


        robot.turnRightTillDegrees(90, true);

        robot.driveForwardTillRotation(2.75 + siteLocationDistanceOffset, 0.6, 90, true);

        robot.motorIntakeRight.setPower(0.25);
        robot.motorIntakeLeft.setPower(0.25);

        sleep(1000);

        robot.motorIntakeRight.setPower(0.10);
        robot.motorIntakeLeft.setPower(0.10);

        robot.turnRightTillDegrees(270, true);

        robot.motorIntakeRight.setPower(0.0);
        robot.motorIntakeLeft.setPower(0.0);

        ////////////////////////////
        //
        // 4. Drive towards second stone
        //
        /////////////////////////////

        robot.driveForwardTillRotation(3.25, 0.50, 270, true);

        robot.driveForwardTillRange(35, 0.30, 270, true);

        ////////////////////////////
        //
        // 5. Pick up second stone
        //
        /////////////////////////////

        robot.turnRightTillDegrees(1,true);

        // Pick up stone

        robot.turnIntakeOn( SkystoneRobot.IntakeDirection.INTAKE_DIRECTION_IN);

        robot.driveForwardTillRotation(1.0, 0.25, -1,false );

        robot.turnIntakeoff();

        robot.driveBackwardTillRotation(2.0, 0.5, true);

        ////////////////////////////
        //
        // 6. Deliver first stone to building site
        //
        /////////////////////////////

        robot.turnRightTillDegrees(90, true);

        robot.driveForwardTillRotation(5.0, 0.60, 90, true);

        robot.motorIntakeRight.setPower(0.25);
        robot.motorIntakeLeft.setPower(0.25);

        sleep(1000);

        robot.motorIntakeRight.setPower(0.10);
        robot.motorIntakeLeft.setPower(0.10);

        ////////////////////////////
        //
        // 7. Park
        //
        /////////////////////////////

        robot.driveBackwardTillRotationOrColor(1.5, SkystoneRobot.Color.COLOR_RED, 0.30, true);

        robot.motorIntakeRight.setPower(0.0);
        robot.motorIntakeLeft.setPower(0.0);

        robot.unlockLiftMotor();

    }

}
