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

        robot.setOpMode( this );
        robot.initMotors();
        robot.initRangeSensors();
        robot.initGyroSensor();
        robot.initColorSensors();
//        robot.initIntakeServos();
        robot.initAttachmentServos();

        while(!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("colorsesnor",  "red %d green %d blue %d", robot.colorSensor.red(), robot.colorSensor.green(), robot.colorSensor.blue());
            telemetry.addData("pos", robot.getCurrentPositionInDegrees());
            telemetry.addData("range", robot.rangeSensorFR.rawUltrasonic());


            telemetry.addData("",  "------------------------------");
            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }


        ///////////////////////////////////////
        // Start of program
        ///////////////////////////////////////

        robot.resetAutonomousTimer();

        robot.lockLiftMotor();


        robot.turnLeftTillDegrees(180, 0.40, true);


        robot.turnLeftTillDegrees(90, 0.40, true);

        robot.driveForwardTillRange(40, 0.40, 90, true);
        /*
        robot.driveForwardTillRange( 15, 0.30, false );

        robot.turnIntakeOn( SkystoneRobot.IntakeDirection.INTAKE_DIRECTION_IN);

        // Pick up stone
        robot.driveForwardTillRotation(1.5,0.15, true );

        robot.turnIntakeoff();


        robot.driveBackwardTillRotation(0.20, 0.25, true);

        robot.turnLeftTillDegrees(270, 0.40, true);

        robot.driveForwardTillRotation(0.75, 0.30, true);

        // Eject stone
        robot.turnIntakeOn( SkystoneRobot.IntakeDirection.INTAKE_DIRECTION_OUT);

        sleep(1000);

        robot.driveBackwardTillRotationOrColor(1.0, SkystoneRobot.Color.COLOR_RED,0.15,true);

        // Eject stone
        robot.turnIntakeoff();
*/


/*        robot.turnLeftTillDegrees(270, 0.40, true);

        telemetry.addData("pos", robot.getCurrentPositionInDegrees());
        telemetry.update();
        sleep(2000);

        robot.turnLeftTillDegrees(210, 0.40, true);

        telemetry.addData("pos", robot.getCurrentPositionInDegrees());
        telemetry.update();
        sleep(2000);

        robot.turnLeftTillDegrees(90, 0.40, true); */

        //        robot.driveForwardTillRotationOrCOlor(5, SkystoneRobot.Color.COLOR_BLUE, 0.25,true);

        while (opModeIsActive() ) {

//            telemetry.addData("color",  "red %d green %d blue %d", robot.colorSensor.red(), robot.colorSensor.green(), robot.colorSensor.blue());
//            telemetry.addData("blue",  robot.isColorBlue());
            telemetry.addData("pos", robot.getCurrentPositionInDegrees());
            telemetry.addData("range", robot.rangeSensorFR.rawUltrasonic());
            telemetry.update();
        }

    }

}
