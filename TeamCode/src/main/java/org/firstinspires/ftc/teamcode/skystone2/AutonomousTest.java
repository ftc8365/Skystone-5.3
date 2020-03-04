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

        robot.setAllianceMode(SkystoneRobot.AllianceMode.ALLIANCE_RED );
        robot.setOpMode( this );
        robot.initGyroSensor();
        robot.initDriveMotors();
        robot.initFoundationServos();
        robot.initIntakeServos();
        robot.initRangeSensors();
        robot.initCameraServo();
        robot.initLiftServos();
        robot.initIntakeMotors();
        robot.raiseFoundationServos();

        robot.servoCamera.setPosition(0.5);

        while (!opModeIsActive() && !isStopRequested()) {

            telemetry.addData("distance",  robot.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData( "Gyro Pos", robot.getCurrentPositionInDegrees());
            telemetry.addData( "MotorFR Pos", robot.motorFR.getCurrentPosition());
            telemetry.addData( "MotorFL Pos", robot.motorFL.getCurrentPosition());

            telemetry.addData("MotorBL Pos", (double)robot.motorBL.getCurrentPosition());
            telemetry.addData("MotorTape Pos", (double)robot.motorTape.getCurrentPosition());

            telemetry.addData("rangeSensorFront", robot.rangeSensorFront.getDistance(DistanceUnit.INCH));
            telemetry.addData("rangeSensorBack", robot.rangeSensorBack.getDistance(DistanceUnit.INCH));

            telemetry.addData("V4BLState", robot.getV4BLState());
            telemetry.addData("V4BLPos", robot.getV4BLServoPosition());

            telemetry.addData("",  "------------------------------");
            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }

        //robot.servoCamera.setPosition(1.0);



        robot.turnRightTillDegrees(75,0.6,false,false);
        robot.driveBackwardTillRange(18,0.70,0.50, 90, true);

        runtime.reset();
        robot.turnRightTillDegrees(170, false, false);
        robot.driveBackwardTillTime(500,0.25,true);
        robot.lowerFoundationServos();
        sleep(500);

        robot.driveForwardTillRotation(0.75,0.4,0.6,180,true,true);

        robot.turnLeftTillDegrees(85, 1.0, true, true);
        robot.driveBackwardTillTime(1000, 0.35, true);
        robot.raiseFoundationServos();


        // 1 - 5.3 sec

        double sec = runtime.seconds();
//        robot.grabStone();
//        sleep(1000);
//        robot.dropStone();

//        robot.driveLeftTillRotation(0.5, 0.50, 0, true, true);
//        robot.driveRightTillRotation(0.5, 0.50, 0, true, true);

//        robot.driveForwardTillRotation(1.75, 0.30, 0, false, false);
//        robot.curveBackwardTillRotation(1.0, 0.5, 90, false, true);

        while (opModeIsActive() ) {

            telemetry.addData( "Sec", sec);

            telemetry.addData( "MotorFR Pos", robot.motorFR.getCurrentPosition());
            telemetry.addData( "MotorFL Pos", robot.motorFL.getCurrentPosition());
            telemetry.addData( "Gyro Pos", robot.getCurrentPositionInDegrees());


            telemetry.addData("rangeSensorFront", robot.rangeSensorFront.getDistance(DistanceUnit.INCH));
            telemetry.addData("rangeSensorBack", robot.rangeSensorBack.getDistance(DistanceUnit.INCH));

            telemetry.addData("MotorBL Pos", (double)robot.motorBL.getCurrentPosition());
            telemetry.addData("MotorTape Pos", (double)robot.motorTape.getCurrentPosition());

            telemetry.update();
        }


    }

}
