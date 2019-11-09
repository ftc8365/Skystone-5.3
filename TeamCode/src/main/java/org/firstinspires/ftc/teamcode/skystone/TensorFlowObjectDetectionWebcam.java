/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "TEST Object Detection Webcam", group = "Test")
//@Disabled
public class TensorFlowObjectDetectionWebcam extends LinearOpMode {

    SkystoneRobot robot = new SkystoneRobot();

    @Override
    public void runOpMode() {

        robot.setOpMode(this);

        robot.initCameraServo();
        robot.servoCamera.setPosition(0.52);

        robot.initTensorFlowObjectDetectionWebcam();

        ElapsedTime runtime = new ElapsedTime();
        double lastSecond = -1;
        boolean skystoneFoundPos1 = false;
        boolean skystoneFoundPos2 = false;

        while (!opModeIsActive() && !isStopRequested()) {

            if ( runtime.seconds() > lastSecond + 1 ) {

                skystoneFoundPos1 = false;
                skystoneFoundPos2 = false;

                skystoneFoundPos1 = false;
                lastSecond = runtime.seconds();

                robot.servoCamera.setPosition(0.52);

                sleep(1000);
                List<Recognition> updatedRecognitions = robot.getTensorFlowUpdatedRecognitions();

                if (updatedRecognitions != null) {
                    telemetry.addData("Pos 1 Camera Pos", robot.servoCamera.getPosition());
                    telemetry.addData("Pos 1 # Object Detected", updatedRecognitions.size());

                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals("Skystone"))
                            skystoneFoundPos1 = true;
                        telemetry.addData(String.format("Pos 1 label (%d)", ++i), recognition.getLabel());

//                            telemetry.addData(String.format("  left,top (%d)", i), "%.01f , %.01f",
//                                    recognition.getLeft(), recognition.getTop());
//                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.01f , %.01f",
//                                    recognition.getRight(), recognition.getBottom());
                    }
                }


                if (!skystoneFoundPos1) {
                    robot.servoCamera.setPosition(0.45);

                    sleep(1000);
                    updatedRecognitions = robot.getTensorFlowUpdatedRecognitions();

                    if (updatedRecognitions != null) {
                        telemetry.addData("Pos 2 Camera Pos", robot.servoCamera.getPosition());
                        telemetry.addData("Pos 2 # Object Detected", updatedRecognitions.size());
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals("Skystone"))
                                skystoneFoundPos2 = true;

                            telemetry.addData(String.format("Pos 2 label (%d)", ++i), recognition.getLabel());
//                                telemetry.addData(String.format("  left,top (%d)", i), "%.01f , %.01f",
//                                        recognition.getLeft(), recognition.getTop());
//                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.01f , %.01f",
//                                        recognition.getRight(), recognition.getBottom());
                        }
                    }
                }

                telemetry.addData( "Found Pos 1", skystoneFoundPos1 );
                telemetry.addData( "Found Pos 2", skystoneFoundPos2 );
                telemetry.addData( "time", lastSecond );
                telemetry.update();
            }
        }


        while (opModeIsActive()) {

            List<Recognition> updatedRecognitions = robot.getTensorFlowUpdatedRecognitions();

            if (updatedRecognitions != null) {
              telemetry.addData("# Object Detected", updatedRecognitions.size());
              // step through the list of recognitions and display boundary info.
              int i = 0;
              for (Recognition recognition : updatedRecognitions) {
                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
              }
              telemetry.update();
            }
        }

        robot.shutdownTensorFlow();
    }

}
