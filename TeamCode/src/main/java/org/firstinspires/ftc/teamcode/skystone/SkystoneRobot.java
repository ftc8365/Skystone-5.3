package org.firstinspires.ftc.teamcode.skystone;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class SkystoneRobot {

    /////////////////////
    // Declare constants
    /////////////////////

    // Neverest 20 motors = 560 ticks per rotation
    // Neverest 40 moteos = 1120 tickers per rotation
    final int TICK_PER_WHEEL_ROTATION   = 560;

    final int AUTONOMOUS_DURATION_MSEC  = 29500;

    final double INTAKE_POWER           = 0.40;

    final double RAMP_UP_RATE_DRIVE     = 0.01;
    final double RAMP_UP_RATE_TURN      = 0.01;
    final double RAMP_UP_RATE_STRAFE    = 0.05;

    final double RAMP_DOWN_DRIVE_TICKS  = 150;
    final double RAMP_DOWN_DRIVE_RANGE  = 20;
    final double RAMP_DOWN_TURN_DEGREES = 30;
    final double RAMP_DOWN_STRAFE_TICKS = 150;

    final double MIN_DRIVE_POWER        = 0.10;
    final double MIN_TURN_POWER         = 0.15;
    final double MIN_STRAFE_POWER       = 0.20;


    final double TURN_TOLERANCE         = 2.0;

    enum IntakeDirection {
        INTAKE_DIRECTION_IN,
        INTAKE_DIRECTION_OUT
    }

    enum Color {
        COLOR_UNKNOWN,
        COLOR_BLUE,
        COLOR_RED,
        COLOR_GREY
    }

    enum SkystonePosition {
        SKYSTONE_POSITION_UNKNOWN,
        SKYSTONE_POSITION_1,
        SKYSTONE_POSITION_2,
        SKYSTONE_POSITION_3
    }

    enum Alliance {
        ALLIANCE_BLUE,
        ALLIANCE_RED
    }

    enum LatchPosition {
        LATCH_POSITION_INITIAL,
        LATCH_POSITION_1,
        LATCH_POSITION_2
    }

    /////////////////////
    // Declare motors variables
    /////////////////////
    public DcMotor motorFR = null;
    public DcMotor motorFL = null;
    public DcMotor motorBR = null;
    public DcMotor motorBL = null;

    public DcMotor motorIntakeRight = null;
    public DcMotor motorIntakeLeft  = null;

    public DcMotor motorGrabber   = null;
    public DcMotor motorLift        = null;

    /////////////////////
    // Declare vuforia tensorflow variables
    /////////////////////

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AaaD61H/////AAABmfJ7OgkkWEWVmniO8RAqZ1cEkXF6bR9ebw4Gw+hUI8s1s5iTA9Hyri+sjoSO/ISwSWxfZCI/iAzZ0RxSQyGQ7xjWaoE4AJgn4pKLKVcOsuglHJQhjQevzdFKWX6cXq4xYL6vzwX7G7zuUP6Iw3/TzZIAj7OxYl49mA30JfoXvq/kKnhDOdM531dbRyZiaNwTGibRl5Dxd4urQ5av3EU1QyFBWR04eKWBrJGffk8bdqjAbB3QDp/7ZAMi2WfkItMOP5ghc5arNRCNt5x+xX7gSq8pMt3ZoC3XPfRNNaEbf24MgaNJXlUARsfAuycgPiY83jbX0Hrctj4wZ20wqah+FNYMqvySokw6/fDmyG0mPmel";
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    /////////////////////
    // Declare sensors
    /////////////////////
    /////////////////////
    ModernRoboticsI2cRangeSensor    rangeSensorFR   = null;
    ModernRoboticsI2cRangeSensor    rangeSensorFL   = null;

    ModernRoboticsI2cRangeSensor    rangeSensor     = null;
//    DistanceSensor                  distanceSensor  = null;
    IntegratingGyroscope            gyro            = null;
    NavxMicroNavigationSensor       navxMicro       = null;


    ColorSensor                     colorSensor     = null;
    DistanceSensor                  colorDistance   = null;


    ElapsedTime autonomusTimer                      = new ElapsedTime();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // DECLARE SERVOS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Servo servoFoundation       = null;
    Servo servoLiftGrabber      = null;
    Servo servoLiftRotator      = null;
    Servo servoIntakeRight      = null;
    Servo servoIntakeLeft       = null;
    Servo servoCamera           = null;
    Servo servoPoker            = null;

    LinearOpMode opMode         = null;

    public void setOpMode(LinearOpMode opMode ) {

        this.opMode = opMode;
    }

    //----------------------------------------------------------------------------------------------
    // Initialization Methods
    //----------------------------------------------------------------------------------------------

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // resetAutonomousTimer
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void resetAutonomousTimer() {
        autonomusTimer.reset();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // initMotors
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void initMotors() {

        motorFR = opMode.hardwareMap.get(DcMotor.class, "motorFR");
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL  = opMode.hardwareMap.get(DcMotor.class, "motorFL");
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBR = opMode.hardwareMap.get(DcMotor.class, "motorBR");
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBL  = opMode.hardwareMap.get(DcMotor.class, "motorBL");
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorIntakeRight  = opMode.hardwareMap.get(DcMotor.class, "motorIntakeRight");
        motorIntakeRight.setDirection(DcMotor.Direction.FORWARD);
        motorIntakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorIntakeLeft  = opMode.hardwareMap.get(DcMotor.class, "motorIntakeLeft");
        motorIntakeLeft.setDirection(DcMotor.Direction.REVERSE);
        motorIntakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLift  = opMode.hardwareMap.get(DcMotor.class, "motorLift");
        motorLift.setDirection(DcMotor.Direction.FORWARD);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorGrabber  = opMode.hardwareMap.get(DcMotor.class, "motorGrabber");
        motorGrabber.setDirection(DcMotor.Direction.FORWARD);
        motorGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void initRangeSensors() {
        rangeSensorFR = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor1");
        rangeSensorFL = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor2");
    }

    public void initColorSensors() {
        // get a reference to the color sensor.
        colorSensor = opMode.hardwareMap.get(ColorSensor.class, "color_sensor");

        // get a reference to the distance sensor that shares the same name.
        colorDistance = opMode.hardwareMap.get(DistanceSensor.class, "color_sensor");
    }

    public void initIntakeServos() {
        servoIntakeRight = opMode.hardwareMap.get(Servo.class, "servoIntakeRight");
        servoIntakeLeft  = opMode.hardwareMap.get(Servo.class, "servoIntakeLeft");
        servoPoker       = opMode.hardwareMap.get(Servo.class, "servoPoker");
    }

    public void initCameraServo() {
        servoCamera = opMode.hardwareMap.get(Servo.class, "servoCamera");
    }

    public void initAttachmentServos() {
        servoFoundation = opMode.hardwareMap.get(Servo.class, "servo0");
        servoLiftGrabber = opMode.hardwareMap.get(Servo.class, "servo1");
        servoLiftRotator = opMode.hardwareMap.get(Servo.class, "servo2");
    }


    public void initGyroSensor() {
        navxMicro = opMode.hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope)navxMicro;

        while (navxMicro.isCalibrating())  {
            opMode.sleep(50);
        }
    }

    public void activateGyroTracking() {
    }


    public void initTensorFlowObjectDetection() {

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }

        if (tfod != null) {
            tfod.activate();
        }
    }

    public void initTensorFlowObjectDetectionWebcam() {

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforiaWebcam();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }

        if (tfod != null) {
            tfod.activate();
        }
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }


    public List<Recognition> getTensorFlowRecognitions() {
        if (tfod != null) {

            List<Recognition> updatedRecognitions = tfod.getRecognitions();

            if (updatedRecognitions != null) {
                return updatedRecognitions;
            }
        }
        return null;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initVuforiaWebcam() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = this.opMode.hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    public void shutdownTensorFlow() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public double getCurrentPositionInDegrees() {

        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double fromDegrees = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);

        // degrees
        // 0, -45, -90, -180
        // convert that to 0, 45, 90, 180

        double toDegrees;

        if (fromDegrees < 0)
            toDegrees = fromDegrees * -1;
        else
            toDegrees = 360 - fromDegrees;

        return toDegrees % 360;
    }


    float getCurrentHeading() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) * -1;
    }


    void setServoPosition(Servo servo, double targetPosition) {
        double currentPos = servo.getPosition();

        if (currentPos > targetPosition) {
            while (servo.getPosition() > targetPosition) {
                servo.setPosition(servo.getPosition() - 0.01);
                opMode.sleep(50);
            }
        } else if (currentPos < targetPosition) {
            while (servo.getPosition() < targetPosition) {
                servo.setPosition(servo.getPosition() + 0.01);
                opMode.sleep(50);
            }
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // stopAllMotors
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void stopAllMotors() {
        for (int i = 0; i < 3; ++i) {
            this.motorFL.setPower(0);
            this.motorFR.setPower(0);
            this.motorBL.setPower(0);
            this.motorBR.setPower(0);
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // continueAutonomus
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    boolean continueAutonomus() {
        return ( opMode.opModeIsActive() ) && ( autonomusTimer.milliseconds() < AUTONOMOUS_DURATION_MSEC );
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveForwardTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveForwardTillRotation( double rotation, double targetPower, int targetHeading, boolean stopMotors )
    {
        boolean useGyroToAlign  = (this.gyro != null && targetHeading >= 0) ? true : false;
        int initPosition        = motorFR.getCurrentPosition();
        int ticksToGo           = 0;
        double power            = 0.0;

        while (continueAutonomus()) {

            ticksToGo = (int)(TICK_PER_WHEEL_ROTATION * rotation) - (motorFR.getCurrentPosition() - initPosition);

            if (ticksToGo <= 0)
                break;

            power = this.getDrivePower(power, ticksToGo, targetPower);

            double powerRight = power;
            double powerLeft  = power;

            // Adjusting motor power based on gyro position
            // to force the robot to move straight
            if (useGyroToAlign) {
                double headingChange  = this.getCurrentPositionInDegrees() - targetHeading;

                if (headingChange > 180 && targetHeading < 90) {
                    headingChange = this.getCurrentPositionInDegrees() - (targetHeading + 360);


//                    opMode.telemetry.addData("pos", getCurrentPositionInDegrees());
//                    opMode.telemetry.addData("targetpos", targetHeading);
//                    opMode.telemetry.addData("change", headingChange);
//                    opMode.telemetry.update();
//                    this.stopAllMotors();
//                    opMode.sleep(100000);
                }

                powerRight +=  2 * (headingChange / 100);
                powerLeft  -=  2 * (headingChange / 100);

            }

            opMode.telemetry.addData( "power", power);
            opMode.telemetry.update();

            motorFR.setPower( powerRight );
            motorFL.setPower( powerLeft );
            motorBR.setPower( powerRight );
            motorBL.setPower( powerLeft );



        }

        if (stopMotors) {
            this.stopAllMotors();
        }
    }



    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveForwardTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveForwardTillRotationOrCOlor( double rotation, Color color, double targetPower, boolean stopMotors )
    {
        boolean useGyroToAlign = false;

        double initHeading = useGyroToAlign ? this.getCurrentHeading() : 0;

        int initPosition = motorFR.getCurrentPosition();
        int ticksToGo = 0;

        double power = 0.0;

        while (continueAutonomus()) {

            if (this.colorDetected(color))
                break;

            ticksToGo = (int)(TICK_PER_WHEEL_ROTATION * rotation) - (motorFR.getCurrentPosition() - initPosition);

            if (ticksToGo <= 0)
                break;

            power = this.getDrivePower(power, ticksToGo, targetPower);

            double powerRight = power;
            double powerLeft  = power;

            // Adjusting motor power based on gyro position
            // to force the robot to move straight
            if (useGyroToAlign && this.gyro != null) {
                double headingChange  = getCurrentHeading() - initHeading;


                powerRight += power * (-1 * headingChange / 100);
                powerLeft  += power * (-1 * headingChange / 100);
            }

            opMode.telemetry.addData( "power", power);
            opMode.telemetry.update();

            motorFR.setPower( powerRight );
            motorFL.setPower( powerLeft );
            motorBR.setPower( powerRight );
            motorBL.setPower( powerLeft );
        }

        if (stopMotors) {
            this.stopAllMotors();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveBackwardTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveBackwardTillRotation( double rotation, double targetPower, boolean stopMotors ) {

        boolean useGyroToAlign = false;
        int initPosition = motorFR.getCurrentPosition();
        int ticksToGo = 0;

        double power = 0.0;

        while (continueAutonomus()) {
            ticksToGo = (int)(TICK_PER_WHEEL_ROTATION * rotation) - (initPosition - motorFR.getCurrentPosition());


            if (ticksToGo <= 0)
                break;

            power = this.getDrivePower(power, ticksToGo, targetPower);

            motorFR.setPower( power * -1);
            motorFL.setPower( power * -1);
            motorBR.setPower( power * -1);
            motorBL.setPower( power * -1);
        }

        if (stopMotors) {
            stopAllMotors();
        }
    }



    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveBackwardTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveBackwardTillRotationOrColor( double rotation, Color color, double targetPower, boolean stopMotors ) {

        boolean useGyroToAlign = false;
        int initPosition = motorFR.getCurrentPosition();
        int ticksToGo = 0;

        double power = 0.0;

        while (continueAutonomus()) {
            if (this.colorDetected(color))
                break;

            ticksToGo = (int)(TICK_PER_WHEEL_ROTATION * rotation) - (initPosition - motorFR.getCurrentPosition());

            if (ticksToGo <= 0)
                break;

            power = this.getDrivePower(power, ticksToGo, targetPower);

            motorFR.setPower( power * -1);
            motorFL.setPower( power * -1);
            motorBR.setPower( power * -1);
            motorBL.setPower( power * -1);
        }

        if (stopMotors) {
            stopAllMotors();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveForwardTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveForwardTillRange( double range, double targetPower, int targetHeading, boolean stopMotors )
    {
        boolean useGyroToAlign  = (this.gyro != null && targetHeading >= 0) ? true : false;
        double power            = 0.0;

        while (continueAutonomus()) {

            double rangeFR = this.rangeSensorFR.rawUltrasonic();
            double rangeFL = this.rangeSensorFL.rawUltrasonic();

            double minRange = Math.min(rangeFR, rangeFL);

            if (minRange <= range )
                break;

            power = getDriveRangePower(power, minRange, targetPower);

            double powerRight = power;
            double powerLeft  = power;

            // Adjusting motor power based on gyro position
            // to force the robot to move straight
            if (useGyroToAlign) {
                double headingChange  = this.getCurrentPositionInDegrees() - targetHeading;
                if (headingChange > 180 && targetHeading < 90)
                    headingChange = this.getCurrentPositionInDegrees() - (targetHeading + 360);

                powerRight +=  2 * (headingChange / 100);
                powerLeft  -=  2 * (headingChange / 100);
            }

            motorFR.setPower( powerRight );
            motorFL.setPower( powerLeft );
            motorBR.setPower( powerRight );
            motorBL.setPower( powerLeft );
        }

        if (stopMotors) {
            this.stopAllMotors();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveLeftTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveLeftTillRotation( double rotation, double targetPower, boolean stopMotors )
    {
        int initPosition = motorFR.getCurrentPosition();
        int ticksToGo = 0;

        double power = 0.0;

        while (continueAutonomus()) {

            ticksToGo = (int)(TICK_PER_WHEEL_ROTATION * rotation) - (motorFR.getCurrentPosition() - initPosition);

            if (ticksToGo <= 0)
                break;

            power = this.getDrivePower(power, ticksToGo, targetPower);

            motorFR.setPower( power );
            motorFL.setPower( power * -1 );
            motorBR.setPower( power * -1 );
            motorBL.setPower( power );
        }

        if (stopMotors) {
            stopAllMotors();
        }
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveRightTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveRightTillRotation( double rotation, double targetPower, boolean stopMotors )
    {
        int initPosition = motorFR.getCurrentPosition();
        int ticksToGo = 0;

        double power = 0.0;

        while (continueAutonomus()) {

            ticksToGo = (int)(TICK_PER_WHEEL_ROTATION * rotation) - (initPosition - motorFR.getCurrentPosition());

            if (ticksToGo <= 0)
                break;

            power = this.getDrivePower( power, ticksToGo, targetPower );

            motorFR.setPower( power * -1 );
            motorFL.setPower( power );
            motorBR.setPower( power );
            motorBL.setPower( power * -1 );
        }

        if (stopMotors) {
            stopAllMotors();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // turnRightTillDegrees
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void turnRightTillDegrees( int targetDegrees, double targetPower, boolean stopMotors )
    {
        double power = 0.05;
        double currentHeading = 0;
        double degressToGo = 0;

        while (continueAutonomus()) {

            currentHeading = getCurrentPositionInDegrees();
            if (currentHeading > 180 && targetDegrees < 180)
                currentHeading -= 360;

            degressToGo = targetDegrees - currentHeading;
            if (degressToGo <= TURN_TOLERANCE)
                break;

            power = this.getTurnPower(power, degressToGo, targetPower);

            motorFR.setPower( -1 * power );
            motorFL.setPower(      power );
            motorBR.setPower( -1 * power );
            motorBL.setPower(      power );
        }

        if (stopMotors) {
            this.stopAllMotors();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // turnLeftTillDegrees
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void turnLeftTillDegrees( int targetDegrees, double targetPower, boolean stopMotors )
    {
        double power = 0.05;
        double currentHeading = 0;
        double degressToGo = 0;

        while (continueAutonomus()) {

            currentHeading = getCurrentPositionInDegrees();
            if (currentHeading < 90) {
               currentHeading += 360;
            }

            if (currentHeading > 270 && targetDegrees < 90) {
                targetDegrees += 360;
            }

            degressToGo = currentHeading - targetDegrees;

            if (degressToGo <= TURN_TOLERANCE)
                break;

            power = this.getTurnPower(power, degressToGo, targetPower);

            motorFR.setPower(      power );
            motorFL.setPower( -1 * power );
            motorBR.setPower(      power );
            motorBL.setPower( -1 * power );

        }


        if (stopMotors) {
            this.stopAllMotors();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // getDrivePower
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private double getDrivePower( double curPower, double ticksToGo, double targetPower ) {
        double power = curPower;


        // Ramp down power
        if ( ticksToGo <= RAMP_DOWN_DRIVE_TICKS ) {
            power = ( ticksToGo / RAMP_DOWN_DRIVE_TICKS)  * curPower;

            if (power < this.MIN_DRIVE_POWER)
                power = MIN_DRIVE_POWER;
        }
        else if (targetPower - power > 0.001) {
            power += this.RAMP_UP_RATE_DRIVE;
        }

        return power;
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // getDrivePower
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private double getDriveRangePower( double curPower, double range, double targetPower ) {
        double power = curPower;


        // Ramp down power
        if ( range <= RAMP_DOWN_DRIVE_RANGE ) {
            power = ( range / RAMP_DOWN_DRIVE_RANGE)  * curPower;

            if (power < this.MIN_DRIVE_POWER)
                power = MIN_DRIVE_POWER;
        }
        else if (targetPower - power > 0.001) {
            power += this.RAMP_UP_RATE_DRIVE;
        }

        return power;
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // getTurnPower
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private double getTurnPower( double curPower, double degreesToGo, double targetPower ) {
        double power = curPower;
        double RAMP_DOWN_DISTANCE = 30;

        // Ramp down power
        if ( degreesToGo <= RAMP_DOWN_DISTANCE ) {
            power = ( degreesToGo / RAMP_DOWN_DISTANCE)  * curPower;

            if (power < MIN_TURN_POWER)
                power = MIN_TURN_POWER;
        }
        else if (targetPower - power > 0.001) {
            power += this.RAMP_UP_RATE_TURN;
        }
        return power;
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // turnIntakeOn
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void turnIntakeOn( IntakeDirection direction ) {

        if (direction == IntakeDirection.INTAKE_DIRECTION_IN) {
            motorIntakeRight.setPower(-1 * INTAKE_POWER);
            motorIntakeLeft.setPower(-1 * INTAKE_POWER);
        } else {
            motorIntakeRight.setPower(0.60);
            motorIntakeLeft.setPower(0.60);
        }
    }

    public boolean isColorBlue() {
        if (this.colorDistance.getDistance(DistanceUnit.CM) < 3.0) {
            if (colorSensor.blue() > colorSensor.green() &&
                    colorSensor.blue() > colorSensor.red())
                return true;
        }
        return false;
    }


    public boolean isColorRed() {
        if (this.colorDistance.getDistance(DistanceUnit.CM) < 1.0) {
            if (colorSensor.red() > colorSensor.green() &&
                    colorSensor.red() > colorSensor.blue())
                return true;
        }
        return false;
    }

    public boolean colorDetected(Color color) {
        if (color == Color.COLOR_BLUE)
            return isColorBlue();
        else if (color == Color.COLOR_RED)
            return isColorRed();

        return false;
    }



    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // turnIntakeff
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void turnIntakeoff() {

        motorIntakeRight.setPower(0);
        motorIntakeLeft.setPower(0);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // lockLiftMotor
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void lockLiftMotor() {

        this.motorLift.setPower( 0.25 );
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // unlockLiftMotor
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void unlockLiftMotor() {

        this.motorLift.setPower( 0 );
    }

    public SkystonePosition scanSkystone( Alliance alliance, SkystonePosition previousPosition ) {

        double servoPos1 = (alliance == Alliance.ALLIANCE_BLUE) ? 0.55 : 0.49;
        double servoPos2 = (alliance == Alliance.ALLIANCE_BLUE) ? 0.50 : 0.54;

        setServoPosition(servoCamera, servoPos1);

        opMode.sleep(500);

        List<Recognition> updatedRecognitions = getTensorFlowRecognitions();

        if (updatedRecognitions != null) {
            for (Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel().equals("Skystone"))
                    return SkystonePosition.SKYSTONE_POSITION_1;
            }
        }

        if (!opMode.opModeIsActive() && !opMode.isStopRequested()) {
            setServoPosition(servoCamera, servoPos2);

            opMode.sleep(500);
            updatedRecognitions = getTensorFlowRecognitions();

            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals("Skystone"))
                        return SkystonePosition.SKYSTONE_POSITION_2;
                }
            }

            return SkystonePosition.SKYSTONE_POSITION_3;
        }

        return previousPosition;
    }


    void setLatchPosition( LatchPosition position ) {

        switch (position) {
            case LATCH_POSITION_INITIAL:
                servoIntakeRight.setPosition(0.15);
                servoIntakeLeft.setPosition(1.0);
                servoPoker.setPosition(0.15);
                break;
            case LATCH_POSITION_1:
                servoIntakeRight.setPosition(0.30);
                servoIntakeLeft.setPosition(0.85);
                break;
            case LATCH_POSITION_2:
                servoIntakeRight.setPosition(0.15);
                servoIntakeLeft.setPosition(1.0);
                servoPoker.setPosition(0.85);
                break;
        }

    }
}