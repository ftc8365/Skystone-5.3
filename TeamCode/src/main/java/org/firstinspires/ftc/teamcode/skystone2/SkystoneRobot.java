package org.firstinspires.ftc.teamcode.skystone2;


import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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

    final int AUTONOMOUS_DURATION_MSEC  = 29800;

    final double INTAKE_POWER           = 0.30;
    final double RAMP_UP_RATE_DRIVE     = 0.01;
    final double RAMP_UP_RATE_TURN      = 0.01;
    final double RAMP_UP_RATE_STRAFE    = 0.05;

    final double RAMP_DOWN_DRIVE_TICKS  = 150;
    final double RAMP_DOWN_DRIVE_RANGE  = 30;
    final double RAMP_DOWN_TURN_DEGREES = 30;
    final double RAMP_DOWN_STRAFE_TICKS = 150;

    final double MIN_DRIVE_POWER        = 0.10;
    final double MIN_TURN_POWER         = 0.35;
    final double MIN_STRAFE_POWER       = 0.20;
    final double TURN_POWER             = 0.70;
    final double TURN_TOLERANCE         = 5.0;




    ///////////////////MAKE CHANGE HERE IF NEEDED//////MAKE CHANGE TO ALL 4 VALUES//////////

/*    enum V4BLState {
        V4BL_STATE_UNKNOWN      (0.00),
        V4BL_STATE_STONE        (0.10),
        V4BL_STATE_INTAKE       (0.25),
        V4BL_STATE_TOP          (0.60),
        V4BL_STATE_FOUNDATION   (0.95);

        public final double servoPos;

        V4BLState(double pos) {
            this.servoPos = pos;
        }
    }
*/
    enum V4BLState {
        V4BL_STATE_UNKNOWN      (0.00),
        V4BL_STATE_STONE        (0.15),
        V4BL_STATE_INTAKE       (0.25),
        V4BL_STATE_TOP          (0.75),
        V4BL_STATE_FOUNDATION   (0.97);

        public final double servoPos;

        V4BLState(double pos) {
            this.servoPos = pos;
        }
    }



    double currentV4BLPos               = 0;
    V4BLState currentV4BLState          = V4BLState.V4BL_STATE_UNKNOWN;

    boolean runningAutonomous           = true;
    AllianceMode allianceMode           = AllianceMode.ALLIANCE_BLUE;

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

    enum LatchPosition {
        LATCH_POSITION_INITIAL,
        LATCH_POSITION_1
    }

    enum AllianceMode {
        ALLIANCE_BLUE,
        ALLIANCE_RED
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
    public DcMotor motorLiftRight  = null;
//    public DcMotor motorLiftLeft   = null;

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
    ModernRoboticsI2cRangeSensor    rangeSensorBR   = null;
    ModernRoboticsI2cRangeSensor    rangeSensorBL   = null;

    DistanceSensor                  distanceSensor  = null;
    IntegratingGyroscope            gyro            = null;
    NavxMicroNavigationSensor       navxMicro       = null;

    ColorSensor                     colorSensor     = null;
    DistanceSensor                  colorDistance   = null;

    ElapsedTime autonomusTimer                      = new ElapsedTime();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // DECLARE SERVOS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Servo servoFoundationLeft   = null;
    Servo servoFoundationRight  = null;
    Servo servoLiftGrabber      = null;
//    Servo servoLiftRotator      = null;
    Servo servoIntakeRight      = null;
    Servo servoIntakeLeft       = null;
    Servo servoCamera           = null;
    Servo servoV4BLUpper        = null;
    Servo servoV4BLLower        = null;
    Servo servoPoker            = null;
    Servo servoCapstone         = null;


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
    // initDriveMotors
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void initDriveMotors() {

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

    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // initIntakeMotors
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void initIntakeMotors() {

        motorIntakeRight = opMode.hardwareMap.get(DcMotor.class, "motorIntakeRight");
        motorIntakeRight.setDirection(DcMotor.Direction.REVERSE);
        motorIntakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorIntakeLeft = opMode.hardwareMap.get(DcMotor.class, "motorIntakeLeft");
        motorIntakeLeft.setDirection(DcMotor.Direction.FORWARD);
        motorIntakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // initLitMotors
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void initLiftMotors() {
        motorLiftRight  = opMode.hardwareMap.get(DcMotor.class, "motorLiftRight");
        motorLiftRight.setDirection(DcMotor.Direction.FORWARD);
        motorLiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        motorLiftLeft  = opMode.hardwareMap.get(DcMotor.class, "motorLiftLeft");
//        motorLiftLeft.setDirection(DcMotor.Direction.REVERSE);
//        motorLiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initRangeSensors() {
        rangeSensorBR = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensorFR");
        rangeSensorBL = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensorFL");
    }

    public void initColorSensors() {
        // get a reference to the color sensor.
        colorSensor = opMode.hardwareMap.get(ColorSensor.class, "colorSensor");

        // get a reference to the distance sensor that shares the same name.
        colorDistance = opMode.hardwareMap.get(DistanceSensor.class, "colorSensor");
    }

    public void initIntakeServos() {
        servoIntakeRight = opMode.hardwareMap.get(Servo.class, "servoIntakeRight");
        servoIntakeLeft  = opMode.hardwareMap.get(Servo.class, "servoIntakeLeft");
    }

    public void initCameraServo() {
        servoCamera = opMode.hardwareMap.get(Servo.class, "servoCamera");
    }

    public void initLiftServos() {
        servoV4BLUpper      = opMode.hardwareMap.get(Servo.class, "servoV4BLUpper");
        servoV4BLLower      = opMode.hardwareMap.get(Servo.class, "servoV4BLLower");
        servoLiftGrabber    = opMode.hardwareMap.get(Servo.class, "servoLiftGrabber");
        servoCapstone       = opMode.hardwareMap.get(Servo.class, "servoCapstone");
        distanceSensor      = opMode.hardwareMap.get(DistanceSensor.class, "distance_sensor");

        servoPoker       = opMode.hardwareMap.get(Servo.class, "servoPoker");
    }

    public void initFoundationServos() {
        this.servoFoundationRight = opMode.hardwareMap.get(Servo.class, "servoFoundationRight");
        this.servoFoundationLeft = opMode.hardwareMap.get(Servo.class, "servoFoundationLeft");
    }


    public void initGyroSensor() {
        navxMicro = opMode.hardwareMap.get(NavxMicroNavigationSensor.class, "gyro_sensor");
        gyro = (IntegratingGyroscope)navxMicro;

        while (navxMicro.isCalibrating())  {
            opMode.sleep(50);
        }
    }

 /*   public void initTensorFlowObjectDetection() {

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }

        if (tfod != null) {
            tfod.activate();
        }
    }*/

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

    public void setAllianceMode( AllianceMode mode ) {
        this.allianceMode = mode;

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


    void setServoPosition(Servo servo, double targetPosition, int delay) {
        double currentPos = servo.getPosition();

        if (currentPos > targetPosition) {
            while (servo.getPosition() > targetPosition) {
                servo.setPosition(servo.getPosition() - 0.01);
                opMode.sleep(delay);
            }
        } else if (currentPos < targetPosition) {
            while (servo.getPosition() < targetPosition) {
                servo.setPosition(servo.getPosition() + 0.01);
                opMode.sleep(delay);
            }
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // setV4BLServoPosition
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void setV4BLState(V4BLState targetState, int delay) {
        setV4BLPosition(targetState.servoPos, delay);
        this.currentV4BLState = targetState;
    }

    void initV4BLState(V4BLState targetState) {
        servoV4BLUpper.setPosition(targetState.servoPos);
        servoV4BLLower.setPosition(1-targetState.servoPos);
        this.currentV4BLState = targetState;
    }

/*    void setV4BLPosition(double targetPosition) {
        if (targetPosition > 1.0 || targetPosition < 0.0)
            return;

        servoV4BLUpper.setPosition(targetPosition);
        servoV4BLLower.setPosition(1-targetPosition);

        this.currentV4BLPos = targetPosition;
    }
*/

    void setV4BLPosition(double targetPosition, int delay) {
        if (targetPosition > 1.0 || targetPosition < 0.0)
            return;

        double currentPos = servoV4BLUpper.getPosition();

        if (currentPos > targetPosition) {
            while (servoV4BLUpper.getPosition() > targetPosition) {
                double targetPosUpper = servoV4BLUpper.getPosition() - 0.02;
                double targetPosLower = 1- targetPosUpper;

                servoV4BLUpper.setPosition(targetPosUpper);
                servoV4BLLower.setPosition(targetPosLower);
                opMode.sleep(delay);
            }
        } else if (currentPos < targetPosition) {
            while (servoV4BLUpper.getPosition() < targetPosition) {
                double targetPosUpper = servoV4BLUpper.getPosition() + 0.02;
                double targetPosLower = 1- targetPosUpper;

                servoV4BLUpper.setPosition(targetPosUpper);
                servoV4BLLower.setPosition(targetPosLower);
                opMode.sleep(delay);
            }
        }

        this.currentV4BLPos = targetPosition;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // stopAllMotors
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void stopDriveMotors() {
        for (int i = 0; i < 3; ++i) {
            this.motorFL.setPower(0);
            this.motorFR.setPower(0);
            this.motorBL.setPower(0);
            this.motorBR.setPower(0);
        }
    }

    public void stopAllMotors() {
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorIntakeRight.setPower(0);
        motorIntakeLeft.setPower(0);
        motorLiftRight.setPower(0);
//        motorLiftLeft.setPower(0);
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // continueAutonomus
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    boolean continueAutonomus() {
        if (!this.runningAutonomous)
            return true;
        else
            return ( opMode.opModeIsActive() ) && ( autonomusTimer.milliseconds() < AUTONOMOUS_DURATION_MSEC );
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveForwardTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveForwardTillRotation( double rotation, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors )
    {
        boolean useGyroToAlign  = (this.gyro != null && targetHeading >= 0) ? true : false;
        int initPosition        = motorFL.getCurrentPosition();
        int ticksToGo           = 0;
        double power            = 0.0;

        while (continueAutonomus()) {

            ticksToGo = (int)(TICK_PER_WHEEL_ROTATION * rotation) - (motorFL.getCurrentPosition() - initPosition);

            if (ticksToGo <= 0)
                break;

            power = this.getDrivePower(power, ticksToGo, targetPower, rampDown);

            double powerRight = power;
            double powerLeft  = power;

            // Adjusting motor power based on gyro position
            // to force the robot to move straight
            if (useGyroToAlign) {
                double currentPosition = this.getCurrentPositionInDegrees();
                double headingChange   = currentPosition - targetHeading;

                if (headingChange > 180 && targetHeading == 0) {
                    headingChange -= 360;
                }
                powerRight +=  2 * (headingChange / 100);
                powerLeft  -=  2 * (headingChange / 100);

            }

            motorFR.setPower( powerRight );
            motorFL.setPower( powerLeft );
            motorBR.setPower( powerRight );
            motorBL.setPower( powerLeft );

        }

        if (stopMotors) {
            this.stopDriveMotors();
        }
    }

    private void setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motorFR.setZeroPowerBehavior(behavior);
        motorFL.setZeroPowerBehavior(behavior);
        motorBR.setZeroPowerBehavior(behavior);
        motorBL.setZeroPowerBehavior(behavior);
    }



    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveLeftDiagionalTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveLeftDiagionalTillRotation( double rotation, double targetPower, boolean rampDown, boolean stopMotors )
    {
        int initPosition        = motorFL.getCurrentPosition();
        int ticksToGo           = 0;
        double power            = 0.30;

        setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while (continueAutonomus()) {

            ticksToGo = (int)(TICK_PER_WHEEL_ROTATION * rotation) - (motorFL.getCurrentPosition() - initPosition);

            if (ticksToGo <= 0)
                break;

            power = this.getDrivePower(power, ticksToGo, targetPower, rampDown);

            motorFR.setPower( power );
            motorFL.setPower( 0 );
            motorBR.setPower( 0 );
            motorBL.setPower( power );
        }

        setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (stopMotors) {
            this.stopDriveMotors();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveRightDiagionalTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveRightDiagionalTillRotation( double rotation, double targetPower, boolean rampDown, boolean stopMotors )
    {
        int initPosition        = motorFR.getCurrentPosition();
        int ticksToGo           = 0;
        double power            = 0.0;

        setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while (continueAutonomus()) {

            ticksToGo = (int)(TICK_PER_WHEEL_ROTATION * rotation) - (motorFR.getCurrentPosition() - initPosition);

            if (ticksToGo <= 0)
                break;

            power = this.getDrivePower(power, ticksToGo, targetPower, rampDown);

            motorFR.setPower( 0 );
            motorFL.setPower( power );
            motorBR.setPower( power );
            motorBL.setPower( 0 );
        }

        setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (stopMotors) {
            this.stopDriveMotors();
        }
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveForwardTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveForwardTillRotationOrCOlor( double rotation, Color color, double targetPower, boolean rampDown, boolean stopMotors )
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

            power = this.getDrivePower(power, ticksToGo, targetPower, rampDown);

            double powerRight = power;
            double powerLeft  = power;

            motorFR.setPower( powerRight );
            motorFL.setPower( powerLeft );
            motorBR.setPower( powerRight );
            motorBL.setPower( powerLeft );
        }

        if (stopMotors) {
            this.stopDriveMotors();
        }
    }

    private int getCurrentDrivePosition() {
        return motorFL.getCurrentPosition();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveBackwardTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveBackwardTillRotation( double rotation, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors )
    {
        boolean useGyroToAlign  = (this.gyro != null && targetHeading >= 0) ? true : false;
        int initPosition        = getCurrentDrivePosition();
        int ticksToGo           = 0;
        double power            = 0.0;

        while (continueAutonomus()) {
            ticksToGo = (int)(TICK_PER_WHEEL_ROTATION * rotation) - (initPosition - getCurrentDrivePosition());

            if (ticksToGo <= 0)
                break;

            power = this.getDrivePower(power, ticksToGo, targetPower, rampDown);

            double powerRight = power;
            double powerLeft  = power;

            // Adjusting motor power based on gyro position
            // to force the robot to move straight
            if (useGyroToAlign) {
                double currentPosition = this.getCurrentPositionInDegrees();
                double headingChange   = currentPosition - targetHeading;

                if (headingChange > 180 && targetHeading == 0) {
                    headingChange -= 360;
                }
                powerRight -=  2 * (headingChange / 100);
                powerLeft  +=  2 * (headingChange / 100);

            }

            motorFR.setPower( powerRight * -1);
            motorFL.setPower( powerLeft * -1);
            motorBR.setPower( powerRight * -1);
            motorBL.setPower( powerLeft * -1);
        }

        if (stopMotors) {
            stopDriveMotors();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveBackwardTillTime
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveBackwardTillTime( long timeInMilliSeconds, double targetPower, boolean stopMotors ) {

        boolean useGyroToAlign = false;

        double power = targetPower;

        motorFR.setPower( power * -1);
        motorFL.setPower( power * -1);
        motorBR.setPower( power * -1);
        motorBL.setPower( power * -1);

        try { Thread.sleep(timeInMilliSeconds); } catch (Exception e) {}

        if (stopMotors) {
            stopDriveMotors();
        }
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveBackwardTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveBackwardTillRotationOrColor( double rotation, Color color, double targetPower, boolean rampDown, boolean stopMotors ) {

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

            power = this.getDrivePower(power, ticksToGo, targetPower, rampDown);

            motorFR.setPower( power * -1);
            motorFL.setPower( power * -1);
            motorBR.setPower( power * -1);
            motorBL.setPower( power * -1);
        }

        if (stopMotors) {
            stopDriveMotors();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveBackwardTillRange
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveBackwardTillRange( double range, double targetPower, int targetHeading, boolean stopMotors )
    {
        boolean useGyroToAlign  = (this.gyro != null && targetHeading >= 0) ? true : false;
        double power            = 0.10;

        while (continueAutonomus()) {

            double rangeBR = this.rangeSensorBR.rawUltrasonic();
            double rangeBL = this.rangeSensorBL.rawUltrasonic();

            double minRange = Math.min(rangeBR, rangeBL);

            if (minRange <= range )
                break;

            power = getDriveRangePower(power, minRange, targetPower);

            double powerRight = power;
            double powerLeft  = power;

            // Adjusting motor power based on gyro position
            // to force the robot to move straight
            if (useGyroToAlign) {
                double currentPosition = this.getCurrentPositionInDegrees();
                double headingChange   = currentPosition - targetHeading;

                if (headingChange > 180 && targetHeading == 0) {
                    headingChange -= 360;
                }
                powerRight -=  2 * (headingChange / 100);
                powerLeft  +=  2 * (headingChange / 100);
            }

            motorFR.setPower( -1 * powerRight );
            motorFL.setPower( -1 * powerLeft );
            motorBR.setPower( -1 * powerRight );
            motorBL.setPower( -1 * powerLeft );
        }

        if (stopMotors) {
            this.stopDriveMotors();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveLeftTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveLeftTillRotation( double rotation, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors )
    {
        boolean useGyroToAlign  = (this.gyro != null && targetHeading >= 0) ? true : false;
        int initPosition        = motorFR.getCurrentPosition();
        int ticksToGo           = 0;
        double power            = 0.0;

        while (continueAutonomus()) {

            ticksToGo = (int)(TICK_PER_WHEEL_ROTATION * rotation) - (motorFR.getCurrentPosition() - initPosition);

            if (ticksToGo <= 0)
                break;

            power = this.getDrivePower(power, ticksToGo, targetPower, rampDown);

            double powerFront = power;
            double powerBack  = power;

            // Adjusting motor power based on gyro position
            // to force the robot to move straight
            if (useGyroToAlign) {
                double currentPosition = this.getCurrentPositionInDegrees();
                double headingChange   = currentPosition - targetHeading;

                if (headingChange > 180 && targetHeading == 0) {
                    headingChange -= 360;
                }
                powerFront +=  2 * (headingChange / 100);
                powerBack  -=  2 * (headingChange / 100);
            }

            motorFR.setPower( powerFront );
            motorFL.setPower( powerFront * -1 );
            motorBR.setPower( powerBack * -1 );
            motorBL.setPower( powerBack );

        }

        if (stopMotors) {
            stopDriveMotors();
        }
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveRightTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveRightTillRotation( double rotation, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors )
    {
        boolean useGyroToAlign  = (this.gyro != null && targetHeading >= 0) ? true : false;
        int initPosition        = motorFR.getCurrentPosition();
        int ticksToGo           = 0;

        double power = 0.0;

        while (continueAutonomus()) {

            ticksToGo = (int)(TICK_PER_WHEEL_ROTATION * rotation) - (initPosition - motorFR.getCurrentPosition());

            if (ticksToGo <= 0)
                break;

            power = this.getDrivePower( power, ticksToGo, targetPower, rampDown );

            double powerFront = power;
            double powerBack  = power;

            // Adjusting motor power based on gyro position
            // to force the robot to move straight
            if (useGyroToAlign) {
                double currentPosition = this.getCurrentPositionInDegrees();
                double headingChange   = currentPosition - targetHeading;

                if (headingChange > 180 && targetHeading == 0) {
                    headingChange -= 360;
                }
                powerFront -=  2 * (headingChange / 100);
                powerBack  +=  2 * (headingChange / 100);
            }

            motorFR.setPower( powerFront * -1 );
            motorFL.setPower( powerFront );
            motorBR.setPower( powerBack );
            motorBL.setPower( powerBack * -1 );
        }

        if (stopMotors) {
            stopDriveMotors();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // turnRightTillDegrees
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void turnRightTillDegrees( int targetDegrees, boolean rampDown, boolean stopMotors ) {
        turnRightTillDegrees(targetDegrees, TURN_POWER, rampDown,stopMotors);
    }

    public void turnRightTillDegrees( int targetDegrees, double targetPower, boolean rampDown, boolean stopMotors )
    {
        double power = 0.30;
        double currentHeading = 0;
        double degressToGo = 0;

        while (continueAutonomus()) {

            currentHeading = getCurrentPositionInDegrees();
            if (currentHeading > 180 && targetDegrees < 180)
                currentHeading -= 360;

            degressToGo = targetDegrees - currentHeading;
            if (degressToGo <= TURN_TOLERANCE)
                break;

            power = this.getTurnPower(power, degressToGo, targetPower, rampDown);

            motorFR.setPower( -1 * power );
            motorFL.setPower(      power );
            motorBR.setPower( -1 * power );
            motorBL.setPower(      power );
        }

        if (stopMotors) {
            this.stopDriveMotors();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // turnLeftTillDegrees
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void turnLeftTillDegrees( int targetDegrees, boolean rampDown, boolean stopMotors ) {
        turnLeftTillDegrees(targetDegrees, TURN_POWER, rampDown,stopMotors);
    }

    public void turnLeftTillDegrees( int targetDegrees, double targetPower, boolean rampDown, boolean stopMotors )
    {
        double power = 0.30;
        double currentHeading = 0;
        double degressToGo = 0;

        while (continueAutonomus()) {

            currentHeading = getCurrentPositionInDegrees();

            degressToGo = currentHeading - targetDegrees;

            if (degressToGo < -120)
                degressToGo += 360;

            if (degressToGo <= TURN_TOLERANCE)
                break;

            power = this.getTurnPower(power, degressToGo, targetPower, rampDown);

            motorFR.setPower(      power );
            motorFL.setPower( -1 * power );
            motorBR.setPower(      power );
            motorBL.setPower( -1 * power );
        }

        if (stopMotors) {
            this.stopDriveMotors();
        }
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // turnLeftTillDegrees
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void curveLeftTillDegrees( int targetDegrees, boolean rampDown, boolean stopMotors )
    {
        double targetPower = 0.20;
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

            power = this.getTurnPower(power, degressToGo, targetPower, rampDown);

            motorFR.setPower( power * 1.5 );
            motorFL.setPower( power * 0.5);
            motorBR.setPower( power * 1.5 );
            motorBL.setPower( power * 0.5 );
        }


        if (stopMotors) {
            this.stopDriveMotors();
        }
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // getDrivePower
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private double getDrivePower( double curPower, double ticksToGo, double targetPower, boolean rampDown ) {
        double power = curPower;

        // Ramp down power
        if ( rampDown && (ticksToGo <= RAMP_DOWN_DRIVE_TICKS) ) {
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
    private double getTurnPower( double curPower, double degreesToGo, double targetPower, boolean rampDown ) {
        double power = curPower;
        double RAMP_DOWN_DISTANCE = 30;

        // Ramp down power
        if ( rampDown && (degreesToGo <= RAMP_DOWN_DISTANCE) ) {
            power = ( degreesToGo / RAMP_DOWN_DISTANCE)  * curPower;

            if (power < MIN_TURN_POWER)
                power = MIN_TURN_POWER;
        }
        else if (targetPower - power > 0.001) {
            power += this.RAMP_UP_RATE_TURN;
        }
        return power;
    }



    public boolean isColorBlue() {
        if (this.colorDistance.getDistance(DistanceUnit.CM) < 6.0) {
            if (colorSensor.blue() > colorSensor.green() &&
                    colorSensor.blue() > colorSensor.red())
                return true;
        }
        return false;
    }


    public boolean isColorRed() {
        if (this.colorDistance.getDistance(DistanceUnit.CM) < 6.0) {
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
    // turnIntakeOn
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void turnIntakeOn( IntakeDirection direction ) {

        if (direction == IntakeDirection.INTAKE_DIRECTION_IN) {
            motorIntakeRight.setPower(-1 * INTAKE_POWER);
            motorIntakeLeft.setPower(-1 * INTAKE_POWER);
        } else {
            motorIntakeRight.setPower(INTAKE_POWER);
            motorIntakeLeft.setPower(INTAKE_POWER);
        }
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
    public void lockLiftMotors() {

//        this.motorLift.setPower( 0.25 );
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // unlockLiftMotor
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void unlockLiftMotors() {

//        this.motorLift.setPower( 0 );
    }

    public SkystonePosition scanSkystone( SkystonePosition previousPosition ) {
        double servoPos1 = (this.allianceMode == SkystoneRobot.AllianceMode.ALLIANCE_BLUE) ? 0.38 : 0.46;
        double servoPos2 = (this.allianceMode == SkystoneRobot.AllianceMode.ALLIANCE_BLUE) ? 0.44 : 0.40;

        setServoPosition(this.servoCamera, servoPos1, 0);

        opMode.sleep(1000);

        List<Recognition> updatedRecognitions = getTensorFlowRecognitions();

        if (updatedRecognitions != null) {
            for (Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel().equals("Skystone"))
                    return SkystonePosition.SKYSTONE_POSITION_1;
            }
        }

        if (!opMode.opModeIsActive() && !opMode.isStopRequested()) {
            setServoPosition(servoCamera, servoPos2, 0);

            opMode.sleep(1000);
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
                servoIntakeRight.setPosition(0.10);
                servoIntakeLeft.setPosition(1.0);
                break;
            case LATCH_POSITION_1:
                servoIntakeRight.setPosition(0.05);
                servoIntakeLeft.setPosition(0.1);
                break;
        }
    }

    void setRunningAutonomous(boolean autonomous) {
        this.runningAutonomous = autonomous;
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveForwardTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveForwardWithOuttakeTillRotation( double rotation, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors )
    {
        boolean useGyroToAlign  = (this.gyro != null && targetHeading >= 0) ? true : false;
        int initPosition        = motorFR.getCurrentPosition();
        int ticksToGo           = 0;
        double power            = 0.0;

        while (continueAutonomus()) {

            ticksToGo = (int)(TICK_PER_WHEEL_ROTATION * rotation) - (motorFR.getCurrentPosition() - initPosition);

            if (ticksToGo <= 0)
                break;

            power = this.getDrivePower(power, ticksToGo, targetPower, rampDown);

            double powerRight = power;
            double powerLeft  = power;

            // Adjusting motor power based on gyro position
            // to force the robot to move straight
            if (useGyroToAlign) {
                double currentPosition = this.getCurrentPositionInDegrees();
                double headingChange   = currentPosition - targetHeading;

                if (headingChange > 180 && targetHeading == 0) {
                    headingChange -= 360;
                }
                powerRight +=  2 * (headingChange / 100);
                powerLeft  -=  2 * (headingChange / 100);

            }

            motorFR.setPower( powerRight );
            motorFL.setPower( powerLeft );
            motorBR.setPower( powerRight );
            motorBL.setPower( powerLeft );

            double intakePower = 0.0;
            if (ticksToGo <= TICK_PER_WHEEL_ROTATION ) {
                intakePower = 0.10;
            } else if (ticksToGo <= TICK_PER_WHEEL_ROTATION * 2.0) {
                intakePower = 0.20;
            } else if (ticksToGo <= TICK_PER_WHEEL_ROTATION * 4.0) {
                intakePower = 0.25;
            }

            motorIntakeRight.setPower(intakePower);
            motorIntakeLeft.setPower(intakePower);
        }

        if (stopMotors) {
            this.stopDriveMotors();

            motorIntakeRight.setPower(0.25);
            motorIntakeLeft.setPower(0.25);
        }
    }

    public void lowerGrabber() {
        servoLiftGrabber.setPosition(0.10);
    }

    public void raiseGrabber() {
        servoLiftGrabber.setPosition(0.45);
    }


    public void lowerFoundationServos() {
        servoFoundationLeft.setPosition(1.00);
        servoFoundationRight.setPosition(0.00);
    }

    public void raiseFoundationServos() {
        servoFoundationLeft.setPosition(0.35);
        servoFoundationRight.setPosition(0.65);
    }

    public void raiseLift() {
        this.motorLiftRight.setPower(0.50);
//        this.motorLiftLeft.setPower(0.50);
    }

    public boolean stoneDetected() {
        return (distanceSensor.getDistance(DistanceUnit.CM) < 3.0);
    }

    public void grabStone() {
        setV4BLState(V4BLState.V4BL_STATE_STONE, 20);
        lowerGrabber();
    }

    public void dropStone() {
        setV4BLState(V4BLState.V4BL_STATE_TOP, 20);
        setV4BLState(V4BLState.V4BL_STATE_FOUNDATION,30);

        try {
            Thread.sleep(100);
            raiseGrabber();
            Thread.sleep(100);

            setV4BLState(V4BLState.V4BL_STATE_TOP,10);
            setV4BLState(V4BLState.V4BL_STATE_INTAKE,20);
        }
        catch (Exception e) {

        }
    }

    public V4BLState getV4BLState() {
        return this.currentV4BLState;
    }

    public double getV4BLServoPosition() {
        return this.currentV4BLPos;
//        return this.currentV4BLState.servoPos;
    }

    public boolean isV4BLState( V4BLState state ) {
        return ((Math.abs(this.currentV4BLPos - state.servoPos)) <= 0.05);
    }


}