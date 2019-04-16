
// Simple autonomous program that drives bot forward for a given amount
// of time then does right-hand 90-deg trun; or does turn when gamepad A,B
// button is pushed.
//
// Also uses IMU to drive in a straight line when not avoiding an obstacle.

package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Drive Encode Imu", group = "Exercises")
@Disabled

public class EncoderTESTS extends LinearOpMode {

    static final double DRIVEPOWER   =  1;
    static final double TURNPOWER    =  1;
    static final int    TURNCORR     =   +1; //degrees corr, pos corrects for oversteer, neg for under
    static final double ENCODE_PPR   = 1020; //NeverRest 40 280 pulse per revolution (ppr)
    static final double WHEEL_CIRCUM = 31.91; // 10cm diameter ~ 31.42 cm/rev

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;
    HardwareOmni robot = new HardwareOmni();

    //DigitalChannel          touch;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;
    boolean aButton, bButton, touched;

    private ElapsedTime runtime = new ElapsedTime();
    private GoldAlignDetector detector;
    double GoldPos;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

//        leftMotor = hardwareMap.dcMotor.get("motorZero");
//
//        rightMotor = hardwareMap.dcMotor.get("motorOne");
//        rightMotor.setDirection(DcMotor.Direction.REVERSE);
//
//        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.init(hardwareMap);

        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        telemetry.addData("Status", "Ready to run");
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();
        detector.alignSize = 100;
        detector.alignPosOffset = 0;
        detector.downscale = 0.4;
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        detector.maxAreaScorer.weight = 0.005;
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;
        detector.enable();
        robot.init(hardwareMap);
        telemetry.addData("IsAligned", detector.getAligned());
        telemetry.addData("X Pos", detector.getXPosition());

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        runtime.reset();

        // drive until end of period.

        while (opModeIsActive()) {

            if(detector.getXPosition() < 200){
                GoldPos = 1;      //left
            }else if (detector.getXPosition() > 400){
                GoldPos = 3;      //right
            }else if (detector.getXPosition() > 200 && detector.getXPosition() < 400){
                GoldPos = 2;      //center
            }
            telemetry.addData("Gold Position: ", GoldPos);



            // Use gyro to drive in a straight line for dist
            straight(5.0 * WHEEL_CIRCUM, DRIVEPOWER);
            sleep(250);
            rotate(90,  TURNPOWER);
            straight(5.0 * WHEEL_CIRCUM, DRIVEPOWER);
            sleep(250);
            rotate(90,  TURNPOWER);
            straight(5.0 * WHEEL_CIRCUM, DRIVEPOWER);
            sleep(250);
            rotate(+90, TURNPOWER);
            straight(5.0 * WHEEL_CIRCUM, DRIVEPOWER);
            sleep(250);
            rotate(+90, TURNPOWER);

        }

        // turn the motors off.
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    /**
     * Calculate number of pulse ticks for a given distance
     */
    private int calcPPR(double dist) {
        return (int) ((dist / WHEEL_CIRCUM) * ENCODE_PPR);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .5;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Drive straight for dist centimeters.
     *
     * @param dist Centimeters to drive
     */

    private void straight(double dist, double power) {
     //   int currentLeftEnc  = leftFront.getCurrentPosition();
        int currentBackLeftEnc = leftRear.getCurrentPosition();

       // int targetLeftEnc  = currentLeftEnc + calcPPR(dist);
        int targetBackLeftEnc = currentBackLeftEnc + calcPPR(dist);

//        while (currentLeftEnc < targetLeftEnc && currentRightEnc < targetRightEnc && opModeIsActive()) {
        while (currentBackLeftEnc < targetBackLeftEnc && opModeIsActive()) {

            correction = checkDirection();
            leftFront.setPower(DRIVEPOWER + correction);
            rightFront.setPower(-DRIVEPOWER - correction);
            leftRear.setPower(DRIVEPOWER - correction);
            rightRear.setPower(-DRIVEPOWER - correction);

           // currentLeftEnc  = leftMotor.getCurrentPosition();  // update enc reading
            currentBackLeftEnc = leftRear.getCurrentPosition();

            telemetry.addData("imu heading firstAngle", lastAngles.firstAngle);
            telemetry.addData("imu heading secondAngle", lastAngles.secondAngle);
            telemetry.addData("imu heading thirdAngle", lastAngles.thirdAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            //telemetry.addData("4 left enc", currentLeftEnc);
            telemetry.addData("5 right enc", currentBackLeftEnc);
            telemetry.update();

            sleep(100);

        }

        sleep(100);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  LeftFrontPower, RightFrontPower, LeftRearPower, RightRearPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            LeftFrontPower = power;
            RightFrontPower = power;
            LeftRearPower = power;
            RightRearPower = power;

        }
        else if (degrees > 0)
        {   // turn left.
            LeftFrontPower = -power;
            RightFrontPower = -power;
            LeftRearPower = -power;
            RightRearPower = -power;
        }
        else return;

        // set power to rotate.
        leftFront.setPower(LeftFrontPower);
        rightFront.setPower(RightFrontPower);
        leftRear.setPower(LeftRearPower);
        rightRear.setPower(RightRearPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                telemetry.addData("imu heading firstAngle", lastAngles.firstAngle);
                telemetry.addData("imu heading secondAngle", lastAngles.secondAngle);
                telemetry.addData("imu heading thirdAngle", lastAngles.thirdAngle);
                telemetry.update();
            }

            while (opModeIsActive() && getAngle() > degrees + TURNCORR) {
                telemetry.addData("imu heading firstAngle", lastAngles.firstAngle);
                telemetry.addData("imu heading secondAngle", lastAngles.secondAngle);
                telemetry.addData("imu heading thirdAngle", lastAngles.thirdAngle);
                telemetry.update();
            }

        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees - TURNCORR) {
                telemetry.addData("imu heading firstAngle", lastAngles.firstAngle);
                telemetry.addData("imu heading secondAngle", lastAngles.secondAngle);
                telemetry.addData("imu heading thirdAngle", lastAngles.thirdAngle);
                telemetry.update();
            }

        // turn the motors off.
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

}
