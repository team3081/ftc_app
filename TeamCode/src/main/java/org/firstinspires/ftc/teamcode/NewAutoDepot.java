
package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="NewAutoDepot", group="Pushbot")
//@Disabled
public class NewAutoDepot extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareOmni robot = new HardwareOmni();
    private ElapsedTime runtime = new ElapsedTime();
    private GoldAlignDetector detector;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

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


        robot.lift.setPower(-1);            //lift up
        robot.tube.setPosition(.4);         //tube up

        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 9.5)){
            telemetry.addData("Path", "Lift Up", runtime.seconds());
            telemetry.update();
        }

        robot.lift.setPower(0);

        //straight
        robot.leftFront.setPower(1);
        robot.rightFront.setPower(-1);
        robot.leftRear.setPower(1);
        robot.rightRear.setPower(-1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .1)){
            telemetry.addData("Path", "Unhook Leg 1", runtime.seconds());
            telemetry.update();
        }
        //strafe left
        robot.leftFront.setPower(-1);
        robot.rightFront.setPower(-1);
        robot.leftRear.setPower(1);
        robot.rightRear.setPower(1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .1)){
            telemetry.addData("Path", "Unhook Leg 2", runtime.seconds());
            telemetry.update();
        }
        //backwards
        robot.leftFront.setPower(-1);
        robot.rightFront.setPower(1);
        robot.leftRear.setPower(-1);
        robot.rightRear.setPower(1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .1)){
            telemetry.addData("Path", "Unhook Leg 3", runtime.seconds());
            telemetry.update();
        }



        if(detector.getXPosition() < 200){  //gold is to the left
            //turn left
            robot.leftFront.setPower(-1);
            robot.rightFront.setPower(-1);
            robot.leftRear.setPower(-1);
            robot.rightRear.setPower(-1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .890625)){
                telemetry.addData("Path", "Left Gold Leg 1", runtime.seconds());
                telemetry.update();
            }
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightRear.setPower(0);

            robot.slide.setPower(-1);
            robot.sweeper.setPower(1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .75)){
                telemetry.addData("Path", "Left Gold Leg 2", runtime.seconds());
                telemetry.update();
            }
            robot.sweeper.setPower(1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .5)){
                telemetry.addData("Path", "Left Gold Leg 3", runtime.seconds());
                telemetry.update();
            }
            robot.slide.setPower(1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .75)){
                telemetry.addData("Path", "Left Gold Leg 4", runtime.seconds());
                telemetry.update();
            }
            robot.leftFront.setPower(1);
            robot.rightFront.setPower(1);
            robot.leftRear.setPower(1);
            robot.rightRear.setPower(1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .178125)){
                telemetry.addData("Path", "Left Gold Leg 5", runtime.seconds());
                telemetry.update();
            }
        }else if (detector.getXPosition() > 400){//gold is to the right
            //turn left
            robot.leftFront.setPower(-1);
            robot.rightFront.setPower(-1);
            robot.leftRear.setPower(-1);
            robot.rightRear.setPower(-1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .534375)){
                telemetry.addData("Path", "Right Gold Leg 1", runtime.seconds());
                telemetry.update();
            }
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightRear.setPower(0);

            robot.slide.setPower(-1);
            robot.sweeper.setPower(1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .75)){
                telemetry.addData("Path", "Right Gold Leg 2", runtime.seconds());
                telemetry.update();
            }
            robot.sweeper.setPower(1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .5)){
                telemetry.addData("Path", "Right Gold Leg 3", runtime.seconds());
                telemetry.update();
            }
            robot.slide.setPower(1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .75)){
                telemetry.addData("Path", "Right Gold Leg 4", runtime.seconds());
                telemetry.update();
            }
            robot.leftFront.setPower(-1);
            robot.rightFront.setPower(-1);
            robot.leftRear.setPower(-1);
            robot.rightRear.setPower(-1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .178125)){
                telemetry.addData("Path", "Right Gold Leg 5", runtime.seconds());
                telemetry.update();
            }
        }else if (detector.getXPosition() > 200 && detector.getXPosition() < 400){//gold is in the center
            //straight
            robot.leftFront.setPower(-1);
            robot.rightFront.setPower(-1);
            robot.leftRear.setPower(-1);
            robot.rightRear.setPower(-1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .7125)){
                telemetry.addData("Path", "Center Gold Leg 1", runtime.seconds());
                telemetry.update();
            }
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightRear.setPower(0);

            robot.slide.setPower(-1);
            robot.sweeper.setPower(1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .75)){
                telemetry.addData("Path", "Center Gold Leg 2", runtime.seconds());
                telemetry.update();
            }
            robot.sweeper.setPower(1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .5)){
                telemetry.addData("Path", "Center Gold Leg 3", runtime.seconds());
                telemetry.update();
            }
            robot.slide.setPower(1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .75)){
                telemetry.addData("Path", "Center Gold Leg 4", runtime.seconds());
                telemetry.update();
            }

        }






        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
        robot.sweeper.setPower(0);
        robot.slide.setPower(0);
        robot.popper.setPower(0);
        robot.lift.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
