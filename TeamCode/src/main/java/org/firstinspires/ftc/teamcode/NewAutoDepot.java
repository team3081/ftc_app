
package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="NewAutoDepot", group="Pushbot")
@Disabled
public class NewAutoDepot extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareOmni robot = new HardwareOmni();
    private ElapsedTime runtime = new ElapsedTime();
    private GoldAlignDetector detector;
    double GoldPos;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

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


        waitForStart();


        if(detector.getXPosition() < 200){
            GoldPos = 1;
        }else if (detector.getXPosition() > 400){
            GoldPos = 3;
        }else if (detector.getXPosition() > 200 && detector.getXPosition() < 400){
            GoldPos = 2;
        }

        //lift up and tube up
        robot.lift.setPower(-1);
        robot.tube.setPosition(.45);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 9.5)){
            telemetry.addData("Path", "Lift Up", runtime.seconds());
            telemetry.addData("Gold Position", GoldPos);
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



        if(GoldPos == 1){  //gold is to the left
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

            //slide out and sweeper on
            robot.slide.setPower(-1);
            robot.sweeper.setPower(1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .5)){
                telemetry.addData("Path", "Left Gold Leg 2", runtime.seconds());
                telemetry.update();
            }
            //sweeper continues
            robot.sweeper.setPower(1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .5)){
                telemetry.addData("Path", "Left Gold Leg 3", runtime.seconds());
                telemetry.update();
            }
            //slide in
            robot.slide.setPower(1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .5)){
                telemetry.addData("Path", "Left Gold Leg 4", runtime.seconds());
                telemetry.update();
            }
            robot.slide.setPower(0);
            robot.sweeper.setPower(0);

            //turn right
            robot.leftFront.setPower(1);
            robot.rightFront.setPower(1);
            robot.leftRear.setPower(1);
            robot.rightRear.setPower(1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .178125)){
                telemetry.addData("Path", "Left Gold Leg 5", runtime.seconds());
                telemetry.update();
            }
        }else if (GoldPos == 3){//gold is to the right
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

            //slide out and sweeper on
            robot.slide.setPower(-1);
            robot.sweeper.setPower(1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .5)){
                telemetry.addData("Path", "Right Gold Leg 2", runtime.seconds());
                telemetry.update();
            }
            //sweeper continues
            robot.sweeper.setPower(1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .5)){
                telemetry.addData("Path", "Right Gold Leg 3", runtime.seconds());
                telemetry.update();
            }
            //slide back
            robot.slide.setPower(1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .5)){
                telemetry.addData("Path", "Right Gold Leg 4", runtime.seconds());
                telemetry.update();
            }
            robot.slide.setPower(0);
            robot.sweeper.setPower(0);

            //turn left
            robot.leftFront.setPower(-1);
            robot.rightFront.setPower(-1);
            robot.leftRear.setPower(-1);
            robot.rightRear.setPower(-1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .178125)){
                telemetry.addData("Path", "Right Gold Leg 5", runtime.seconds());
                telemetry.update();
            }
        }else if (GoldPos == 2){//gold is in the center
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

            //slide out and sweeper on
            robot.slide.setPower(-1);
            robot.sweeper.setPower(-1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .5)){
                telemetry.addData("Path", "Center Gold Leg 2", runtime.seconds());
                telemetry.update();
            }
            //sweeper continues
            robot.sweeper.setPower(-1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .5)){
                telemetry.addData("Path", "Center Gold Leg 3", runtime.seconds());
                telemetry.update();
            }
            //slide in
            robot.slide.setPower(1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .5)){
                telemetry.addData("Path", "Center Gold Leg 4", runtime.seconds());
                telemetry.update();
            }
            robot.slide.setPower(0);
            robot.sweeper.setPower(0);

        }


        

        //straight
        robot.leftFront.setPower(1);
        robot.rightFront.setPower(-1);
        robot.leftRear.setPower(1);
        robot.rightRear.setPower(-1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .9)){
            telemetry.addData("Path", "Unhook Leg 1", runtime.seconds());
            telemetry.update();
        }

        //turn right
        robot.leftFront.setPower(1);
        robot.rightFront.setPower(1);
        robot.leftRear.setPower(1);
        robot.rightRear.setPower(1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .35625)){
            telemetry.addData("Path", "Left Gold Leg 5", runtime.seconds());
            telemetry.update();
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
