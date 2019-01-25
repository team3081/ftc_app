
package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoStartDepot", group="Pushbot")
@Disabled
public class AutoStartDepot extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareOmni         robot   = new HardwareOmni();
    private ElapsedTime     runtime = new ElapsedTime();
    private GoldAlignDetector detector;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();



        waitForStart();




        robot.lift.setPower(-1);                //lift up
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 6)){
            telemetry.addData("Path", "Lift Up", runtime.seconds());
            telemetry.update();
        }

        robot.leftFront.setPower(.25);          //forward
        robot.rightFront.setPower(-.25);
        robot.leftRear.setPower(.25);
        robot.rightRear.setPower(-.25);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .25)){
            telemetry.addData("Path", "Forward", runtime.seconds());
            telemetry.update();
        }


        robot.leftFront.setPower(0);          //pause
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .25)){
            telemetry.addData("Path", "Forward", runtime.seconds());
            telemetry.update();
        }


        robot.lift.setPower(1);            //lift down
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 6)){
            telemetry.addData("Path", "Lift Down", runtime.seconds());
            telemetry.update();
        }

        robot.leftFront.setPower(-.25);          //back
        robot.rightFront.setPower(.25);
        robot.leftRear.setPower(-.25);
        robot.rightRear.setPower(.25);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .25)){
            telemetry.addData("Path", "backwards", runtime.seconds());
            telemetry.update();
        }


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

        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        telemetry.addData("IsAligned", detector.getAligned());
        telemetry.addData("X Pos", detector.getXPosition());


        robot.leftFront.setPower(0);          //wait for camera to awaken
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 4)){
            telemetry.addData("Path", "Forward", runtime.seconds());
            telemetry.update();
        }



        if(detector.getXPosition() < 200){          //Gold is in left spot
            //turn left
            robot.leftFront.setPower(-.25);
            robot.rightFront.setPower(-.25);
            robot.leftRear.setPower(-.25);
            robot.rightRear.setPower(-.25);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .25)){
                telemetry.addData("Path", "Left Gold Leg 1", runtime.seconds());
                telemetry.update();
            }

            //straight
            robot.leftFront.setPower(.25);
            robot.rightFront.setPower(-.25);
            robot.leftRear.setPower(.25);
            robot.rightRear.setPower(-.25);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 2)){
                telemetry.addData("Path", "Left Gold Leg 2", runtime.seconds());
                telemetry.update();
            }

            //turn right
            robot.leftFront.setPower(.25);
            robot.rightFront.setPower(.25);
            robot.leftRear.setPower(.25);
            robot.rightRear.setPower(.25);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .6)){
                telemetry.addData("Path", "Left Gold Leg 3", runtime.seconds());
                telemetry.update();
            }

            //straight
            robot.leftFront.setPower(.25);
            robot.rightFront.setPower(-.25);
            robot.leftRear.setPower(.25);
            robot.rightRear.setPower(-.25);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.5)){
                telemetry.addData("Path", "Left Gold Leg 4", runtime.seconds());
                telemetry.update();
            }

            //turn right
            robot.leftFront.setPower(.25);
            robot.rightFront.setPower(.25);
            robot.leftRear.setPower(.25);
            robot.rightRear.setPower(.25);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .8)){
                telemetry.addData("Path", "Left Gold Leg 5", runtime.seconds());
                telemetry.update();
            }

            //strafe left
            robot.leftFront.setPower(-.25);
            robot.rightFront.setPower(-.25);
            robot.leftRear.setPower(.25);
            robot.rightRear.setPower(.25);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1)){
                telemetry.addData("Path", "left Gold Gold Leg 6", runtime.seconds());
                telemetry.update();
            }

            //strafe right
            robot.leftFront.setPower(.25);
            robot.rightFront.setPower(.25);
            robot.leftRear.setPower(-.25);
            robot.rightRear.setPower(-.25);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .2)){
                telemetry.addData("Path", "left Gold Gold Leg 7", runtime.seconds());
                telemetry.update();
            }

            //straight
            robot.leftFront.setPower(1);
            robot.rightFront.setPower(-1);
            robot.leftRear.setPower(1);
            robot.rightRear.setPower(-1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 4)){
                telemetry.addData("Path", "Left Gold Leg 8", runtime.seconds());
                telemetry.update();
            }


        }else if (detector.getXPosition() > 400){       //Gold is in right spot
            //turn right
            robot.leftFront.setPower(.25);
            robot.rightFront.setPower(.25);
            robot.leftRear.setPower(.25);
            robot.rightRear.setPower(.25);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .25)){
                telemetry.addData("Path", "Right Gold Leg 1", runtime.seconds());
                telemetry.update();
            }

            //straight
            robot.leftFront.setPower(.25);
            robot.rightFront.setPower(-.25);
            robot.leftRear.setPower(.25);
            robot.rightRear.setPower(-.25);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 2)){
                telemetry.addData("Path", "Right Gold Leg 2", runtime.seconds());
                telemetry.update();
            }

            //turn left
            robot.leftFront.setPower(-.25);
            robot.rightFront.setPower(-.25);
            robot.leftRear.setPower(-.25);
            robot.rightRear.setPower(-.25);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .6)){
                telemetry.addData("Path", "Right Gold Leg 3", runtime.seconds());
                telemetry.update();
            }

            //straight
            robot.leftFront.setPower(.25);
            robot.rightFront.setPower(-.25);
            robot.leftRear.setPower(.25);
            robot.rightRear.setPower(-.25);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.5)){
                telemetry.addData("Path", "Right Gold Leg 4", runtime.seconds());
                telemetry.update();
            }

            //turn right
            robot.leftFront.setPower(.25);
            robot.rightFront.setPower(.25);
            robot.leftRear.setPower(.25);
            robot.rightRear.setPower(.25);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.35)){
                telemetry.addData("Path", "Right Gold Leg 5", runtime.seconds());
                telemetry.update();
            }

            //strafe left
            robot.leftFront.setPower(-.25);
            robot.rightFront.setPower(-.25);
            robot.leftRear.setPower(.25);
            robot.rightRear.setPower(.25);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .75)){
                telemetry.addData("Path", "Right Gold Gold Leg 6", runtime.seconds());
                telemetry.update();
            }

            //strafe right
            robot.leftFront.setPower(.25);
            robot.rightFront.setPower(.25);
            robot.leftRear.setPower(-.25);
            robot.rightRear.setPower(-.25);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .5)){
                telemetry.addData("Path", "Right Gold Gold Leg 7", runtime.seconds());
                telemetry.update();
            }

            //straight
            robot.leftFront.setPower(1);
            robot.rightFront.setPower(-1);
            robot.leftRear.setPower(1);
            robot.rightRear.setPower(-1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3.5)){
                telemetry.addData("Path", "Right Gold Leg 8", runtime.seconds());
                telemetry.update();
            }
   

        }else if (detector.getXPosition() > 200 && detector.getXPosition() < 400){      //Gold is in Center spot
            //straight
            robot.leftFront.setPower(.25);
            robot.rightFront.setPower(-.25);
            robot.leftRear.setPower(.25);
            robot.rightRear.setPower(-.25);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3)){
                telemetry.addData("Path", "Center Gold Leg 1", runtime.seconds());
                telemetry.update();
            }

            //turn right
            robot.leftFront.setPower(.25);
            robot.rightFront.setPower(.25);
            robot.leftRear.setPower(.25);
            robot.rightRear.setPower(.25);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.1)){
                telemetry.addData("Path", "Center Gold Gold Leg 2", runtime.seconds());
                telemetry.update();
            }

            //strafe left
            robot.leftFront.setPower(-.25);
            robot.rightFront.setPower(-.25);
            robot.leftRear.setPower(.25);
            robot.rightRear.setPower(.25);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .75)){
                telemetry.addData("Path", "Center Gold Gold Leg 3", runtime.seconds());
                telemetry.update();
            }

            //strafe right
            robot.leftFront.setPower(.25);
            robot.rightFront.setPower(.25);
            robot.leftRear.setPower(-.25);
            robot.rightRear.setPower(-.25);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .25)){
                telemetry.addData("Path", "Center Gold Gold Leg 4", runtime.seconds());
                telemetry.update();
            }

            //straight
            robot.leftFront.setPower(1);
            robot.rightFront.setPower(-1);
            robot.leftRear.setPower(1);
            robot.rightRear.setPower(-1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3.5)){
                telemetry.addData("Path", "Center Gold Leg 5", runtime.seconds());
                telemetry.update();
            }

        }
        robot.rightFront.setPower(0);
        robot.rightRear.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.lift.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
