
package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="CleanDepot", group="Pushbot")
//@Disabled
public class CleanDepot extends LinearOpMode {

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
            GoldPos = 1;                    //left
        }else if (detector.getXPosition() > 400){
            GoldPos = 3;                     //right
        }else if (detector.getXPosition() > 200 && detector.getXPosition() < 400){
            GoldPos = 2;                     //center
        }
        telemetry.addData("Gold Position: ", GoldPos);


        robot.startauto(5.8);
        telemetry.addData("Stage: ", "Lower Boot and Lift Tube");

        robot.backward(.1);
        telemetry.addData("Stage: ", "Unhook 1");

        robot.strafeleft(.1);
        telemetry.addData("Stage: ", "Unhook 2");

        robot.forward(.1);
        telemetry.addData("Stage: ", "Unhook 3");

        if(GoldPos == 1){  //gold is to the left
            robot.turnleft(.890625);
            telemetry.addData("Stage: ", "Left Turn");

            robot.slideandsweepout(.5);
            telemetry.addData("Stage: ", "Slide Out and Sweep in");

            robot.sweepin(.5);
            telemetry.addData("Stage: ", "Continue Sweeping in");

            robot.slideandsweepin(.5);
            telemetry.addData("Stage: ", "Slide in and Sweep in");

            robot.turnright(.178125);
            telemetry.addData("Stage: ", "Turn Right");

        }else if (GoldPos == 3){//gold is to the right
            robot.turnleft(.534375);
            telemetry.addData("Stage: ", "Turn Left");

            robot.slideandsweepout(.5);
            telemetry.addData("Stage: ", "Slide Out and Sweeper Out");

            robot.sweepin(.5);
            telemetry.addData("Stage: ", "Continue Sweeping in");

            robot.slideandsweepin(.5);
            telemetry.addData("Stage: ", "Slide in and Sweep in");

            robot.turnleft(.178125);
            telemetry.addData("Stage: ", "Turn Left");

        }else if (GoldPos == 2){//gold is in the center
            robot.turnleft(.7125);
            telemetry.addData("Stage: ", "Turn Left");

            robot.slideandsweepout(.5);
            telemetry.addData("Stage: ", "Slide Out and Sweep in");

            robot.sweepin(.5);
            telemetry.addData("Stage: ", "Continue Sweeping in");

            robot.slideandsweepin(.5);
            telemetry.addData("Stage: ", "Slide in and Sweep in");
        }







        robot.sleep(100000.0);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
