
package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="FINALDEPOT", group="Pushbot")
//@Disabled
public class FINALDEPOT extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareOmni robot = new HardwareOmni();
    private ElapsedTime runtime = new ElapsedTime();
    private GoldAlignDetector detector;
    double GoldPos;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
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


        waitForStart();


        if(detector.getXPosition() < 200){
            GoldPos = 1;                    //left
        }else if (detector.getXPosition() > 400){
            GoldPos = 3;                     //right
        }else if (detector.getXPosition() > 200 && detector.getXPosition() < 400){
            GoldPos = 2;                     //center
        }
        telemetry.addData("Gold Position: ", GoldPos);


        robot.startauto(5.75);
        telemetry.addData("Stage: ", "Lower Boot and Lift Tube");
                robot.sleep(1.0);

        robot.backward(.06);
        telemetry.addData("Stage: ", "Unhook 1");
                robot.sleep(1.0);

        robot.strafeleft(.2);
        telemetry.addData("Stage: ", "Unhook 2");
                robot.sleep(1.0);

        robot.forward(.06);
        telemetry.addData("Stage: ", "Unhook 3");
                robot.sleep(1.0);




        if(GoldPos == 1){  //gold is to the left
            robot.turnleft(1.2);
            telemetry.addData("Stage: ", "Left Turn");
                    robot.sleep(1.0);

            robot.slideandsweepout(3.0);
            telemetry.addData("Stage: ", "Slide Out and Sweep in");
                    robot.sleep(1.0);

            robot.sweepin(1.0);
            telemetry.addData("Stage: ", "Continue Sweeping in");
                    robot.sleep(1.0);

            robot.slideandsweepin(3.0);
            telemetry.addData("Stage: ", "Slide in and Sweep in");
                    robot.sleep(1.0);

            robot.turnright(.2);
            telemetry.addData("Stage: ", "Turn Right");
                    robot.sleep(1.0);

        }else if (GoldPos == 3){//gold is to the right
            robot.turnleft(.45);
            telemetry.addData("Stage: ", "Turn Left");
                    robot.sleep(1.0);

            robot.slideandsweepout(3.0);
            telemetry.addData("Stage: ", "Slide Out and Sweeper Out");
                    robot.sleep(1.0);

            robot.sweepin(1.0);
            telemetry.addData("Stage: ", "Continue Sweeping in");
                    robot.sleep(1.0);

            robot.slideandsweepin(3.0);
            telemetry.addData("Stage: ", "Slide in and Sweep in");
                    robot.sleep(1.0);

            robot.turnleft(.1);
            telemetry.addData("Stage: ", "Turn Left");
                    robot.sleep(1.0);

        }else if (GoldPos == 2){//gold is in the center
            robot.turnleft(.82);
            telemetry.addData("Stage: ", "Turn Left");
                    robot.sleep(1.0);

            robot.slideandsweepout(3.0);
            telemetry.addData("Stage: ", "Slide Out and Sweep in");
                    robot.sleep(1.0);

            robot.sweepin(1.0);
            telemetry.addData("Stage: ", "Continue Sweeping in");
                    robot.sleep(1.0);

            robot.slideandsweepin(3.0);
            telemetry.addData("Stage: ", "Slide in and Sweep in");
                    robot.sleep(1.0);
        }


        robot.turnleft(.3);
        telemetry.addData("Stage: ", "Turn Left");
                robot.sleep(1.0);

        robot.forward(1.0);
        telemetry.addData("Stage: ", "Forward");
                robot.sleep(1.0);

        robot.turnright(1.0);
        telemetry.addData("Stage: ", "Turn Right");
                robot.sleep(1.0);

        robot.forward(1.32);
        telemetry.addData("Stage: ", "Forward");
                robot.sleep(1.0);

        robot.drop(.3);
        telemetry.addData("Stage: ", "Drop");
                robot.sleep(1.0);

        robot.turnleft(1.35);
        telemetry.addData("Stage: ", "Turn Left");
                robot.sleep(1.0);

        robot.forward(2.0);
        telemetry.addData("Stage: ", "Forward");
                robot.sleep(1.0);




        robot.sleep(100000.0);
        telemetry.addData("Path", "Complete");
        sleep(1000);
    }
}
