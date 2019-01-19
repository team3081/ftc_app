
package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.ftccommon.SoundPlayer;

@Autonomous(name="AutoTests", group="Pushbot")
//@Disabled
public class AutoTests extends LinearOpMode {

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

        //Things to note:
        //1 sec forward == about 3 feet at full power
        //2.85 sec turn == 360 turn


        robot.forward(1.0);
        telemetry.addData("Path", "Forward");


        robot.backward(1.0);
        telemetry.addData("Path", "Backward");


        robot.sleep(1.0);
        telemetry.addData("Path", "Sleep");

    }
}




