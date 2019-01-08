
package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
//Tings to note:
        //1 sec forward == about 3 feet at full power
        //




        robot.leftFront.setPower(-1);
        robot.rightFront.setPower(-1);
        robot.leftRear.setPower(-1);
        robot.rightRear.setPower(-1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.85)){
            telemetry.addData("Path", "Left Gold Leg 1", runtime.seconds());
            telemetry.update();
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




