
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@TeleOp(name="OmniLight", group="Pushbot")
@Disabled
public class OmniLight extends OpMode{

    HardwareOmni robot       = new HardwareOmni();

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");    //
    }
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        double rightx = .75*gamepad1.left_stick_x;
        double righty = .75*gamepad1.left_stick_y;
        double leftx = .75*gamepad1.right_stick_x;
        double lefty = -.75*gamepad1.right_stick_y;
        double diagUpRight;
        double diagUpLeft;
        double diagDownRight;
        double diagDownLeft;




        if(gamepad1.left_stick_y < 0){                     //backward
            robot.leftFront.setPower(-righty); //+
            robot.rightFront.setPower(righty); //-
            robot.leftRear.setPower(-righty); //+
            robot.rightRear.setPower(righty); //-
        }else if(gamepad1.left_stick_y > 0) {              //forward
            robot.leftFront.setPower(-righty); //-
            robot.rightFront.setPower(righty); //+
            robot.leftRear.setPower(-righty); //-
            robot.rightRear.setPower(righty); //+
        }else if(gamepad1.left_stick_x > 0){               //strafe right
            robot.leftFront.setPower(rightx);//+
            robot.rightFront.setPower(rightx);//+
            robot.leftRear.setPower(-rightx);//-
            robot.rightRear.setPower(-rightx);//-
        }else if(gamepad1.left_stick_x < 0) {              //strafe left
            robot.leftFront.setPower(rightx);//-
            robot.rightFront.setPower(rightx);//-
            robot.leftRear.setPower(-rightx);//+
            robot.rightRear.setPower(-rightx);//+
        }else if(gamepad1.right_stick_x < 0){                //spin left
            robot.leftFront.setPower(leftx);//-
            robot.rightFront.setPower(leftx);//-
            robot.leftRear.setPower(leftx);//-
            robot.rightRear.setPower(leftx);//-
        }else if(gamepad1.right_stick_x > 0) {                //spin right
            robot.leftFront.setPower(leftx);//+
            robot.rightFront.setPower(leftx);//+
            robot.leftRear.setPower(leftx);//+
            robot.rightRear.setPower(leftx);//+
        }

//        else if(gamepad1.left_stick_y > 0 && gamepad1.left_stick_x > 0){                //Strafe up/right
//            robot.leftFront.setPower(diagUpRight);//-
//            robot.rightFront.setPower(0);
//            robot.leftRear.setPower(0);
//            robot.rightRear.setPower(diagUpRight);
//        }else if(gamepad1.left_stick_y > 0 && gamepad1.left_stick_x < 0){                //Strafe up/left
//            robot.leftFront.setPower(0);
//            robot.rightFront.setPower(diagUpLeft);//+
//            robot.leftRear.setPower(diagUpLeft);//-
//            robot.rightRear.setPower(0);
//        }else if(gamepad1.left_stick_y > 0 && gamepad1.left_stick_x > 0){                //Strafe down/right
//            robot.leftFront.setPower(diagDownRight);//+
//            robot.rightFront.setPower(0);
//            robot.leftRear.setPower(0);
//            robot.rightRear.setPower(diagDownRight);//-
//        }else if(gamepad1.left_stick_y > 0 && gamepad1.left_stick_x > 0){                //Strafe down/left
//            robot.leftFront.setPower(0);
//            robot.rightFront.setPower(diagDownLeft);//-
//            robot.leftRear.setPower(diagDownLeft);//+
//            robot.rightRear.setPower(0);
//        }
//

else{                                              //stop
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightRear.setPower(0);
        }

        if(gamepad1.right_bumper){                          //lift up
            robot.lift.setPower(-1);
        }else if(gamepad1.left_bumper){                     //lift down
            robot.lift.setPower(1);
        }else{
            robot.lift.setPower(0);
        }
        telemetry.addData("lefty",  "%.2f", lefty);
        telemetry.addData("leftx", "%.2f", leftx);
        telemetry.addData("rightx", "%.2f", rightx);
    }

    @Override
    public void stop() {
    }
}
