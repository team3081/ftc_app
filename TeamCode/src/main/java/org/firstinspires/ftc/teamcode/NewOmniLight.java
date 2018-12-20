
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="NewOmniLight", group="Pushbot")
public class NewOmniLight extends OpMode{

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

        double leftx = .5*gamepad1.left_stick_x;
        double lefty = .5*gamepad1.left_stick_y;
        double rightx = .5*gamepad1.right_stick_x;
        double righty = -.5*gamepad1.right_stick_y;


        if(gamepad1.left_stick_y < 0){                     //backward
            robot.leftFront.setPower(-lefty); //+
            robot.rightFront.setPower(lefty); //-
            robot.leftRear.setPower(-lefty); //+
            robot.rightRear.setPower(lefty); //-
        }else if(gamepad1.left_stick_y > 0) {              //forward
            robot.leftFront.setPower(-lefty); //-
            robot.rightFront.setPower(lefty); //+
            robot.leftRear.setPower(-lefty); //-
            robot.rightRear.setPower(lefty); //+
        }else if(gamepad1.left_stick_x > 0){               //strafe right
            robot.leftFront.setPower(leftx);//+
            robot.rightFront.setPower(leftx);//+
            robot.leftRear.setPower(-leftx);//-
            robot.rightRear.setPower(-leftx);//-
        }else if(gamepad1.left_stick_x < 0) {              //strafe left
            robot.leftFront.setPower(leftx);//-
            robot.rightFront.setPower(leftx);//-
            robot.leftRear.setPower(-leftx);//+
            robot.rightRear.setPower(-leftx);//+
        }else if(gamepad1.dpad_left){                //spin left
            robot.leftFront.setPower(-.5);//-
            robot.rightFront.setPower(-.5);//-
            robot.leftRear.setPower(-.5);//-
            robot.rightRear.setPower(-.5);//-
        }else if(gamepad1.dpad_right) {                //spin right
            robot.leftFront.setPower(.5);//+
            robot.rightFront.setPower(.5);//+
            robot.leftRear.setPower(.5);//+
            robot.rightRear.setPower(.5);//+
        }else{                                              //stop
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
