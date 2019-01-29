
package org.firstinspires.ftc.teamcode;

import android.media.AudioManager;
import android.media.MediaPlayer;
import android.media.SoundPool;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;





@TeleOp(name="Omni2019", group="Pushbot")
public class Omni2019 extends OpMode{

    HardwareOmni robot       = new HardwareOmni();
    MediaPlayer mediaPlayer;


    @Override
    public void init() {
        robot.init(hardwareMap);
        mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.avengers);
        telemetry.addData("Say", "Hello Driver");
    }
    @Override
    public void init_loop() {
        robot.tube.setPosition(.41);
    }
    double position = .5;


    @Override
    public void start() {

    }
    String AA;
    @Override
    public void loop() {


        if(gamepad1.back && gamepad1.x){
            mediaPlayer.start();
        }

        if(gamepad1.back && gamepad1.y){
            mediaPlayer.pause();
        }

        if(gamepad1.back && gamepad1.a){
            mediaPlayer.seekTo(0);
        }

        if(gamepad1.back && gamepad1.b){
            mediaPlayer.seekTo(31600);
            mediaPlayer.start();
            AA = "Avengers Assemble!";
        }

        if(gamepad1.dpad_up){
            mediaPlayer.setVolume(1,1);
        }

        if(gamepad1.dpad_down){
            mediaPlayer.setVolume(0,0);
        }



        double leftx = gamepad1.left_stick_x;
        double lefty = gamepad1.left_stick_y;
        double rightx = gamepad1.right_stick_x;
        double righty = gamepad1.right_stick_y;
        double slide = -gamepad2.left_stick_y * .75;


        if(gamepad1.left_stick_y < 0){                      //backward
            robot.leftFront.setPower(-lefty); //+
            robot.rightFront.setPower(lefty); //-
            robot.leftRear.setPower(-lefty); //+
            robot.rightRear.setPower(lefty); //-
        }else if(gamepad1.left_stick_y > 0) {               //forward
            robot.leftFront.setPower(-lefty); //-
            robot.rightFront.setPower(lefty); //+
            robot.leftRear.setPower(-lefty); //-
            robot.rightRear.setPower(lefty); //+
        }else if(gamepad1.left_stick_x > 0){//strafe right
            lefty = 0;
            robot.leftFront.setPower(leftx);//+
            robot.rightFront.setPower(leftx);//+
            robot.leftRear.setPower(-leftx);//-
            robot.rightRear.setPower(-leftx);//-
        }else if(gamepad1.left_stick_x < 0) {               //strafe left
            lefty = 0;
            robot.leftFront.setPower(leftx);//-
            robot.rightFront.setPower(leftx);//-
            robot.leftRear.setPower(-leftx);//+
            robot.rightRear.setPower(-leftx);//+
        }else if(gamepad1.right_stick_x < 0){                     //spin left
            robot.leftFront.setPower(rightx);//-
            robot.rightFront.setPower(rightx);//-
            robot.leftRear.setPower(rightx);//-
            robot.rightRear.setPower(rightx);//-
        }else if(gamepad1.right_stick_x > 0) {                   //spin right
            robot.leftFront.setPower(rightx);//+
            robot.rightFront.setPower(rightx);//+
            robot.leftRear.setPower(rightx);//+
            robot.rightRear.setPower(rightx);//+
        }else{                                              //stop
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightRear.setPower(0);
        }

        if(gamepad2.left_stick_y > 0){                      //slide out
            robot.slide.setPower(slide);//-
        }else if(gamepad2.left_stick_y <  0){                //slide in
            robot.slide.setPower(slide);//+
        }else{                                              //stop
            robot.slide.setPower(0);
        }

        if(gamepad2.right_bumper){                          //sweep in
            robot.sweeper.setPower(-1);
        }else if(gamepad2.left_bumper){                     //sweep out
            robot.sweeper.setPower(1);
        }else{                                              //stop
            robot.sweeper.setPower(0);
        }

        if(gamepad2.dpad_up){                               //tube up
            robot.tube.setPosition(.41);
        }else if(gamepad2. dpad_down){                      //tube down
            robot.tube.setPosition(.975);
        }

        if(gamepad1.y){
            position = position +.5;
            robot.marker.setPosition(position);                    //marker out
        }if (gamepad1.a){
            position = position -.5;
            robot.marker.setPosition(0);                    //marker in

        }



        if(gamepad2.x){                                     //pop
            robot.popper.setPower(-1);
        }else if(gamepad2.a){                               //opposite pop
            robot.popper.setPower(1);
        }else{                                              //stop
            robot.popper.setPower(0);
        }



        if(gamepad1.right_bumper){                               //lift up
            robot.lift.setPower(-1);
        }else if(gamepad1.left_bumper){                       //lift down
            robot.lift.setPower(1);
        }else{                                              //stop
            robot.lift.setPower(0);
        }


        telemetry.addData("lefty",  "%.2f", lefty);
        telemetry.addData("leftx", "%.2f", leftx);
        telemetry.addData("rightx", "%.2f", rightx);
        telemetry.addData("", AA);


    }

    @Override
    public void stop() {
        mediaPlayer.stop();
    }
}
