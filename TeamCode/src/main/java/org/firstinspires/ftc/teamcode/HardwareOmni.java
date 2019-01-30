//package org.firstinspires.ftc.teamcode;
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;






public class HardwareOmni
{
    /* Public OpMode members. */
    public DcMotor  leftFront   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  leftRear    = null;
    public DcMotor  rightRear   = null;
    public DcMotor  sweeper     = null;
    public DcMotor  slide       = null;
    public DcMotor  popper      = null;
    public DcMotor  lift        = null;

    public Servo    tube        = null;
    public Servo    marker  = null;



    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public HardwareOmni(){

    }
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        leftFront  = hwMap.get(DcMotor.class, "leftFront");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        leftRear   = hwMap.get(DcMotor.class, "leftRear");
        rightRear  = hwMap.get(DcMotor.class, "rightRear");
        sweeper    = hwMap.get(DcMotor.class, "sweeper");
        slide      = hwMap.get(DcMotor.class, "slide");
        popper     = hwMap.get(DcMotor.class, "popper");
        lift       = hwMap.get(DcMotor.class, "lift");

        tube       = hwMap.get(Servo.class, "tube");
        marker = hwMap.get(Servo.class, "leftmarker");

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        sweeper.setPower(0);
        slide.setPower(0);
        popper.setPower(0);
        lift.setPower(0);

        tube.setPosition(.975);
        marker.setPosition(.6);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        popper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }


    public void rotpop(Double time) {
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < time) {

        }
    }

    public void backward(Double time){
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < time){
            leftFront.setPower(-1); //-
            rightFront.setPower(1); //+
            leftRear.setPower(-1); //-
            rightRear.setPower(1); //+
        }
    }

    public void forward(Double time){
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < time){
            leftFront.setPower(1); //+
            rightFront.setPower(-1); //-
            leftRear.setPower(1); //+
            rightRear.setPower(-1); //-
        }
    }

    public void strafeleft(Double time){
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < time){
            leftFront.setPower(-1);//-
            rightFront.setPower(-1);//-
            leftRear.setPower(1);//+
            rightRear.setPower(1);//+
        }
    }

    public void straferight(Double time){
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < time){
            leftFront.setPower(1);//+
            rightFront.setPower(1);//+
            leftRear.setPower(-1);//-
            rightRear.setPower(-1);//-
        }
    }

    public void turnleft(Double time){
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < time){
            leftFront.setPower(-1);//-
            rightFront.setPower(-1);//-
            leftRear.setPower(-1);//-
            rightRear.setPower(-1);//-
        }
    }

    public void turnright(Double time){
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < time){
            leftFront.setPower(1);//+
            rightFront.setPower(1);//+
            leftRear.setPower(1);//+
            rightRear.setPower(1);//+
        }
    }

    public void sleep(Double time){
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < time){
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
            sweeper.setPower(0);
            slide.setPower(0);
            popper.setPower(0);
            lift.setPower(0);
        }
    }

    public void slideandsweepout(Double time){
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < time){
            slide.setPower(-1);//-
            sweeper.setPower(-1);//-
        }
    }

    public void slideandsweepin(Double time){
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < time){
            slide.setPower(1);//+
            sweeper.setPower(-1);//-
        }
    }

    public void sweepin(Double time){
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < time){
            sweeper.setPower(-1);//-
        }
    }

    public void sweepout(Double time){
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < time){
            sweeper.setPower(1);//+
        }
    }

    public void pop(Double time){
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < time){
            popper.setPower(-1);
        }
    }

    public void startauto(Double time){
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < time){
            lift.setPower(-1);
            //tube.setPosition(.41);
        }
    }

    public void drop(Double time){
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < time){
            marker.setPosition(0);
        }
    }


}

