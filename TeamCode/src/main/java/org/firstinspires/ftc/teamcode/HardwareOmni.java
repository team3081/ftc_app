//package org.firstinspires.ftc.teamcode;
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public CRServo  tube        = null;

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
        tube       = hwMap.get(CRServo.class, "tube");

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        sweeper.setPower(0);
        slide.setPower(0);
        popper.setPower(0);
        lift.setPower(0);
        tube.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        popper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

