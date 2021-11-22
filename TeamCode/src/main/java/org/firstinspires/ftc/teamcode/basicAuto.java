package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Create by Owen Bachyrycz on 11/17/2021.
 */

public class basicAuto extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor rearLeft;
    DcMotor frontRight;
    DcMotor rearRight;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        rearLeft = hardwareMap.dcMotor.get("rearLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        rearRight = hardwareMap.dcMotor.get("rearRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        moveDrivetrain(0, 0, 0.5, 5000);

    }

    /**
     * Sets drive train to move in x, y, and rx, for time milliseconds and then stops
     * @param x horizontal speed
     * @param y vertical speed
     * @param rx rotational speed
     * @param time how long to apply these movements for
     */
    public void moveDrivetrain(double x, double y, double rx, int time){
        frontLeft.setPower(y + x + rx);
        rearLeft.setPower(y - x + rx);
        frontRight.setPower(y - x - rx);
        rearRight.setPower(y + x -rx);
        sleep(time);
        stopDrivetrain();
    }

    public void stopDrivetrain() {
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);
    }
}
