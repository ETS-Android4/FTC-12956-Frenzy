package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Owen Bachyrycz on 12/5/2021.
 */


@TeleOp(name="MotorTest", group="default")

public class MotorTest extends OpMode {

    //Declares motor variables
    DcMotor frontLeft;
    DcMotor rearLeft;
    DcMotor frontRight;
    DcMotor rearRight;

    //The power each motor should be set to
    double frontLeftPower;
    double rearLeftPower;
    double frontRightPower;
    double rearRightPower;

    @Override
    public void init() {

        //Initializes the motors and assigns them to a motor in the hardwareMap
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        rearLeft = hardwareMap.dcMotor.get("rearLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        rearRight = hardwareMap.dcMotor.get("rearRight");

        //Reverses the front left motor
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {

        //Takes input from controller joysticks and assigns them to
        //variables x, y, and rx
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        //Calculates the power that should be sent to each motor
        //based on the controller inputs.
        frontLeftPower = (y + x + rx);
        rearLeftPower = (y - x + rx);
        frontRightPower = (y - x - rx);
        rearRightPower = (y + x -rx);

        if(gamepad1.a) {
            frontLeft.setPower(1.0);
        }

        if(gamepad1.b) {
            rearLeft.setPower(1.0);
        }

        if(gamepad1.x) {
            frontRight.setPower(1.0);
        }

        if(gamepad1.y) {
            rearRight.setPower(1.0);
        }


        updatePower();
    }

    @Override
    public void stop() {
        // Sets all motors to zero, stopping the drivetrain
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);
    }

    public void updatePower() {
        frontLeft.setPower(frontLeftPower);
        rearLeft.setPower(rearLeftPower);
        frontRight.setPower(frontRightPower);
        rearRight.setPower(rearRightPower);
    }
}
