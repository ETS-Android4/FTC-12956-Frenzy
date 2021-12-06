package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Owen Bachyrycz on 12/1/2021.
 */

@TeleOp(name="MagneticEncoderTest", group="default")

public class MagneticEncoderTest extends OpMode {

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
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

        //Takes input from controller joysticks and assigns them to
        //variables x, y, and rx
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        int enc0 = frontLeft.getCurrentPosition();
        double enc0mm = ((double)enc0 / 8192) * 38 * Math.PI;
        int enc1 = rearLeft.getCurrentPosition();
        double enc1mm = ((double)enc1 / 8192) * 38 * Math.PI;
        int enc2 = frontRight.getCurrentPosition();
        double enc2mm = ((double)enc2 / 8192) * 38 * Math.PI;

        //Calculates the power that should be sent to each motor
        //based on the controller inputs.
        frontLeftPower = (y + x + rx);
        rearLeftPower = (y - x + rx);
        frontRightPower = (y - x - rx);
        rearRightPower = (y + x -rx);

        //Applies the calculated power to the motors
        frontLeft.setPower(frontLeftPower);
        rearLeft.setPower(rearLeftPower);
        frontRight.setPower(frontRightPower);
        rearRight.setPower(rearRightPower);

        telemetry.addData("Left Encoder Travel (mm): ", enc0mm);
        telemetry.addData("Middle Encoder Travel (mm): ", enc1mm);
        telemetry.addData("Right Encoder Travel (mm): ", enc2mm);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Sets all motors to zero, stopping the drivetrain
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);
    }
}
