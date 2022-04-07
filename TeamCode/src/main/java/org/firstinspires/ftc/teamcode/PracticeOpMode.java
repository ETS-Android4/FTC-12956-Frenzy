package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "PracticeOpMode", group = "default")

public class PracticeOpMode extends OpMode
{
    //Declare motors, servos, etc.
    DcMotor frontLeft;
    DcMotor rearLeft;
    DcMotor frontRight;
    DcMotor rearRight;
    //Declare motor powers; these are the values we will modify when we want to change the power/direction
    //of the motors
    double frontLeftPower;
    double frontRightPower;
    double rearLeftPower;
    double rearRightPower;

    @Override
    //Code to execute when "init" button is pressed
    public void init()
    {
        //Creating a hardwareMap;
        //Tells control hub to "create" motors, servos, etc.
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        rearRight = hardwareMap.dcMotor.get("rearRight");
        frontRight = hardwareMap.dcMotor. get("frontRight");
        rearLeft = hardwareMap.dcMotor.get("frontLeft");
        //Reverse motor direction
        //Not necessary, but helps in programming because
        //the motor powers can all be set to the same value
        //Instead of right being 1 and left being -1, they can
        //both be 1
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    //Code to execute when "start" button is pressed
    //Not super useful, but it is there if you want to use it
    public void start()
    {

    }

    @Override
    //Code to execute after the "start" button is pressed
    //and before "stop" button is pressed
    public void loop() {
        //Creating variables to use controller inputs
        //to set motor powers
        //gamepad1 refers to the first controller, gamepad2 refers to second controller
        //(if you decide to use two controllers)
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;
        //Sets motor power variables using doubles created above
        frontLeftPower = (y - x - rx);
        frontRightPower = (y + x + rx);
        rearRightPower = (y - x + rx);
        rearLeftPower = (y + x - rx);
        //Sets the power of each motor accordingly
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        rearLeft.setPower(rearLeftPower);
        rearRight.setPower(rearRightPower);
    }

    @Override
    //Code to execute when "stop" button is pressed
    //This method is SUPER important!
    //You need to stop all motors, servos, etc. here or else
    //they will just keep going based on the last buttons you
    //have pressed until the robot is turned off
    public void stop()
    {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }
}