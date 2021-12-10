package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "LiftCalibration", group = "default")
public class LiftCalibration extends OpMode {

    DcMotorEx liftSpool;

    @Override
    public void init() {
        liftSpool = (DcMotorEx) hardwareMap.dcMotor.get("liftSpool");
        liftSpool.setDirection(DcMotor.Direction.REVERSE);
        liftSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        telemetry.addData("Encoder Count: ", liftSpool.getCurrentPosition());
        telemetry.addData("Right Trigger: ", gamepad1.right_trigger);
        telemetry.addData("Left Trigger", gamepad1.left_trigger);
        telemetry.update();
        if(gamepad1.left_trigger > 0 && liftSpool.getCurrentPosition() < -50) {
            liftSpool.setVelocity(-10);
        }
        else if(gamepad1.right_trigger > 0 && liftSpool.getCurrentPosition() > -1000) {
            liftSpool.setVelocity(10);
        }
        else {
            liftSpool.setPower(0);
        }
    }
}
