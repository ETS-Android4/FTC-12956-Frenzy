package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;

@TeleOp(name = "OdometryOpMode")
public class OdometryOpmode extends LinearOpMode {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 8192.0 / ((38.0 / 25.4) * Math.PI);

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "frontRight", rbName = "rearRight", lfName = "frontLeft", lbName = "rearLeft";
    String verticalLeftEncoderName = lfName, verticalRightEncoderName = rfName, horizontalEncoderName = lbName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();
        globalPositionUpdate.reverseLeftEncoder();

        goToPosition(0, 12, 0.2, 0, 0.5);
        goToPosition(12, 12, 0.2, 45, 0.5);
        goToPosition(0, 0, 0.2, 0, 0.5);

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    public void goToPosition(double targetX, double targetY, double power, double targetOrientation, double accuracy) {
        targetX *= COUNTS_PER_INCH;
        targetY *= COUNTS_PER_INCH;
        accuracy *= COUNTS_PER_INCH;
        double distanceToX = targetX - globalPositionUpdate.returnXCoordinate();
        double distanceToY = targetY - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToX, distanceToY);

        while(opModeIsActive() && distance > accuracy) {
            distanceToX = targetX - globalPositionUpdate.returnXCoordinate();
            distanceToY = targetY - globalPositionUpdate.returnYCoordinate();
            distance = Math.hypot(distanceToX, distanceToY);

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToX, distanceToY));

            double robotMovementX = calculateX(robotMovementAngle, power);
            double robotMovementY = calculateY(robotMovementAngle, power);
            double pivotCorrection = targetOrientation - globalPositionUpdate.returnOrientation();
            double pivotPower = pivotCorrection / 90;
            if(pivotCorrection >= 90) {
                pivotPower = 0.5;
            }
            else if(pivotCorrection < 90 && pivotCorrection >= 45) {
                pivotPower = 0.3;
            }
            else if(pivotCorrection < 45 && pivotCorrection >= 10) {
                pivotPower = 0.2;
            }
            else {
                pivotCorrection = 0.1;
            }
            telemetry.addData("Movement x: ", robotMovementX);
            telemetry.addData("Movement Y: ", robotMovementY);
            telemetry.addData("Rotation Correction: ", pivotCorrection);
            telemetry.addData("Distance to Point: ", distance);
            telemetry.update();
            moveDrivetrain(robotMovementX, robotMovementY, pivotPower);
        }
        stopDrivetrain();
    }

    public void moveDrivetrain(double x, double y, double rx) {
        left_front.setPower(y + x + rx);
        left_back.setPower(y - x + rx);
        right_front.setPower(y - x - rx);
        right_back.setPower(y + x -rx);
    }

    public void stopDrivetrain() {
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);
    }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
}