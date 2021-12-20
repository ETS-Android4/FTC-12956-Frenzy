package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "FrenzyAuto", group = "default")

public class FrenzyAuto extends LinearOpMode {

    //Motor variables
    DcMotor frontLeft;
    DcMotor rearLeft;
    DcMotor frontRight;
    DcMotor rearRight;
    DcMotor carouselMover;
    DcMotor liftSpool;

    //Encoders
    DcMotor verticalLeft, verticalRight, horizontal;
    final double COUNTS_PER_INCH = 8192.0 / ((38.0 / 25.4) * Math.PI);
    String frontRightName = "frontRight", rearRightName = "rearRight", frontLeftName = "frontLeft", rearLeftName = "rearLeft";
    String verticalLeftEncoderName = "frontLeft", verticalRightEncoderName = "frontRight", horizontalEncoderName = "rearLeft";
    OdometryGlobalCoordinatePosition globalPositionUpdate;

    //Drivetrain speed variables
    double frontLeftPower;
    double rearLeftPower;
    double frontRightPower;
    double rearRightPower;

    OpenCvCamera webcam;
    FrenzyPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        initDriveHardwareMap(frontRightName, rearRightName, frontLeftName, rearLeftName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        //Initializes the motors and assigns them to a motor in the hardwareMap
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        rearLeft = hardwareMap.dcMotor.get("rearLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        rearRight = hardwareMap.dcMotor.get("rearRight");
        carouselMover = hardwareMap.dcMotor.get("carouselMover");
        liftSpool = hardwareMap.dcMotor.get("liftSpool");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftSpool.setDirection(DcMotorSimple.Direction.REVERSE);

        //Initializes webcam and computer vision pipeline
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new FrenzyPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("OpenCV Error: ", errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();
        globalPositionUpdate.reverseLeftEncoder();

        for(int i = 0; i < 50 && opModeIsActive(); i++) {
            telemetry.addData("Avg1: ", pipeline.getAvg1());
            telemetry.addData("Avg2: ", pipeline.getAvg2());
            telemetry.addData("Avg3: ", pipeline.getAvg3());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();
            sleep(20);
        }
        int artifactPosition = pipeline.getPosition();
        sleep(100);

        telemetry.addData("Temp Position: ", artifactPosition);
        sleep(1000);

        if(artifactPosition == 0){
            telemetry.addData("Position: ", "LEFT");
            telemetry.update();
            leftRoute();
        }
        else if(artifactPosition == 1){
            telemetry.addData("Position: ", "MIDDLE");
            telemetry.update();
            middleRoute();
        }
        else if(artifactPosition == 2){
            telemetry.addData("Position: ", "RIGHT");
            telemetry.update();
            rightRoute();
        }
        else{
            telemetry.addData("Position: ", "UNKNOWN");
            telemetry.update();
        }
    }

    public void leftRoute() {
        goToPosition(-12, 0, 0.2, 0, 0.5);
    }

    public void middleRoute() {
        goToPosition(0, 12, 0.2, 0, 0.5);
    }

    public void rightRoute() {
        goToPosition(12, 0, 0.2, 0, 0.5);
    }

    public void moveDrivetrain(double x, double y, double rx) {

        frontLeftPower = (y + x + rx);
        rearLeftPower = (y - x + rx);
        frontRightPower = (y - x - rx);
        rearRightPower = (y + x -rx);

        frontLeft.setPower(frontLeftPower);
        rearLeft.setPower(rearLeftPower);
        frontRight.setPower(frontRightPower);
        rearRight.setPower(rearRightPower);

    }

    public void stopDrivetrain() {
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);
    }

    public static class FrenzyPipeline extends OpenCvPipeline {

        /*
         * Some color constants
         */
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar DARK_RED = new Scalar(150, 0, 0);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar DARK_GREEN = new Scalar(0, 150, 0);
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar DARK_BLUE = new Scalar(0, 0, 255);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(225,320);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(640,320);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(1100,320);

        static final int REGION_WIDTH = 100;
        static final int REGION_HEIGHT = 100;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Mat region1_Cb;
        Mat region2_Cb;
        Mat region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;
        int avg2;
        int avg3;

        private int position = 0;

        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    DARK_RED, // The color the rectangle is drawn in
                    8); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    DARK_GREEN, // The color the rectangle is drawn in
                    8); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    DARK_BLUE, // The color the rectangle is drawn in
                    8); // Thickness of the rectangle lines

            if(avg1 > avg2 && avg1 > avg3) {
                position = 0;
            }
            else if(avg2 > avg1 && avg2 > avg3) {
                position = 1;
            }
            else if(avg3 > avg1 && avg3 > avg2) {
                position = 2;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getPosition() {
            return position;
        }

        public int getAvg1() {
            return avg1;
        }

        public int getAvg2() {
            return avg2;
        }

        public int getAvg3() {
            return avg3;
        }
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

    private void initDriveHardwareMap(String frontRightName, String rearRightName, String frontLeftName, String rearLeftName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        frontRight = hardwareMap.dcMotor.get(frontRightName);
        rearRight = hardwareMap.dcMotor.get(rearRightName);
        frontLeft = hardwareMap.dcMotor.get(frontLeftName);
        rearLeft = hardwareMap.dcMotor.get(rearLeftName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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