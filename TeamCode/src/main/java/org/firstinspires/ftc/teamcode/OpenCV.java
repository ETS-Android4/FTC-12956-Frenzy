package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

/**
 * Created by Owen Bachyrycz on 11/28/2021.
 */

@Autonomous(name = "OpenCV", group = "default")

public class OpenCV extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor rearLeft;
    DcMotor frontRight;
    DcMotor rearRight;

    OpenCvCamera webcam;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

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

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        rearLeft = hardwareMap.dcMotor.get("rearLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        rearRight = hardwareMap.dcMotor.get("rearRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        for(int i = 0; i < 10 && opModeIsActive(); i++)
        {
            telemetry.addData("Analysis", FrenzyPipeline.getAnalysis());
            telemetry.update();
            sleep(20);
        }

        sleep(100);

        int artifactPosition = FrenzyPipeline.getAnalysis();

        if(artifactPosition == 0){
            telemetry.addData("Position: ", "RIGHT");
            telemetry.update();
        }
        else if(artifactPosition == 1){
            telemetry.addData("Position: ", "MIDDLE");
            telemetry.update();
        }
        else if(artifactPosition == 2){
            telemetry.addData("Position: ", "RIGHT");
            telemetry.update();
        }
        else{
            telemetry.addData("Position: ", "UNKNOWN");
            telemetry.update();
        }
    }

    /**
     * Sets drive train to move in x, y, and rx, for time milliseconds and then stops
     * @param x horizontal speed
     * @param y vertical speed
     * @param rx rotational speed
     * @param time how long to apply these movements for (milliseconds)
     */
    public void moveDrivetrain(double x, double y, double rx, int time){
        frontLeft.setPower(y + x + rx);
        rearLeft.setPower(y - x + rx);
        frontRight.setPower(y - x - rx);
        rearRight.setPower(y + x -rx);
        sleep(time);
        stopDrivetrain();
    }

    //Stops all drivetrain motors
    public void stopDrivetrain() {
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);
    }

    public static class FrenzyPipeline extends OpenCvPipeline {

        /*
        public enum artifactPosition {
            LEFT,
            MIDDLE,
            RIGHT
        }
        */
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar BLUE = new Scalar(0, 0, 255);

        static final Point FIRST_TOPLEFT_ANCHOR = new Point(200, 225);
        static final Point SECOND_TOPLEFT_ANCHOR = new Point(600, 225);
        static final Point THIRD_TOPLEFT_ANCHOR = new Point(1000, 225);

        static final int REGION_WIDTH = 50;
        static final int REGION_HEIGHT = 50;

        Point first_point_a = new Point(
                FIRST_TOPLEFT_ANCHOR.x,
                FIRST_TOPLEFT_ANCHOR.y);
        Point first_point_b = new Point(
                FIRST_TOPLEFT_ANCHOR.x + REGION_WIDTH,
                FIRST_TOPLEFT_ANCHOR.y + REGION_HEIGHT);

        Point second_point_a = new Point(
                SECOND_TOPLEFT_ANCHOR.x,
                SECOND_TOPLEFT_ANCHOR.y);
        Point second_point_b = new Point(
                SECOND_TOPLEFT_ANCHOR.x + REGION_WIDTH,
                SECOND_TOPLEFT_ANCHOR.y + REGION_HEIGHT);

        Point third_point_a = new Point(
                THIRD_TOPLEFT_ANCHOR.x,
                THIRD_TOPLEFT_ANCHOR.y);
        Point third_point_b = new Point(
                THIRD_TOPLEFT_ANCHOR.x + REGION_WIDTH,
                THIRD_TOPLEFT_ANCHOR.y + REGION_HEIGHT);

        Mat first_Cb;
        Mat second_Cb;
        Mat third_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;
        int avg2;
        int avg3;

        //private volatile OpenCV.FrenzyPipeline.artifactPosition position = OpenCV.FrenzyPipeline.artifactPosition.LEFT;
        public static int position = 0;

        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            first_Cb = Cb.submat(new Rect(first_point_a, first_point_b));
            second_Cb = Cb.submat(new Rect(second_point_a, second_point_b));
            third_Cb = Cb.submat(new Rect(third_point_a, third_point_b));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(first_Cb).val[0];
            avg2 = (int) Core.mean(second_Cb).val[0];
            avg3 = (int) Core.mean(third_Cb).val[0];

            Imgproc.rectangle(
                    input,
                    first_point_a,
                    first_point_b,
                    RED,
                    2);
            Imgproc.rectangle(
                    input,
                    second_point_a,
                    second_point_b,
                    GREEN,
                    2);
            Imgproc.rectangle(
                    input,
                    third_point_a,
                    third_point_b,
                    BLUE,
                    2);

            //position = OpenCV.FrenzyPipeline.artifactPosition.LEFT;
            if(avg1 > avg2 && avg1 > avg3){
                position = 0;
            }
            if(avg2 > avg1 && avg2 > avg3) {
                position = 1;
            }
            if(avg3 > avg1 && avg3 > avg2) {
                position = 2;
            }
            return input;
        }

        public static int getAnalysis() {
            return position;
        }
    }
}
