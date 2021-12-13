package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp
public class OpenCVExample extends LinearOpMode
{
    OpenCvCamera webcam;
    FrenzyPipeline pipeline;

    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new FrenzyPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280,720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("OpenCV Error: ", errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Avg1: ", pipeline.getAvg1());
            telemetry.addData("Avg2: ", pipeline.getAvg2());
            telemetry.addData("Avg3: ", pipeline.getAvg3());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    public static class FrenzyPipeline extends OpenCvPipeline
    {
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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(245,310);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(590,310);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(935,310);

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

        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    DARK_RED,
                    8);

            Imgproc.rectangle(
                    input,
                    region2_pointA,
                    region2_pointB,
                    DARK_GREEN,
                    8);

            Imgproc.rectangle(
                    input,
                    region3_pointA,
                    region3_pointB,
                    DARK_BLUE,
                    8);

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
                    input,
                    region1_pointA,
                    region1_pointB,
                    RED,
                    -1);

            Imgproc.rectangle(
                    input,
                    region2_pointA,
                    region2_pointB,
                    GREEN,
                    -1);

            Imgproc.rectangle(
                    input,
                    region3_pointA,
                    region3_pointB,
                    BLUE,
                    -1);

            return input;
        }

        public int getPosition() {
            return position;
        }

        public int getAvg1()
        {
            return avg1;
        }

        public int getAvg2()
        {
            return avg2;
        }

        public int getAvg3()
        {
            return avg3;
        }
    }
}