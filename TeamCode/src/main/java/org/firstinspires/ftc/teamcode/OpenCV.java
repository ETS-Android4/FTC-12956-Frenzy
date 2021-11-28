package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.openftc.easyopencv.*;

/**
 * Created by Owen Bachyrycz on 11/28/2021.
 */

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

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("OpenCV Error: ", errorCode);
            }
        });
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        rearLeft = hardwareMap.dcMotor.get("rearLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        rearRight = hardwareMap.dcMotor.get("rearRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Sets drive train to move in x, y, and rx, for time milliseconds and then stops
     * @param x horizontal speed
     * @param y vertical speed
     * @param rx rotational speed
     * @param time how long to apply these movements for (millis)
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
         * An enum for each position the artifact can start in
         */
        public enum artifactPosition {
            LEFT,
            MIDDLE,
            RIGHT
        }

        static final Point FIRST_TOPLEFT_ANCHOR = new Point(200, 200);
        static final Point SECOND_TOPLEFT_ANCHOR = new Point(600, 200);
        static final Point THIRD_TOPLEFT_ANCHOR = new Point(1000, 200);

        static final int REGION_WIDTH = 50;
        static final int REGION_HEIGHT = 50;

        final int DETECTION_THRESHOLD = 150;

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


        @Override
        public Mat processFrame(Mat input) {
            return null;
        }
    }

}
