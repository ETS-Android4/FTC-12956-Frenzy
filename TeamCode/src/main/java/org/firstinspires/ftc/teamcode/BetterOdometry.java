package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rrdrive.SampleMecanumDrive;

@Autonomous(group="Autonomous")
public class BetterOdometry extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory trajectoryA = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(30)
                .forward(20)
                .build();

        drive.followTrajectory(trajectoryA);

        while (!isStopRequested() && opModeIsActive());
    }
}