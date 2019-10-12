package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.tank.SampleTankDriveBase;
import org.firstinspires.ftc.teamcode.drive.tank.SampleTriRobotDriveREV;

@Config
@Autonomous(group = "drive")
public class StraightTestTriRobot extends LinearOpMode {

    public static double DISTANCE = 60;
    @Override
    public void runOpMode() throws InterruptedException {

        SampleTankDriveBase drive = new SampleTriRobotDriveREV(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory);

    }
}
