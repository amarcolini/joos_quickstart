package org.firstinspires.ftc.teamcode.drive.forwardKinematics;

import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.followers.TrajectoryFollower;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.hardware.Motor;
import com.amarcolini.joos.kinematics.MecanumKinematics;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SampleBot;

import java.util.Arrays;

@JoosConfig
@Autonomous(group = "forwardKinematics")
public class TurnTest extends CommandOpMode {
    public static Angle ANGLE = Angle.deg(90);
    private SampleBot robot;

    @Override
    public void preInit() {
        robot = registerRobot(new SampleBot());

        schedule(robot.drive.followTrajectory(robot.drive.trajectoryBuilder(new Pose2d())
                .turn(ANGLE)
                .build()
        ).onExecute(() -> {
            TrajectoryFollower follower = robot.drive.getTrajectoryFollower();
            Pose2d vel = follower.getTrajectory().velocity(follower.elapsedTime());
            telemetry.addData("vel", vel);
            telemetry.addData("converted", Math.toDegrees(vel.heading.degrees()));
            telemetry.addData("wheelVel", MecanumKinematics.robotToWheelVelocities(vel, robot.drive.getConstraints().trackWidth));
            telemetry.addData("trackWidth", -MecanumKinematics.robotToWheelVelocities(vel, robot.drive.getConstraints().trackWidth).get(0) / vel.heading.radians());
            telemetry.addData("powers", Arrays.toString(Arrays.stream(robot.drive.motors.getMotors()).mapToDouble(Motor::getPower).toArray()));
        }));

        telemetry.addLine("ready!");
    }
}
