package org.firstinspires.ftc.teamcode.drive.forwardKinematics;

import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.command.WaitCommand;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.hardware.Motor;
import com.amarcolini.joos.util.MathUtil;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SampleBot;

import java.util.ArrayList;
import java.util.Objects;

@JoosConfig
@Autonomous(group = "forwardKinematics")
public class MaxVelocityTuner extends CommandOpMode {
    public static double RUNTIME = 1;
    private SampleBot robot;

    private double maxVel = 0.0;
    @Override
    public void preInit() {
        robot = registerRobot(new SampleBot());
        telemetry.addLine("Your robot will drive at full speed for " + RUNTIME + " seconds.");
        telemetry.addLine("Make sure you have enough room for the robot to move.");

        schedule(
                new WaitCommand(RUNTIME)
                        .onInit(() -> robot.drive.setDrivePower(
                                new Pose2d(1)
                        ))
                        .onExecute(() -> {
                            double vel = Objects.requireNonNull(robot.drive.getPoseVelocity(), "getPoseVelocity() must not return null.").vec().norm();
                            maxVel = Math.max(vel, maxVel);
                        })
                        .onEnd(interrupted -> {
                            robot.drive.setDrivePower(new Pose2d(0));
                            telemetry.setAutoClear(false);
                            telemetry.addData("Maximum Velocity", maxVel);

                            ArrayList<Double> kFs = new ArrayList<>();
                            double initialkF = Double.NaN;
                            boolean areSame = true;
                            for (Motor motor : robot.drive.motors.getMotors()) {
                                double kF = 32767 / motor.distanceVelocityToTicks(maxVel);
                                if (Double.isNaN(initialkF)) initialkF = kF;
                                areSame = MathUtil.epsilonEquals(kF, initialkF) && areSame;
                                kFs.add(kF);
                            }
                            telemetry.addData("Voltage Compensated kF" + (areSame ? "" : "s"), areSame ? initialkF : kFs);
                        })
                        .requires(robot.drive)
        );
    }
}