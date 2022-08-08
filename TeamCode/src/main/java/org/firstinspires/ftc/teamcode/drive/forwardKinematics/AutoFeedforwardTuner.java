package org.firstinspires.ftc.teamcode.drive.forwardKinematics;

import android.os.Build;
import android.util.Base64;

import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;

import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.command.ListenerCommand;
import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.command.SequentialCommand;
import com.amarcolini.joos.command.TimeCommand;
import com.amarcolini.joos.command.WaitCommand;
import com.amarcolini.joos.control.FeedforwardCoefficients;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.drive.DriveSignal;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.hardware.Motor;
import com.amarcolini.joos.profile.MotionProfile;
import com.amarcolini.joos.profile.MotionProfileGenerator;
import com.amarcolini.joos.profile.MotionSegment;
import com.amarcolini.joos.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Util;
import com.sun.tools.javac.util.StringUtils;

import org.apache.commons.math3.stat.regression.AbstractMultipleLinearRegression;
import org.apache.commons.math3.stat.regression.OLSMultipleLinearRegression;
import org.firstinspires.ftc.teamcode.SampleBot;

import java.sql.Time;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.function.IntFunction;
import java.util.stream.Collectors;

import kotlin.Pair;

@JoosConfig
@Autonomous(group = "forwardKinematics")
public class AutoFeedforwardTuner extends CommandOpMode {
    public static double DISTANCE = 30;

    private SampleBot robot;

    private double lastPos = 0.0;
    private double kVEstimate = 1;
    @Override
    public void preInit() {
        robot = registerRobot(new SampleBot());

        robot.drive.motors.setFeedforwardCoefficients(new FeedforwardCoefficients(1 / robot.drive.motors.getMaxDistanceVelocity()));
        robot.drive.setRunMode(Motor.RunMode.RUN_WITHOUT_ENCODER);
        robot.drive.setPoseEstimate(new Pose2d());

        schedule(
                new SequentialCommand(
                        new TimeCommand((t, dt) -> {
                            robot.drive.setDriveSignal(new DriveSignal(new Pose2d(testVel)));
                            testVel = 10 * t * t;
                            double estimate = robot.drive.getPoseEstimate().x;
                            if (dt <= 0) return false;
                            double vel = (robot.drive.getPoseEstimate().x - lastPos) / dt;
                            lastPos = estimate;
                            kVEstimate = testVel / vel * 0.4;
                            telemetry.addData("velocity", vel);
                            return Math.abs(vel) >= 10;
                        }),
                        Command.select(() -> {
                            double x = robot.drive.getPoseEstimate().x;
                            double target = (DISTANCE - x) * kVEstimate;
                            return new SequentialCommand(
//                                        new WaitCommand(5).onInit(() -> {
//                                            robot.drive.setDrivePower(new Pose2d());
//                                            telemetry.setAutoClear(false);
//                                            telemetry.addData("x", x);
//                                            telemetry.addData("target", target);
//                                            telemetry.addData("distance", DISTANCE);
//                                            telemetry.addData("testVel", testVel);
//                                            telemetry.addData("kVEstimate", kVEstimate);
//                                            telemetry.addData("maxAccel", robot.drive.getConstraints().maxAccel * 0.5 * kVEstimate);
//                                        }).onEnd(interrupted -> telemetry.setAutoClear(true)),
                                    Command.select(() -> followMotionProfile(MotionProfileGenerator.generateSimpleMotionProfile(
                                            new MotionState(0.0, testVel, 0),
                                            new MotionState(target, 0, 0),
                                            testVel, robot.drive.getConstraints().maxAccel * kVEstimate
                                    )))
                            );
                        }),
                        new WaitCommand(0.5),
                        Command.select(() ->
                                followMotionProfile(MotionProfileGenerator.generateSimpleMotionProfile(
                                        new MotionState(robot.drive.getPoseEstimate().x * kVEstimate, 0, 0),
                                        new MotionState(0.0,0,0),
                                        testVel * 2, robot.drive.getConstraints().maxAccel
                                ))
                        )
                )
                        .onEnd(interrupted -> {
                            telemetry.clearAll();
                            telemetry.setAutoClear(false);
                            if (interrupted) return;
                            robot.drive.setDrivePower(new Pose2d(0));
                            OLSMultipleLinearRegression regression = new OLSMultipleLinearRegression();
                            double[] voltage = data.parallelStream().mapToDouble(it -> it.voltage).toArray();
                            regression.newSampleData(voltage,
                                    data.parallelStream().map(it -> {
                                        double accel = it.accel;
                                        if (Double.isNaN(accel) || Double.isInfinite(accel)) {
                                            accel = 0.0;
                                        }
                                        return new double[]{it.vel, accel};
                                    }).toArray(double[][]::new));
                            regression.setNoIntercept(false);
                            double[] results = regression.estimateRegressionParameters();
                            telemetry.addLine(String.format("kV = %.4f", results[1]));
                            telemetry.addLine(String.format("kA = %.4f", results[2]));
                            telemetry.addLine(String.format("kStatic = %.4f", results[0]));
                            telemetry.addLine("_");
                            StringBuilder builder = new StringBuilder();
                            for (DataPoint dataPoint : data) {
                                builder.append(dataPoint.id).append('*')
                                        .append(dataPoint.voltage).append(',')
                                        .append(dataPoint.vel).append(',')
                                        .append(dataPoint.accel).append(',')
                                        .append(dataPoint.targetVel).append(',')
                                        .append(dataPoint.targetAccel).append('n');
                            }
                            telemetry.addLine(builder.toString());
                            telemetry.addLine("_");
                        })
        );

        telemetry.addLine("ready!");
        telemetry.addData("distancePerRev", robot.drive.motors.getMotors()[0].getDistancePerRev());
    }

    private class DataPoint {
        final double targetVel;
        final double targetAccel;
        final double voltage;
        final double vel;
        final double accel;
        final int id;

        public DataPoint(int id, MotionState targetState, double voltage, double vel, double accel) {
            targetVel = targetState.v;
            targetAccel = targetState.a;
            this.voltage = voltage;
            this.vel = vel;
            this.id = id;
            this.accel = accel;
        }
    }

    ArrayList<DataPoint> data = new ArrayList<>();
    private double lastVel = 0.0;
    private double testVel = 0.0;
    private Command followMotionProfile(MotionProfile profile) {
        return new TimeCommand((t, dt) -> {
            MotionState state = profile.get(t);
            robot.drive.setDriveSignal(new DriveSignal(new Pose2d(state.v), new Pose2d(state.a)));
            double vel = robot.drive.getPoseVelocity().x;
            double accel = (vel - lastVel) / dt;
            lastVel = vel;
            telemetry.addData("duration", profile.duration());
            telemetry.addData("x", robot.drive.getPoseEstimate().x);
            double voltage = robot.drive.motors.getMotors()[0].getPower();
            telemetry.addData("voltage", voltage);
            data.add(new DataPoint(profile.hashCode(), state,
                    voltage, vel, accel));
            return t > profile.duration();
        });
    }
}