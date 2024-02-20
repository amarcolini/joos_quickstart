package org.firstinspires.ftc.teamcode;

import com.amarcolini.joos.command.Component;
import com.amarcolini.joos.control.PIDCoefficients;
import com.amarcolini.joos.drive.AbstractSwerveDrive;
import com.amarcolini.joos.drive.PIDSwerveModule;
import com.amarcolini.joos.drive.SwerveModule;
import com.amarcolini.joos.followers.HolonomicPIDVAFollower;
import com.amarcolini.joos.followers.TrajectoryFollower;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.hardware.Motor;
import com.amarcolini.joos.hardware.MotorGroup;
import com.amarcolini.joos.hardware.drive.DriveComponent;
import com.amarcolini.joos.hardware.drive.DriveTrajectoryFollower;
import com.amarcolini.joos.localization.AngleSensor;
import com.amarcolini.joos.trajectory.constraints.GenericConstraints;
import com.amarcolini.joos.trajectory.constraints.TrajectoryConstraints;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

public class SampleSwerveDrive extends AbstractSwerveDrive implements DriveTrajectoryFollower {
    public static double trackWidth = 13.0;
    public static double wheelBase = 13.0;
    private final List<SampleSwerveModule> modules;
    public final MotorGroup motorGroup;

    public SampleSwerveDrive(
            @NotNull SampleSwerveModule frontLeft,
            @NotNull SampleSwerveModule backLeft,
            @NotNull SampleSwerveModule backRight,
            @NotNull SampleSwerveModule frontRight
    ) {
        super(frontLeft, backLeft, backRight, frontRight, trackWidth, wheelBase);
        modules = Arrays.asList(
                frontLeft, backLeft, backRight, frontRight
        );
        motorGroup = new MotorGroup(
                modules.stream().map((i) -> i.motor).collect(Collectors.toList())
        );
    }

    @Override
    public void update() {
        for (PIDSwerveModule module : modules) {
            module.update();
        }
    }

    @Override
    public void setRunMode(@NotNull Motor.RunMode runMode) {
        motorGroup.setRunMode(runMode);
    }

    @Override
    public void setZeroPowerBehavior(@NotNull Motor.ZeroPowerBehavior zeroPowerBehavior) {
        motorGroup.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @NotNull
    @Override
    public TrajectoryConstraints getConstraints() {
        return new GenericConstraints();
    }

    @NotNull
    @Override
    public TrajectoryFollower getTrajectoryFollower() {
        return new HolonomicPIDVAFollower(
                new PIDCoefficients(),
                new PIDCoefficients(),
                new PIDCoefficients()
        );
    }
}