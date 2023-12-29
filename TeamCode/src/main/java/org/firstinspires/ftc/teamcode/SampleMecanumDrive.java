package org.firstinspires.ftc.teamcode;

import com.amarcolini.joos.control.PIDCoefficients;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.followers.HolonomicPIDVAFollower;
import com.amarcolini.joos.followers.TrajectoryFollower;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.hardware.MotorGroup;
import com.amarcolini.joos.hardware.drive.DriveTrajectoryFollower;
import com.amarcolini.joos.hardware.drive.MecanumDrive;
import com.amarcolini.joos.localization.AngleSensor;
import com.amarcolini.joos.trajectory.constraints.MecanumConstraints;
import com.amarcolini.joos.trajectory.constraints.TrajectoryConstraints;
import org.jetbrains.annotations.NotNull;

@JoosConfig
public class SampleMecanumDrive extends MecanumDrive implements DriveTrajectoryFollower {
    public static double trackWidth = 18.0;
    public static double lateralMultiplier = 1.0;
    public static MecanumConstraints constraints = new MecanumConstraints(
            40.0,
            18.0,
            18.0, 1.0,
            40.0,
            40.0,
            Angle.deg(180.0),
            Angle.deg(180.0)
    );
    public static final PIDCoefficients axialCoeffs = new PIDCoefficients();
    public static final PIDCoefficients lateralCoeffs = new PIDCoefficients();
    public static final PIDCoefficients headingCoeffs = new PIDCoefficients();
    private final HolonomicPIDVAFollower trajectoryFollower = new HolonomicPIDVAFollower(
            axialCoeffs, lateralCoeffs, headingCoeffs,
            new Pose2d(0.5, 0.5, Angle.deg(5)), 0.5
    );

    public SampleMecanumDrive(MotorGroup motors, AngleSensor headingSensor) {
        super(motors, trackWidth, trackWidth, lateralMultiplier, headingSensor);
    }

    public final MotorGroup motorGroup = motors;
    public final MotorGroup leftMotors = new MotorGroup(motors.subList(0, 1));
    public final MotorGroup rightMotors = new MotorGroup(motors.subList(1, 2));

    @NotNull
    @Override
    public TrajectoryConstraints getConstraints() {
        return constraints;
    }

    @NotNull
    @Override
    public TrajectoryFollower getTrajectoryFollower() {
        return trajectoryFollower;
    }
}