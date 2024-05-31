package org.firstinspires.ftc.teamcode;

import com.amarcolini.joos.control.DCMotorFeedforward;
import com.amarcolini.joos.control.PIDCoefficients;
import com.amarcolini.joos.dashboard.Immutable;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.drive.AbstractSwerveDrive;
import com.amarcolini.joos.drive.PIDSwerveModule;
import com.amarcolini.joos.followers.HolonomicPIDVAFollower;
import com.amarcolini.joos.followers.TrajectoryFollower;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.hardware.CRServo;
import com.amarcolini.joos.hardware.Motor;
import com.amarcolini.joos.hardware.MotorGroup;
import com.amarcolini.joos.hardware.drive.DriveTrajectoryFollower;
import com.amarcolini.joos.trajectory.constraints.GenericConstraints;
import com.amarcolini.joos.trajectory.constraints.MecanumConstraints;
import com.amarcolini.joos.trajectory.constraints.SwerveConstraints;
import com.amarcolini.joos.trajectory.constraints.TrajectoryConstraints;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

@JoosConfig
public class SampleSwerveDrive extends AbstractSwerveDrive implements DriveTrajectoryFollower {
    // @TODO Update swerve trackwidth/wheelbase to match your drive configuration.
    public static double trackWidth = 12.0;
    public static double wheelBase = 12.0;
    private final List<SampleSwerveModule> modules;
    private final MotorGroup motorGroup;

    @NotNull
    @Override
    public MotorGroup getMotors() {
        return motorGroup;
    }

    @Immutable
    public static SwerveConstraints constraints = new SwerveConstraints(
            40.0,
            18.0,
            18.0,
            40.0,
            40.0,
            Angle.deg(180.0),
            Angle.deg(180.0)
    );

    // @TODO Tune swerve module offsets.
    public static Angle frontLeftOffset = Angle.deg(0);
    public static Angle backLeftOffset = Angle.deg(0);
    public static Angle backRightOffset = Angle.deg(0);
    public static Angle frontRightOffset = Angle.deg(0);

    public static final PIDCoefficients axialCoeffs = new PIDCoefficients();
    public static final PIDCoefficients lateralCoeffs = new PIDCoefficients();
    public static final PIDCoefficients headingCoeffs = new PIDCoefficients();
    private final HolonomicPIDVAFollower trajectoryFollower = new HolonomicPIDVAFollower(
            axialCoeffs, lateralCoeffs, headingCoeffs,
            new Pose2d(0.5, 0.5, Angle.deg(5)), 0.5
    );
    public static final DCMotorFeedforward feedforward = new DCMotorFeedforward();
    public static double distancePerTick = 1.0;

    public SampleSwerveDrive(HardwareMap hMap) {
        this(
                new SampleSwerveModule(
                        new AxonAngleSensor(
                                hMap.get(AnalogInput.class, "front_left_angle"),
                                frontLeftOffset, false
                        ),
                        new Motor(hMap, "front_left_motor", Motor.Type.GOBILDA_MATRIX).reversed(), //Add .reversed() as necessary
                                        new CRServo(hMap, "front_left_servo")
                ),
                new SampleSwerveModule(
                        new AxonAngleSensor(
                                hMap.get(AnalogInput.class, "back_left_angle"),
                                backLeftOffset, false
                        ),
                        new Motor(hMap, "back_left_motor", Motor.Type.GOBILDA_MATRIX).reversed(),
                        new CRServo(hMap, "back_left_servo")
                ),
                new SampleSwerveModule(
                        new AxonAngleSensor(
                                hMap.get(AnalogInput.class, "back_right_angle"),
                                backRightOffset, false
                        ),
                        new Motor(hMap, "back_right_motor", Motor.Type.GOBILDA_MATRIX),
                        new CRServo(hMap, "back_right_servo")
                ),
                new SampleSwerveModule(
                        new AxonAngleSensor(
                                hMap.get(AnalogInput.class, "front_right_angle"),
                                frontRightOffset, false
                        ),
                        new Motor(hMap, "front_right_motor", Motor.Type.GOBILDA_MATRIX),
                        new CRServo(hMap, "front_right_servo")
                )
        );
    }

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
        getMotors().setDistancePerTick(distancePerTick);
        getMotors().setFeedforward(feedforward);
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
        return constraints;
    }

    @NotNull
    @Override
    public TrajectoryFollower getTrajectoryFollower() {
        return trajectoryFollower;
    }
}