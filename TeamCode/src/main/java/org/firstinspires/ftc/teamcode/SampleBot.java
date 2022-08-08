package org.firstinspires.ftc.teamcode;

import com.amarcolini.joos.command.Robot;
import com.amarcolini.joos.control.FeedforwardCoefficients;
import com.amarcolini.joos.control.PIDCoefficients;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.hardware.Motor;
import com.amarcolini.joos.hardware.drive.MecanumDrive;
import com.amarcolini.joos.hardware.drive.Standard3WheelLocalizer;
import com.amarcolini.joos.trajectory.constraints.MecanumConstraints;

import kotlin.Pair;

@JoosConfig
public class SampleBot extends Robot {
    public static final FeedforwardCoefficients ffcoeffs = new FeedforwardCoefficients(0.016, 0.003, 0);
    public static double FORWARD_OFFSET = -8;
    public static double xMultiplier = 0.92;
    public static double yMultiplier = 0.92;
    public static double lateralDistance = 14;
    public static double trackWidth = 14;
    public static final PIDCoefficients HEADING_PID = new PIDCoefficients(100, 0, 0.5, 3);
    public static final PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0);

    public final MecanumDrive drive;

    public SampleBot() {
        drive = new MecanumDrive(
                new Motor(hMap, "left_front", Motor.Kind.GOBILDA_312, 2).reversed(),
                new Motor(hMap, "left_back", Motor.Kind.GOBILDA_312, 2),
                new Motor(hMap, "right_back", Motor.Kind.GOBILDA_312, 2).reversed(),
                new Motor(hMap, "right_front", Motor.Kind.GOBILDA_312, 2),
                null, new MecanumConstraints(
                        trackWidth, trackWidth, 1.0,
                40.0, 40.0, Angle.deg(180),
                Angle.deg(180)
                ), TRANSLATIONAL_PID, HEADING_PID
        );
        drive.setLocalizer(new Standard3WheelLocalizer(
                Motor.Encoder.multiple(hMap, 8192, 0.75,
                        new Pair<>("right_front", false),
                        new Pair<>("left_back", false),
                        new Pair<>("left_front", false)
                ),
                lateralDistance, FORWARD_OFFSET, xMultiplier, yMultiplier
        ));
        drive.motors.resetEncoder();
        drive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        drive.motors.setFeedforwardCoefficients(ffcoeffs);
        register(drive);
    }

    @Override
    public void init() {
    }

    @Override
    public void start() {
    }
}
