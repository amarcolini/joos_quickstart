package org.firstinspires.ftc.teamcode.drive.forwardKinematics;

import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.command.InstantCommand;
import com.amarcolini.joos.command.SequentialCommand;
import com.amarcolini.joos.command.WaitCommand;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.SampleBot;

@JoosConfig
@Autonomous(group = "forwardKinematics")
public class TrackWidthTuner extends CommandOpMode {
    public static final Angle ANGLE = Angle.deg(180);
    public static int NUM_TRIALS = 5;
    public static int DELAY = 1; // s

    private SampleBot robot;

    private MovingStatistics trackWidthStats;
    private Angle headingAccumulator = new Angle();
    private Angle lastHeading = new Angle();
    @Override
    public void preInit() {
        robot = registerRobot(new SampleBot());

        trackWidthStats = new MovingStatistics(NUM_TRIALS);
        telemetry.clearAll();

        schedule(
                new SequentialCommand(
                        new InstantCommand(() -> {
                            robot.drive.setPoseEstimate(new Pose2d());
                            headingAccumulator = new Angle();
                            lastHeading = new Angle();
                        }),
                        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(new Pose2d())
                                .turn(ANGLE)
                                .build()
                        ).onExecute(() -> {
                            Angle heading = robot.drive.getPoseEstimate().heading;
                            headingAccumulator = headingAccumulator.plus(heading.minus(lastHeading).normDelta());
                            lastHeading = heading;
                        }).onEnd(interrupted -> {
                            if (!interrupted)
                                trackWidthStats.add(robot.drive.getConstraints().trackWidth * ANGLE.div(headingAccumulator));
                        }),
                        new WaitCommand(DELAY)
                ).repeat(NUM_TRIALS)
                        .onEnd( interrupted -> {
                            if (interrupted) return;
                            telemetry.setAutoClear(false);
                            telemetry.addLine(Misc.formatInvariant("Effective track width = %.2f (SE = %.3f)",
                                    trackWidthStats.getMean(),
                                    trackWidthStats.getStandardDeviation() / Math.sqrt(NUM_TRIALS)));
                        })
        );

        telemetry.addLine(Misc.formatInvariant("Your robot will turn %.2f degrees %d times.", ANGLE.degrees(), NUM_TRIALS));
        telemetry.addLine("Make sure your robot has enough clearance to turn smoothly");
    }
}
