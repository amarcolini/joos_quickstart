package org.firstinspires.ftc.teamcode.tuning;

import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.command.TimeCommand;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.drive.DriveSignal;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SampleRobot;

@TeleOp(group = "Tuning")
@JoosConfig
public class TurnTest extends CommandOpMode {
    @Register
    private SampleRobot robot;
    public static Angle angle = Angle.deg(90);

    public void preInit() {
        Trajectory trajectory = robot.drive.trajectoryBuilder()
                .turn(angle)
                .build();

        new TimeCommand((t, dt) -> {
            robot.drive.setDriveSignal(
                    new DriveSignal(
                            trajectory.velocity(t),
                            trajectory.acceleration(t)
                    )
            );
            return t >= trajectory.duration();
        }).onEnd((interrupted) -> robot.drive.setDriveSignal(new DriveSignal()))
                .thenStopOpMode()
                .schedule();
    }
}