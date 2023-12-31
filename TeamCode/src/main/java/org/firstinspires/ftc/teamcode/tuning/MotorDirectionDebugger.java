package org.firstinspires.ftc.teamcode.tuning;

import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SampleRobot;

import java.util.Arrays;
import java.util.List;
import java.util.function.Function;

@TeleOp(group = "Tuning")
public class MotorDirectionDebugger extends CommandOpMode {
    @Register
    private SampleRobot robot;

    @Override
    public void preInit() {
        List<Function<GamepadEx, Boolean>> buttonMap = Arrays.asList(
                (p1) -> p1.x0.isActive(),
                (p1) -> p1.a0.isActive(),
                (p1) -> p1.b0.isActive(),
                (p1) -> p1.y0.isActive()
        );

        schedule(true, () -> {
            for (int i = 0; i < robot.drive.motorGroup.size(); i++) {
                robot.drive.motorGroup.get(i).setPower(
                        buttonMap.get(i).apply(gamepad().p1) ? 1.0 : 0.0
                );
            }
        });
    }
}