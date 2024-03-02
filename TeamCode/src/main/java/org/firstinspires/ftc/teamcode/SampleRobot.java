package org.firstinspires.ftc.teamcode;

import com.amarcolini.joos.command.Robot;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.hardware.CRServo;
import com.amarcolini.joos.hardware.IMUAngleSensor;
import com.amarcolini.joos.hardware.Motor;
import com.amarcolini.joos.hardware.MotorGroup;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import kotlin.Pair;

import java.util.List;

//TODO: change all IDs to what they are in your robot configuration
@JoosConfig
public class SampleRobot extends Robot {
    public final IMUAngleSensor headingSensor = new IMUAngleSensor(
            hMap, "imu",
            RevHubOrientationOnRobot.LogoFacingDirection.UP, // TODO: Set these to what they are on your robot
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
    );

    public static Angle frontLeftOffset = Angle.deg(355);
    public static Angle backLeftOffset = Angle.deg(357);
    public static Angle backRightOffset = Angle.deg(18);
    public static Angle frontRightOffset = Angle.deg(132);

    public final SampleSwerveDrive drive = new SampleSwerveDrive(
            new SampleSwerveModule(
                    new AxonAngleSensor(
                            hMap.get(AnalogInput.class, "front_left_angle"),
                            frontLeftOffset, false
                    ),
                    new Motor(hMap, "FrontLeftDrive", Motor.Type.GOBILDA_MATRIX).reversed(),
                    new CRServo(hMap, "frontLeftTurn")
            ),
            new SampleSwerveModule(
                    new AxonAngleSensor(
                            hMap.get(AnalogInput.class, "back_left_angle"),
                            backLeftOffset, false
                    ),
                    new Motor(hMap, "BackLeftDrive", Motor.Type.GOBILDA_MATRIX).reversed(),
                    new CRServo(hMap, "backLeftTurn")
            ),
            new SampleSwerveModule(
                    new AxonAngleSensor(
                            hMap.get(AnalogInput.class, "back_right_angle"),
                            backRightOffset, false
                    ),
                    new Motor(hMap, "BackRightDrive", Motor.Type.GOBILDA_MATRIX),
                    new CRServo(hMap, "backRightTurn")
            ),
            new SampleSwerveModule(
                    new AxonAngleSensor(
                            hMap.get(AnalogInput.class, "front_right_angle"),
                            frontRightOffset, false
                    ),
                    new Motor(hMap, "FrontRightDrive", Motor.Type.GOBILDA_MATRIX),
                    new CRServo(hMap, "frontRightTurn")
            )
    );

    @Override
    public void init() {
        register(drive);
    }
}