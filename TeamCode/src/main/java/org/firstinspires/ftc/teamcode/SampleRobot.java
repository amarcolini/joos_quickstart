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

//TODO: change all IDs to what they are in your robot configuration
@JoosConfig
public class SampleRobot extends Robot {
    public final IMUAngleSensor headingSensor = new IMUAngleSensor(
            hMap, "imu",
            RevHubOrientationOnRobot.LogoFacingDirection.UP, // TODO: Set these to what they are on your robot
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
    );

    public static Angle frontLeftOffset = Angle.deg(0);
    public static Angle backLeftOffset = Angle.deg(0);
    public static Angle backRightOffset = Angle.deg(0);
    public static Angle frontRightOffset = Angle.deg(0);

    public final SampleSwerveDrive drive = new SampleSwerveDrive(
            new SampleSwerveModule(
                    new AxonAngleSensor(
                            hMap.get(AnalogInput.class, "front_left_angle"),
                            frontLeftOffset, false
                    ),
                    new Motor(hMap, "FrontLeftDrive", 500, 1),
                    new CRServo(hMap, "frontLeftTurn")
            ),
            new SampleSwerveModule(
                    new AxonAngleSensor(
                            hMap.get(AnalogInput.class, "back_left_angle"),
                            backLeftOffset, false
                    ),
                    new Motor(hMap, "BackLeftDrive", 500, 1),
                    new CRServo(hMap, "backLeftTurn")
            ),
            new SampleSwerveModule(
                    new AxonAngleSensor(
                            hMap.get(AnalogInput.class, "back_right_angle"),
                            backRightOffset, false
                    ),
                    new Motor(hMap, "BackRightDrive", 500, 1),
                    new CRServo(hMap, "backRightTurn")
            ),
            new SampleSwerveModule(
                    new AxonAngleSensor(
                            hMap.get(AnalogInput.class, "front_right_angle"),
                            frontRightOffset, false
                    ),
                    new Motor(hMap, "FrontRightDrive", 500, 1),
                    new CRServo(hMap, "frontRightTurn")
            )
    );

    @Override
    public void init() {
        register(drive);
    }
}