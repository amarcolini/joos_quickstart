package org.firstinspires.ftc.teamcode;

import com.amarcolini.joos.command.Robot;
import com.amarcolini.joos.hardware.IMUAngleSensor;
import com.amarcolini.joos.hardware.Motor;
import com.amarcolini.joos.hardware.MotorGroup;
import com.amarcolini.joos.localization.AngleSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import kotlin.Pair;

public class SampleRobot extends Robot {
    public final IMUAngleSensor headingSensor = new IMUAngleSensor(
            hMap, "imu",
            RevHubOrientationOnRobot.LogoFacingDirection.UP, // TODO: Set these to what they are on your robot
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
    );

    //TODO: change all IDs to what they are in your robot configuration
    public final SampleMecanumDrive drive = new SampleMecanumDrive(
            new MotorGroup(
                    hMap,
                    Motor.Type.GOBILDA_312,
                    new Pair<>("back_left", false), // TODO: Change from false to true to reverse each motor as necessary
                    new Pair<>("back_left", false),
                    new Pair<>("back_left", false),
                    new Pair<>("back_left", false)
            ),
            headingSensor
    );
}