package org.firstinspires.ftc.teamcode;

import com.amarcolini.joos.command.Robot;
import com.amarcolini.joos.hardware.IMUAngleSensor;
import com.amarcolini.joos.hardware.Motor;
import com.amarcolini.joos.hardware.MotorGroup;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import kotlin.Pair;

//TODO: change all IDs to what they are in your robot configuration
public class SampleRobot extends Robot {
    public final IMUAngleSensor headingSensor = new IMUAngleSensor(
            hMap, "imu",
            RevHubOrientationOnRobot.LogoFacingDirection.UP, // TODO: Set these to what they are on your robot
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
    );

    public final SampleMecanumDrive drive = new SampleMecanumDrive(
            new MotorGroup(
                    hMap,
                    Motor.Type.GOBILDA_312,
                    new Pair<>("front_left", true), // TODO: Change from false to true to reverse each motor as necessary
                    new Pair<>("back_left", true),
                    new Pair<>("back_right", false),
                    new Pair<>("front_right", false)
            ),
            headingSensor
    );
}