package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;


@Config
public class DriveConfig {
    // IMU orientation
    // TODO: fill in these values based on
    //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
    public static volatile RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static volatile RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // drive model parameters
    public static volatile double inPerTick = .003;
    public static volatile double lateralInPerTick = inPerTick;
    public static volatile double trackWidthTicks = 0;

    // feedforward parameters (in tick units)
    public static volatile double kS = 0;
    public static volatile double kV = 0;
    public static volatile double kA = 0;

    // path profile parameters (in inches)
    public static volatile double maxWheelVel = 50;
    public static volatile double minProfileAccel = -30;
    public static volatile double maxProfileAccel = 50;

    // turn profile parameters (in radians)
    public static volatile double maxAngVel = Math.PI; // shared with path
    public static volatile double maxAngAccel = Math.PI;

    // path controller gains
    public static volatile double axialGain = 0.0;
    public static volatile double lateralGain = 0.0;
    public static volatile double headingGain = 0.0; // shared with turn

    public static volatile double axialVelGain = 0.0;
    public static volatile double lateralVelGain = 0.0;
    public static volatile double headingVelGain = 0.0; // shared with turn


}