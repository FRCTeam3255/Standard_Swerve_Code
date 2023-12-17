// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Physical constants for Swerve Modules */
public class SwerveConstants {
    public double steerGearRatio;
    public double wheelCircumference;
    public double driveGearRatio;
    public double maxSpeed;

    public SwerveConstants(double steerGearRatio, double wheelCircumference, double driveGearRatio, double maxSpeed) {
        this.steerGearRatio = steerGearRatio;
        this.wheelCircumference = wheelCircumference;
        this.driveGearRatio = driveGearRatio;
        this.maxSpeed = maxSpeed;
    }

    public static final SwerveConstants MK4I_L1 = new SwerveConstants(150.0 / 7.0, 0.30322652292, 8.14, 13.7);
    public static final SwerveConstants MK4I_L2 = new SwerveConstants(150.0 / 7.0, 0.30322652292, 6.75, 16.5);
    public static final SwerveConstants MK4I_L3 = new SwerveConstants(150.0 / 7.0, 0.30322652292, 6.12, 18.2);

    public static final SwerveConstants MK4I_L1_FOC = new SwerveConstants(150.0 / 7.0, 0.30322652292, 8.14, 13.0);
    public static final SwerveConstants MK4I_L2_FOC = new SwerveConstants(150.0 / 7.0, 0.30322652292, 6.75, 15.7);
    public static final SwerveConstants MK4I_L3_FOC = new SwerveConstants(150.0 / 7.0, 0.30322652292, 6.12, 17.3);

}
