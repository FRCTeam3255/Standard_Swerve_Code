// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Physical constants for Swerve Modules */
public class SwerveConstants {
        private static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(3.8) * Math.PI;
        private static final double STEER_GEAR_RATIO = 150.0 / 7.0;
        public double steerGearRatio;
        public double wheelCircumference;
        public double driveGearRatio;
        public double maxSpeedMeters;

        public SwerveConstants(double steerGearRatio, double wheelCircumference, double driveGearRatio,
                        double maxSpeedMeters) {
                this.steerGearRatio = steerGearRatio;
                this.wheelCircumference = wheelCircumference;
                this.driveGearRatio = driveGearRatio;
                this.maxSpeedMeters = maxSpeedMeters;
        }

        public static final SwerveConstants MK4I_L1 = new SwerveConstants(STEER_GEAR_RATIO, WHEEL_CIRCUMFERENCE,
                        8.14, Units.feetToMeters(13.7));
        public static final SwerveConstants MK4I_L2 = new SwerveConstants(STEER_GEAR_RATIO, WHEEL_CIRCUMFERENCE,
                        6.75, Units.feetToMeters(16.5));
        public static final SwerveConstants MK4I_L3 = new SwerveConstants(STEER_GEAR_RATIO, WHEEL_CIRCUMFERENCE,
                        6.12, Units.feetToMeters(18.2));

        public static final SwerveConstants MK4I_L1_FOC = new SwerveConstants(STEER_GEAR_RATIO, WHEEL_CIRCUMFERENCE,
                        8.14, Units.feetToMeters(13.0));
        public static final SwerveConstants MK4I_L2_FOC = new SwerveConstants(STEER_GEAR_RATIO, WHEEL_CIRCUMFERENCE,
                        6.75, Units.feetToMeters(15.7));
        public static final SwerveConstants MK4I_L3_FOC = new SwerveConstants(STEER_GEAR_RATIO, WHEEL_CIRCUMFERENCE,
                        6.12, Units.feetToMeters(17.3));

}
