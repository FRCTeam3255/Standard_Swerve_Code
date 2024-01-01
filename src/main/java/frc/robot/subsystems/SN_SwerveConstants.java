// Note to future readers!
// Do not copy paste this file into your code! Instead, import the SuperCORE library into your code and reference it from there.
// This repository is for simulating changes to SuperCORE prior to updating the library.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;

/**
 * <p>
 * A wrapper for a swerve module's physical constants. Contains preset constants
 * as well as a constructor for custom constants.
 * </p>
 * <p>
 * Currently, there are presets for all MK4I Falcon 500 modules, with and
 * without FOC. Values are taken directly from their website.
 * (https://www.swervedrivespecialties.com/products/mk4i-swerve-module)
 * </p>
 */
public class SN_SwerveConstants {
	private static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(3.8) * Math.PI;
	private static final double STEER_GEAR_RATIO = 150.0 / 7.0;
	public double steerGearRatio;
	public double wheelCircumference;
	public double driveGearRatio;
	public double maxSpeedMeters;

	/**
	 * A wrapper for Swerve Module constants. You can create your own custom
	 * constants using this constructor.
	 *
	 * @param steerGearRatio
	 *                           The gear ratio between the wheel and the steer
	 *                           motor
	 * @param wheelCircumference
	 *                           The circumference of the wheel, in meters
	 * @param driveGearRatio
	 *                           The gear ratio between the wheel and the drive
	 *                           motor
	 * @param maxSpeedMeters
	 *                           The maximum speed of the module, in meters per
	 *                           second
	 */
	public SN_SwerveConstants(double steerGearRatio, double wheelCircumference, double driveGearRatio,
			double maxSpeedMeters) {
		this.steerGearRatio = steerGearRatio;
		this.wheelCircumference = wheelCircumference;
		this.driveGearRatio = driveGearRatio;
		this.maxSpeedMeters = maxSpeedMeters;
	}

	/**
	 * Preset Constants for a MK4I with L1 gearing and Falcon 500 Motors
	 */
	public static final SN_SwerveConstants MK4I_L1 = new SN_SwerveConstants(STEER_GEAR_RATIO, WHEEL_CIRCUMFERENCE, 8.14,
			Units.feetToMeters(13.7));

	/**
	 * Preset Constants for a MK4I with L2 gearing and Falcon 500 Motors
	 */
	public static final SN_SwerveConstants MK4I_L2 = new SN_SwerveConstants(STEER_GEAR_RATIO, WHEEL_CIRCUMFERENCE, 6.75,
			Units.feetToMeters(16.5));

	/**
	 * Preset Constants for a MK4I with L3 gearing and Falcon 500 Motors
	 */
	public static final SN_SwerveConstants MK4I_L3 = new SN_SwerveConstants(STEER_GEAR_RATIO, WHEEL_CIRCUMFERENCE, 6.12,
			Units.feetToMeters(18.2));

	/**
	 * Preset Constants for a MK4I with L1 gearing, Falcon 500 Motors, and FOC
	 * (Field Oriented Control, a Phoenix Pro Feature) enabled
	 */
	public static final SN_SwerveConstants MK4I_L1_FOC = new SN_SwerveConstants(STEER_GEAR_RATIO, WHEEL_CIRCUMFERENCE,
			8.14, Units.feetToMeters(13.0));
	/**
	 * Preset Constants for a MK4I with L2 gearing, Falcon 500 Motors, and FOC
	 * (Field Oriented Control, a Phoenix Pro Feature) enabled
	 */
	public static final SN_SwerveConstants MK4I_L2_FOC = new SN_SwerveConstants(STEER_GEAR_RATIO, WHEEL_CIRCUMFERENCE,
			6.75, Units.feetToMeters(15.7));
	/**
	 * Preset Constants for a MK4I with L3 gearing, Falcon 500 Motors, and FOC
	 * (Field Oriented Control, a Phoenix Pro Feature) enabled
	 */
	public static final SN_SwerveConstants MK4I_L3_FOC = new SN_SwerveConstants(STEER_GEAR_RATIO, WHEEL_CIRCUMFERENCE,
			6.12, Units.feetToMeters(17.3));

}
