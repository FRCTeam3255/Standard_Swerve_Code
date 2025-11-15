// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Contains constants related to vision processing for the robot.
 * <p>
 * This class provides standard deviation values used by the pose estimator
 * for vision data, including position (in meters) and heading (in radians).
 */
public class ConstVision {
  /**
   * <p>
   * Pose estimator standard deviation for vision data
   * <p>
   * <b>Units:</b> Meters
   */
  public static final double STD_DEVS_POS = 0.7;

  /**
   * <p>
   * Pose estimator standard deviation for vision data
   * </p>
   * <b>Units:</b> Radians
   */
  public static final double STD_DEVS_HEADING = 9999999;

}
