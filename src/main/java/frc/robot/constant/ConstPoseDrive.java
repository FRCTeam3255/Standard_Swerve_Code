// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constant;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.DriverStateMachine.DriverState;

/** Add your docs here. */
public class ConstPoseDrive {
  public static class PoseDriveGroup {
    public Distance minDistanceBeforeDrive;
    public List<Pose2d> targetPoseGroup;
    public DriverState driveState;
    public DriverState snapState;
    public boolean lockX;
    public boolean lockY;
    public Distance distanceTolerance = Units.Inches.of(0);
    public Angle rotationTolerance = Units.Degrees.of(0);
  }

  public static final PoseDriveGroup EXAMPLE_POSE_DRIVE_GROUP = new PoseDriveGroup();

  static {
    EXAMPLE_POSE_DRIVE_GROUP.minDistanceBeforeDrive = Units.Inches.of(12);
    EXAMPLE_POSE_DRIVE_GROUP.targetPoseGroup = ConstField.FieldElementGroups.RESET_POSE_SET.getAll();
    EXAMPLE_POSE_DRIVE_GROUP.driveState = DriverState.EXAMPLE_POSE_DRIVE;
    EXAMPLE_POSE_DRIVE_GROUP.snapState = DriverState.EXAMPLE_ROTATION_SNAP;
    EXAMPLE_POSE_DRIVE_GROUP.lockX = false;
    EXAMPLE_POSE_DRIVE_GROUP.lockY = false;
    EXAMPLE_POSE_DRIVE_GROUP.distanceTolerance = Units.Inches.of(6);
    EXAMPLE_POSE_DRIVE_GROUP.rotationTolerance = Units.Degrees.of(5);
  }
}
