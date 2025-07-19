// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotPoses extends SubsystemBase {
  /** Creates a new RobotPoses. */

  Drivetrain subDrivetrain;

  Pose3d comp0Drivetrain = Pose3d.kZero;

  public RobotPoses(Drivetrain subDrivetrain) {
    this.subDrivetrain = subDrivetrain;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Robot Positions
    comp0Drivetrain = new Pose3d(subDrivetrain.getPose());

  }
}
