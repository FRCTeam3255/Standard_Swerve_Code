// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstSystem;

@Logged
public class RobotPoses extends SubsystemBase {
  /** Creates a new RobotPoses. */
  Field2d field2d = new Field2d();
  FieldObject2d robotObject = field2d.getObject("Robot");

  Pose3d comp0Drivetrain = Pose3d.kZero;
  Pose3d comp1Bumpers = Pose3d.kZero.plus(ConstSystem.ROBOT_TO_BUMPERS);

  public RobotPoses() {

    SmartDashboard.putData("Field", field2d);
  }

  @Override
  public void periodic() {
    robotObject.setPose(RobotContainer.drivetrainInstance.getPose());
    // This method will be called once per scheduler run

    // Robot Positions
    comp0Drivetrain = new Pose3d(RobotContainer.drivetrainInstance.getPose());
  }
}
