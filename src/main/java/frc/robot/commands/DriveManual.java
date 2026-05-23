// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstDrivetrain;
import frc.robot.constants.ConstField;
import frc.robot.subsystems.DriverStateMachine.DriverState;

public class DriveManual extends Command {
  DoubleSupplier xAxis, yAxis, rotationAxis;
  boolean isOpenLoop;
  BooleanSupplier slowMode;

  public DriveManual(DoubleSupplier xAxis, DoubleSupplier yAxis,
      DoubleSupplier rotationAxis, BooleanSupplier slowMode) {

    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    this.slowMode = slowMode;

    isOpenLoop = true;

    addRequirements(RobotContainer.driverStateMachineInstance);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    ChassisSpeeds velocities = RobotContainer.drivetrainInstance.calculateVelocitiesFromInput(
        xAxis,
        yAxis,
        rotationAxis,
        slowMode,
        ConstField.isRedAlliance(),
        ConstDrivetrain.SLOW_MODE_MULTIPLIER,
        ConstDrivetrain.REAL_DRIVE_SPEED,
        ConstDrivetrain.TURN_SPEED);

    RobotContainer.driverStateMachineInstance.setDriverState(DriverState.MANUAL);

    RobotContainer.drivetrainInstance.drive(velocities);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
