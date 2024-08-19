// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class ExampleAuto extends SequentialCommandGroup {
  Drivetrain subDrivetrain;

  /** Creates a new ExampleAuto. */
  public ExampleAuto(Drivetrain subDrivetrain) {
    addCommands(
        Commands.runOnce(
            () -> subDrivetrain.resetPoseToPose(new Pose2d(0, 1, new Rotation2d()))),
        new Translate(subDrivetrain, Units.Meters.of(1), Units.Degrees.of(0)));
  }
}
