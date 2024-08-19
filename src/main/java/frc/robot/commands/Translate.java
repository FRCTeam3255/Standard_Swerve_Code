// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constDrivetrain;
import frc.robot.subsystems.Drivetrain;

public class Translate extends Command {
  Drivetrain subDrivetrain;
  Pose2d currentPose;
  Pose2d finalPose;
  Measure<Distance> desiredDistance;
  Measure<Angle> desiredAngle;
  final boolean isOpenLoop = false;

  Measure<Velocity<Distance>> xVelocity;
  Measure<Velocity<Distance>> yVelocity;

  /** Creates a new Translate. */
  public Translate(Drivetrain subDrivetrain, Measure<Distance> desiredDistance, Measure<Angle> desiredAngle) {
    this.subDrivetrain = subDrivetrain;
    this.desiredDistance = desiredDistance;
    this.desiredAngle = desiredAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Calculate the desired velocities without regard for distance
    xVelocity = Units.MetersPerSecond.of(Math.cos(desiredAngle.in(Units.Radians)));
    yVelocity = Units.MetersPerSecond.of(Math.sin(desiredAngle.in(Units.Radians)));

    // Calculate our final pose
    // this is where im losing it because it hangs itself if desiredAngle isnt 0
    // TODO: STOP LOSING IT
    currentPose = subDrivetrain.getPose();
    Translation2d translation = new Translation2d(desiredDistance.in(Units.Meters), new Rotation2d(desiredAngle));
    Transform2d transform = new Transform2d(translation, new Rotation2d());
    finalPose = currentPose.transformBy(transform);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("CURRENT POSE", currentPose.toString());
    SmartDashboard.putString("FINAL POSE", finalPose.toString());

    currentPose = subDrivetrain.getPose();
    subDrivetrain.drive(new Translation2d(xVelocity.in(Units.MetersPerSecond), yVelocity.in(Units.MetersPerSecond)), 0,
        isOpenLoop);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subDrivetrain.drive(new Translation2d(0, 0), 0, isOpenLoop);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distance = Math.sqrt(
        Math.pow((currentPose.getX() - finalPose.getX()), 2) + Math.pow((currentPose.getY() - finalPose.getY()), 2));
    SmartDashboard.putNumber("DISTANCE", distance);
    return distance < constDrivetrain.AT_POINT_TOLERANCE.in(Units.Meters);
  }
}
