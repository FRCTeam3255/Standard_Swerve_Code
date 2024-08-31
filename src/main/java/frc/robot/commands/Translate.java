// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
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

  Measure<Dimensionless> percentSpeed;

  /**
   * Automatically translate the robot for a given distance, at a specified angle
   * in the Field Coordinate System, and at a percentage of our max speed.
   * 
   * @param subDrivetrain   The instance of the subsystem we are making run this
   *                        command
   * @param desiredDistance The desired distance you would like to travel in that
   *                        direction
   * @param desiredAngle    The desired angle you would like to travel in
   *                        (Field-Relative Angle)
   * @param percentSpeed    The percent of our predefined max speed to go at (± 1
   *                        meter per second)
   * 
   * @see <a href=
   *      "https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html">Field
   *      Coordinate System</a>
   */
  public Translate(Drivetrain subDrivetrain, Measure<Distance> desiredDistance, Measure<Angle> desiredAngle,
      Measure<Dimensionless> percentSpeed) {
    this.subDrivetrain = subDrivetrain;
    this.desiredDistance = desiredDistance;
    this.desiredAngle = desiredAngle;
    this.percentSpeed = percentSpeed;

    addRequirements(subDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Calculate the desired velocities without regard for distance
    xVelocity = Units.MetersPerSecond.of(Math.cos(desiredAngle.in(Units.Radians)) * percentSpeed.in(Units.Percent) * 2);
    yVelocity = Units.MetersPerSecond.of(Math.sin(desiredAngle.in(Units.Radians)) * percentSpeed.in(Units.Percent) * 2);

    // Calculate our final pose
    currentPose = new Pose2d(subDrivetrain.getPose().getTranslation(), new Rotation2d());
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
