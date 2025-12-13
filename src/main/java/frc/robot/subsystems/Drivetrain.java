// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.frcteam3255.components.swerve.SN_SuperSwerve;
import com.frcteam3255.components.swerve.SN_SwerveModule;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap.mapDrivetrain;
import frc.robot.constants.ConstDrivetrain;
import frc.robot.constants.ConstField;
import frc.robot.constants.ConstVision;
import frc.robot.constants.ConstPoseDrive.PoseDriveGroup;

@Logged
public class Drivetrain extends SN_SuperSwerve {
  StructPublisher<Pose2d> robotPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("/SmartDashboard/Drivetrain/Robot Pose", Pose2d.struct).publish();

  public PoseDriveGroup lastDesiredPoseGroup;
  public Pose2d lastDesiredTarget;
  public Orchestra badApple = new Orchestra();

  // TODO: VERY CRUSIAL!!!!! Set this based on battery location
  public static boolean isBatteryFront = true;

  private static int FRONT_LEFT_MODULE = isBatteryFront ? 3 : 0;
  private static int FRONT_RIGHT_MODULE = isBatteryFront ? 2 : 1;
  private static int BACK_LEFT_MODULE = isBatteryFront ? 1 : 2;
  private static int BACK_RIGHT_MODULE = isBatteryFront ? 0 : 3;

  private static SN_SwerveModule[] modules = new SN_SwerveModule[] {
      new SN_SwerveModule(FRONT_LEFT_MODULE, mapDrivetrain.FRONT_LEFT_DRIVE_CAN, mapDrivetrain.FRONT_LEFT_STEER_CAN,
          mapDrivetrain.FRONT_LEFT_ABSOLUTE_ENCODER_CAN, ConstDrivetrain.FRONT_LEFT_ABS_ENCODER_OFFSET,
          mapDrivetrain.CAN_BUS_NAME),
      new SN_SwerveModule(FRONT_RIGHT_MODULE, mapDrivetrain.FRONT_RIGHT_DRIVE_CAN, mapDrivetrain.FRONT_RIGHT_STEER_CAN,
          mapDrivetrain.FRONT_RIGHT_ABSOLUTE_ENCODER_CAN, ConstDrivetrain.FRONT_RIGHT_ABS_ENCODER_OFFSET,
          mapDrivetrain.CAN_BUS_NAME),
      new SN_SwerveModule(BACK_LEFT_MODULE, mapDrivetrain.BACK_LEFT_DRIVE_CAN, mapDrivetrain.BACK_LEFT_STEER_CAN,
          mapDrivetrain.BACK_LEFT_ABSOLUTE_ENCODER_CAN, ConstDrivetrain.BACK_LEFT_ABS_ENCODER_OFFSET,
          mapDrivetrain.CAN_BUS_NAME),
      new SN_SwerveModule(BACK_RIGHT_MODULE, mapDrivetrain.BACK_RIGHT_DRIVE_CAN, mapDrivetrain.BACK_RIGHT_STEER_CAN,
          mapDrivetrain.BACK_RIGHT_ABSOLUTE_ENCODER_CAN, ConstDrivetrain.BACK_RIGHT_ABS_ENCODER_OFFSET,
          mapDrivetrain.CAN_BUS_NAME),
  };

  public Drivetrain() {
    super(
        ConstDrivetrain.SWERVE_CONSTANTS,
        modules,
        ConstDrivetrain.WHEELBASE,
        ConstDrivetrain.TRACK_WIDTH,
        mapDrivetrain.CAN_BUS_NAME,
        mapDrivetrain.PIGEON_CAN,
        ConstDrivetrain.MIN_STEER_PERCENT,
        ConstDrivetrain.DRIVE_CONFIG,
        ConstDrivetrain.STEER_CONFIG,
        ConstDrivetrain.CANCODER_CONFIG,
        VecBuilder.fill(
            ConstDrivetrain.MEASUREMENT_STD_DEVS_POS,
            ConstDrivetrain.MEASUREMENT_STD_DEVS_POS,
            ConstDrivetrain.MEASUREMENT_STD_DEV_HEADING),
        VecBuilder.fill(
            ConstVision.STD_DEVS_POS,
            ConstVision.STD_DEVS_POS,
            ConstVision.STD_DEVS_HEADING),
        ConstDrivetrain.AUTO.AUTO_DRIVE_PID,
        ConstDrivetrain.AUTO.AUTO_STEER_PID,
        ConstDrivetrain.AUTO_ALIGN.POSE_AUTO_ALIGN_CONTROLLER,
        ConstDrivetrain.TURN_SPEED,
        ConstDrivetrain.AUTO.ROBOT_CONFIG,
        () -> ConstField.isRedAlliance(),
        Robot.isSimulation());

    badApple.addInstrument(modules[0].driveMotor);
    var status = badApple.loadMusic("src/main/java/frc/robot/BadApple.chrp");

    if (!status.isOK()) {
      System.out.println("Failed to load bad apple music: " + status);
    }
  }

  /**
   * Determines whether the robot is within the specified auto-drive zone based on
   * the distance
   * to a target pose.
   *
   * @param autoDriveMaxDistance The maximum allowable distance for the auto-drive
   *                             zone. If null,
   *                             the method will return false.
   * @param target               The target pose to calculate the distance from
   *                             the robot's current pose.
   * @return True if the robot's current pose is within the specified maximum
   *         distance from the target pose,
   *         false otherwise.
   */
  public boolean isInAutoDriveZone(Distance autoDriveMaxDistance, Pose2d target) {
    if (autoDriveMaxDistance == null) {
      return false;
    }
    Distance distanceFromPose = Units.Meters
        .of(getPose().getTranslation().getDistance(target.getTranslation()));
    return distanceFromPose.lt(autoDriveMaxDistance);
  }

  /**
   * Checks if the drivetrain is at the last desired field position.
   * 
   * This method verifies whether the drivetrain has reached the last desired
   * target position and orientation within the specified tolerances. If no
   * target or pose group has been set, it will return false.
   * 
   * @return {@code true} if the drivetrain is at the last desired target position
   *         and orientation within the defined tolerances, {@code false}
   *         otherwise.
   */
  public boolean atLastDesiredFieldPosition() {
    if (lastDesiredTarget == null || lastDesiredPoseGroup == null) {
      return false;
    }
    return isAtPosition(lastDesiredTarget, lastDesiredPoseGroup.distanceTolerance)
        && isAtRotation(lastDesiredTarget.getRotation(), lastDesiredPoseGroup.rotationTolerance);
  }

  /**
   * Follows a trajectory by calculating the desired chassis speeds based on the
   * current pose
   * of the robot and the target pose provided in the trajectory sample.
   *
   * @param sample The trajectory sample containing the desired target pose and
   *               other relevant data.
   *               This is used to determine the robot's next movement.
   */
  public void followTrajectory(SwerveSample sample) {
    // Get the current pose of the robot
    Pose2d desiredTarget = sample.getPose();
    ChassisSpeeds automatedDTVelocity = ConstDrivetrain.AUTO_ALIGN.PATH_AUTO_ALIGN_CONTROLLER.calculate(getPose(),
        desiredTarget, 0,
        desiredTarget.getRotation());

    // Apply the generated speeds
    if (ConstDrivetrain.INVERT_ROTATION) {
      automatedDTVelocity.omegaRadiansPerSecond = -automatedDTVelocity.omegaRadiansPerSecond;
    }
    drive(automatedDTVelocity, true);
  }

  @Override
  public void configure() {
    SN_SwerveModule.driveConfiguration = ConstDrivetrain.DRIVE_CONFIG;
    SN_SwerveModule.steerConfiguration = ConstDrivetrain.STEER_CONFIG;
    SN_SwerveModule.cancoderConfiguration = ConstDrivetrain.CANCODER_CONFIG;
    super.configure();
  }

  public Angle getRotationMeasure() {
    return Units.Degrees.of(getRotation().getDegrees());
  }

  @Override
  public void periodic() {
    super.periodic();

    for (SN_SwerveModule mod : modules) {
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Desired Speed (FPS)",
          Units.Meters.convertFrom(Math.abs(getDesiredModuleStates()[mod.moduleNumber].speedMetersPerSecond),
              Units.Feet));
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Actual Speed (FPS)",
          Units.Meters.convertFrom(Math.abs(getActualModuleStates()[mod.moduleNumber].speedMetersPerSecond),
              Units.Feet));

      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Desired Angle (Degrees)",
          Math.abs(
              Units.Meters.convertFrom(getDesiredModuleStates()[mod.moduleNumber].angle.getDegrees(), Units.Feet)));
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Actual Angle (Degrees)",
          Math.abs(Units.Meters.convertFrom(getActualModuleStates()[mod.moduleNumber].angle.getDegrees(), Units.Feet)));

      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Offset Absolute Encoder Angle (Rotations)",
          mod.getAbsoluteEncoder());
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Absolute Encoder Raw Value (Rotations)",
          mod.getRawAbsoluteEncoder());
    }

    field.setRobotPose(getPose());
    robotPosePublisher.set(getPose());

    SmartDashboard.putData(field);
    SmartDashboard.putNumber("Drivetrain/Rotation", getRotationMeasure().in(Units.Degrees));
  }
}
