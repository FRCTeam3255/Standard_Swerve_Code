// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.SwerveConstants;

public class SuperSwerve extends SubsystemBase {
  private SwerveModule[] modules;
  private SwerveDrivePoseEstimator swervePoseEstimator;
  private SwerveDriveKinematics swerveKinematics;
  public SwerveAutoBuilder swerveAutoBuilder;
  private Pigeon2 pigeon;
  private boolean isFieldRelative;

  private SwerveConstants swerveConstants;
  private PIDConstants autoDrivePID;
  private PIDConstants autoSteerPID;
  private Matrix<N3, N1> stateStdDevs;
  private Matrix<N3, N1> visionStdDevs;
  private boolean autoFlipWithAllianceColor;
  public HashMap<String, Command> autoEventMap = new HashMap<>();

  public PathPlannerTrajectory exampleAuto;

  double[] swerveRealStates = new double[8];
  double[] swerveDesiredStates = new double[8];
  double simAngle = 0;
  SwerveModuleState[] lastDesiredStates;
  private double timeFromLastUpdate;
  private double lastSimTime;
  private Timer simTimer;
  private Field2d field;

  /**
   * Create a new Swerve Drive!
   * 
   * @param swerveConstants           The constants for the modules, such as the
   *                                  gear ratio and max speed
   * @param driveConfig               - The configuration for every drive motor.
   *                                  <br>
   *                                  <b>IMPORTANT: Configure PID and
   *                                  FeedForward!</b>
   * @param steerConfig               - The configuration for every steer motor.
   *                                  <br>
   *                                  <b>IMPORTANT: Configure PID!</b>
   * @param modules
   * @param wheelbase                 Physically measured distance between Left &
   *                                  Right Wheels
   * @param trackWidth                Physically measured distance between Front &
   *                                  Back Wheels
   * @param CANBusName
   * @param pigeonCANId               The pigeon MUST be on the same CANBus as the
   *                                  modules
   * @param minimumSteerPercent
   * @param isDriveInverted
   * @param isSteerInverted
   * @param driveNeutralMode          The behavior of the drive motor when set to
   *                                  neutral-output
   * @param steerNeutralMode          The behavior of the steer motor when set to
   *                                  neutral-output
   * @param stateStdDevs              Standard deviations of the pose estimate (x
   *                                  position in meters, y position
   *                                  in meters, and heading in radians). Increase
   *                                  these numbers to trust your state estimate
   *                                  less.
   * @param visionStdDevs             Standard deviations of the vision pose
   *                                  measurement (x position
   *                                  in meters, y position in meters, and heading
   *                                  in radians). Increase these numbers to trust
   *                                  the vision pose measurement less.
   * @param autoDrivePID
   * @param autoSteerPID
   * @param autoFlipWithAllianceColor
   */
  public SuperSwerve(
      SwerveConstants swerveConstants,
      SwerveModule[] modules,
      double wheelbase,
      double trackWidth,
      String CANBusName, int pigeonCANId,
      double minimumSteerPercent,
      boolean isDriveInverted, boolean isSteerInverted,
      NeutralMode driveNeutralMode, NeutralMode steerNeutralMode,
      Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> visionStdDevs,
      PIDConstants autoDrivePID, PIDConstants autoSteerPID,
      boolean autoFlipWithAllianceColor) {

    if (Robot.isSimulation()) {
      simTimer = new Timer();
      simTimer.start();
      lastSimTime = simTimer.get();
      timeFromLastUpdate = 0;
    }

    isFieldRelative = true;
    field = new Field2d();

    // Location of all modules in the WPILib robot coordinate system
    swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelbase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelbase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelbase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelbase / 2.0, -trackWidth / 2.0));

    this.modules = modules;
    this.stateStdDevs = stateStdDevs;
    this.visionStdDevs = visionStdDevs;
    this.autoFlipWithAllianceColor = autoFlipWithAllianceColor;
    this.swerveConstants = swerveConstants;
    this.autoDrivePID = autoDrivePID;
    this.autoSteerPID = autoSteerPID;

    SwerveModule.driveGearRatio = swerveConstants.driveGearRatio;

    SwerveModule.CANBusName = CANBusName;
    SwerveModule.minimumSteerSpeedPercent = minimumSteerPercent;
    SwerveModule.wheelCircumference = swerveConstants.wheelCircumference;
    SwerveModule.maxModuleSpeed = swerveConstants.maxSpeed;

    SwerveModule.isDriveInverted = isDriveInverted;
    SwerveModule.driveNeutralMode = driveNeutralMode;
    SwerveModule.driveGearRatio = swerveConstants.driveGearRatio;

    SwerveModule.isSteerInverted = isSteerInverted;
    SwerveModule.steerNeutralMode = steerNeutralMode;
    SwerveModule.steerGearRatio = swerveConstants.steerGearRatio;

    pigeon = new Pigeon2(pigeonCANId, CANBusName);

    // The absolute encoders need time to initialize
    Timer.delay(2.5);
    resetModulesToAbsolute();
    configure();
  }

  public void configure() {

    for (SwerveModule mod : modules) {
      mod.configure();
    }
    swervePoseEstimator = new SwerveDrivePoseEstimator(
        swerveKinematics,
        getRotation(),
        getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionStdDevs);

    swerveAutoBuilder = new SwerveAutoBuilder(
        this::getPose,
        this::resetPoseToPose,
        swerveKinematics,
        autoDrivePID,
        autoSteerPID,
        this::setModuleStatesAuto,
        autoEventMap,
        autoFlipWithAllianceColor,
        this);

  }

  /**
   * Reset all of the steer motors to the absolute encoder values.
   */
  public void resetModulesToAbsolute() {
    for (SwerveModule mod : modules) {
      mod.resetSteerMotorToAbsolute();
    }
  }

  /**
   * Get the position (distance, angle) of each module.
   * 
   * @return An Array of Swerve module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (SwerveModule mod : modules) {
      positions[mod.moduleNumber] = mod.getModulePosition();
    }

    return positions;
  }

  /**
   * Set the state of the modules
   * 
   * @param desiredModuleStates Desired states to set the modules to
   * @param isOpenLoop          Are the modules being set based on open loop or
   *                            closed loop control
   * 
   */
  public void setModuleStates(SwerveModuleState[] desiredModuleStates, boolean isOpenLoop) {
    lastDesiredStates = desiredModuleStates;

    // Lowers the speeds so that they are actually achievable
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, swerveConstants.maxSpeed);

    for (SwerveModule mod : modules) {
      mod.setModuleState(desiredModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /**
   * Set the state of the modules in autonomous. Always set with open-loop
   * control.
   * 
   * @param desiredModuleStates Desired states to set the modules to
   * 
   */
  private void setModuleStatesAuto(SwerveModuleState[] desiredModuleStates) {
    setModuleStates(desiredModuleStates, false);
  }

  /**
   * Drive the drivetrain
   * 
   * @param translation Desired translational velocity in meters per second
   * @param rotation    Desired rotational velocity in radians per second
   * @param isOpenLoop  Are the modules being set based on open loop or closed
   *                    loop control
   * 
   */
  public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
    ChassisSpeeds chassisSpeeds;

    if (isFieldRelative) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          translation.getX(),
          translation.getY(),
          rotation,
          getRotation());
    } else {
      chassisSpeeds = new ChassisSpeeds(
          translation.getX(),
          translation.getY(),
          rotation);
    }

    SwerveModuleState[] desiredModuleStates = swerveKinematics.toSwerveModuleStates(discretize(chassisSpeeds));
    setModuleStates(desiredModuleStates, isOpenLoop);
  }

  /**
   * This is the most theoretical thing that is in the code.
   * It takes our current position and then adds an offset to it, knowing that the
   * robot's estimated position
   * is not following the exact position of the robot.
   * 
   * @param speeds the speeds about to be inputted into the robot.
   * @return the same thing as we input.
   *         Think of this method as an interceptor,
   *         not changing the parameter but using it for calculations.
   */
  /**
   * Credit: WPIlib 2024 and 4738
   * Discretizes a continuous-time chassis speed.
   *
   * @param vx    Forward velocity.
   * @param vy    Sideways velocity.
   * @param omega Angular velocity.
   */
  public ChassisSpeeds discretize(ChassisSpeeds speeds) {
    double dt = 0.02;

    var desiredDeltaPose = new Pose2d(
        speeds.vxMetersPerSecond * dt,
        speeds.vyMetersPerSecond * dt,
        new Rotation2d(speeds.omegaRadiansPerSecond * dt * 4));

    var twist = new Pose2d().log(desiredDeltaPose);

    return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
  }

  /**
   * Sets all modules to neutral output
   */
  public void neutralDriveOutputs() {
    for (SwerveModule mod : modules) {
      mod.neutralDriveOutput();
    }
  }

  /**
   * Set the drive method to use field relative drive controls
   */
  public void setFieldRelative() {
    isFieldRelative = true;
  }

  /**
   * Set the drive method to use robot relative drive controls
   */
  public void setRobotRelative() {
    isFieldRelative = false;
  }

  /**
   * Updates the pose estimator with the current robot uptime, the gyro yaw, and
   * each swerve module position.
   * <p>
   * This method MUST be called every loop (or else pose estimator breaks)
   */
  public void updatePoseEstimator() {
    swervePoseEstimator.updateWithTime(
        Timer.getFPGATimestamp(),
        getRotation(),
        getModulePositions());
  }

  /**
   * Return the current estimated pose from the pose estimator.
   * 
   * @return The current estimated pose
   */
  public Pose2d getPose() {
    return swervePoseEstimator.getEstimatedPosition();
  }

  /**
   * Reset the pose estimator's pose to a given pose.
   * 
   * @param pose The pose you would like to reset the pose estimator to
   */
  public void resetPoseToPose(Pose2d pose) {
    swervePoseEstimator.resetPosition(getRotation(), getModulePositions(), pose);
  }

  /**
   * Get the rotation of the drivetrain using the Pigeon.
   * 
   * @return Rotation of drivetrain, stored as Radians
   */
  public Rotation2d getRotation() {
    if (Robot.isSimulation() && lastDesiredStates != null) {
      timeFromLastUpdate = simTimer.get() - lastSimTime;
      lastSimTime = simTimer.get();
      simAngle += swerveKinematics.toChassisSpeeds(lastDesiredStates).omegaRadiansPerSecond * timeFromLastUpdate;
      return new Rotation2d(simAngle);
    }
    return Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(pigeon.getYaw())));
  }

  /**
   * Resets the Yaw of the Pigeon to 0
   */
  public void resetYaw() {
    pigeon.setYaw(0);
  }

  /**
   * Resets the Yaw of the Pigeon to the given value
   * 
   * @param yaw The yaw (in degrees) to reset the Pigeon to
   */
  public void resetYaw(double yaw) {
    pigeon.setYaw(yaw);
  }

  @Override
  public void periodic() {
    updatePoseEstimator();
    SmartDashboard.putNumber("Drivetrain/Yaw Degrees", getRotation().getDegrees());
    SmartDashboard.putBoolean("Drivetrain/Is Field Relative", isFieldRelative);
    for (SwerveModule mod : modules) {
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Speed",
          Units.metersToFeet(mod.getModuleState().speedMetersPerSecond));
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Distance",
          Units.metersToFeet(mod.getModulePosition().distanceMeters));
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Angle",
          mod.getModuleState().angle.getDegrees());
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Absolute Encoder Angle (WITH OFFSET)",
          mod.getAbsoluteEncoder());
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Absolute Encoder Raw Value",
          mod.getRawAbsoluteEncoder());
    }

    field.setRobotPose(getPose());
    SmartDashboard.putData(field);
  }
}