// Note to future readers!
// Do not copy paste this file into your code! Instead, import the SuperCORE library into your code and reference it from there.
// This repository is for simulating changes to SuperCORE prior to updating the library.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

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

public class SN_SuperSwerve extends SubsystemBase {
	private SN_SwerveModule[] modules;
	private SwerveDrivePoseEstimator swervePoseEstimator;
	private SwerveDriveKinematics swerveKinematics;
	private Pigeon2 pigeon;
	private boolean isFieldRelative;

	private SN_SwerveConstants swerveConstants;
	/**
	 * Drive base radius in meters. Distance from robot center to furthest module.
	 */
	private double driveBaseRadius;
	private PIDConstants autoDrivePID;
	private PIDConstants autoSteerPID;
	private Matrix<N3, N1> stateStdDevs;
	private Matrix<N3, N1> visionStdDevs;
	public HashMap<String, Command> autoEventMap = new HashMap<>();
	private ReplanningConfig autoReplanningConfig;

	public PathPlannerTrajectory exampleAuto;

	private boolean isSimulation;
	private double simAngle = 0;
	private SwerveModuleState[] lastDesiredStates;
	private double timeFromLastUpdate = 0;
	private Timer simTimer = new Timer();
	private double lastSimTime = simTimer.get();
	private Field2d field;

	// TODO: Create Example Implementation
	/**
	 * <p>
	 * A superclass for a Swerve Drive. Your Drivetrain subsystem should extend from
	 * this class. View an example implementation here: REALLY COOL LINK HERE
	 * </p>
	 * Derived from https://github.com/ACat701/SuperSwerve23
	 *
	 * @param swerveConstants
	 *                             The constants for all of your modules, such as
	 *                             gear ratio and max
	 *                             speed. You can create your own SN_SwerveConstants
	 *                             object or use a
	 *                             preset.
	 * @param modules
	 *                             An array of SN_SwerveModules.
	 * @param wheelbase
	 *                             Physically measured distance (center to center)
	 *                             between the Left
	 *                             and Right wheels in meters
	 * @param trackWidth
	 *                             Physically measured distance (center to center)
	 *                             between the Front
	 *                             and Back wheels in meters
	 * @param CANBusName
	 *                             The name of the CANBus that all of the swerve
	 *                             components are on
	 * @param pigeonCANId
	 *                             The CAN id of the Pigeon. The Pigeon MUST be on
	 *                             the same CANBus as
	 *                             the modules
	 * @param minimumSteerPercent
	 *                             The minimum PercentOutput required to make the
	 *                             steer motor move
	 * @param isDriveInverted
	 *                             If the drive motors on every module should be
	 *                             inverted
	 * @param isSteerInverted
	 *                             If the steer motors on every module should be
	 *                             inverted
	 * @param driveNeutralMode
	 *                             The behavior of every drive motor when set to
	 *                             neutral-output
	 * @param steerNeutralMode
	 *                             The behavior of every steer motor when set to
	 *                             neutral-output
	 * @param stateStdDevs
	 *                             Standard deviations of the pose estimate (x
	 *                             position in meters, y
	 *                             position in meters, and heading in radians).
	 *                             Increase these
	 *                             numbers to trust your state estimate less.
	 * @param visionStdDevs
	 *                             Standard deviations of vision pose measurements
	 *                             (x position in
	 *                             meters, y position in meters, and heading in
	 *                             radians). Increase
	 *                             these numbers to trust the vision pose
	 *                             measurement less.
	 * @param autoDrivePID
	 *                             The translational PID constants applied to the
	 *                             entire Drivetrain
	 *                             during autonomous in order to reach the correct
	 *                             pose
	 * @param autoSteerPID
	 *                             The rotational PID constants applied to the
	 *                             entire Drivetrain
	 *                             during autonomous in order to reach the correct
	 *                             pose
	 * @param autoReplanningConfig
	 *                             The configuration for replanning paths in
	 *                             autonomous. See the
	 *                             <a href=
	 *                             "https://mjansen4857.com/pathplanner/docs/java/com/pathplanner/lib/util/ReplanningConfig.html">PathPlanner
	 *                             API</a> for more information
	 * @param isSimulation
	 *                             If your robot is running in Simulation. As of
	 *                             2023, you can supply
	 *                             this with Robot.isSimulation();
	 */
	public SN_SuperSwerve(SN_SwerveConstants swerveConstants, SN_SwerveModule[] modules, double wheelbase,
			double trackWidth, String CANBusName, int pigeonCANId, double minimumSteerPercent, boolean isDriveInverted,
			boolean isSteerInverted, NeutralMode driveNeutralMode, NeutralMode steerNeutralMode,
			Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> visionStdDevs, PIDConstants autoDrivePID,
			PIDConstants autoSteerPID, ReplanningConfig autoReplanningConfig, boolean isSimulation) {

		simTimer.start();

		isFieldRelative = true;
		field = new Field2d();

		// Location of all modules in the WPILib robot coordinate system
		swerveKinematics = new SwerveDriveKinematics(new Translation2d(wheelbase / 2.0, trackWidth / 2.0),
				new Translation2d(wheelbase / 2.0, -trackWidth / 2.0),
				new Translation2d(-wheelbase / 2.0, trackWidth / 2.0),
				new Translation2d(-wheelbase / 2.0, -trackWidth / 2.0));

		this.modules = modules;
		this.stateStdDevs = stateStdDevs;
		this.visionStdDevs = visionStdDevs;
		this.swerveConstants = swerveConstants;
		this.autoDrivePID = autoDrivePID;
		this.autoSteerPID = autoSteerPID;
		this.isSimulation = isSimulation;
		this.autoReplanningConfig = autoReplanningConfig;

		SN_SwerveModule.isSimulation = isSimulation;
		SN_SwerveModule.wheelCircumference = swerveConstants.wheelCircumference;
		SN_SwerveModule.maxModuleSpeedMeters = swerveConstants.maxSpeedMeters;
		SN_SwerveModule.driveGearRatio = swerveConstants.driveGearRatio;
		SN_SwerveModule.steerGearRatio = swerveConstants.steerGearRatio;

		SN_SwerveModule.CANBusName = CANBusName;
		SN_SwerveModule.minimumSteerSpeedPercent = minimumSteerPercent;

		SN_SwerveModule.isDriveInverted = isDriveInverted;
		SN_SwerveModule.driveNeutralMode = driveNeutralMode;

		SN_SwerveModule.isSteerInverted = isSteerInverted;
		SN_SwerveModule.steerNeutralMode = steerNeutralMode;

		driveBaseRadius = Math.sqrt(Math.pow((wheelbase / 2), 2) + Math.pow((trackWidth / 2), 2));
		pigeon = new Pigeon2(pigeonCANId, CANBusName);

		// The absolute encoders need time to initialize
		Timer.delay(2.5);
		resetModulesToAbsolute();
		configure();
	}

	public void configure() {
		for (SN_SwerveModule mod : modules) {
			mod.configure();
		}

		swervePoseEstimator = new SwerveDrivePoseEstimator(swerveKinematics, getRotation(), getModulePositions(),
				new Pose2d(), stateStdDevs, visionStdDevs);

		AutoBuilder.configureHolonomic(this::getPose, this::resetPoseToPose, this::getChassisSpeeds,
				this::driveAutonomous, new HolonomicPathFollowerConfig(autoDrivePID, autoSteerPID,
						swerveConstants.maxSpeedMeters, driveBaseRadius, autoReplanningConfig),
				this);

	}

	/**
	 * Reset all of the steer motors's encoders to the absolute encoder values.
	 */
	public void resetModulesToAbsolute() {
		for (SN_SwerveModule mod : modules) {
			mod.resetSteerMotorToAbsolute();
		}
	}

	/**
	 * Get the position (distance, angle) of each module.
	 *
	 * @return An Array of Swerve module positions (distance, angle)
	 */
	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];

		for (SN_SwerveModule mod : modules) {
			positions[mod.moduleNumber] = mod.getModulePosition();
		}

		return positions;
	}

	/**
	 * Get the state (velocity, angle) of each module.
	 *
	 * @return An Array of Swerve module states (velocity, angle)
	 */
	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];

		for (SN_SwerveModule mod : modules) {
			states[mod.moduleNumber] = mod.getModuleState();
		}

		return states;
	}

	/**
	 * Returns the robot-relative chassis speeds.
	 *
	 * @return The robot-relative chassis speeds
	 */
	public ChassisSpeeds getChassisSpeeds() {
		return swerveKinematics.toChassisSpeeds(getModuleStates());
	}

	/**
	 * Set the states (velocity and position) of the modules.
	 *
	 * @param desiredModuleStates
	 *                            Desired states to set the modules to
	 * @param isOpenLoop
	 *                            Are the modules being set based on open loop or
	 *                            closed loop
	 *                            control
	 *
	 */
	public void setModuleStates(SwerveModuleState[] desiredModuleStates, boolean isOpenLoop) {
		lastDesiredStates = desiredModuleStates;

		// Lowers the speeds if needed so that they are actually achievable. This has to
		// be done here because speeds must be lowered relative to the other speeds as
		// well
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, swerveConstants.maxSpeedMeters);

		for (SN_SwerveModule mod : modules) {
			mod.setModuleState(desiredModuleStates[mod.moduleNumber], isOpenLoop);
		}
	}

	/**
	 * Drive the drivetrain!
	 *
	 * @param translation
	 *                    Desired translational velocity in meters per second
	 * @param rotation
	 *                    Desired rotational velocity in radians per second
	 * @param isOpenLoop
	 *                    Are the modules being set based on open loop or closed
	 *                    loop
	 *                    control
	 *
	 */
	public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
		ChassisSpeeds chassisSpeeds;

		if (isFieldRelative) {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
					getRotation());
		} else {
			chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
		}

		SwerveModuleState[] desiredModuleStates = swerveKinematics.toSwerveModuleStates(discretize(chassisSpeeds));
		setModuleStates(desiredModuleStates, isOpenLoop);
	}

	/**
	 * Drive the drivetrain in autonomous. Autonomous driving is always closed loop.
	 *
	 * @param chassisSpeeds
	 *                      Desired robot-relative chassis speeds
	 *
	 */
	public void driveAutonomous(ChassisSpeeds chassisSpeeds) {
		SwerveModuleState[] desiredModuleStates = swerveKinematics.toSwerveModuleStates(discretize(chassisSpeeds));
		setModuleStates(desiredModuleStates, false);
	}

	// TODO: Replace with WPILib ChassisSpeeds.discretize when 2024 releases
	// https://github.wpilib.org/allwpilib/docs/development/java/edu/wpi/first/math/kinematics/ChassisSpeeds.html#discretize(double,double,double,double)
	/**
	 * Credit: WPILib 2024 and 4738. <br>
	 * <p>
	 * When we are commanding motors in our code, we typically do not account for
	 * the delay in the motors receiving inputs. However, for Swerve, we want to
	 * account for this delay, especially when we are turning. "Discretizing" a
	 * ChassisSpeed means that take what we would have inputted and account for the
	 * time period being discrete (not continuous).
	 * </p>
	 *
	 * @param speeds
	 *               The speeds about to be inputted into the robot.
	 * @return The same desired speeds, but adjusted to our current location.
	 */
	public ChassisSpeeds discretize(ChassisSpeeds speeds) {
		double dt = 0.02;

		var desiredDeltaPose = new Pose2d(speeds.vxMetersPerSecond * dt, speeds.vyMetersPerSecond * dt,
				new Rotation2d(speeds.omegaRadiansPerSecond * dt * 4));

		var twist = new Pose2d().log(desiredDeltaPose);

		return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
	}

	/**
	 * Sets all modules to neutral output
	 */
	public void neutralDriveOutputs() {
		for (SN_SwerveModule mod : modules) {
			mod.neutralDriveOutput();
		}
	}

	/**
	 * Sets the drive method to use field relative drive controls
	 */
	public void setFieldRelative() {
		isFieldRelative = true;
	}

	/**
	 * Sets the drive method to use robot relative drive controls
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
		swervePoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getRotation(), getModulePositions());
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
	 * @param pose
	 *             The pose you would like to reset the pose estimator to
	 */
	public void resetPoseToPose(Pose2d pose) {
		swervePoseEstimator.resetPosition(getRotation(), getModulePositions(), pose);
	}

	/**
	 * Get the rotation of the drivetrain using the Pigeon. In simulation, this will
	 * return a simulated value.
	 *
	 * @return Rotation of drivetrain. The Rotation2d object will deal with units
	 *         for you as long as you specify your desired unit (ex.
	 *         rotation.getDegrees)
	 */
	public Rotation2d getRotation() {
		if (isSimulation && lastDesiredStates != null) {
			timeFromLastUpdate = simTimer.get() - lastSimTime;
			lastSimTime = simTimer.get();
			simAngle += swerveKinematics.toChassisSpeeds(lastDesiredStates).omegaRadiansPerSecond * timeFromLastUpdate;
			return new Rotation2d(simAngle);
		}
		return Rotation2d.fromDegrees(pigeon.getYaw());
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
	 * @param yaw
	 *            The yaw (in degrees) to reset the Pigeon to
	 */
	public void resetYaw(double yaw) {
		pigeon.setYaw(yaw);
	}

	@Override
	public void periodic() {
		updatePoseEstimator();
		SmartDashboard.putNumber("Drivetrain/Yaw Degrees", getRotation().getDegrees());
		SmartDashboard.putBoolean("Drivetrain/Is Field Relative", isFieldRelative);
		for (SN_SwerveModule mod : modules) {
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
