// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.*;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.states.*;
import frc.robot.subsystems.*;

public class StateMachine extends SubsystemBase {
  public static DriverState currentDriverState;
  public static RobotState currentRobotState;
  Drivetrain subDrivetrain;
  StateMachine subStateMachine = this;

  /** Creates a new StateMachine. */
  public StateMachine(Drivetrain subDrivetrain) {
    currentRobotState = RobotState.NONE;
    currentDriverState = DriverState.MANUAL;

    this.subDrivetrain = subDrivetrain;
  }

  public void setRobotState(RobotState robotState) {
    currentRobotState = robotState;
  }

  public RobotState getRobotState() {
    return currentRobotState;
  }

  public DriverState getDriverState() {
    return currentDriverState;
  }

  public void setDriverState(DriverState driverState) {
    currentDriverState = driverState;
  }

  public Command tryState(RobotState desiredState) {
    switch (desiredState) {
      case NONE:
        switch (currentRobotState) {
          case NONE:
            return new None(subStateMachine);
        }
        break;
    }
    return Commands
        .print("ITS SO OVER D: Invalid State Provided, Blame Eli. Attempted to go to: " + desiredState.toString()
            + " while at " + currentRobotState.toString());
  }

  public enum DriverState {
    MANUAL
    // TODO: Add other driver states as needed
  }

  public enum RobotState {
    NONE
    // TODO: Add other robot states as needed
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
