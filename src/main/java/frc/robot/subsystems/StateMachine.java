// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateMachine extends SubsystemBase {
  public static DriverState currentDriverState;
  public static RobotState currentRobotState;

  /** Creates a new StateMachine. */
  public StateMachine() {
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
            // TODO: return Commands
        }
        break;
    }
    return Commands
        .print("ITS SO OVER D: Invalid State Provided, Blame Eli. Attempted to go to: " + desiredState.toString()
            + " while at " + currentRobotState.toString());
  }

  public enum DriverState {
    NONE
  }

  public enum RobotState {
    NONE
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
