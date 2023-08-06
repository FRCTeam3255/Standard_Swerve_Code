// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.constControllers;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

  private final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);
  private final Drivetrain subDrivetain = new Drivetrain();

  public RobotContainer() {
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);

    // The Left Y and X Axies are swapped because from behind the glass, the X Axis
    // is actually in front of you
    subDrivetain
        .setDefaultCommand(new Drive(subDrivetain, conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX));

    configureBindings();

    Timer.delay(2.5);
    // NOTE: Not quite sure if this delay is needed. I added a delay in Drivetrain
    // so idk
    subDrivetain.resetModulesToAbsolute();
  }

  private void configureBindings() {
    conDriver.btn_B.onTrue(Commands.runOnce(() -> subDrivetain.resetModulesToAbsolute()));
    conDriver.btn_A.onTrue(Commands.runOnce(() -> subDrivetain.resetYaw()));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
