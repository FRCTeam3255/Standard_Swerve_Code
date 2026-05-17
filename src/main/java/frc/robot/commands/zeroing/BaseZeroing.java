// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.zeroing;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class BaseZeroing extends Command {
  boolean zeroingSuccess = false;
  Timer zeroingTimer = new Timer();
  AngularVelocity lastRotorVelocity;
  AngularVelocity currentRotorVelocity;
  TalonFX motor;
  BooleanSupplier attemptingZeroing;
  Consumer<Boolean> setAttemptingZeroing;
  Consumer<Boolean> setHasZeroed;
  String name;
  double zeroedPosition;

  /**
   * <b>
   * Constructs a BaseZeroing instance which encapsulates the logic and state
   * needed
   * to zero a single TalonFX motor.
   *
   * <p>
   * This constructor stores the supplied motor and state callbacks and registers
   * a dependency on the shared motion subsystem (RobotContainer.motionInstance).
   * The provided {@code attemptingZeroing} supplier and
   * {@code setAttemptingZeroing}
   * / {@code setHasZeroed} consumers are retained as callbacks for reading and
   * updating zeroing state during the command's lifecycle.
   *
   * @param mainMotor            the TalonFX *MASTER* motor instance that this
   *                             command will attempt to zero
   * @param attemptingZeroing    a BooleanSupplier that returns true when a
   *                             zeroing attempt is in progress (used to query
   *                             current attempt state)
   * @param setAttemptingZeroing a Consumer<Boolean> invoked to mark whether a
   *                             zeroing attempt is being started or stopped
   * @param setHasZeroed         a Consumer<Boolean> invoked to record whether the
   *                             motor
   *                             has successfully been zeroed
   * @param name                 a name for this zeroing mechanism (e.g.
   *                             "Elevator" or "Pivot")
   *                             (used for logging or telemetry)
   * 
   * @param zeroedPosition       the encoder position that corresponds to the
   *                             zeroed state of the motor (used to set the
   *                             motor's position to the zeroed value)
   *
   * @implNote the constructor retains references to the supplied objects (does
   *           not
   *           make defensive copies), so callers should ensure the supplied
   *           callbacks and
   *           motor remain valid for the command's lifetime. The constructor also
   *           calls
   *           addRequirements(RobotContainer.motionInstance) to declare a
   *           dependency on the
   *           motion subsystem.
   *           </b>
   */
  public BaseZeroing(TalonFX mainMotor,
      BooleanSupplier attemptingZeroing,
      Consumer<Boolean> setAttemptingZeroing,
      Consumer<Boolean> setHasZeroed,
      String name,
      double zeroedPosition) {
    this.motor = mainMotor;
    this.attemptingZeroing = attemptingZeroing;
    this.setAttemptingZeroing = setAttemptingZeroing;
    this.setHasZeroed = setHasZeroed;
    this.name = name;
    this.zeroedPosition = zeroedPosition;
    addRequirements(RobotContainer.motionInstance);
  }

  @Override
  public void initialize() {
    zeroingSuccess = false;
    zeroingTimer.reset();
  }

  @Override
  public void execute() {
    // Check if we have raised the elevator above a certain speed
    currentRotorVelocity = motor.getVelocity().getValue();
    if (currentRotorVelocity.gte(Units.RotationsPerSecond.of(0.5))
        || attemptingZeroing.getAsBoolean()) {
      // Enter zeroing mode!
      if (!attemptingZeroing.getAsBoolean()) {
        setAttemptingZeroing.accept(true);
        zeroingTimer.start();
        System.out.println(name + " Zeroing Started!");
      }

      // Check if time elapsed is too high (zeroing timeout)
      if (zeroingTimer.hasElapsed(3.0)) {
        setAttemptingZeroing.accept(false);
        System.out.println(name + " Zeroing Failed :(");
      } else {
        boolean deltaRotorVelocity = motor.getVelocity().getValue().minus(lastRotorVelocity)
            .lte(Units.RotationsPerSecond.of(-0.5));

        if (deltaRotorVelocity && lastRotorVelocity.lte(Units.RotationsPerSecond.of(0))) {
          zeroingSuccess = true;
        } else {
          lastRotorVelocity = motor.getVelocity().getValue();
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      setHasZeroed.accept(true);
      motor.setPosition(zeroedPosition);
      System.out.println(name + " Zeroing Successful!!!! Yippee and hooray!!! :3");
    } else {
      System.out.println(name + " was never zeroed :((( blame eli");
    }
  }

  @Override
  public boolean isFinished() {
    boolean rotorVelocityIsZero = motor.getVelocity().getValue().isNear(Units.RotationsPerSecond.zero(), 0.01);
    SmartDashboard.putBoolean("Zeroing/Pivot/Is Rotor Velocity Zero", rotorVelocityIsZero);
    return zeroingSuccess && rotorVelocityIsZero;
  }
}