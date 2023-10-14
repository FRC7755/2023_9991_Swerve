// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class IntakeCommand extends CommandBase {
  Pneumatics pneumaticsSubsystem;

  BooleanSupplier deployIntake, retractIntake, openLid, closeLid;

  Timer timer;

  public IntakeCommand(Pneumatics pneumaticsSubsystem, BooleanSupplier deployIntake, BooleanSupplier retractIntake, BooleanSupplier openLid, BooleanSupplier closeLid) {
    this.pneumaticsSubsystem = pneumaticsSubsystem;
    this.deployIntake = deployIntake;
    this.retractIntake = retractIntake;
    this.openLid = openLid;
    this.closeLid = closeLid;

    timer = new Timer();
    addRequirements(pneumaticsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pneumaticsSubsystem.enableSubsystem();
    timer.reset();
  }

  @Override
  public void execute() {
    /**If the BooleanSupplier returns a true value, run the toggleSolenoid()
     method in the pneumaticsSubsystem */
     
      if (deployIntake.getAsBoolean()) {
        pneumaticsSubsystem.changeState(Value.kForward);
      }
      else if (retractIntake.getAsBoolean()) {
        pneumaticsSubsystem.changeState(Value.kReverse);
      }
      else if (openLid.getAsBoolean()) {
        pneumaticsSubsystem.changeStateLid(Value.kForward);
      }
      else if (closeLid.getAsBoolean()) {
        pneumaticsSubsystem.changeStateLid(Value.kReverse);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pneumaticsSubsystem.disableSubsystem();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
