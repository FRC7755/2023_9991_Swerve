// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class BalanceAuto extends CommandBase {
  /** Creates a new BalanceAuto. */
  private Swerve m_Swerve;
  
  public BalanceAuto(Swerve m_Swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Swerve = m_Swerve;
    addRequirements(m_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Swerve.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_Swerve.getGyroRoll() >= 10){
      m_Swerve.drive(new Translation2d(-0.7, 0), 0, true, true);
    } else if (m_Swerve.getGyroRoll() <= -10){
      m_Swerve.drive(new Translation2d(0.7, 0), 0, true, true);
    } else {
      m_Swerve.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
m_Swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
