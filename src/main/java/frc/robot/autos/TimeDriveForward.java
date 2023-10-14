// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class TimeDriveForward extends CommandBase {
  /** Creates a new TimeDriveForward. */
  private Timer m_timer;
  private Swerve m_swerve;
  private double speed;
  private double time;

  public TimeDriveForward(Swerve m_swerve, double time, double speed) {
    this.m_swerve = m_swerve;
    m_timer = new Timer();
    this.time = time;
    this.speed = speed;
    addRequirements(m_swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerve.drive(new Translation2d(speed, 0), 0, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.drive(new Translation2d(0, 0), 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() >= time ? true : false;
  }
}
