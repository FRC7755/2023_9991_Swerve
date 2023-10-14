// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MobilityBalanceLow extends SequentialCommandGroup {
  /** Creates a new MobilityBalanceLow. */
  public MobilityBalanceLow(Swerve m_swerve, Pneumatics A_Pneumatics) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()-> m_swerve.zeroGyro()),
      new TimeDriveForward(m_swerve, 1, 1),
      new TimeDriveForward(m_swerve, 1.5, - 1),
      new TimeDriveForward(m_swerve, 2, 2),
      new TimeDriveForward(m_swerve, 2, 1),
      new TimeDriveForward(m_swerve, 1, -2),
      new BalanceAuto(m_swerve)
    );
  }
}
