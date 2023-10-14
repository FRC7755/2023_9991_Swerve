package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends CommandBase {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private BooleanSupplier slowSpeedSup;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  public TeleopSwerve(
      Swerve s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup,
      BooleanSupplier slowSpeedSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.slowSpeedSup = slowSpeedSup;
  }

  @Override
  public void execute() {

    double speedMultiplier = slowSpeedSup.getAsBoolean() ? 0.2 : 1.0;
    double tIN = Math.abs(translationSup.getAsDouble()) < 0.15 ? 0 : translationSup.getAsDouble();
    //double tIN = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband);
    //tIN = tIN * tIN * tIN;
    double sIN = Math.abs(strafeSup.getAsDouble()) < 0.15 ? 0 : strafeSup.getAsDouble();
    //double sIN = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband);
    //sIN = sIN * sIN * sIN;
    double rIN = Math.abs(rotationSup.getAsDouble()) < 0.15 ? 0 : rotationSup.getAsDouble();
    //double rIN = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband);
    //rIN = rIN * rIN * rIN;
    /* Get Values, Deadband*/
    double translationVal =
        translationLimiter.calculate(
            speedMultiplier *
            tIN);
    double strafeVal =
        strafeLimiter.calculate(
            speedMultiplier *
            sIN);
    double rotationVal =
        rotationLimiter.calculate(
            speedMultiplier *
            rIN);
    //translationVal = translationVal * translationVal * translationVal;
    //strafeVal = strafeVal * strafeVal * strafeVal;
    //rotationVal = rotationVal * rotationVal * rotationVal;

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity,
        !robotCentricSup.getAsBoolean(),
        true);
  }
}
