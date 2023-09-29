package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.Swerve;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

public class exampleAuto extends SequentialCommandGroup {
  public exampleAuto(Swerve s_Swerve) {


    // PathPlannerTrajectory trajectory1 = PathPlanner.generatePath(
    //   new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
    //                       AutoConstants.kMaxAccelerationMetersPerSecondSquared), 
    //   new PathPoint(new Translation2d(0,0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0)),
    //   new PathPoint(new Translation2d(-1.5,0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(90)),
    //   new PathPoint(new Translation2d(-3,0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180)),
    //   new PathPoint(new Translation2d(-5,0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180))
    // );
    // PathPlannerTrajectory trajectory2 = PathPlanner.generatePath(
    //   new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
    //                       AutoConstants.kMaxAccelerationMetersPerSecondSquared), 
    //   new PathPoint(new Translation2d(-5,0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)),
    //   new PathPoint(new Translation2d(-3.5,0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(270)),
    //   new PathPoint(new Translation2d(-2,0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(360)),
    //   new PathPoint(new Translation2d(0,0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(360))
    // );


    // PathPlannerTrajectory trajectory1 = PathPlanner.generatePath(
    //   new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
    //                       AutoConstants.kMaxAccelerationMetersPerSecondSquared), 
    //   new PathPoint(new Translation2d(0,0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0)),
    //   new PathPoint(new Translation2d(-5,0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0))
    // );
    // PathPlannerTrajectory trajectory2 = PathPlanner.generatePath(
    //   new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
    //                       AutoConstants.kMaxAccelerationMetersPerSecondSquared), 
    //   new PathPoint(new Translation2d(-5,0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
    //   new PathPoint(new Translation2d(0,0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))
    // );

    PathPlannerTrajectory trajectory1 = PathPlanner.generatePath(
      new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
                          AutoConstants.kMaxAccelerationMetersPerSecondSquared), 
      new PathPoint(new Translation2d(0,0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0)),
      new PathPoint(new Translation2d(-1,0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0))
    );
    PathPlannerTrajectory trajectory2 = PathPlanner.generatePath(
      new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
                          AutoConstants.kMaxAccelerationMetersPerSecondSquared), 
      new PathPoint(new Translation2d(-1,0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180)),
      new PathPoint(new Translation2d(-5,0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180))
    );
    PathPlannerTrajectory trajectory3 = PathPlanner.generatePath(
      new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
                          AutoConstants.kMaxAccelerationMetersPerSecondSquared), 
      new PathPoint(new Translation2d(-5,0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)),
      new PathPoint(new Translation2d(-1,0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180))
    );
    PathPlannerTrajectory trajectory4 = PathPlanner.generatePath(
      new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
                          AutoConstants.kMaxAccelerationMetersPerSecondSquared), 
      new PathPoint(new Translation2d(-1,0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
      new PathPoint(new Translation2d(0,0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))
    );
    // PathPlannerTrajectory trajectory2 = PathPlanner.generatePath(
    //   new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
    //                       AutoConstants.kMaxAccelerationMetersPerSecondSquared), 
    //   new PathPoint(new Translation2d(-5,0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
    //   new PathPoint(new Translation2d(0,0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))
    // );

    // // An example trajectory to follow.  All units in meters.
    // Trajectory exampleTrajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         // Pass through these two interior waypoints, making an 's' curve path
    //         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //         // End 3 meters straight ahead of where we started, facing forward
    //         new Pose2d(3, 0, new Rotation2d(0)),
    //         config);

    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(0, 2*Math.PI);

    PPSwerveControllerCommand swerveControllerCommand1 =
        new PPSwerveControllerCommand(
            trajectory1,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            new PIDController(Constants.AutoConstants.kPThetaController, 0, 0),
            s_Swerve::setModuleStates,
            s_Swerve);
    PPSwerveControllerCommand swerveControllerCommand2 =
          new PPSwerveControllerCommand(
              trajectory2,
              s_Swerve::getPose,
              Constants.Swerve.swerveKinematics,
              new PIDController(Constants.AutoConstants.kPXController, 0, 0),
              new PIDController(Constants.AutoConstants.kPYController, 0, 0),
              new PIDController(Constants.AutoConstants.kPThetaController, 0, 0),
              s_Swerve::setModuleStates,
              s_Swerve);
    PPSwerveControllerCommand swerveControllerCommand3 =
          new PPSwerveControllerCommand(
              trajectory3,
              s_Swerve::getPose,
              Constants.Swerve.swerveKinematics,
              new PIDController(Constants.AutoConstants.kPXController, 0, 0),
              new PIDController(Constants.AutoConstants.kPYController, 0, 0),
              new PIDController(Constants.AutoConstants.kPThetaController, 0, 0),
              s_Swerve::setModuleStates,
              s_Swerve);
    PPSwerveControllerCommand swerveControllerCommand4 =
            new PPSwerveControllerCommand(
                trajectory4,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                new PIDController(Constants.AutoConstants.kPThetaController, 0, 0),
                s_Swerve::setModuleStates,
                s_Swerve);

    addCommands(
      //new InstantCommand(() -> s_Swerve.resetOdometry(trajectory1.getInitialPose())),
      new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0)))),
      swerveControllerCommand1,
      new TurnToAngleCommand(s_Swerve, 180, 2),
      swerveControllerCommand2,
      new InstantCommand(() -> s_Swerve.drive(new Translation2d(0,0), 0, false, true)),
      new WaitCommand(1),
      swerveControllerCommand3,
      new TurnToAngleCommand(s_Swerve, 0, 2),
      swerveControllerCommand4,
      new InstantCommand(() -> s_Swerve.drive(new Translation2d(0,0), 0, false, true))

      // ,
      // new InstantCommand(() -> s_Swerve.drive(new ChassisSpeeds(0,0,0)))
    );

    // addCommands(
    //   //new InstantCommand(() -> s_Swerve.resetOdometry(trajectory1.getInitialPose())),
    //   new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0)))),
    //   swerveControllerCommand1,
    //   new InstantCommand(() -> s_Swerve.drive(new Translation2d(0,0), 0, false, true)),
    //   new WaitCommand(1),
    //   new InstantCommand(() -> s_Swerve.drive(new Translation2d(0,0), 0, false, true)),
    //   swerveControllerCommand2
    //   // ,
    //   // new InstantCommand(() -> s_Swerve.drive(new ChassisSpeeds(0,0,0)))
    // );

    // addCommands(
    //     new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
    //     swerveControllerCommand);
  }
}
