package frc.robot.autos;

import frc.lib.util.PIDConfig;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(Swerve swerveDrive){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.maxSpeedMetersPerSecond,
                    Constants.AutoConstants.maxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        PIDConfig thetaConfig = AutoConstants.thetaController;        
        var thetaController =
            new ProfiledPIDController(
                thetaConfig.getKp(), thetaConfig.getKi(),thetaConfig.getKd(), Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                swerveDrive::getPose,
                Constants.Swerve.swerveKinematics,
                AutoConstants.xController.getController(Constants.loopPeriod),
                AutoConstants.yController.getController(Constants.loopPeriod),
                thetaController,
                swerveDrive::setModuleStates,
                swerveDrive);


        addCommands(
            new InstantCommand(() -> swerveDrive.resetOdometry(exampleTrajectory.getInitialPose())),
            swerveControllerCommand
        );
    }
}