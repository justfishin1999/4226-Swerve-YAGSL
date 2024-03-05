package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(SwerveSubsystem s_Swerve){
    double fwdMetersPerSecond, sideMetersPerSecond, omegaRadiansPerSecond;

    fwdMetersPerSecond = 0.5;
    sideMetersPerSecond = 0.0;
    omegaRadiansPerSecond = 0.0;

    ChassisSpeeds zero = new ChassisSpeeds(0.0,0.0,0.0);
    ChassisSpeeds driveSpeed = new ChassisSpeeds(sideMetersPerSecond, fwdMetersPerSecond, omegaRadiansPerSecond);
        s_Swerve.drive(driveSpeed);
        Timer.delay(1.0);
        s_Swerve.drive(zero);
    }
}