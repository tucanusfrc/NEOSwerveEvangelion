// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {

  private final SwerveDriveSubsystem swerve = new SwerveDriveSubsystem();
  private final CommandXboxController controll = new CommandXboxController(0);

  public RobotContainer() {
    swerve.setDefaultCommand(
      new RunCommand(() -> {
        double xSpeed = MathUtil.applyDeadband(controll.getLeftY(), 0.1) * SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
        double ySpeed = MathUtil.applyDeadband(controll.getLeftX(), 0.1) * SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
        double rotation = MathUtil.applyDeadband(controll.getRightX(), 0.1) * SwerveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

        swerve.drive(xSpeed, ySpeed, rotation);
      }, swerve)
    );
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
