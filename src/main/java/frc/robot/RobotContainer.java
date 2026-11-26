// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {

  private SendableChooser<Command> autoChooser = 
    new SendableChooser<>();

  private final SwerveDriveSubsystem swerve = new SwerveDriveSubsystem();
  private final CommandXboxController controll = new CommandXboxController(0);

  public RobotContainer() {

    configureAuto();

    swerve.setDefaultCommand(
      new RunCommand(() -> {
        double xSpeed = MathUtil.applyDeadband(controll.getLeftY(), 0.1) * SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
        double ySpeed = MathUtil.applyDeadband(controll.getLeftX(), 0.1) * SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
        double rotation = MathUtil.applyDeadband(controll.getRightX(), 0.1) * SwerveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

        swerve.drive(xSpeed, ySpeed, rotation);
      }, swerve)
    );
    configureBindings();

      SmartDashboard.putData("auto Chooser", autoChooser);

  }

  private void configureBindings() {}

  private void configureAuto(){

    autoChooser.setDefaultOption(
      "default",
      AutoBuilder.buildAuto("Autoteste")
    );

    autoChooser.addOption(
      "auto1", null
      );

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
