// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ignore this file
package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DifferentialDriveKinematicsCmd extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final DoubleSupplier kForwardSpeed, kSidewaysSpeed, kAngularSpeed;

  /** Creates a new ArcadeDriveCommand. */
  public DifferentialDriveKinematicsCmd(DriveSubsystem driveSubsystem, DoubleSupplier xVelocity, DoubleSupplier yVelocity, DoubleSupplier angularRate) {
    this.driveSubsystem = driveSubsystem;
    this.kForwardSpeed = xVelocity;
    this.kSidewaysSpeed = yVelocity;
    this.kAngularSpeed = angularRate;
    addRequirements(driveSubsystem);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.kDrive(kForwardSpeed.getAsDouble(), kSidewaysSpeed.getAsDouble(), kAngularSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setMotor(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
