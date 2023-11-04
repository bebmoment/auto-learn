// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPIDCommand extends CommandBase {
  private final ElevatorSubsystem elevatorSubsystem;
  private final double setpoint;

  /** Creates a new ElevatorCommand. */

  public ElevatorPIDCommand(ElevatorSubsystem elevatorSubsystem, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    this.setpoint = setpoint;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { // no initialization needed
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setMotor(elevatorSubsystem.getElevatorControllerSpeed(setpoint)); // calls the .setMotor() command, with the PID's calculated speed as the parameter
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setMotor(ElevatorConstants.ELEVATOR_STALL_SPEED); // set a stall speed for the elevator once the command ends, to work against gravity
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    return false;
  }
}