// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotationSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.RotationCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MyNewAutoCommand extends SequentialCommandGroup {
  /** Creates a new EngageAuto. */
  public MyNewAutoCommand(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem,
      RotationSubsystem rotationSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new ElevatorCommand(elevatorSubsystem, ElevatorConstants.ELEVATOR_UP_SPEED).withTimeout(0.75),
        new RotationCommand(rotationSubsystem, true)
        ),
        new IntakeCommand(intakeSubsystem, true),
        new RetractAutomation(rotationSubsystem, elevatorSubsystem, intakeSubsystem)
        // new ConeAutomation(rotationSubsystem, elevatorSubsystem),
        // new IntakeCommand(intakeSubsystem, true),
        // new ArcadeDriveCommand(driveSubsystem, () -> 0.0, () -> 0.6).withTimeout(3.0)
        )
        ;
  }
}
