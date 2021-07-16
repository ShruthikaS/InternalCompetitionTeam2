// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.DriveSystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

/** An example command that uses an example subsystem. */
public class AutoCommands extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSystem driveSystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoCommands(DriveSystem subsystem) {
    driveSystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  public static Command driveDistanceCommand(double distance, String direction) {
    return new FunctionalCommand(
        () -> Robot.drive.resetDistance(),
        () -> Robot.drive.driveDistance(distance, direction),
        (interrupt) -> Robot.drive.tank(0, 0),
        () -> Robot.drive.getDistance() >= distance, 
        Robot.drive
    );
  }

  public static Command angleTurnCommand(double speed, double angle, String direction) {
    return new FunctionalCommand(() -> Robot.drive.resetAngle(),
        () -> Robot.drive.turn(speed, direction),
        (interrupt) -> Robot.drive.tank(0, 0), 
        () -> Robot.drive.getAngle() >= angle, 
        Robot.drive
    );
  }

  public static Command closeAuto(){
    return new SequentialCommandGroup(
      driveDistanceCommand(23.56, "forward"), 
      angleTurnCommand(0.2, 90, "left"),
      driveDistanceCommand(85.88, "forward"),
      angleTurnCommand(0.2, 90, "right"),
      driveDistanceCommand(40, "forward"),
      ShooterCommands.shootCommand(),
      driveDistanceCommand(60, "backward")
    );
  }

  public static Command farAuto(){
    return new SequentialCommandGroup(
      driveDistanceCommand(23.56, "forward"), 
      angleTurnCommand(0.2, 90, "right"),
      driveDistanceCommand(85.88, "forward"),
      angleTurnCommand(0.2, 90, "right"),
      driveDistanceCommand(181, "forward"),
      ShooterCommands.shootCommand()
    );
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
