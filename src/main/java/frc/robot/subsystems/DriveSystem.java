// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class DriveSystem extends PIDSubsystem {
  // create motor controller objects
  private final WPI_TalonSRX leftFront = new WPI_TalonSRX(1);
  private final WPI_TalonSRX leftRear = new WPI_TalonSRX(2);
  private final WPI_TalonSRX rightFront = new WPI_TalonSRX(3);
  private final WPI_TalonSRX rightRear = new WPI_TalonSRX(4);

  // create control groups for the tank drive
  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(this.leftFront, this.leftRear);
  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(this.rightFront, this.rightRear);
  
  // create a drive drivetrain
  private final DifferentialDrive driveTrain = new DifferentialDrive(this.leftMotors, this.rightMotors);
  /** Creates a new DriveSystem. */
  public DriveSystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));
    
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
