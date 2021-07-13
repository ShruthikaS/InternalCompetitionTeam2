// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class IntakeSystem {
 private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(5);
 private final double MAX_OUTPUT = 0.7;
 
  public IntakeSystem() {
    this.intakeMotor.setNeutralMode(NeutralMode.brake);       
    
  }

  public void diskIn(){
    this.intakeMotor.set(ControlMode.PercentOutput, MAX_OUTPUT);    
  }

  public void shootOut(){
    this.intakeMotor.set(ControlMode.PercentOutput, -MAX_OUTPUT);    
  }

  public void stopIntake(){
    this.intakeMotor.set(ControlMode.PercentOutput, 0);    
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
