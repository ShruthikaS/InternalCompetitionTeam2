// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class ShooterSystem {
 private WPI_TalonSRX shooterMotor = new WPI_TalonSRX(6);
 private final double MAX_OUTPUT = 0.7;
 
  public ShooterSystem() {
    this.shooterMotor.setNeutralMode(NeutralMode.brake);       
    
  }

  public void shoot(){
    this.shooterMotor.set(ControlMode.PercentOutput, MAX_OUTPUT);    
  }


  public void stopShoot(){
    this.shooterMotor.set(ControlMode.PercentOutput, 0);    
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
