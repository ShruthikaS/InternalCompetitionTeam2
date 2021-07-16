// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSystem extends SubsystemBase {
 private WPI_TalonSRX hopperMotor = new WPI_TalonSRX(5);
 private final double MAX_OUTPUT = 0.5;
 
  public HopperSystem() {
    this.hopperMotor.setNeutralMode(NeutralMode.Brake);       
    
  }

  public void hopperIn(){
    this.hopperMotor.set(ControlMode.PercentOutput, MAX_OUTPUT);    
  }

  public void hopperOut(){
    this.hopperMotor.set(ControlMode.PercentOutput, -MAX_OUTPUT);    
  }

  public void stopHopper(){
    this.hopperMotor.set(ControlMode.PercentOutput, 0);    
  }
}
