// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class DriveSystem extends PIDSubsystem {
  // create motor controller objects
  private final WPI_TalonSRX leftFront = new WPI_TalonSRX(1);
  private final WPI_TalonSRX leftRear = new WPI_TalonSRX(2);
  private final WPI_TalonSRX rightFront = new WPI_TalonSRX(3);
  private final WPI_TalonSRX rightRear = new WPI_TalonSRX(4);

  private final Encoder leftEncoder;
  private final Encoder rightEncoder;

  private final WPI_TalonSRX[] motors = {leftFront, leftRear, rightFront, rightRear};

  private final double MAX_VELOCITY = 400; // in RPM
  private final double TURBO_VELOCITY = 450; 

  private boolean turbo = false;

  private final double TICKS_PER_INCH = 40; // actual value depends on encoder used and wheel size.

  // PID constant values. Would need to test new values
  public static final double VELOCITY_P = 0.000213;
	public static final double VELOCITY_I = 0.0;
	public static final double VELOCITY_D = 0.0;

	public static final double POSITION_P = 0.06;
	public static final double POSITION_I = 0.0;
	public static final double POSITION_D = 0.0;

  /** Creates a new DriveSystem. */
  public DriveSystem() {
    super(new PIDController(VELOCITY_P, VELOCITY_I, VELOCITY_D));
    
    leftEncoder = new Encoder(0, 1);
    rightEncoder = new Encoder(0, 1);
  }

  public void configMotorControllers() {
    for (WPI_TalonSRX motor: motors) {
      motor.setNeutralMode(NeutralMode.Brake);
      
      // set individual motors inverted if needed with motor.setInverted(true);

    }
  }

  public void setPID(double p, double i, double d) {
    for (WPI_TalonSRX motorController: motors) {
      motorController.config_kP(0, p, 100);
      motorController.config_kI(0, i, 100);
      motorController.config_kD(0, d, 100);
    }
  }

  public void toggleTurbo() {
    this.turbo = !this.turbo;
  }

  public void tank(double l, double r) {
    double targetVelocity = MAX_VELOCITY;

    if (this.turbo) {
      targetVelocity = TURBO_VELOCITY;
    }

    this.leftFront.set(ControlMode.Velocity, targetVelocity);
    this.leftRear.set(ControlMode.Velocity, targetVelocity);
    this.rightFront.set(ControlMode.Velocity, targetVelocity);
    this.rightRear.set(ControlMode.Velocity, targetVelocity);
  }

  public void percent(double left, double right) {
		this.leftFront.set(ControlMode.PercentOutput, left);
		this.leftRear.set(ControlMode.PercentOutput, left);
		this.rightFront.set(ControlMode.PercentOutput, right);
		this.rightRear.set(ControlMode.PercentOutput, right);
  }
  
  public void driveDistance(double inches, String direction) {
    double targetPosition;
    // negative may change based on motor direction in wiring
		if (direction.equals("forward")) {
			targetPosition = -1 * inches * TICKS_PER_INCH;
		} else if (direction.equals("backward")) {
			targetPosition = inches * TICKS_PER_INCH;
		} else {
			targetPosition = 0;
		}

		this.leftFront.set(ControlMode.Position, targetPosition);
    this.leftRear.set(ControlMode.Position, targetPosition);
    this.rightFront.set(ControlMode.Position, targetPosition);
		this.rightRear.set(ControlMode.Position, targetPosition);
	}

    @Override
    protected void useOutput(double output, double setpoint) {
      // TODO Auto-generated method stub

    }

    @Override
    protected double getMeasurement() {
      // TODO Auto-generated method stub
      return 0;
    }
}
