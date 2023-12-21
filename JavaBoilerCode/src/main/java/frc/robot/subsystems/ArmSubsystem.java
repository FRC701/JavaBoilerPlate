// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  public TalonFX ArmMaster;
  public TalonFX ArmFollower;
  public PIDController pidControl;
  public SimpleMotorFeedforward feedforwardControl;
  public ArmSubsystem() {

    ArmMaster = new TalonFX(Constants.Arm.ArmMotor1);
    ArmFollower = new TalonFX(Constants.Arm.ArmMotor2);
    pidControl = new PIDController(Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD);
    feedforwardControl = new SimpleMotorFeedforward(Constants.Arm.kS, Constants.Arm.kA);

    

    ArmFollower.follow(ArmMaster);

    ArmMaster.setInverted(Constants.Arm.am1InvertMode);
    ArmFollower.setInverted(Constants.Arm.am2InvertMode);

    ArmMaster.setNeutralMode(Constants.Arm.am1NeutralMode);
    ArmFollower.setNeutralMode(Constants.Arm.am2NeutralMode);
  }

  public enum ArmState {
    pos1, pos2, pos3
  }

  public void setState(ArmState state) {

    switch (state) {
      case pos1: 
        break;
      case pos2:
        break;
      case pos3:
        break;
    }
  }

public double SetPercentOutput(double AppliedOutput) {
  ArmMaster.set(ControlMode.PercentOutput, AppliedOutput);
  return AppliedOutput;
}

public double OutputPosePID(double setPointPose, double setPointVel){
  return pidControl.calculate(setPointPose, ArmMaster.getSelectedSensorPosition()) 
  + feedforwardControl.calculate(setPointVel) / 12.0;
}

public boolean AtSetpoint(double setPoint, double tolerance){
  {
    pidControl.setTolerance(tolerance);
    pidControl.setSetpoint(setPoint);
    return pidControl.atSetpoint();
  }
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Angle", ArmMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("Speed", ArmMaster.getSelectedSensorVelocity());
  }
}