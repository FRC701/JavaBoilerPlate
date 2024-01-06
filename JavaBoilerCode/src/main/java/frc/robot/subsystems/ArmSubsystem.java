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
  private TalonFX ArmMaster;
  private TalonFX ArmFollower;
  private PIDController pidControl;
  private SimpleMotorFeedforward feedforwardControl;

  public ArmState mState;
  public ArmSubsystem() {

    mState = ArmState.pos1;
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
    pos1, pos2, pos3, disable
  }

  public String runState() {
    String currentState;
    switch (mState) {
      case pos1:
      SetPercentOutput(0.1);
      currentState = "Position 1"; 
        break;
      case pos2:
      SetPercentOutput(0.2);
      currentState = "Position 2";
        break;
      case pos3:
      SetPercentOutput(0.3);
      currentState = "Position 3";
        break;
      case disable:
      SetPercentOutput(0.0);
      currentState = "disable";
        break;
      default:
      currentState = "null";
    }
    return currentState;
  }

  public void setPosition1(){

    mState = ArmState.pos1;
  }

  public void setPosition2(){
    mState = ArmState.pos2;
  }

  public void setPosition3(){
    mState = ArmState.pos3;
  }

  public void disableArmState(){
    mState = ArmState.disable;
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

    SmartDashboard.putString("Current State =", runState());
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Angle", ArmMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("Speed", ArmMaster.getSelectedSensorVelocity());
  }
}