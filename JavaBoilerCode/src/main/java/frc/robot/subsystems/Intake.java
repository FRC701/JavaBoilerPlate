// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  TalonFX IntakeMotor;

  public Intake() {
    IntakeMotor = new TalonFX(Constants.Intake.IntakeMotor);
  }

  public enum ObjectState {
    S_WaitingforBall, S_HasBall
  };

  public String HasObjectState(ObjectState State) {
    String CurrentState = "none";

    switch (State) {
      case S_WaitingforBall:
        CurrentState = "Waiting For Ball";
        if (WaitingforBall()) {
          State = ObjectState.S_HasBall;
        }
        break;
      case S_HasBall:
        CurrentState = "Has Ball";
        if (HasBall()) {
          State = ObjectState.S_WaitingforBall;
        }
        break;
    }
    return CurrentState;
  }

  public boolean WaitingforBall() {
    if (ThresholdInMet()) {
      return true;
    }
    SpinIntake(1000);
    return false;
  }

  public boolean HasBall() {
    SpinIntake(0);
    if (ThresholdOutMet()) {
      return true;
    }
    return false;
  }

  public boolean ThresholdOutMet() {
    return true;
  }

  public boolean ThresholdInMet() {
    return true;
  }

  public void SpinIntake(double MotorSpeed) {
    IntakeMotor.set(ControlMode.Velocity, MotorSpeed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("CurrentIntakeState", HasObjectState(ObjectState.S_WaitingforBall));
    // This method will be called once per scheduler run
  }
}
