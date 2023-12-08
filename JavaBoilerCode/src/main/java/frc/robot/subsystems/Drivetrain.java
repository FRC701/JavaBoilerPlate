// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  TalonFX leftFrontTalonFX = null;
  TalonFX leftBackTalonFX = null;
  TalonFX rightFrontTalonFX = null;
  TalonFX rightBackTalonFX = null;


  public Drivetrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
