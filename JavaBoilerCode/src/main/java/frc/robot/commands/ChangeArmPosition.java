// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class ChangeArmPosition extends CommandBase {
  /** Creates a new ChangeArmPosition. */
  private ArmSubsystem mArm;

  private double mSetpoint;

  private TrapezoidProfile.State ArmProfilingState;

  public ChangeArmPosition(TrapezoidProfile.State ArmProfilingState,  ArmSubsystem mArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var ArmProfile = new TrapezoidProfile(Constants.Arm.ArmProfilingConstraint,
        ArmProfilingState,
        Constants.Arm.ArmProfilingState1);

    var setpoint = ArmProfile.calculate(5);
    mArm.SetPercentOutput(mArm.OutputPosePID(setpoint.position, setpoint.velocity));

    mSetpoint = setpoint.position;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mArm.AtSetpoint(mSetpoint, 0.5);
  }
}
