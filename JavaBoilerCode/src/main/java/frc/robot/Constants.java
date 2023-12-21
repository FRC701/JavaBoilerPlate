// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
public static final class Arm {
  
  public static final int ArmMotor1 = 1;
  public static final int ArmMotor2 = 2;

  public static final NeutralMode am1NeutralMode = NeutralMode.Brake;
  public static final NeutralMode am2NeutralMode = NeutralMode.Brake;

  public static final InvertType am1InvertMode = InvertType.InvertMotorOutput;
  public static final InvertType am2InvertMode = InvertType.FollowMaster;

  public static final TrapezoidProfile.Constraints ArmProfilingConstraint = 
  new TrapezoidProfile.Constraints(10, 20);

  public static final TrapezoidProfile.State ArmProfilingState1 =
  new TrapezoidProfile.State(0, 0);

  public static final TrapezoidProfile.State ArmProfilingState2 =
  new TrapezoidProfile.State(5, 0);

  public static final TrapezoidProfile.State ArmProfilingState3 =
  new TrapezoidProfile.State(10, 0);

  public static final double kP = 0.01;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  
  public static final double kS = 0.01;
  public static final double kV = 0.0;
  public static final double kA = 0.0;
}
  
}
