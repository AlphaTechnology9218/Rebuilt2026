// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

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
    public static final int kSystemControllerPort = 1;

    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
//mudar massa do robô
  public static class SwerveConstants{
    public static final double kSwerveSteeringRatio = 21.428571428571428571428571428571;
    public static final double ROBOT_MASS = 40.5;
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, 0), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13;
    public static final double MAX_SPEED  = Units.feetToMeters(3);
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(6.5,0.00000008,1.39);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0, 0, 0);
    public static RobotConfig robotConfig;

  }

  public static final class AutonConstants
  {
    
  }

  public static final class DrivebaseConstants
  {

    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class IntakeSubsystem
  {
    // Collector motor (NEO 550)
    public static final int IntakeMotorID = 14;
    public static final boolean intakeInverted = false;
    public static final double intakeConversionFactor = 1.0 / 4.0;
    
    // PID constants para SparkClosedLoopController (velocidade)
    public static final double collectorKp = 0.0001;
    public static final double collectorKi = 0.000001;
    public static final double collectorKd = 0.0;
    
    // Feedforward constants (RPM-based)
    public static final double collectorKsVolts = 0.0;
    public static final double collectorKvVoltPerRpm = 0.0001;
    public static final double collectorKaVoltPerRpmPerSec = 0.0;
    
    // Ramp rate para proteger transmissão (segundos)
    public static final double collectorClosedLoopRampRate = 0.2;
    
    // Tolerância para isAtSetpoint (RPM)
    public static final double collectorVelocityTolerance = 50.0;
    
    public static final double collectorTargetRpm = 1500.0;
    
    // Deploy motor (linear)
    public static final int LinearMotorID = 11;
    public static final boolean LinearInverted = false;  
    public static final double LinearConversionFactor = 1.0 / 27.0;
    
    // PID constants para SparkClosedLoopController (posição)
    public static final double linearKp = 0.1;
    public static final double linearKi = 0.0;
    public static final double linearKd = 0.0;
    
    // Ramp rate para deploy (segundos)
    public static final double linearClosedLoopRampRate = 0.1;
    
    // Tolerância para posição (rotações)
    public static final double linearPositionTolerance = 0.05;
    
    public static final double deployVoltage = 4.0;
    public static final double deployMaxPosition = 1.0;
    public static final double deployHoldEpsilon = 0.02;
  }
  public static final class ShooterSubsystem
  {
    public static final int ShooterID = 10;
    public static final boolean ShooterInverted = false;
    public static final double ShooterConversionFactor = 1.0;

    public static final int shooterCurrentLimitContinuousAmps = 40;
    public static final int shooterCurrentLimitPeakAmps = 60;
    
    // PID constants para SparkClosedLoopController (velocidade)
    public static final double kP = 0.0001;
    public static final double kI = 0.000001;
    public static final double kD = 0.0;
    
    // Feedforward constants (RPM-based)
    public static final double kSVolts = 0.0;
    public static final double kVVoltPerRpm = 0.0001;
    public static final double kAVoltPerRpmPerSec = 0.0;
    
    // Ramp rate para proteger transmissão (segundos)
    public static final double closedLoopRampRate = 0.2;
    
    // Tolerância para isAtSetpoint (RPM)
    public static final double velocityTolerance = 50.0;
    
    // Target RPM padrão
    public static final double targetRpm = 2500.0;
  }
  public static final class ClimberSubsystem
  {
    public static final int ClimberID = 13;
    public static final boolean ClimberInverted = false;
    public static final double ClimberConversionFactor = 1.0;
    
    // PID constants para SparkClosedLoopController (posição)
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    
    // Ramp rate para proteger transmissão (segundos)
    public static final double closedLoopRampRate = 0.1;
    
    // Tolerância para posição (rotações)
    public static final double positionTolerance = 0.05;
  }
}
