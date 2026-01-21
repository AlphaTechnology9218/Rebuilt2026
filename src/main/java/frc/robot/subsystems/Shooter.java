package frc.robot.subsystems;
import frc.robot.Constants.ShooterSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.MathUtil;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Shooter extends SubsystemBase{
    SparkMax Shooter = new SparkMax(ShooterSubsystem.ShooterID, MotorType.kBrushless);
    SparkMaxConfig ShooterConfig;

    private final PIDController velocityPid = new PIDController(
        ShooterSubsystem.kP,
        ShooterSubsystem.kI,
        ShooterSubsystem.kD
    );
    private final SimpleMotorFeedforward velocityFF = new SimpleMotorFeedforward(
        ShooterSubsystem.kSVolts,
        ShooterSubsystem.kVVoltPerRpm,
        ShooterSubsystem.kAVoltPerRpmPerSec
    );

    private double targetRpm = 0.0;
    private boolean closedLoopEnabled = false;

    public Shooter(){
        ShooterConfig = new SparkMaxConfig();
        ShooterConfig.idleMode(IdleMode.kCoast)
        .inverted(ShooterSubsystem.ShooterInverted)
        .smartCurrentLimit(20, 40);

        ShooterConfig.encoder.velocityConversionFactor(ShooterSubsystem.ShooterConversionFactor);


        ShooterConfig.closedLoop.pid(ShooterSubsystem.P,ShooterSubsystem.I,ShooterSubsystem.D, ClosedLoopSlot.kSlot0);
        

        Shooter.configure(ShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    public void VelocitySetpoint(double setpoint)
    {
        Shooter.getClosedLoopController().setSetpoint(setpoint, ControlType.kVelocity);
    }

    public void setTargetRpmWithFF(double rpm)
    {
        targetRpm = rpm;
        closedLoopEnabled = true;
        velocityPid.reset();
    }

    public void disableClosedLoop()
    {
        closedLoopEnabled = false;
    }

    public double getVelocity()
    {
        return Shooter.getEncoder().getVelocity();
    }

    public void StopMotor(){
        closedLoopEnabled = false;
        targetRpm = 0.0;
        Shooter.stopMotor();
    } 

    @Override
    public void periodic() {
        double currentRpm = getVelocity();
        SmartDashboard.putNumber("ShooterSpeed", currentRpm);
        SmartDashboard.putNumber("ShooterTargetRpm", targetRpm);
        SmartDashboard.putBoolean("ShooterClosedLoopEnabled", closedLoopEnabled);

        if (closedLoopEnabled)
        {

            double pidOut = velocityPid.calculate(currentRpm, targetRpm);
            double ffVolts = velocityFF.calculate(targetRpm);

            double volts = ffVolts + pidOut;
            volts = MathUtil.clamp(volts, -12.0, 12.0);
            Shooter.setVoltage(volts);
            SmartDashboard.putNumber("ShooterAppliedVolts", volts);
        }
    }
    
    
}
