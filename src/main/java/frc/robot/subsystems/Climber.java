package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberSubsystem;


public class Climber extends SubsystemBase{
    double climberSetpoint = 0.0;
    SparkMax Climber = new SparkMax(ClimberSubsystem.ClimberID, MotorType.kBrushless);
    SparkMaxConfig ClimberConfig;

    public Climber()
    {
        ClimberConfig = new SparkMaxConfig();

        ClimberConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40,60).inverted(ClimberSubsystem.ClimberInverted);
        
        ClimberConfig.encoder.positionConversionFactor(ClimberSubsystem.ClimberConversionFactor).velocityConversionFactor(ClimberSubsystem.ClimberConversionFactor);

        ClimberConfig.closedLoop.pid(ClimberSubsystem.P, ClimberSubsystem.D, ClimberSubsystem.I, ClosedLoopSlot.kSlot0);

        
        Climber.configure(ClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    public void ClimberToCollect()
    {
        Climber.getClosedLoopController().setSetpoint(climberSetpoint, ControlType.kPosition);
    }

    public void ClimberStop()
    {
        Climber.stopMotor();
    }

    public boolean ClimberAtsetpoint()
    {
        return Climber.getClosedLoopController().isAtSetpoint();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ClimberPosition", Climber.getEncoder().getPosition());
    }
}
