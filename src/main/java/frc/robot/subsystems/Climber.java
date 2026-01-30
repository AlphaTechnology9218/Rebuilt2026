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

public class Climber extends SubsystemBase {
    private final SparkMax climber = new SparkMax(ClimberSubsystem.ClimberID, MotorType.kBrushless);
    private final SparkMaxConfig climberConfig;
    
    private double climberSetpoint = 0.0;

    public Climber() {
        climberConfig = new SparkMaxConfig();

        // Configuração de hardware e segurança
        climberConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40, 60)
            .inverted(ClimberSubsystem.ClimberInverted);
        
        // Configuração de encoder
        climberConfig.encoder
            .positionConversionFactor(ClimberSubsystem.ClimberConversionFactor)
            .velocityConversionFactor(ClimberSubsystem.ClimberConversionFactor);
        
        // Ramp rate para proteger transmissão
        climberConfig.openLoopRampRate(ClimberSubsystem.closedLoopRampRate);
        
        // Malha fechada interna para posição
        climberConfig.closedLoop
            .pid(ClimberSubsystem.kP, ClimberSubsystem.kI, ClimberSubsystem.kD, ClosedLoopSlot.kSlot0);

        climber.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Move o climber para a posição de coleta.
     */
    public void climberToCollect() {
        climber.getClosedLoopController().setSetpoint(climberSetpoint, ControlType.kPosition);
    }

    /**
     * Define a posição alvo do climber.
     * @param setpoint Posição alvo em rotações
     */
    public void setSetpoint(double setpoint) {
        climberSetpoint = setpoint;
        climber.getClosedLoopController().setSetpoint(setpoint, ControlType.kPosition);
    }

    /**
     * Para o motor do climber.
     */
    public void climberStop() {
        climber.stopMotor();
    }

    /**
     * Verifica se o climber está no setpoint com tolerância configurada.
     */
    public boolean isAtSetpoint() {
        return climber.getClosedLoopController().isAtSetpoint() || 
               Math.abs(climber.getEncoder().getPosition() - climberSetpoint) <= ClimberSubsystem.positionTolerance;
    }

    /**
     * Retorna a posição atual do climber em rotações.
     */
    public double getPosition() {
        return climber.getEncoder().getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ClimberPosition", getPosition());
        SmartDashboard.putNumber("ClimberSetpoint", climberSetpoint);
        SmartDashboard.putBoolean("ClimberAtSetpoint", isAtSetpoint());
    }
}
