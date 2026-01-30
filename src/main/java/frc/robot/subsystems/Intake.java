package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.IntakeSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final SparkMax intakeMotor = new SparkMax(IntakeSubsystem.IntakeMotorID, MotorType.kBrushless);
    private final SparkMax linearMotor = new SparkMax(IntakeSubsystem.LinearMotorID, MotorType.kBrushless);
    private final SparkMaxConfig intakeConfig;
    private final SparkMaxConfig linearConfig;

    private boolean isDeployed = false;
    private double collectorTargetRpm = 0.0;

    public Intake() {
        intakeConfig = new SparkMaxConfig();
        linearConfig = new SparkMaxConfig();

        // Configuração do collector motor (NEO 550)
        intakeConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20, 40)  // 20A contínuo, 40A pico para NEO 550
            .inverted(IntakeSubsystem.intakeInverted);
        
        intakeConfig.encoder.velocityConversionFactor(IntakeSubsystem.intakeConversionFactor);
        
        // Ramp rate para proteger transmissão
        intakeConfig.openLoopRampRate(IntakeSubsystem.collectorClosedLoopRampRate);
        
        // Malha fechada interna para velocidade
        intakeConfig.closedLoop
            .pid(IntakeSubsystem.collectorKp, IntakeSubsystem.collectorKi, IntakeSubsystem.collectorKd, ClosedLoopSlot.kSlot0);

        // Configuração do linear motor (deploy)
        linearConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40, 60)
            .inverted(IntakeSubsystem.LinearInverted);
        
        linearConfig.encoder
            .positionConversionFactor(IntakeSubsystem.LinearConversionFactor)
            .velocityConversionFactor(IntakeSubsystem.LinearConversionFactor);
        
        // Ramp rate para proteger transmissão
        linearConfig.openLoopRampRate(IntakeSubsystem.linearClosedLoopRampRate);
        
        // Malha fechada interna para posição
        linearConfig.closedLoop
            .pid(IntakeSubsystem.linearKp, IntakeSubsystem.linearKi, IntakeSubsystem.linearKd, ClosedLoopSlot.kSlot0);

        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        linearMotor.configure(linearConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Métodos do collector (intake roller)
    
    /**
     * Define a velocidade alvo do collector usando malha fechada interna do SPARK MAX.
     * O feedforward pode ser configurado no PID config se necessário.
     * @param rpm Velocidade alvo em RPM
     */
    public void setCollectorTargetRpm(double rpm) {
        collectorTargetRpm = rpm;
        intakeMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
    }

    /**
     * Retorna a velocidade atual do collector em RPM.
     */
    public double getCollectorVelocity() {
        return intakeMotor.getEncoder().getVelocity();
    }

    /**
     * Verifica se o collector está no setpoint com tolerância configurada.
     */
    public boolean isCollectorAtSetpoint() {
        return Math.abs(getCollectorVelocity() - collectorTargetRpm) <= IntakeSubsystem.collectorVelocityTolerance;
    }

    /**
     * Para o motor do collector.
     */
    public void stopCollector() {
        collectorTargetRpm = 0.0;
        intakeMotor.stopMotor();
    }

    // Métodos do linear motor (deploy)
    
    /**
     * Move o deploy para a posição de coleta.
     */
    public void linearToCollect() {
        linearMotor.getClosedLoopController().setSetpoint(IntakeSubsystem.deployMaxPosition, ControlType.kPosition);
    }

    /**
     * Move o deploy para a posição inicial (retraído).
     */
    public void linearToInitial() {
        linearMotor.getClosedLoopController().setSetpoint(0.0, ControlType.kPosition);
    }

    /**
     * Para o motor linear.
     */
    public void linearStop() {
        linearMotor.stopMotor();
    }

    /**
     * Verifica se o deploy está no setpoint com tolerância configurada.
     */
    public boolean isLinearAtSetpoint() {
        return linearMotor.getClosedLoopController().isAtSetpoint() || 
               Math.abs(linearMotor.getEncoder().getPosition() - linearMotor.getClosedLoopController().getSetpoint()) <= IntakeSubsystem.linearPositionTolerance;
    }

    /**
     * Retorna a posição atual do deploy em rotações.
     */
    public double getLinearPosition() {
        return linearMotor.getEncoder().getPosition();
    }

    // Métodos de deploy/retract (open-loop voltage control)
    
    public boolean isDeployed() {
        return isDeployed;
    }

    public void stopDeploy() {
        linearMotor.stopMotor();
    }

    public void retractUntilZero() {
        isDeployed = false;
        setDeployVoltage(-IntakeSubsystem.deployVoltage);
    }

    public void deployUntilMax() {
        isDeployed = true;
        setDeployVoltage(IntakeSubsystem.deployVoltage);
    }

    public void toggleDeploy() {
        isDeployed = !isDeployed;
    }

    public void applyDeploy() {
        if (isDeployed) {
            deployUntilMax();
        } else {
            retractUntilZero();
        }
    }

    public void setDeployVoltage(double volts) {
        double pos = getLinearPosition();

        if (volts > 0 && pos >= IntakeSubsystem.deployMaxPosition) {
            stopDeploy();
            return;
        }

        if (volts < 0 && pos <= 0.0) {
            stopDeploy();
            return;
        }

        linearMotor.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }

    public boolean isAtDeployLimit() {
        double pos = getLinearPosition();
        return pos >= IntakeSubsystem.deployMaxPosition - IntakeSubsystem.deployHoldEpsilon;
    }

    public boolean isAtRetractLimit() {
        double pos = getLinearPosition();
        return pos <= IntakeSubsystem.deployHoldEpsilon;
    }

    @Override
    public void periodic() {
        double collectorRpm = getCollectorVelocity();
        SmartDashboard.putNumber("IntakeSpeed", collectorRpm);
        SmartDashboard.putNumber("CollectorTargetRpm", collectorTargetRpm);
        SmartDashboard.putBoolean("CollectorAtSetpoint", isCollectorAtSetpoint());
        SmartDashboard.putNumber("LinearPos", getLinearPosition());
        SmartDashboard.putBoolean("LinearAtSetpoint", isLinearAtSetpoint());
        SmartDashboard.putBoolean("DeployIsDeployed", isDeployed);
    }
}
