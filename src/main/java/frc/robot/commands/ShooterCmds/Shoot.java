package frc.robot.commands.ShooterCmds;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
    private final Shooter shooter;
    private final double targetRpm;

    /**
     * Comando para atirar usando velocidade alvo específica.
     * @param shooter Subsystem do shooter
     * @param targetRpm Velocidade alvo em RPM
     */
    public Shoot(Shooter shooter, double targetRpm) {
        this.shooter = shooter;
        this.targetRpm = targetRpm;
        addRequirements(shooter);
    }

    /**
     * Comando para atirar usando velocidade padrão de Constants.
     */
    public Shoot(Shooter shooter) {
        this(shooter, Constants.ShooterSubsystem.targetRpm);
    }

    @Override
    public void initialize() {
        shooter.setTargetRpm(targetRpm);
    }

    @Override
    public void execute() {
        // Telemetria: indica se está pronto para atirar
        SmartDashboard.putBoolean("ShooterReadyToShoot", shooter.isAtSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopMotor();
    }

    @Override
    public boolean isFinished() {
        // Mantém rodando enquanto o botão estiver pressionado
        return false;
    }
}
