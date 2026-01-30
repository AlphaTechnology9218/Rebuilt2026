package frc.robot.commands.IntakeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeCollector extends Command {
    private final Intake intake;
    private final double targetRpm;

    /**
     * Comando para coletar usando velocidade alvo específica.
     * @param intake Subsystem do intake
     * @param targetRpm Velocidade alvo em RPM
     */
    public IntakeCollector(Intake intake, double targetRpm) {
        this.intake = intake;
        this.targetRpm = targetRpm;
        addRequirements(intake);
    }

    /**
     * Comando para coletar usando velocidade padrão de Constants.
     */
    public IntakeCollector(Intake intake) {
        this(intake, Constants.IntakeSubsystem.collectorTargetRpm);
    }

    @Override
    public void initialize() {
        intake.setCollectorTargetRpm(targetRpm);
    }

    @Override
    public void execute() {
        // Malha fechada roda internamente no SPARK MAX
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopCollector();
    }

    @Override
    public boolean isFinished() {
        // Mantém rodando enquanto o botão estiver pressionado
        return false;
    }
}
