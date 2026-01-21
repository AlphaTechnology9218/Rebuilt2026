package frc.robot.commands.IntakeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;


public class IntakeDeployToggle extends Command {
    private final Intake intake;

    public IntakeDeployToggle(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

        intake.toggleDeploy();
    }

    @Override
    public void execute() {
        intake.applyDeploy();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopDeploy();
    }

    @Override
    public boolean isFinished() {
        if (intake.isDeployed())
        {
            return intake.getLinearPosition() >= frc.robot.Constants.IntakeSubsystem.deployMaxPosition - frc.robot.Constants.IntakeSubsystem.deployHoldEpsilon;
        }
        return intake.getLinearPosition() <= frc.robot.Constants.IntakeSubsystem.deployHoldEpsilon;
    }
}

