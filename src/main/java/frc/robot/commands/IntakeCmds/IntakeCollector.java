package frc.robot.commands.IntakeCmds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCollector extends Command{
       Intake IntakeSubsystem;
       Timer timer;
       


    public IntakeCollector(Intake subsystem){
        this.IntakeSubsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.reset();
    }

    @Override
    public void execute() {
        IntakeSubsystem.intakemotion (-1);
        timer.start();
        
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.intakestop();

    }

}