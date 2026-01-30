package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;

public class IntakeCoraloutCmd extends Command{
       CoralIntake coralintakeSubsystem;
       Timer timer;
       


    public IntakeCoraloutCmd(CoralIntake subsystem){
        this.coralintakeSubsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.reset();
    }

    @Override
    public void execute() {
        coralintakeSubsystem.intakemotion (-1);
        timer.start();
        
    }

    @Override
    public void end(boolean interrupted) {
        coralintakeSubsystem.intakestop();
       if (coralintakeSubsystem.getcurrent()>=40) {
            coralintakeSubsystem.intakestop();
        }

    }

}