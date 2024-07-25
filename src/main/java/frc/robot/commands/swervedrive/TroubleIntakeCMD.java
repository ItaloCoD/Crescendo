package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.TroubleIntake;

public class TroubleIntakeCMD extends Command {
    
    private final TroubleIntake troubleIntakeSubsystem;

    public TroubleIntakeCMD(TroubleIntake troubleIntakeSubsystem){
        this.troubleIntakeSubsystem = troubleIntakeSubsystem;
        addRequirements(troubleIntakeSubsystem);
    }

    @Override
    public void initialize(){
        troubleIntakeSubsystem.cospeNota();
        System.out.println("Entrou Cospe");
    }

    @Override
    public void execute(){
        
    }

    @Override
    public void end(boolean interrupted){
        troubleIntakeSubsystem.stopMotor();
        System.out.println("terminou Cospe");
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
