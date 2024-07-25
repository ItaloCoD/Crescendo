package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.TiltShutter;

public class TiltCMD extends Command {
    
    private final Pivot pivotSubsystem;
    private final TiltShutter tiltSubsystem;

    public TiltCMD(TiltShutter tiltSubsystem, Pivot pivotSubsystem){
        this.tiltSubsystem = tiltSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        addRequirements(tiltSubsystem);
    }

    @Override
    public void initialize(){
        pivotSubsystem.setBrake();
        System.out.println("entrou");
    }

    @Override
    public void execute(){
        tiltSubsystem.calculate();
        pivotSubsystem.setPosition(tiltSubsystem.getTilt());
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("terminou");
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
