package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class PivotCMD extends Command {
    
    private final Pivot pivotSubsystem;
    private final double setPoint;

    public PivotCMD(Pivot pivotSubsystem, double setPoint){
        this.pivotSubsystem = pivotSubsystem;
        this.setPoint = setPoint;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize(){
        pivotSubsystem.zeroFinish();
        System.out.println("entrou");
    }

    @Override
    public void execute(){
       // System.out.println(pivotSubsystem.erro());
       pivotSubsystem.setPosition(setPoint);
    }

    @Override
    public void end(boolean interrupted){
        //pivotSubsystem.disablePivot();
        System.out.println("terminou Pivot");
    }

    @Override
    public boolean isFinished(){
        boolean atSetpoint = pivotSubsystem.isPivotAtSetpoint();
        return atSetpoint;
    }
}
