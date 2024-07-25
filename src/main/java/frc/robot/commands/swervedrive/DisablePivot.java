package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class DisablePivot extends Command {
    
    private final Pivot pivotSubsystem;
    private final double setPoint;

    public DisablePivot(Pivot pivotSubsystem, double setPoint){
        this.pivotSubsystem = pivotSubsystem;
        this.setPoint = setPoint;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize(){
        pivotSubsystem.zeroFinish();
    }

    @Override
    public void execute(){
        pivotSubsystem.disablePivot();
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("terminou Disable");
    }

    @Override
    public boolean isFinished(){
        boolean atSetpoint = pivotSubsystem.isPivotDisabled();
        return atSetpoint;
    }
}
