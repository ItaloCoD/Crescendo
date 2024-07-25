package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intake;

public class TroubleIntake extends SubsystemBase{


public TroubleIntake(){
}

@Override
    public void periodic(){
    }

public void cospeNota(){
    intake.m_itk.set(-1);
}
public void stopMotor(){
    intake.m_itk.stopMotor();
}
}
