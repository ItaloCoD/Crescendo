package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.pivot;

public class Pivot extends SubsystemBase{

    private SparkPIDController pivotPID = pivot.m_pvL.getPIDController();
    private double currentSetpoint;
    private RelativeEncoder encoderPivot;
    private final double tolerance = 1;//0.7;
    private boolean isFinish;
    private boolean isFinishDisable;

    public Pivot(){
        pivotPID.setP(0.1);
        pivotPID.setI(0);
        pivotPID.setD(0);
        pivotPID.setIZone(0);
        pivotPID.setFF(0);
        pivotPID.setOutputRange(-0.4, 0.1);
        pivot.m_pvL.setIdleMode(IdleMode.kBrake);
        pivot.m_pvR.setIdleMode(IdleMode.kBrake);
        pivot.m_pvR.follow(pivot.m_pvL, true);
        encoderPivot = pivot.m_pvL.getEncoder();

    }


    @Override
    public void periodic(){

        if (Math.abs(getCurrentPosition() - currentSetpoint) < tolerance) {
            isFinish = true;
        }

        SmartDashboard.putNumber("Encoder Value: ", getCurrentPosition());
    }
    public void setPosition(double pivotsetPoint){
        pivot.m_pvL.setIdleMode(IdleMode.kBrake);
        pivot.m_pvR.setIdleMode(IdleMode.kBrake);
        currentSetpoint = pivotsetPoint;
        pivotPID.setReference(pivotsetPoint, ControlType.kPosition);
    }

    public void disablePivot(){
        pivotPID.setReference(1, ControlType.kPosition);
        if (Math.abs(encoderPivot.getPosition()) <= 1) {
            pivotPID.setReference(0, ControlType.kDutyCycle);
            isFinishDisable = true;
        }
    }
    
    public void pivotsetZero(){
        encoderPivot.setPosition(0);
        if (Math.abs(encoderPivot.getPosition()) >= 0) {
            isFinish = true;
        }
    }

    public double getCurrentPosition(){
        return pivot.m_pvL.getEncoder().getPosition();
    }
    
    public boolean isPivotAtSetpoint(){
        return isFinish;
    }

    public boolean isPivotDisabled(){
        return isFinishDisable;
    }

    public double erro(){
        return (Math.abs(getCurrentPosition() - currentSetpoint));
    } 

    public void zeroFinish(){
        isFinish = false;
        isFinishDisable = false;
    }

    public void setBrake(){
        pivot.m_pvL.setIdleMode(IdleMode.kBrake);
        pivot.m_pvR.setIdleMode(IdleMode.kBrake);
    }
    public void setCoast(){
        pivot.m_pvL.setIdleMode(IdleMode.kCoast);
        pivot.m_pvR.setIdleMode(IdleMode.kCoast);
    }
}