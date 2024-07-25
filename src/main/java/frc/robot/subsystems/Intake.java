package frc.robot.subsystems;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.intake;
import frc.robot.Constants.shutter;

public class Intake{
    private int aux1;
    private double tempoCorrido;
    private double timeOut;
    public Intake(){
        aux1 = 0;
        timeOut = 0;
        tempoCorrido = Timer.getFPGATimestamp();
    }
    
    public class Pega extends Thread {
    
    @Override
    public void run (){
        while (!interrupted()) {
            tempoCorrido = Timer.getFPGATimestamp();
            if (aux1 == 0) {
                shutter.m_shut_in.setIdleMode(IdleMode.kBrake);
                shutter.m_shut_in2.setIdleMode(IdleMode.kBrake);
                intake.m_itk.set(1);
                shutter.m_shut_in.set(0.3);
                shutter.m_shut_in2.set(-0.3); //-0.15
                intake.m_itk_left.set(0.1);
                intake.m_itk_right.set(-0.1);
                aux1 = 1;
                timeOut = Timer.getFPGATimestamp();
            }
            if (aux1==1 && tempoCorrido - timeOut >= 5 ) {
                shutter.m_shut_in.stopMotor();
                shutter.m_shut_in2.stopMotor();
                intake.m_itk.stopMotor();
                intake.m_itk_left.stopMotor();
                intake.m_itk_right.stopMotor();
                aux1 = 0;
                interrupt();
            }
            if (aux1 == 1 && shutter.sensor_shu.get() == true) {
                shutter.m_shut_in.stopMotor();
                shutter.m_shut_in2.stopMotor();
                intake.m_itk.stopMotor();
                intake.m_itk_left.stopMotor();
                intake.m_itk_right.stopMotor();
                Timer.delay(0.3);
                aux1 = 2;
            }
            if (aux1 == 2) {
                shutter.m_shut_in.set(-0.1);
                shutter.m_shut_in2.set(0.1);
                intake.m_itk_left.setIdleMode(IdleMode.kCoast);
                intake.m_itk_right.setIdleMode(IdleMode.kCoast);   
                //intake.m_itk_left.set(-0.1);
                //intake.m_itk_right.set(0.1);
                aux1 = 3;
            }
            if (aux1 == 3 && shutter.sensor_shu.get() == true) {
                shutter.m_shut_in.stopMotor();
                shutter.m_shut_in2.stopMotor();
                intake.m_itk_left.stopMotor();
                intake.m_itk_right.stopMotor();
                aux1 = 0;
                interrupt();
            }
        } try{
            Thread.sleep(100);
        } catch(InterruptedException e){
            interrupt();
            return;
        }
    }
    }



public void getNotaSolo(){
    if (shutter.sensor_shu.get() == true || aux1 !=0 ) {
        return;
    }
    Pega go = new Pega();
    go.start();
}

}
