package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intake;
import frc.robot.Constants.shutter;

public class Shutter extends SubsystemBase{
private int aux1;
private int aux2;
private int aux3;
double tempo;
int counter;
private boolean isFinishSpeaker;
private boolean isFinishAmp;
private boolean isFinishSource;

public Shutter(){
    this.aux1 = 0;
    this.aux2 = 0;
    this.aux3 = 0;
    tempo = 0;
}

@Override
    public void periodic(){
    }

public class Speaker extends Thread{
@Override
public void run(){
    while (!interrupted()) {
        if (aux1 == 0) {
            shutter.m_shut_up.set(-1 * TiltShutter.shutterVel);
            shutter.m_shut_dw.set(-1 * TiltShutter.shutterVel);
            Timer.delay(0.5);
            aux1 = 1;
        }
        if (aux1 == 1) {
            shutter.m_shut_in.set(1 * TiltShutter.shutterVel);
            shutter.m_shut_in2.set(-1 * TiltShutter.shutterVel);
            Timer.delay(0.5);
            aux1 = 2;
        }
        if (aux1 == 2) {
            shutter.m_shut_in.stopMotor();
            shutter.m_shut_in2.stopMotor();
            shutter.m_shut_up.stopMotor();
            shutter.m_shut_dw.stopMotor();
            shutter.m_shut_dw.setIdleMode(IdleMode.kBrake);
            shutter.m_shut_up.setIdleMode(IdleMode.kBrake);
            aux1 =0;
            isFinishSpeaker = true;
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

public class Amp extends Thread{
@Override
public void run(){
    while (!interrupted()) {

          if(aux2 == 0){
           shutter.m_shut_up.set(-0.4);
           shutter.m_shut_dw.set(-0.4);
           shutter.m_shut_in.set(0.6);
           shutter.m_shut_in2.set(-0.6);
           Timer.delay(0.6);
           aux2 = 1;
          }
          if(aux2 == 1){
           shutter.m_shut_in.stopMotor();
           shutter.m_shut_in2.stopMotor();
           shutter.m_shut_up.stopMotor();
           shutter.m_shut_dw.stopMotor();
           aux2 = 0;
           isFinishAmp = true;
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

public class Source extends Thread{
@Override
public void run(){
    while (!interrupted()) {

          if(aux3 == 0){
           shutter.m_shut_up.set(0.5);
           shutter.m_shut_dw.set(0.5);
           shutter.m_shut_in.set(-0.5);
           shutter.m_shut_in2.set(0.5);
           aux3 = 1;
          }
          if(aux3 == 1 && shutter.sensor_shu.get() == true){
           Timer.delay(0.2);
           aux3 = 2;
          }
          if (aux3 == 2) {
            shutter.m_shut_dw.setIdleMode(IdleMode.kBrake);
            shutter.m_shut_up.setIdleMode(IdleMode.kBrake);
            shutter.m_shut_up.stopMotor();
            shutter.m_shut_dw.stopMotor();
            shutter.m_shut_in.stopMotor();
            shutter.m_shut_in2.stopMotor();
            Timer.delay(0.2);
            aux3 = 3;
          }
          if (aux3 == 3) {
            shutter.m_shut_in.set(0.3);
            shutter.m_shut_in2.set(-0.3); //-0.15
            intake.m_itk_left.set(0.1);
            intake.m_itk_right.set(-0.1);
            aux3 = 4;
          }
          if (aux3 == 4 && shutter.sensor_shu.get() == true) {
            shutter.m_shut_in.stopMotor();
            shutter.m_shut_in2.stopMotor();
            intake.m_itk_left.stopMotor();
            intake.m_itk_right.stopMotor();
            Timer.delay(0.3);
            aux3 = 5;
        }
        if (aux3 == 5) {
            shutter.m_shut_in.set(-0.1);
            shutter.m_shut_in2.set(0.1);
            intake.m_itk_left.setIdleMode(IdleMode.kCoast);
            intake.m_itk_right.setIdleMode(IdleMode.kCoast);   
            aux3 = 6;
        }
        if (aux3 == 6 && shutter.sensor_shu.get() == true) {
            shutter.m_shut_in.stopMotor();
            shutter.m_shut_in2.stopMotor();
            intake.m_itk_left.stopMotor();
            intake.m_itk_right.stopMotor();
            aux3 = 0;
            isFinishSource = true;
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

public void setNotaSperker(){
    if (aux1 != 0) {
    return;
}
    Speaker go = new Speaker();
    go.start();
}

public void setNotaAmp(){
    if (aux2 != 0) {
    return;
}
    Amp go = new Amp();
    go.start();
}

public void setNotaSource(){
    if (aux3 != 0) {
    return;
}
    Source go = new Source();
    go.start();
}

public boolean speakerIsFinish(){
    return isFinishSpeaker;
}

public void zeroFinishSpeaker(){
    isFinishSpeaker = false;
}

public boolean ampIsFinish(){
    return isFinishAmp;
}

public void zeroFinishAmp(){
    isFinishAmp = false;
}

public boolean sourceIsFinish(){
    return isFinishSource;
}

public void zeroFinishSource(){
    isFinishSource = false;
}



}
