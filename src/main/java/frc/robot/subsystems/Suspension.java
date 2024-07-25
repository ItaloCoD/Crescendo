package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class Suspension {
  
  int aux1 = 0;
  private Solenoid ledIntake;
  private Solenoid solenoide0;
  private Solenoid solenoide2;
  private Solenoid solenoide3;
  private Solenoid solenoide4;
  private Solenoid solenoide5;

  private Boolean outputState = false;
  private Timer timer = new Timer();


  public Suspension(){
    ledIntake = new Solenoid(59, PneumaticsModuleType.REVPH, 0);
    solenoide0 = new Solenoid(59, PneumaticsModuleType.REVPH, 1);
    solenoide2 = new Solenoid(59, PneumaticsModuleType.REVPH, 2);
    solenoide3 = new Solenoid(59, PneumaticsModuleType.REVPH, 3);
    solenoide4 = new Solenoid(59, PneumaticsModuleType.REVPH, 4);
    solenoide5 = new Solenoid(59, PneumaticsModuleType.REVPH, 5);
  }

  public void start(){
    timer.start();
  }

  public void pisca(){
    if (timer.hasElapsed(0.05)) {
      outputState = !outputState;
      if (outputState) {
          ledIntake.set(true);
      } else {
          ledIntake.set(false);
      }
      timer.reset();
  }
  }
}
