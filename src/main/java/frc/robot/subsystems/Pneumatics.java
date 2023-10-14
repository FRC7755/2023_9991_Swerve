// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  
  DoubleSolenoid bucket, lid;

  PneumaticsControlModule PCM;

  /**Creates a new Pneumatics subsystem with a PCM and two solenoids */
  public Pneumatics(PneumaticsControlModule PCM) {
    this.PCM = PCM;

    
    bucket = PCM.makeDoubleSolenoid(5, 4); 
    lid = PCM.makeDoubleSolenoid(3, 2);
  }

  /**Runs the toggle() method on both solenoids.*/
  public void toggleSolenoid () {
    bucket.toggle();
    lid.toggle();
  }

  public void changeState (Value state) {
    if (bucket.get() != state && bucket.get() != Value.kOff) bucket.set(state);
    if (lid.get() != state && lid.get() != Value.kOff) lid.set(state);
  }

  public void changeStateLid (Value state) {
    if (lid.get() != state && lid.get() != Value.kOff) lid.set(state);
  }

  /**Enables the subsystem. Returns true if the compressor was enabled, false if not. 
   * Note: compressor would only not be enabled if it already was.*/
  public boolean enableSubsystem() {
    if (bucket.get() == Value.kOff || bucket.get() == Value.kForward) bucket.set(Value.kReverse);
    if (lid.get() == Value.kOff || lid.get() == Value.kForward) lid.set(Value.kReverse);
    if (!PCM.getCompressor()) {
    PCM.enableCompressorDigital();
    return true;
    }
    else return false;
  }

  /**Sets both solenoids to "off", then disables the compressor. */
  public void disableSubsystem () {
    bucket.set(Value.kReverse); 
    lid.set(Value.kReverse);
    bucket.set(Value.kOff);
    lid.set(Value.kOff);
    PCM.disableCompressor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
