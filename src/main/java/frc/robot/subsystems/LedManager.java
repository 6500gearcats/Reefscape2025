// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import frc.robot.ledConstants;

// This class is designed to manage the 8x32 LED board I purchased.
// It is to be imported into WPILib as a subsystem and tested.

// REMINDER: Make sure to initialize in RobotContainer.

public class LedManager extends SubsystemBase {
  Timer timer = new Timer();
  int fps;
  int currentFrame = -1;
  double lastTimestamp;
  AddressableLED m_led = new AddressableLED(ledConstants.kLedPort); //REPLACE WITH PORT
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(ledConstants.kLedLength);
  m_led.setLength(ledConstants.kLedLength);
  int[][][][] displayChoice;
  

  /** Initializes the LED board. 
  @param displayChoice The animation you would like the screen to cycle through. See frc\robot\leds\ledConstants for more details.
  @param fps The amount frames to cycle per second. Alternatively, enter 0 to stay on the first frame.
  **/
  public LedManager(String displayChoice, int fps) {
    try {
        displayChoice = ledConstants.makeDisplayArray(displayChoice);
    } catch(Exception e) {
        System.out.println("Failed to find LED display option. " + e);
        displayChoice = ledConstants.makeDisplayArray("setherror");
    } 
    this.fps = fps;
    lastTimestamp = timer.getTimestamp();
    m_led.start();
    timer.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // hello whoever is reading this!!! - joseph
    if(fps == 0) {
        if(currentFrame == -1) {
          nextFrame();
        }
    }
    
    else if(currentTime >= lastTimestamp + (1.0 / fps)\) {
        nextFrame();
        lastTimestamp = timer.getTimestamp();
    }
  }
  
  public void nextFrame() {
      currentFrame++;
      if(currentFrame >= displayChoice.length) { 
          currentFrame = 0;
      }
      // Might not work with the panel. Will have to see how it considers indexes.
      for(int row = 0; row < 8; row++) {
          for(int col = 0; col < 32; col++) {
              m_led.setRGB(
                  col + (row * 32), // Index
                  displayChoice[currentFrame][row][col][0], // Red
                  displayChoice[currentFrame][row][col][1], // Green
                  displayChoice[currentFrame][row][col][2]  // Blue
              );
          }
      }
  }
}
