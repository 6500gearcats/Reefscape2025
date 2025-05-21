// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import frc.robot.leds.ledConstants;

// This class is designed to manage the 8x32 LED board I purchased.
// It is to be imported into WPILib as a subsystem and tested.

// REMINDER: Make sure to initialize in RobotContainer.

public class LedPanel extends SubsystemBase {
  Timer timer = new Timer();
  int fps;
  int currentFrame;
  double lastTimestamp;
  AddressableLED m_led = new AddressableLED(ledConstants.kLedPort); //REPLACE WITH PORT
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(ledConstants.kLedLength);
  int[][][][] displayChoice;
  String currentOption;
  

  /** Initializes the LED board. 
  @param displayChoice The name of the animation you would like the screen to cycle through. Options above.
  @param fps The amount frames to cycle per second. Alternatively, enter 0 to stay on the first frame.
  **/
  public LedPanel(String choice, int fps) {
    this.currentOption = choice;
    currentFrame = 0;
    try {
        this.displayChoice = ledConstants.makeDisplayArray(choice);
    } catch(Exception e) {
        System.out.println("Failed to find LED display option. " + e);
        this.displayChoice = ledConstants.makeDisplayArray("setherror");
    } 
    this.fps = fps;

    m_led.setLength(ledConstants.kLedLength);
    m_led.start();

    timer.start();
    lastTimestamp = timer.get();
  }

  @Override
  public void periodic() {
    if(displayChoice != null) {
        if(this.currentOption != RobotContainer.LedChooser.getSelected()) {
            this.displayChoice = ledConstants.makeDisplayArray(RobotContainer.LedChooser.getSelected());
        }

        double currentTime = timer.get();
        if(fps == 0) {
            if(currentFrame == -1) {
            nextFrame();
            }
        }
        
        else if(currentTime >= lastTimestamp + (1.0 / fps)) {
            nextFrame();
            lastTimestamp = currentTime;
        }
    }
  }
  
  public void nextFrame() {
      currentFrame++;
      if(currentFrame == displayChoice.length) { 
          currentFrame = 0;
      }
      
      // Might not work with the panel, will have to see how it considers indexes
      for(int row = 0; row < 8; row++) {
          for(int col = 0; col < 32; col++) {
              m_ledBuffer.setRGB(
                  col + (row * 32), // Index
                  displayChoice[currentFrame][row][col][0], // Red
                  displayChoice[currentFrame][row][col][1], // Green
                  displayChoice[currentFrame][row][col][2]  // Blue
              );
          }
      }

      m_led.setData(m_ledBuffer);
  }
}
