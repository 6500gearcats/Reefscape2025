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

import frc.robot.leds.ledConstants;

// This class is designed to manage the 8x32 LED board I purchased.
// It is to be imported into WPILib as a subsystem and tested.

// REMINDER: Make sure to initialize in RobotContainer.

public class LedPanel extends SubsystemBase {
  Timer timer = new Timer();
  double fps;
  int currentFrame;
  double lastTimestamp;
  AddressableLED m_led = new AddressableLED(ledConstants.kLedPort); //REPLACE WITH PORT
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(ledConstants.kLedLength);
  int[][][][] displayChoice;
  String currentOption;
  String ledSimOutput;
  

  /** Initializes the LED board. 
  @param displayChoice The name of the animation you would like the screen to cycle through or the custom input.
  @param fps The amount frames to cycle per second. Alternatively, enter 0 to stay on the first frame.
  **/
  public LedPanel(String choice, double fps) {
    this.currentOption = choice;
    this.fps = fps;
    currentFrame = 0;
    ledSimOutput = "";

    if(isAnimation(choice)) {
        try {
            this.displayChoice = ledConstants.makeDisplayArrayFromImages(choice);
        } catch(Exception e) {
            System.out.println("Error creating LED animation. " + e);
            this.displayChoice = ledConstants.makeDisplayArrayFromImages("setherror");
        }
    }

    else {
        try { 
            this.displayChoice = ledConstants.makeDisplayArrayFromString(choice);
        } catch (Exception e) {
            System.out.println("Error creating custom LED output. " + e);
            this.displayChoice = ledConstants.makeDisplayArrayFromImages("setherror");
        }
    }

    m_led.setLength(ledConstants.kLedLength);
    m_led.start();
    timer.start();
    lastTimestamp = timer.get();

    if(displayChoice == null) {
        System.out.println("DisplayChoice is Null");
    }

    System.out.println("LEDs set to " + currentOption);
    SmartDashboard.putString("Led Choice", "6500Teal");
    SmartDashboard.putNumber("Led Frames Per Second", fps);
    SmartDashboard.putStringArray("Led Options", ledConstants.ledOptions);
  }

  private boolean isAnimation(String choice) {
    for(int i = 0; i < ledConstants.ledOptions.length; i++)
    if(ledConstants.ledOptions[i].equals(choice)) {
        return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    if(displayChoice != null) {
        String dashboardInput = SmartDashboard.getString("Led Choice", "6500Teal");
        if(!(this.currentOption.equals(dashboardInput))) {
            currentFrame = -1;
            if(isAnimation(dashboardInput)) {
                this.displayChoice = ledConstants.makeDisplayArrayFromImages(dashboardInput);
            } else {
                try { 
                    displayChoice = ledConstants.makeDisplayArrayFromString(dashboardInput);
                } catch (Exception e) {
                    System.out.println("Error creating custom LED output; " + e);
                    this.displayChoice = ledConstants.makeDisplayArrayFromImages("setherror");
                }
            }
            this.currentOption = dashboardInput;
            System.out.println("LEDs set to " + currentOption);
        }

        if(SmartDashboard.getNumber("Led Frames Per Second", 1.0) != this.fps) {
            this.fps = SmartDashboard.getNumber("Led Frames Per Second", 1.0);
        }

        double currentTime = timer.get();
        if(fps == 0 && currentFrame == -1) {
            nextFrame();
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
      if(Robot.isReal()) {
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
      }

      if(Robot.isSimulation()) {
        //System.out.println("Current LED Frame: ");
        ledSimOutput = "";
        for(int row = 0; row < 8; row++) {
            for(int col = 0; col < 32; col++){
                int colorAvg = (displayChoice[currentFrame][row][col][0] + displayChoice[currentFrame][row][col][1] + displayChoice[currentFrame][row][col][2]) / 3;
                // Check if white
                if(colorAvg == 255) 
                {
                    ledSimOutput += "▓" + " ";
                // Check if black
                } else if(colorAvg == 0) {
                    ledSimOutput += "░" + " ";
                } else {
                    ledSimOutput += "▒" + " ";
                }
            }
            ledSimOutput += "\n";
        }
        SmartDashboard.putString("Led Sim Output", ledSimOutput);
      }
      
      m_led.setData(m_ledBuffer);
  }
}
