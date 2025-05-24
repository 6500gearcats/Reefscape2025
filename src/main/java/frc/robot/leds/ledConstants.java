package frc.robot.leds;
import frc.robot.leds.ledLetters;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;

public class ledConstants { //TODO: correct kLedPort
    public static final int kLedPort = 8;
    public static final int kLedLength = 256;

    private static String[] files = new File("src\\main\\java\\frc\\robot\\leds\\images").list();

    /*
     * List of display options:
     * { name } { [#frames] } --> { desc }
     * test [2] --> displays hello world
     * setherror [1] --> displays setherror, displayed if an error occurs
     * pongBlueWin [64] --> depicts blue team winning in pong
     * pranavCreeper [29] --> pranav blowing up to a creeper (LOL)
     * 6500Blue [1] --> Resembles the blue bumpers
     * 6500Red [1] --> ^^^ but red
     * 6500Teal [1] --> ^^^ but teal from our logo
     */

     public static final String[] ledOptions = {"test", "setherror", "pongBlueWin", "pranavCreeper", "6500Blue", "6500Red", "6500Teal"};

    
    public static int[][][][] makeDisplayArrayFromImages(String choice) {

        int frames = 0;
        for (int i = 0; i < files.length; i++) {
            if (files[i].indexOf(choice) != -1) {
                frames++;
            }
        }

        int[][][][] display = new int[frames][][][];

        for(int i = 0; i < frames; i++) {
            BufferedImage img = null;
            try {
                img = ImageIO.read(new File("src\\main\\java\\frc\\robot\\leds\\images\\" + choice + i + ".png"));
            } catch (IOException e) {
                System.out.println("Error with image in ledConstants: " + e);
                break;
            }
            
            int[][][] pixels = new int[8][32][3];

            for (int row = 0; row < 8; row++) {
                for (int col = 0; col < 32; col++) {
                    int rgb = img.getRGB(col, row);
                    pixels[row][col][0] = (rgb >> 16) & 0xFF; // Red
                    pixels[row][col][1] = (rgb >> 8) & 0xFF;  // Green
                    pixels[row][col][2] = rgb & 0xFF;         // Blue
                }
            }
            /* 
            for(int x = 0; x < 8; x++) {
                for (int y = 0; y < 32; y++) {
                    System.out.println("Pixel at (" + x + ", " + y + "): (" + pixels[x][y][0] + ", " + pixels[x][y][1] + ", " + pixels[x][y][2] +")");
                }
            } */
            display[i] = pixels;
            //System.out.println(pixels);
        }

        return display;
    }

    /** Turns the parameter into an int[][][][] display for the LED board.
    @param text The String to be put on the array. Screen seperation marked by @. To use colors, begin the string with (r, g, b). Long words may have issues.
    **/
    public static int[][][][] makeDisplayArrayFromString(String text) {
        int r, g, b;
        if(text.indexOf("(") != -1) { // (red, green, blue)
            try {
                text = text.substring(1);
                // Red
                r = Integer.parseInt(text.substring(0, text.indexOf(",")), 10);

                // Green
                text = text.substring(text.indexOf(",") + 2);
                g = Integer.parseInt(text.substring(0, text.indexOf(",")), 10);

                // Blue
                text = text.substring(text.indexOf(",") + 2);
                b = Integer.parseInt(text.substring((0), text.indexOf(")")), 10);
                
                text = text.substring(text.indexOf(")") + 1);
            } catch(Exception e) {
                System.out.println("Error with custom text input. Check your RGB notation. " + e);
                return makeDisplayArrayFromImages("setherror");
            }
        } else {
            r = 255;
            g = 255;
            b = 255;
        }

        // Count number of frames
        int frames = 0;
        text += "@";
        for(int i = 0; i < text.length(); i++) {
            if(text.substring(i, i + 1).equals("@")) {
                frames++;
            }
        }

        int[][][][] display = new int[frames][8][32][3];

        int frame = 0;
        while(text.indexOf("@") != -1) {
            String currentWindow = text.substring(0, text.indexOf("@"));

            // Values automatically 0
            int[][][] pixels = new int[8][32][3];
            
            // Turn the input text into display array
            // Starting at [1] not [0]
            int nextLetterCol = 1;
            for(int i = 0; i < currentWindow.length(); i++) {
                int[][] letter = ledLetters.letterMap.get(currentWindow.substring(i, i + 1));
                
                // Stop if letter would be out of bounds
                for(int row = 0; row < 8; row++) {
                    // Add letter in
                    for(int col = 0; col < letter[0].length; col++) {
                        if(nextLetterCol + col > 31) {
                            break;
                        }
                        pixels[row][col + nextLetterCol][0] = r * letter[row][col];
                        pixels[row][col + nextLetterCol][1] = g * letter[row][col];
                        pixels[row][col + nextLetterCol][2] = b * letter[row][col];
                    }
                    //nextLetterCol++;
                }
                nextLetterCol += letter[0].length + 1;
            }
            nextLetterCol++;
            
            //System.out.println(frame);
            //System.out.println(text);
            display[frame] = pixels;
            frame++;

            if(text.length() != text.indexOf("@") + 1) {
                text = text.substring(text.indexOf("@") + 1);
            } else {
                text =  "";
            }
        }
        return display;
    }
}
