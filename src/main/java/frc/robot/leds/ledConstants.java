package frc.robot.leds;
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

     public static String[] LedOptions = {"test", "setherror", "pongBlueWin", "pranavCreeper", "6500Blue", "6500Red", "6500Teal"};

    
    public static int[][][][] makeDisplayArray(String choice) {

        int frames = 0;
        for (int i = 0; i < files.length; i++) {
            if (files[i].indexOf(choice) != -1) {
                frames++;
            }
        }

        int[][][][] display = new int[frames][][][];
        System.out.println(frames);

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
}
