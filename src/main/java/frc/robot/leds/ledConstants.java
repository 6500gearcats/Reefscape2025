import java.util.ArrayList;
import java.util.HashMap;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class ledConstants {
    public static final int kLedPort = 99;
    public static final int kLedLength = 256;

    private String[] files = new File("frc/robot/leds/images/").list();

    /*
     * List of display options:
     * test --> displays hello world, two frames
     * setherror --> displays setherror, one frame, used to denote an error
     */


    public static int[][][][] makeDisplayArray(String choice) {

        int frames = 0;
        for (int i = 0; i < files.length; i++) {
            if (files[i].indexOf(choice) != -1) {
                frames++;
            }
        }

        int[][][][] display = new int[frames][][][];

        for(int i = 0; i < frames; i++) {
            BufferedImage img = ImageIO.read(new File("frc/robot/leds/images/" + choice + (i + 1) + ".png"));
            int[][][] pixels = new int[8][32][3];

            for (int x = 0; x < 8; x++) {
                for (int y = 0; y < 32; y++) {
                    int rgb = img.getRGB(y, x);
                    pixels[x][y][0] = (rgb >> 16) & 0xFF; // Red
                    pixels[x][y][1] = (rgb >> 8) & 0xFF;  // Green
                    pixels[x][y][2] = rgb & 0xFF;         // Blue
                }
            }
            /* 
            for(int x = 0; x < 8; x++) {
                for (int y = 0; y < 32; y++) {
                    System.out.println("Pixel at (" + x + ", " + y + "): (" + pixels[x][y][0] + ", " + pixels[x][y][1] + ", " + pixels[x][y][2] +")");
                }
            } */
            display[i] = pixels;
        }

        return display;
    }
}