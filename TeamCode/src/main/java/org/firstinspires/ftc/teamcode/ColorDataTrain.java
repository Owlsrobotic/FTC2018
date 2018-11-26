package org.firstinspires.ftc.teamcode;

import java.io.Serializable;

public class ColorDataTrain implements Serializable {
    public static int COLOR_NONE = 0;
    public static int COLOR_WHITE = 1;
    public static int COLOR_GOLD = 2;

    public float[] hsl;
    public int color;

    public ColorDataTrain(float[] hsl, int color) {
        this.hsl = hsl;
        this.color = color;
    }
}
