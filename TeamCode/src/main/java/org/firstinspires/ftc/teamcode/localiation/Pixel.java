package org.firstinspires.ftc.teamcode.localiation;

import android.graphics.Color;

public class Pixel {
    enum Color {
        PURPLE,
        WHITE,
        YELLOW,
        GREEN
    }

    Color color;

    public Pixel(Color color) {
        this.color = color;
    }
}
