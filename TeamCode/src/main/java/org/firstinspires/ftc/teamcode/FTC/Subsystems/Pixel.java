package org.firstinspires.ftc.teamcode.FTC.Subsystems;

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
