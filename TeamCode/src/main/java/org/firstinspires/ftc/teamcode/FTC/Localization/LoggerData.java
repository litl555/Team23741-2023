package org.firstinspires.ftc.teamcode.FTC.Localization;

public class LoggerData {
    public final String name, section;
    public final Object value;
    public final int order;

    public LoggerData(String name, Object value, String section, int order) {
        this.name = name;
        this.value = value;
        this.section = section;
        this.order = order;
    }

    public LoggerData(String name, Object value, String section) {
        this.name = name;
        this.value = value;
        this.section = section;
        this.order = 10_000;
    }
}
