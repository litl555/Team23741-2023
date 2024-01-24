package org.firstinspires.ftc.teamcode.FTC.Localization;

import java.util.Comparator;

public class LoggerData {
    public final String name, section;
    public final Object value;

    public LoggerData(String name, Object value, String section) {
        this.name = name;
        this.value = value;
        this.section = section;
    }

    @Override
    public int hashCode() {
        int hash = 7;
        int len = name.length();
        for (int i = 0; i < len; i++) hash = hash * 31 + name.charAt(i);

        return hash;
    }

    @Override // dont question it
    public boolean equals(Object o) { return o.hashCode() == this.hashCode(); }
}
