package org.firstinspires.ftc.teamcode.FTC.Pixels.Types;

import org.firstinspires.ftc.teamcode.FTC.Pixels.Constants.BoardConstants;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Constants.PixelConstants;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Pixel;
import org.opencv.core.Point;
import org.opencv.core.Point3;

public class Hex {
    // x, y = odd-r offset
    public int x, y;

    // irlX = distance x on board plane from center of center april tag
    // irlY = distance y on board plane from center of center april tag
    public double irlX, irlY;
    public Point3 irl; // so that we dont have to reallocate every time
    public PixelColor color;

    public Hex(int x, int y) {
        this.x = x;
        this.y = y;

        color = PixelColor.empty;

        irlY = -(BoardConstants.tagCenterToFirstRowCenter + y * PixelConstants.vert);
        if (y % 2 == 0) {
            // even, only 6 pixels
            irlX = (x - 3) * PixelConstants.horz + (PixelConstants.horz / 2.0);
        } else {
            // odd, 7 pixels
            irlX = (x - 3) * PixelConstants.horz;
        }

        irl = new Point3(irlX, -irlY, 0);
    }

    @Override
    public int hashCode() { return x ^ (y << 2); }

    @Override
    public boolean equals(Object o) {
        if (o == this) return true;
        if (!(o instanceof Hex)) return false;

        Hex h = (Hex) o;
        return h.hashCode() == hashCode();
    }
}
