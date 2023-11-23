package org.firstinspires.ftc.teamcode.FTC.Pixels.Types;

import org.firstinspires.ftc.teamcode.FTC.Pixels.Constants.BoardConstants;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Constants.PixelConstants;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Pixel;
import org.opencv.core.Point;
import org.opencv.core.Point3;

public class Hex {
    // q, r, s = cube
    // x, y = odd-r offset
    public int q, r, s, x, y;

    // irlX = distance x on board plane from center of center april tag
    // irlY = distance y on board plane from center of center april tag
    public double irlX, irlY;
    public Point3 irl; // so that we dont have to reallocate every time

    public Hex(int q, int r) {
        this.q = q;
        this.r = r;
        this.s = -q - r;

        this.x = q + (r - (r % 2)) / 2;
        this.y = r;

        irlY = BoardConstants.tagCenterToFirstRowCenter + y * PixelConstants.vert;
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
    public int hashCode() {
        // szudziks hashing function
        // is overkill lmao
        int A = q >= 0 ? 2 * q : -2 * q - 1;
        int B = r >= 0 ? 2 * r : -2 * r - 1;
        int C = ((A >= B ? A * A + A + B : A + B * B) / 2);

        return q < 0 && r < 0 || q >= 0 && r >= 0 ? C : -C - 1;
    }

    @Override
    public boolean equals(Object o) {
        if (o == this) return true;
        if (!(o instanceof Hex)) return false;

        Hex h = (Hex) o;
        return h.hashCode() == hashCode();
    }
}
