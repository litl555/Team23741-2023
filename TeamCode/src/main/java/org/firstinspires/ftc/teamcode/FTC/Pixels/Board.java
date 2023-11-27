package org.firstinspires.ftc.teamcode.FTC.Pixels;

import org.firstinspires.ftc.teamcode.FTC.Pixels.Types.Hex;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Types.PixelColor;

import java.util.ArrayList;
import java.util.Hashtable;
import java.util.Map;

public class Board {
    private int nrow = 11, ncolL = 7, ncolS = 6;
    // using offset coordinates
    private Hex[][] _board = new Hex[nrow][ncolL];

    public Board() {
        // 0 is bottom, 11 is top
        for (int row = 0; row < nrow; row++) {
            for (int col = 0; col < ncol(row); col++) {
                Hex h = new Hex(col, row);

                _board[row][col] = h;
            }
        }
    }

    public ArrayList<Hex> getValidPositions() { // this can be optimized but it should be fast enough?
        ArrayList<Hex> out = new ArrayList<>();

        // 1. all positions at the very bottom of the board
        // 2. pixels that are supported by two other pixels
        // 3. pixels on long rows at the very start/end that are supported by one pixel

        // 1
        for (int i = 0; i < ncolS; i++) {
            Hex h = _board[0][i];
            if (h.color == PixelColor.empty) out.add(h);
        }

        // 2
        for (int r = 0; r < nrow; r++) {
            for (int c = 0; c < ncol(r); c++) {
                if (_board[r][c].color != PixelColor.empty) continue;

                // L -> (-1, -1), (0, -1)
                // S -> (0, -1), (1, -1)

                // check
                int[] v1 = ncol(r) == ncolL ? dirL[4] : dirS[3];
                int[] v2 = dirL[3];

                Hex h1 = tryGetHex(r + v1[1], c + v1[0]);
                Hex h2 = tryGetHex(r + v2[1], c + v2[0]);

                if (h1 == null || h2 == null) continue;
                if (h1.color == PixelColor.empty || h2.color == PixelColor.empty) continue;

                out.add(_board[r][c]);
            }
        }

        // 3
        for (int r = 1; r < nrow; r += 2) {
            // right edge
            if (_board[r][ncolL - 1].color == PixelColor.empty && _board[r - 1][ncolL - 1 - 1].color != PixelColor.empty) out.add(_board[r][ncolL - 1]);
            // left edge
            if (_board[r][0].color == PixelColor.empty && _board[r - 1][0].color != PixelColor.empty) out.add(_board[r][0]);
        }

        return out;
    }

    public Hex tryGetHex(int r, int c) {
        if (!inBounds(r, c)) return null;

        return _board[r][c];
    }

    public boolean inBounds(int r, int c) {
        if (r < 0 || c < 0) return false;
        if (r >= nrow) return false;
        if (c >= ncol(r)) return false;

        return true;
    }

    // hexagons :(
    private int[][] dirL = new int[][] {
            {-1, 1}, {0, 1}, {1, 0}, {0, -1}, {-1, -1}, {-1, 0}
    };

    private int[][] dirS = new int[][] {
            {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, 0}
    };

    private int ncol(int row) { return row % 2 == 0 ? ncolS : ncolL; }
}
