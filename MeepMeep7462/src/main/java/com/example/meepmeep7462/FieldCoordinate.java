package com.example.meepmeep7462;

public class FieldCoordinate {
    double x, y;
    int heading;

    public FieldCoordinate(final double _x, final double _y) {
        x = _x;
        y = _y;
    }

    public FieldCoordinate(final double _x, final double _y, final int _heading) {
        x = _x;
        y = _y;
        heading = _heading;
    }
}
