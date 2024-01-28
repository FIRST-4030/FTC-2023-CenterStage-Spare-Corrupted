package com.example.meepmeep7462;

public class FieldSide {
    boolean isBlue;
    boolean isAudience;
    int spike;

    public int maxVel = 40, maxAccel = 40;
    public double trackWidth = 13.45;

    public FieldSide(final boolean _isBlue, final boolean _isAudience, final int _spike ) {
        isBlue     = _isBlue;
        isAudience = _isAudience;
        spike      = _spike;
    }
}
