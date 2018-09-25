package org.firstinspires.ftc.teamcode;

public class PosRot {
    //Meters and degrees
    public double x = 0.0;
    public double y = 0.0;
    public double rot = 0.0;

    public PosRot() {
        this.x = 0;
        this.y = 0;
        this.rot = 0;
    }

    public PosRot(double x, double y, double rot) {
        this.x = x;
        this.y = y;
        this.rot = rot;
    }

    public void setPosition(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void setRot(double angle) {
        this.rot = angle;
    }

    public PosRot rotate(double angle) {
        double cos = Math.cos(Math.toRadians(angle));
        double sin = Math.sin(Math.toRadians(angle));

        return new PosRot(x * cos + -1.0 * sin * y, sin * x + cos * y, rot);
    }
}
