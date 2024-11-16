package frc.lib.models;

public class MotionParameters {

    private int _accerlaterion = 0;
    private int _cruiseVelocity = 0;
    private int _minVelocity = 0;
    private FXGains _gains = new FXGains(0, 0, 0, 0, 0, 0);

    public MotionParameters(int acceleration, int cruiseVelocity, FXGains gains) {
        _accerlaterion = acceleration;
        _cruiseVelocity = cruiseVelocity;
        _gains = gains;
    }

    public MotionParameters(int acceleration, int cruiseVelocity, FXGains gains, int minVelocity) {
        this(acceleration, cruiseVelocity, gains);
        _minVelocity = minVelocity;
    }

    public FXGains getGains() {
        return _gains;
    }

    public int getAcceleration() {
        return _accerlaterion;
    }

    public int getCruiseVelocity() {
        return _cruiseVelocity;
    }

    public int getMinVelocity() {
        return _minVelocity;
    }

    public void setGains(FXGains gains) {
        _gains = gains;
    }

    public void setAcceleration(int acceleration) {
        _accerlaterion = acceleration;
    }

    public void setCruiseVelocity(int cruiseVelocity) {
        _cruiseVelocity = cruiseVelocity;
    }

    public void setMinVelocity(int minVelocity) {
        _minVelocity = minVelocity;
    }
    
}
