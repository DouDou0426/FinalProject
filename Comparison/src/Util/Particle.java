package Util;

public class Particle {
    private double fitness;
    private double[] velocity;
    private double[] location;
    private int index;

    public Particle(){
        super();
    }

    public void setIndex(int index) {
        this.index = index;
    }

    public void setVelocity(double[] v) {
        this.velocity = v;
    }

    public void setLocation(double[] l) {
        this.location = l;
    }

    public double getFitness() {
        fitness = Parameters.evaluate(location);
        return fitness;
    }

    public double[] getVelocity() {
        return velocity;
    }

    public double[] getLocation() {
        return location;
    }

    public int getIndex() {
        return index;
    }
}