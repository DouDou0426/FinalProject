package Algorithm;

import Util.*;

import java.util.ArrayList;
import java.util.Random;
import java.util.Vector;

public class PSO implements Parameters {
    private Vector<Particle> swarm = new Vector<>();
    private double[] pBest = new double[Parameters.SWARM_SIZE];
    private ArrayList<double[]> pBestLocation = new ArrayList<>();
    private double gBest;
    private double[] gBestLocation;
    private double[] fitnessValueList = new double[Parameters.SWARM_SIZE];

    Random random = new Random();

    public int execute(){
        //initialize the population
        initSwarm();
        updateFitnessValueList();

        for (int i = 0; i < Parameters.SWARM_SIZE; i++) {
            pBest[i] = fitnessValueList[i];
            pBestLocation.add(swarm.get(i).getLocation());
        }

        //execution
        double err = 10000;
        double w;

        int iter = 0;

        while(iter < Parameters.MAX_ITERATION && err > Parameters.ERR_TOLERANCE) {

            //update gbest
            int bestIndex = getBestPos(fitnessValueList);
            if(iter == 0 || fitnessValueList[bestIndex] < gBest){
                gBest = fitnessValueList[bestIndex];
                gBestLocation = swarm.get(bestIndex).getLocation();
            }

            //Inertia Weight
            w = Parameters.W_UPPERBOUND - (((double) iter) / Parameters.MAX_ITERATION) * (Parameters.W_UPPERBOUND - Parameters.W_LOWERBOUND);

            //update velocity and position
            double r1 = random.nextDouble();
            double r2 = random.nextDouble();

            for (int i = 0; i < Parameters.SWARM_SIZE; i++) {
                Particle p = swarm.get(i);

                double[] vel = new double[2];
                vel[0] = w * p.getVelocity()[0]
                        + Parameters.C1 * r1 * (pBestLocation.get(i)[0] - p.getLocation()[0])
                        + Parameters.C2 * r2 * (gBestLocation[0] - p.getLocation()[0]);
                vel[1] = w * p.getVelocity()[1]
                        + Parameters.C1 * r1 * (pBestLocation.get(i)[1] - p.getLocation()[1])
                        + Parameters.C2 * r2 * (gBestLocation[1] - p.getLocation()[1]);
                p.setVelocity(vel);

                double[] loc = new double[2];
                loc[0] = p.getLocation()[0] + vel[0];
                loc[1] = p.getLocation()[1] + vel[1];
                p.setLocation(loc);

                if (fitnessValueList[i]<pBest[i]){
                    pBest[i] = fitnessValueList[i];
                    pBestLocation.set(i, swarm.get(i).getLocation());
                }
            }

            iter++;
            err = Parameters.evaluate(gBestLocation) - 0;

//            System.out.println("ITERATION " + iter + ": ");
//            System.out.println("     Best X: " + gBestLocation[0]);
//            System.out.println("     Best Y: " + gBestLocation[1]);
//            System.out.println("     Value: " + Util.Parameters.evaluate(gBestLocation));

            updateFitnessValueList();
        }

//        System.out.println("\nSolution founded, the solutions is:");
//        System.out.println("     Best X: " + gBestLocation[0]);
//        System.out.println("     Best Y: " + gBestLocation[1]);
        return iter;

    }

    public void initSwarm(){
        Particle p = new Particle();
        for (int i = 0; i < Parameters.SWARM_SIZE; i++) {
            double[] loc = new double[2];
            loc[0] = Parameters.LOC_X_LOW + random.nextDouble() * (Parameters.LOC_X_HIGH - Parameters.LOC_X_LOW);
            loc[1] = Parameters.LOC_Y_LOW + random.nextDouble() * (Parameters.LOC_Y_HIGH - Parameters.LOC_Y_LOW);

            double[] v = new double[2];
            v[0] = Parameters.VEL_LOW + random.nextDouble() * (Parameters.VEL_HIGH - Parameters.VEL_LOW);
            v[1] = Parameters.VEL_LOW + random.nextDouble() * (Parameters.VEL_HIGH - Parameters.VEL_LOW);

            p.setLocation(loc);
            p.setVelocity(v);
            swarm.add(p);
            p = new Particle();
        }
    }

    public void updateFitnessValueList(){
        for (int i = 0; i < Parameters.SWARM_SIZE; i++) {
            fitnessValueList[i] = swarm.get(i).getFitness();
        }
    }

    public int getBestPos(double[] list){
        int pos = 0;
        double bestValue = list[0];

        for (int i = 0; i < list.length; i++) {
            if (list[i]< bestValue){
                bestValue = list[i];
                pos = i;
            }
        }
        return pos;
    }
}
