package Algorithm;

import Util.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;
import java.util.Vector;

public class StarlingOpt implements Parameters {
    private Vector<Particle> swarm = new Vector<>();
    private double[] pBest = new double[Parameters.SWARM_SIZE];
    private ArrayList<double[]> pBestLocation = new ArrayList<>();
    private double gBest;
    private double[] gBestLocation;
    private double[] fitnessValueList = new double[Parameters.SWARM_SIZE];
    private double[][] distance = new double[Parameters.SWARM_SIZE][Parameters.SWARM_SIZE];

    Random random = new Random();

   // StringBuilder res = new StringBuilder();

    public int execute() {
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

        double previous_gbest = 1000;
        int stagnent_count = 0;

        int bestIndex = getBestPos(fitnessValueList);
        gBest = fitnessValueList[bestIndex];
        gBestLocation = swarm.get(bestIndex).getLocation();

        int iter = 0;
        while(iter < Parameters.MAX_ITERATION && err > Parameters.ERR_TOLERANCE){

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

                if (fitnessValueList[i] < pBest[i]) {
                    pBest[i] = fitnessValueList[i];
                    pBestLocation.set(i, swarm.get(i).getLocation());
                }

            }

            updateFitnessValueList();

            //update gbest
            bestIndex = getBestPos(fitnessValueList);
            if (fitnessValueList[bestIndex] < gBest) {
                gBest = fitnessValueList[bestIndex];
                gBestLocation = swarm.get(bestIndex).getLocation();
            }

            if (gBest >= previous_gbest) {
                stagnent_count = stagnent_count + 1;
            } else {
                stagnent_count = 0;
            }
            previous_gbest = gBest;

            //Algorithm.StarlingOpt
            if (stagnent_count > Parameters.STAGNANT_LIMIT) {
                for (int i = 0; i < Parameters.SWARM_SIZE; i++) {
                    updateDistance(distance, swarm, i);
                }

                for (int i = 0; i < Parameters.SWARM_SIZE; i++) {
                    updateStarlingOpt(swarm.get(i), distance, swarm);
                }

                updateFitnessValueList();
                stagnent_count = 0;

            }


            err = Parameters.evaluate(gBestLocation) - 0;
//
//            System.out.println("ITERATION " + iter + ": ");
//            System.out.println("     Best X: " + gBestLocation[0]);
//            System.out.println("     Best Y: " + gBestLocation[1]);
//            System.out.println("     Value: " + Util.Parameters.evaluate(gBestLocation));

            iter++;

   //         res.append(iter).append("\t").append(gBest).append("\r\n");

        }

//        System.out.println("\nSolution founded, the solutions is:");
//        System.out.println("     Best X: " + gBestLocation[0]);
//        System.out.println("     Best Y: " + gBestLocation[1]);

        return iter;
        //return res.toString();
    }

    public void initSwarm() {
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
            p.setIndex(i);
            swarm.add(p);
            p = new Particle();
        }
    }

    public void updateFitnessValueList() {
        for (int i = 0; i < Parameters.SWARM_SIZE; i++) {
            fitnessValueList[i] = swarm.get(i).getFitness();
        }
    }

    public int getBestPos(double[] list) {
        int pos = 0;
        double bestValue = list[0];

        for (int i = 0; i < list.length; i++) {
            if (list[i] < bestValue) {
                bestValue = list[i];
                pos = i;
            }
        }
        return pos;
    }

    public void updateStarlingOpt(Particle p, double[][] d, Vector<Particle> set) {
        int index = p.getIndex();
        Particle newP = new Particle();

        double[] newList = new double[Parameters.SWARM_SIZE];
        for (int i = 0; i < Parameters.SWARM_SIZE; i++) {
            newList[i] = d[index][i];
        }
        Arrays.sort(newList);
        double ans = newList[7];
        double dij = 0;

        double[] force = new double[2];
        force[0] = 0;
        force[1] = 0;

        for (int i = 0; i < Parameters.SWARM_SIZE; i++) {
            dij = d[index][i];
            if (dij < ans && dij > 0) {
                if (set.get(i).getFitness() < set.get(index).getFitness()){
                    force[0] = force[0] + Parameters.Ws / Parameters.NumNeighbor * (set.get(i).getLocation()[0] - p.getLocation()[0]) / dij;
                    force[1] = force[1] + Parameters.Ws / Parameters.NumNeighbor * (set.get(i).getLocation()[1] - p.getLocation()[1]) / dij;
                } else{
                    force[0] = force[0] - Parameters.Wc / Parameters.NumNeighbor * (set.get(i).getLocation()[0] - p.getLocation()[0]) / dij;
                    force[1] = force[1] - Parameters.Wc / Parameters.NumNeighbor * (set.get(i).getLocation()[1] - p.getLocation()[1]) / dij;
                }
            }
        }

        double[] newVel = new double[2];

        newVel[0] = p.getVelocity()[0] + force[0] / Parameters.m;
        newVel[1] = p.getVelocity()[1] + force[1] / Parameters.m;
        newP.setVelocity(newVel);

        double[] newLoc = new double[2];
        newLoc[0] = p.getLocation()[0] + newVel[0];
        newLoc[1] = p.getLocation()[1] + newVel[1];
        newP.setLocation(newLoc);

        if (newP.getFitness() < fitnessValueList[index]){
            pBest[index] = p.getFitness();
            pBestLocation.set(index, newLoc);

            p.setVelocity(newVel);
            p.setLocation(newLoc);
        }
        if (newP.getFitness() < gBest){
            gBest = p.getFitness();
            gBestLocation[0] = newLoc[0];
            gBestLocation[1] = newLoc[1];
        }
    }


    public void updateDistance(double[][] d, Vector<Particle> set, int index){
        double res = 0;
        for (int i = 0; i < Parameters.SWARM_SIZE; i++) {
            res = Math.pow(set.get(i).getLocation()[0] - set.get(index).getLocation()[0] ,2)
                    + Math.pow(set.get(i).getLocation()[1] - set.get(index).getLocation()[1] ,2);
            d[index][i] = Math.sqrt(res);
            d[i][index] = Math.sqrt(res);
        }
    }


}
