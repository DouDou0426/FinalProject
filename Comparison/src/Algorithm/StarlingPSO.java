package Algorithm;

import Util.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;
import java.util.Vector;

public class StarlingPSO implements Parameters {
    private Vector<Particle> swarm = new Vector<>();
    private double[] pBest = new double[Parameters.SWARM_SIZE];
    private ArrayList<double[]> pBestLocation = new ArrayList<>();
    private double[] fitnessValueList = new double[Parameters.SWARM_SIZE];

    Particle gBest = new Particle();

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
        int stagnant_count = 0;
        double previous_gBest;
        Particle best_so_far = new Particle();

        //initialize gBest
        int bestIndex = getBestPos(fitnessValueList);
        gBest.setLocation(swarm.get(bestIndex).getLocation());
        gBest.setVelocity(swarm.get(bestIndex).getVelocity());

        best_so_far.setVelocity(gBest.getVelocity());
        best_so_far.setLocation(gBest.getLocation());

        previous_gBest = gBest.getFitness();

        int iter = 0;
        while (iter < Parameters.MAX_ITERATION && err > Parameters.ERR_TOLERANCE){

            //Inertia Weight
            w = Parameters.W_UPPERBOUND - (((double) iter) / Parameters.MAX_ITERATION) * (Parameters.W_UPPERBOUND - Parameters.W_LOWERBOUND);

            //update velocity and position
            double r1 = random.nextDouble();
            double r2 = random.nextDouble();

            //normal Algorithm.PSO
            for (int i = 0; i < Parameters.SWARM_SIZE; i++) {
                Particle p = swarm.get(i);

                double[] vel = new double[2];
                vel[0] = w * p.getVelocity()[0]
                        + Parameters.C1 * r1 * (pBestLocation.get(i)[0] - p.getLocation()[0])
                        + Parameters.C2 * r2 * (gBest.getLocation()[0] - p.getLocation()[0]);
                vel[1] = w * p.getVelocity()[1]
                        + Parameters.C1 * r1 * (pBestLocation.get(i)[1] - p.getLocation()[1])
                        + Parameters.C2 * r2 * (gBest.getLocation()[1] - p.getLocation()[1]);
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

            //update gBest
            updateFitnessValueList();
            bestIndex = getBestPos(fitnessValueList);
            if(fitnessValueList[bestIndex] < gBest.getFitness()){
                gBest.setLocation(swarm.get(bestIndex).getLocation());
                gBest.setVelocity(swarm.get(bestIndex).getVelocity());
            }

            if (gBest.getFitness() < best_so_far.getFitness()){
                best_so_far.setVelocity(gBest.getVelocity());
                best_so_far.setLocation(gBest.getLocation());
            }

            if (gBest.getFitness() >= previous_gBest){
                stagnant_count++;
            }else {
                stagnant_count = 0;
            }

            previous_gBest = gBest.getFitness();

            //Starling Algorithm.PSO
            if (stagnant_count > Parameters.STAGNANT_LIMIT){
                double[][] distance = new double[Parameters.SWARM_SIZE][Parameters.SWARM_SIZE];
                Vector<Particle> copy;
                Vector<Particle> newSwarm = new Vector<>();
                Vector<Integer> neighbors;
                double newGbest = gBest.getFitness();

                for (int i = 0; i < Parameters.MAX_NUM; i++) {
                    copy = new Vector<>();
                    copyVector(swarm, copy);
                    calDistance(distance, copy);
                    double newPbest = 100;

                    for (int j = 0; j < Parameters.SWARM_SIZE; j++) {
                        neighbors = new Vector<>();
                        sortNeighbors(distance[j], j, neighbors);
                        Particle newP = copy.get(j);

                        double[] vel = new double[2];
                        vel[0] = newP.getVelocity()[0] + random.nextDouble() / 7
                                * calSumNeighbor("velocity", 0 , copy,neighbors);
                        vel[1] = newP.getVelocity()[1] + random.nextDouble() / 7
                                * calSumNeighbor("velocity", 1 , copy,neighbors);
                        newP.setVelocity(vel);

                        double[] loc = new double[2];
                        loc[0] = newP.getLocation()[0] + (random.nextDouble() * 2 - 1) / 7
                                * calSumNeighbor("location", 0 , copy,neighbors);
                        loc[1] = newP.getLocation()[1] + (random.nextDouble() * 2 - 1) / 7
                                * calSumNeighbor("location", 1 , copy,neighbors);
                        newP.setLocation(loc);

                        updateDistance(distance, copy, j);

                        if (newP.getFitness() < newPbest){
                            newPbest = newP.getFitness();
                        }
                    }

                    if (newPbest < newGbest){
                        newGbest = newPbest;
                        copyVector(copy, newSwarm);
                    }
                }

                copyVector(newSwarm, swarm);

                stagnant_count = 0;
            }

            err = Parameters.evaluate(best_so_far.getLocation()) - 0;

            iter++;

//            System.out.println("ITERATION " + iter + ": ");
//            System.out.println("     Best X: " + best_so_far.getLocation()[0]);
//            System.out.println("     Best Y: " + best_so_far.getLocation()[1]);
//            System.out.println("     Value: " + Util.Parameters.evaluate(best_so_far.getLocation()));
        }

//        System.out.println("\nSolution founded, the solutions is:");
//        System.out.println("     Best X: " + best_so_far.getLocation()[0]);
//        System.out.println("     Best Y: " + best_so_far.getLocation()[1] + "\r");
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

    public void copyVector(Vector<Particle> origin, Vector<Particle>copy){
        Particle p = new Particle();
        for (Particle particle : origin) {
            p.setLocation(particle.getLocation());
            p.setVelocity(particle.getVelocity());
            copy.add(p);
        }
    }

    public void calDistance(double[][] distance, Vector<Particle>set){
        double res;
        for (int i = 0; i < set.size(); i++) {
            for (int j = 0; j < set.size(); j++) {
                res = Math.pow(set.get(i).getLocation()[0] - set.get(j).getLocation()[0],2)
                        + Math.pow(set.get(i).getLocation()[1] - set.get(j).getLocation()[1],2);
                distance[i][j] = Math.sqrt(res);
            }
        }
    }

    public void updateDistance(double[][]distance, Vector<Particle>set, int k){
        double res;
        for (int i = 0; i < set.size() && i != k; i++) {
            res = Math.pow(set.get(k).getLocation()[0] - set.get(i).getLocation()[0],2)
                    + Math.pow(set.get(k).getLocation()[1] - set.get(i).getLocation()[1],2);
            distance[k][i] = Math.sqrt(res);
        }
    }

    public void sortNeighbors(double[] list,int k,Vector<Integer> result){
        double[] newList = new double[Parameters.SWARM_SIZE];
        for (int i = 0; i < Parameters.SWARM_SIZE; i++) {
            newList[i] = list[i];
        }
        Arrays.sort(newList);
        double ans = newList[7];
        for (int i = 0; i < Parameters.SWARM_SIZE && i != k; i++) {
            if (list[i] < ans){
                result.add(i);
            }
        }
    }

    public double calSumNeighbor(String type, int k, Vector<Particle>origin, Vector<Integer>neighbors){
        double sum = 0;
        if (type.equals("velocity")){
            for (Integer neighbor : neighbors) {
                sum = sum + origin.get(neighbor).getVelocity()[k];
            }
        }
        if(type.equals("location")){
            for (Integer neighbor : neighbors) {
                sum = sum + origin.get(neighbor).getLocation()[k];
            }
        }
        return sum;
    }
}
