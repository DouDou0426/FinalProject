package Algorithm;

import Util.*;

import java.util.Random;
import java.util.Vector;

public class FlockOpt implements Parameters {
    private Vector<Particle> swarm = new Vector<>();
    private Vector<double[]> p = new Vector<>(); //pi, pbest term
    private double[] pbest = new double[Parameters.SWARM_SIZE]; // personal best value
    private Vector<double[]> xbest = new Vector<>(); // xpi
    private double alpha = Parameters.Alpha;
    private double err = 10000;

    Random random = new Random();

    public void execute(){
        initSwarm();

        // main loop
        double[] vel = new double[2];
        Vector<double[]> VUpdate;
        Vector<Vector<double[]>> VUpdateList;

        Particle best_so_far = new Particle();

        int i , j;
        int evaluationTime = Parameters.SWARM_SIZE/20;

        double[][] distance = new double[Parameters.SWARM_SIZE][Parameters.SWARM_SIZE];

        for (int iter = 0; (iter < Parameters.MAX_ITERATION) && (err > Parameters.ERR_TOLERANCE); iter++) {
            VUpdateList = new Vector<>();

            // vi_up = vi
            for (int k = 0; k < Parameters.SWARM_SIZE; k++) {
                vel = new double[2];
                VUpdate = new Vector<>();
                vel[0] = swarm.get(k).getVelocity()[0];
                vel[1] = swarm.get(k).getVelocity()[1];

                VUpdate.add(vel);
                VUpdateList.add(VUpdate);
            }

            // calculate  distance between all agents
            calDistance(distance, swarm);

            for (int ii = 0; ii < Parameters.Npicks; ii++) {
                i = random.nextInt(Parameters.SWARM_SIZE); // pick agent at random i
                j = roulette(distance[i]); // pick partener based on roulette
                double U = random.nextDouble();

                //update xpi
                if(pbest[j] < pbest[i]){
                    xbest.get(i)[0] = xbest.get(j)[0];
                    xbest.get(i)[1] = xbest.get(j)[1];
                }

                vel[0] = VUpdateList.get(i).get(0)[0];
                vel[1] = VUpdateList.get(i).get(0)[1];

                if (distance[i][j] < Parameters.rRpct){
                    vel[0] = vel[0] + alpha * Parameters.V0 * U
                            * (swarm.get(i).getLocation()[0] - swarm.get(j).getLocation()[0]) / distance[i][j]
                            + (1 - alpha) * Parameters.PB1 * U * (p.get(i)[0] - swarm.get(i).getLocation()[0]);
                    vel[1] = vel[1] + alpha * Parameters.V0 * U
                            * (swarm.get(i).getLocation()[1] - swarm.get(j).getLocation()[1]) / distance[i][j]
                            + (1 - alpha) * Parameters.PB1 * U * (p.get(i)[0] - swarm.get(i).getLocation()[1]);
                }
                else if ((distance[i][j] < Parameters.rOpct) && (distance[i][j] > Parameters.rRpct)){
                    vel[0] = vel[0] + alpha * Parameters.V0 * U * swarm.get(j).getVelocity()[0]
                            / Math.sqrt(Math.pow(swarm.get(j).getVelocity()[0], 2) + Math.pow(swarm.get(j).getVelocity()[1], 2))
                            + (1 - alpha) * Parameters.PB1 * U * (p.get(i)[0] - swarm.get(i).getLocation()[0]);
                    vel[1] = vel[1] + alpha * Parameters.V0 * U * swarm.get(j).getVelocity()[1]
                            / Math.sqrt(Math.pow(swarm.get(j).getVelocity()[0], 2) + Math.pow(swarm.get(j).getVelocity()[1], 2))
                            + (1 - alpha) * Parameters.PB1 * U * (p.get(i)[0] - swarm.get(i).getLocation()[1]);
                }else {
                    vel[0] = vel[0] + alpha * 2 * Parameters.VA * U
                            * (swarm.get(j).getLocation()[0] - swarm.get(i).getLocation()[0]) / distance[i][j]
                            + (1 - alpha) * Parameters.PB1 * U * (p.get(i)[0] - swarm.get(i).getLocation()[0]);
                    vel[1] = vel[1] + alpha * 2 * Parameters.VA * U
                            * (swarm.get(j).getLocation()[0] - swarm.get(i).getLocation()[1]) / distance[i][j]
                            + (1 - alpha) * Parameters.PB1 * U * (p.get(i)[0] - swarm.get(i).getLocation()[1]);
                }

                VUpdate = VUpdateList.get(i);
                VUpdate.add(vel);
                VUpdateList.set(i,VUpdate);

                if ((ii !=0) && (ii % evaluationTime == 0)){
                    alpha = alpha * Parameters.ADecay;
                }
            }

            for (int k = 0; k < Parameters.SWARM_SIZE; k++) {
                double[] newVel = new double[2];

                // vi = average of vi_up
                Vector<double[]> newVUpdate = VUpdateList.get(k);
                double[] vsum = new double[2];
                for (double[] vi: newVUpdate) {
                    vsum[0] = vsum[0] + vi[0];
                    vsum[1] = vsum[1] + vi[1];
                }
                newVel[0] = vsum[0]/newVUpdate.size();
                newVel[1] = vsum[1]/newVUpdate.size();
                swarm.get(k).setVelocity(newVel);

                // xi = xi + vi
                double[] newLoc = new double[2];
                newLoc[0] = swarm.get(k).getLocation()[0] + newVel[0];
                newLoc[1] = swarm.get(k).getLocation()[1] + newVel[1];
                swarm.get(k).setLocation(newLoc);

                // pbest term should be used
                p.get(k)[0] = xbest.get(k)[0] - newLoc[0];
                p.get(k)[1] = xbest.get(k)[1] - newLoc[1];

                // update personal best value
                if (pbest[k] > swarm.get(k).getFitness()){
                    pbest[k] = swarm.get(k).getFitness();
                    err = pbest[k];
                }

            }
        }

    }

    public void initSwarm(){
        Particle particle;
        for (int i = 0; i < Parameters.SWARM_SIZE; i++) {
            particle = new Particle();

            double[] loc = new double[2];
            loc[0] = Parameters.LOC_X_LOW + random.nextDouble() * (Parameters.LOC_X_HIGH - Parameters.LOC_X_LOW);
            loc[1] = Parameters.LOC_Y_LOW + random.nextDouble() * (Parameters.LOC_Y_HIGH - Parameters.LOC_Y_LOW);

            double[] v = new double[2];
            v[0] = -Parameters.V0 + random.nextDouble() * 2 * Parameters.V0;
            v[1] = -Parameters.V0 + random.nextDouble() * 2 * Parameters.V0;

            particle.setLocation(loc);
            particle.setVelocity(v);
            swarm.add(particle);
            xbest.add(loc);

            double[] newP = new double[2];
            newP[0] = xbest.get(i)[0] - particle.getLocation()[0];
            newP[1] = xbest.get(i)[1] - particle.getLocation()[1];
            p.add(newP);

            pbest[i] = particle.getFitness();
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

    public int roulette(double[] d){
        double res = 0;
        double rand = random.nextDouble();

        for (int k = 0; k < Parameters.SWARM_SIZE; k++) {
            res = res + d[k];
        }

        double ans = res * rand;
        double sum = 0;

        for (int k = 0; k < Parameters.SWARM_SIZE; k++) {
            sum = sum + d[k];
            if (sum > ans){
                return k;
            }
        }
        return 0;
    }

}