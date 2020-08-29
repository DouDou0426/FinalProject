package Util;

public interface Parameters {
    int SWARM_SIZE = 500;
    int MAX_ITERATION = 200;
    double C1 = 2.0;
    double C2 = 2.0;
    double W_UPPERBOUND = 1.0;
    double W_LOWERBOUND = 0.0;

    double LOC_X_LOW = -5;
    double LOC_X_HIGH = 5;
    double LOC_Y_LOW = -5;
    double LOC_Y_HIGH = 5;
    double VEL_LOW = -1;
    double VEL_HIGH = 1;

    double ERR_TOLERANCE = 1E-10;

    //Algorithm.FSO
    double C3 = 0.05;
    int NumNeighbor = 7; //Algorithm.StarlingOpt

    //Algorithm.FlockOpt
    int Npicks = 800; // 2 * SWARM_SIZE

    double V0 = 1.4; // max velocity for each agent
    double VA = 2.8; // attraction velocity, 2V0

    double rRpct = 0.08; // repulsion radius
    double rOpct = 0.5; // orientation radius
    double rApct = 1; // attraction radius

    double Alpha = 0.5; // weight between bird model velocity term and pbest update term
    double ADecay = 0.8; // alpha decay term

    double PB1 = 0.0;

    //Algorithm.StarlingPSO
    int MAX_NUM = 14;
    int STAGNANT_LIMIT = 2; //Algorithm.StarlingOpt

    //Algorithm.StarlingOpt
    double Ws = 0.1;
    double Wc = 0.1;
    double m = 1;



    static double evaluate(double[] location) {
        double result;
        double x = location[0]; // the "x" part of the location
        double y = location[1]; // the "y" part of the location

        result = Math.pow(2.8125 - x + x * Math.pow(y, 4), 2) +
                Math.pow(2.25 - x + x * Math.pow(y, 2), 2) +
                Math.pow(1.5 - x + x * y, 2);


        //Beale function ???
        //result = Math.pow(1.5-x+ x*y*y,2) + Math.pow(2.25-x+x*y*y,2) + Math.pow(2.625-x+x*y*y*y,2);

        //Sphere function
        //result = Math.pow(x,2) + Math.pow(y,2);

        //F2: Shifted Schwefel’s Problem 1.2
     //   result =Math.pow(x,2) + Math.pow(x+y,2);

        //Himmelblau's function
//        result = Math.pow(Math.pow(x,2) + y - 11, 2) + Math.pow(x+Math.pow(y,2)-7,2);

        //Rosenbrock function constrained to a disk
  //      result = Math.pow(1-x,2)+ 100 * Math.pow(y-Math.pow(x,2),2);

        //Schaffer function N. 2 too good
//        result = 0.5 + (Math.pow(Math.sin(Math.pow(x,2)-Math.pow(y,2)),2) - 0.5)/
//                (1+0.001*Math.pow(Math.pow(x,2) + Math.pow(y,2),2));

        //Lévi function N.13
//        result = Math.pow(Math.sin(3*Math.PI*x) ,2) + Math.pow(x-1,2)*(1+Math.pow(Math.sin(3*Math.PI*y) ,2))
//                +Math.pow(y-1,2)*(1+Math.pow(Math.sin(2*Math.PI*y) ,2));

        //Mishra's Bird function - constrained
//        result = Math.sin(y) * Math.exp(Math.pow(1-Math.cos(x),2)) + Math.cos(x) * Math.exp(Math.pow(1-Math.sin(y),2))
//                + Math.pow(x-y,2) + 106.7645367;

        //Rastrigin function
//        result = 20 + Math.pow(x,2) - 10 * Math.cos(2*Math.PI*x) + Math.pow(y,2) - 10 * Math.cos(2*Math.PI*y);

        //Matyas function
//        result = 0.26*(x*x + y*y) -0.48*x*y;

        //Booth Function
//       result = Math.pow(x+2*y-7,2) + Math.pow(2*x+y-5,2);

        //Griewank Function
       // result = Math.pow(x,2)/4000 + Math.pow(y,2)/4000 - Math.cos(x)*Math.cos(y/2) + 1;

        return result;
    }
}
