import Algorithm.*;
import Util.*;

public class Main {
    public static void main(String[] args) {
        StringBuilder iterRes = new StringBuilder();
        int iteration;

        StringBuilder timeRes = new StringBuilder();
        double startTime;
        double endTime;


        int sOptRes;
        StringBuilder sOptAns = new StringBuilder();

        for (int i = 0; i < 100; i++) {
            PSO pso = new PSO();
            FSO fso = new FSO();
            StarlingPSO sPSO = new StarlingPSO();
            StarlingOpt sOpt = new StarlingOpt();

//            sOptRes = sOpt.execute();
//            sOptAns.append(sOptRes[0]).append("\t").append(sOptRes[1]).append("\r\n");
//

            //pso.execute();
            //sPSO.execute();
//            iterRes.append(i+1).append("\t");
//            timeRes.append(i+1).append("\t");
//
//            startTime = System.currentTimeMillis();
//            iteration = pso.execute();
//            endTime = System.currentTimeMillis();
//            iterRes.append(iteration).append("\t");
//            timeRes.append(endTime - startTime).append("\t");
//
//            startTime = System.currentTimeMillis();
//            iteration = fso.execute();
//            endTime = System.currentTimeMillis();
//            iterRes.append(iteration).append("\t");
//            timeRes.append(endTime - startTime).append("\t");
//
//            startTime = System.currentTimeMillis();
//            iteration = sPSO.execute();
//            endTime = System.currentTimeMillis();
//            iterRes.append(iteration).append("\t");
//            timeRes.append(endTime - startTime).append("\t");
//
//            startTime = System.currentTimeMillis();
//            iteration = sOpt.execute();
//            endTime = System.currentTimeMillis();
//            iterRes.append(iteration).append("\r\n");
//            timeRes.append(endTime - startTime).append("\r\n");


            System.out.println("Iteration time : " + i);
        }
//
        FileOutPut.writeMethod("data/Comparison/iterationSPSO.txt", iterRes.toString());
        FileOutPut.writeMethod("data/Comparison/runningTimeSPSO.txt", timeRes.toString());
//        FileOutPut.writeMethod("data/BenchMark/Booth.txt", sOptRes);

    }

}
