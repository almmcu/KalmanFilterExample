import org.apache.commons.math3.filter.*;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.random.JDKRandomGenerator;
import org.apache.commons.math3.random.RandomGenerator;

/**
 * Created by Oda114 on 2.12.2015.
 */

public class Queueu {

    public static void main(String[] args) {
        System.out.println("args = " + args);

        // discrete time interval
        double dt = 0.1d;
// position measurement noise (meter)
        double measurementNoise = 10d;
// acceleration noise (meter/sec^2)
        double accelNoise = 0.2d;

// A = [ 1 dt ]
//     [ 0  1 ]
        RealMatrix A = new Array2DRowRealMatrix(new double[][] { { 1, dt }, { 0, 1 } });
// B = [ dt^2/2 ]
//     [ dt     ]
        RealMatrix B = new Array2DRowRealMatrix(new double[][] { { Math.pow(dt, 2d) / 2d }, { dt } });
// H = [ 1 0 ]
        RealMatrix H = new Array2DRowRealMatrix(new double[][] { { 1d, 0d } });
// x = [ 0 0 ]
        RealVector x = new ArrayRealVector(new double[] { 0, 0 });

        RealMatrix tmp = new Array2DRowRealMatrix(new double[][] {
                { Math.pow(dt, 4d) / 4d, Math.pow(dt, 3d) / 2d },
                { Math.pow(dt, 3d) / 2d, Math.pow(dt, 2d) } });
// Q = [ dt^4/4 dt^3/2 ]
//     [ dt^3/2 dt^2   ]
        RealMatrix Q = tmp.scalarMultiply(Math.pow(accelNoise, 2));
// P0 = [ 1 1 ]
//      [ 1 1 ]
        RealMatrix P0 = new Array2DRowRealMatrix(new double[][] { { 1, 1 }, { 1, 1 } });
// R = [ measurementNoise^2 ]
        RealMatrix R = new Array2DRowRealMatrix(new double[] { Math.pow(measurementNoise, 2) });

// constant control input, increase velocity by 0.1 m/s per cycle
        RealVector u = new ArrayRealVector(new double[] { 0.1d });

        ProcessModel pm = new DefaultProcessModel(A, B, Q, x, P0);
        MeasurementModel mm = new DefaultMeasurementModel(H, R);
        KalmanFilter filter = new KalmanFilter(pm, mm);

        RandomGenerator rand = new JDKRandomGenerator();

        RealVector tmpPNoise = new ArrayRealVector(new double[] { Math.pow(dt, 2d) / 2d, dt });
        RealVector mNoise = new ArrayRealVector(1);
double [] A1 = new double[60];
double [] A2 = new double[60];
double [] A3 = new double[60];
double [] A4 = new double[60];
// iterate 60 steps
        for (int i = 0; i < 60; i++) {
            filter.predict(u);

            // simulate the process
            double guess1 = rand.nextGaussian();
            RealVector pNoise = tmpPNoise.mapMultiply(accelNoise * guess1);

            // x = A * x + B * u + pNoise
            x = A.operate(x).add(B.operate(u)).add(pNoise);

            // simulate the measurement
            double guess2 = rand.nextGaussian();
            mNoise.setEntry(0, measurementNoise * guess2);

            // z = H * x + m_noise
            RealVector z = H.operate(x).add(mNoise);

            filter.correct(z);

            double position = filter.getStateEstimation()[0];
            double velocity = filter.getStateEstimation()[1];
            System.out.println(position);
            System.out.println(velocity);
            A1[i] = guess1 ;
            A2[i] = position;
            A3[i] = guess2;
            A4[i] = velocity;
        }
    }


}
