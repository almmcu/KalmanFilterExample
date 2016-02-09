import org.apache.commons.math3.filter.*;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.random.JDKRandomGenerator;
import org.apache.commons.math3.random.RandomGenerator;

/**
 * Created by Oda114 on 9.2.2016.
 */
// lineer voltaj deðerlerinin üzerine kalman filtresi uygulanmasý iþlemidir.
public class CostantVoltage {
    public static void main(String[] args) {
        double constantVoltage = 10d;
        double measurementNoise = 0.1d;
        double processNoise = 1e-5d;

// A = [ 1 ]
        RealMatrix A = new Array2DRowRealMatrix(new double[] { 1d });
// B = null
        RealMatrix B = null;
// H = [ 1 ]
        RealMatrix H = new Array2DRowRealMatrix(new double[] { 1d });
// x = [ 10 ]
        RealVector x = new ArrayRealVector(new double[] { constantVoltage });
// Q = [ 1e-5 ]
        RealMatrix Q = new Array2DRowRealMatrix(new double[] { processNoise });
// P = [ 1 ]
        RealMatrix P0 = new Array2DRowRealMatrix(new double[] { 1d });
// R = [ 0.1 ]
        RealMatrix R = new Array2DRowRealMatrix(new double[] { measurementNoise });

        ProcessModel pm = new DefaultProcessModel(A, B, Q, x, P0);
        MeasurementModel mm = new DefaultMeasurementModel(H, R);
        KalmanFilter filter = new KalmanFilter(pm, mm);

// process and measurement noise vectors
        RealVector pNoise = new ArrayRealVector(1);
        RealVector mNoise = new ArrayRealVector(1);

        RandomGenerator rand = new JDKRandomGenerator();
// iterate 60 steps
        for (int i = 0; i < 60; i++) {
            filter.predict();

            // simulate the process
            pNoise.setEntry(0, processNoise * rand.nextGaussian());

            // x = A * x + p_noise
            x = A.operate(x).add(pNoise);

            // simulate the measurement
            mNoise.setEntry(0, measurementNoise * rand.nextGaussian());

            // z = H * x + m_noise
            RealVector z = H.operate(x).add(mNoise);

            filter.correct(z);


            double voltage = filter.getStateEstimation()[0];
        }
    }
}
