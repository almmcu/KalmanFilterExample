import jama.Matrix;
import jkalman.JKalman;

/**
 * Created by Oda114 on 8.2.2016.
 * Kendi kalman filteren
 */
public class MyKalmanFilter {
    private int variables;
    private JKalman kalman;
    private Matrix s; // state [x, y, dx, dy, dxy]
    private Matrix c; // corrected state [x, y, dx, dy, dxy]
    private Matrix m; // measurement [x]

    /*
     * Inicializa el filtro kalman con 2 variables
     */
    public void initialize2() throws Exception{
        double dx, dy;

        if(variables != 0){
            throw new RuntimeException();
        }
        variables = 2;
        kalman = new JKalman(4, 2);

        // constant velocity
        dx = 0.2;
        dy = 0.2;

        s = new Matrix(4, 1); // state [x, y, dx, dy, dxy]
        c = new Matrix(4, 1); // corrected state [x, y, dx, dy, dxy]

        m = new Matrix(2, 1); // measurement [x]
        m.set(0, 0, 0);
        m.set(1, 0, 0);

        // transitions for x, y, dx, dy
        double[][] tr = { {1, 0, dx, 0},
                {0, 1, 0, dy},
                {0, 0, 1, 0},
                {0, 0, 0, 1} };
        kalman.setTransition_matrix(new Matrix(tr));

        // 1s somewhere?
        kalman.setError_cov_post(kalman.getError_cov_post().identity());

    }

    /*
     * Aplica Filtro a variables
     */
    public void push(double x,double y) throws Exception{
        m.set(0, 0, x);
        m.set(1, 0, y);

        c = kalman.Correct(m);
        s = kalman.Predict();
    }

    /*
     * obtiene arreglo con datos filtrados.
     */
    public double[] getKalmanPoint2() throws Exception{
        double[] point = new double[2];
        point[0] = c.get(0,0);
        point[1] = c.get(1,0);
        return point;
    }

    /*
     * obtiene arreglo con prediccion de punto.
     */
    public double[] getPredict2() throws Exception{
        double[] point = new double[2];
        point[0] = s.get(0,0);
        point[1] = s.get(1,0);
        return point;
    }

    /*
     * obtiene cantidad de variables del objeto
     */
    public int getNVariables() throws Exception{
        return this.variables;
    }

}
