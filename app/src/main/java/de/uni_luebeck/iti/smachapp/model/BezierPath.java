package de.uni_luebeck.iti.smachapp.model;

import android.graphics.Path;
import android.graphics.PointF;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * Created by Morten Mey on 05.05.2014.
 */
public class BezierPath {

    private float[] fx,fy,sx,sy;
    private List<PointF> knots;

    public BezierPath(List<PointF> knots){
        this.knots=new ArrayList<PointF>(knots);
        int n=knots.size()-1;
        fx=new float[n];
        fy=new float[n];
        sx=new float[n];
        sy=new float[n];
        updateCurveControlPoints();
    }

    public List<PointF> getPoints(){
        return knots;
    }

    public void fillPath(Path path){
        path.moveTo(knots.get(0).x,knots.get(0).y);

        for(int i=0;i<fx.length;i++){
            PointF curr=knots.get(i+1);
            path.cubicTo(fx[i],fy[i],sx[i],sy[i],curr.x,curr.y);
        }
    }

    /*
        Code adapted from:http://www.codeproject.com/Articles/31859/Draw-a-Smooth-Curve-through-a-Set-of-2D-Points-wit
     */
    public void updateCurveControlPoints()
    {
        if (knots == null){
            throw new NullPointerException("knots");
        }

        int n = knots.size() - 1;
        if (n < 1){
            throw new IllegalArgumentException("At least two knot points required");
        }

        if(fx.length!=n||fy.length!=n||sx.length!=n||sy.length!=n){
            throw new IllegalArgumentException("One array has a wrong size.");
        }

        if (n == 1)
        { // Special case: Bezier curve should be a straight line
            // 3P1 = 2P0 + P3
            fx[0] = (2 * knots.get(0).x + knots.get(1).x) / 3;
            fy[0] = (2 * knots.get(0).y + knots.get(1).y) / 3;

            // P2 = 2P1 – P0
            sx[0] = 2 *	fx[0] - knots.get(0).x;
            sy[0] = 2 * fy[0] - knots.get(0).y;
            return;
        }

        // Calculate first Bezier control points
        // Right hand side vector
        float[] rhs = new float[n];

        // Set right hand side X values
        for (int i = 1; i < n - 1; ++i){
            rhs[i] = 4 * knots.get(i).x + 2 * knots.get(i+1).x;
        }
        rhs[0] = knots.get(0).x + 2 * knots.get(1).x;
        rhs[n - 1] = (8 * knots.get(n-1).x + knots.get(n).x) / 2.0f;
        // Get first control points X-values
        getFirstControlPoints(rhs,fx);

        // Set right hand side Y values
        for (int i = 1; i < n - 1; ++i){
            rhs[i] = 4 * knots.get(i).y + 2 * knots.get(i+1).y;
        }
        rhs[0] = knots.get(0).y + 2 * knots.get(1).y;
        rhs[n - 1] = (8 * knots.get(n-1).y + knots.get(n).y) / 2.0f;
        // Get first control points Y-values
        getFirstControlPoints(rhs,fy);

        for (int i = 0; i < n; ++i)
        {
            // Second control point
            if (i < n - 1){
                sx[i]=2 * knots.get(i+1).x - fx[i + 1];
                sy[i]=2 * knots.get(i+1).y - fy[i + 1];
            }
            else{
                sx[i]=(knots.get(n).x + fx[n - 1]) / 2f;
                sy[i]=(knots.get(n).y + fy[n - 1]) / 2f;
            }
        }
    }

    private static void getFirstControlPoints(float[] rhs, float[] x)
    {
        if(rhs.length!=x.length){
            throw new IllegalArgumentException("Both array must have the same length.");
        }
        int n = rhs.length; // Solution vector.
        float[] tmp = new float[n]; // Temp workspace.

        float b = 2.0f;
        x[0] = rhs[0] / b;
        for (int i = 1; i < n; i++) // Decomposition and forward substitution.
        {
            tmp[i] = 1 / b;
            b = (i < n - 1 ? 4.0f : 3.5f) - tmp[i];
            x[i] = (rhs[i] - x[i - 1]) / b;
        }
        for (int i = 1; i < n; i++)
            x[n - i - 1] -= tmp[n - i] * x[n - i]; // Backsubstitution.

    }


    private static final double MIN_DISTANCE=100;
    private static final double END_MIN_DISTANCE=50;

    public static void filterPoints(List<PointF> points) {
        if(points.size()<=2){
            return;
        }

        Iterator<PointF> iter=points.iterator();

        PointF last=iter.next();//Never remove first point

        int i=0;
        while(iter.hasNext()){
            PointF curr=iter.next();

            if(!iter.hasNext()){
                break; //Never remove last point
            }

            double x=curr.x-last.x;
            double y=curr.y-last.y;
            double dist=Math.sqrt(x*x+y*y);

            if(dist<MIN_DISTANCE){
                iter.remove();
                i++;
            }else{
                last=curr;
            }
        }

        if(points.size()==2){//Only the start and end survived
            return;
        }

        PointF end=points.get(points.size()-1);

        double x=end.x-last.x;
        double y=end.y-last.y;
        double dist=Math.sqrt(x*x+y*y);

        if(dist<END_MIN_DISTANCE){
            points.remove(last);
            i++;
        }
    }
}
