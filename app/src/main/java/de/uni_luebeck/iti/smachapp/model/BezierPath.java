package de.uni_luebeck.iti.smachapp.model;

import android.graphics.Path;
import android.graphics.PointF;
import android.graphics.RectF;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import de.uni_luebeck.iti.smachapp.utils.PointUtils;

/**
 * A class for creating smooth bezier splines.
 * Created by Morten Mey on 05.05.2014.
 */
public class BezierPath {

    private float[] fx,fy,sx,sy;
    private List<PointF> knots;

    public static final float CLOSE_DISTANCE=50;
    public static final float MIN_DISTANCE=100;
    public static final float RELAXED_MIN_DISTANCE=150;

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

    public void resize(){
        int n=knots.size()-1;
        fx=new float[n];
        fy=new float[n];
        sx=new float[n];
        sy=new float[n];
        updateCurveControlPoints();
    }

    public void moveKnots(float x, float y, int closestPoint){
        if(closestPoint<0){
            closestPoint=knots.size()-1;
        }

        PointF last=knots.get(closestPoint);
        last.x+=x;
        last.y+=y;


        for(int i=closestPoint+1;i<knots.size()-1;i++){
            PointF curr=knots.get(i);
            float dist=PointUtils.distance(last,curr);
            if(dist > RELAXED_MIN_DISTANCE || dist < CLOSE_DISTANCE){
                curr.x+=x;
                curr.y+=y;
            }
            last=curr;
        }
        last=knots.get(closestPoint);
        for(int i=closestPoint-1;i>0;i--){
            PointF curr=knots.get(i);
            float dist=PointUtils.distance(last,curr);
            if(dist > RELAXED_MIN_DISTANCE || dist < CLOSE_DISTANCE){
                curr.x+=x;
                curr.y+=y;
            }
            last=curr;
        }

        this.updateCurveControlPoints();

    }

    public void fixBeginning(){
        float distance=PointUtils.distance(knots.get(0),knots.get(1));

        if(distance>RELAXED_MIN_DISTANCE){
            int pointsToAdd=(int)(distance/MIN_DISTANCE);
            float baseT=1.0f/(pointsToAdd+1);

            List<PointF> points=new ArrayList<PointF>(pointsToAdd);
            for(int i=1;i<=pointsToAdd;i++){
                points.add(calculatePointOnBezier(0,i*baseT));
            }

            knots.addAll(1,points);
            resize();
        }
    }

    public void fixEnd(){
        int lastIndex=knots.size()-2;
        float distance=PointUtils.distance(knots.get(lastIndex),knots.get(lastIndex+1));

        if(distance>RELAXED_MIN_DISTANCE){
            int pointsToAdd=(int)(distance/MIN_DISTANCE);
            float baseT=1.0f/(pointsToAdd+1);
            List<PointF> points=new ArrayList<PointF>(pointsToAdd);
            for(int i=1;i<=pointsToAdd;i++){
                points.add(calculatePointOnBezier(lastIndex,i*baseT));
            }

            knots.addAll(lastIndex+1,points);
            resize();
        }
    }

    public PointF calculatePointOnBezier(int index, float t){
        if(index<0){
            index=knots.size()-2;
        }

        if(t<0&&t>1){
            throw new IllegalArgumentException("The bezier parameter must be between 0 and 1 inclusive.");
        }

        PointF res=new PointF();

        res.x=(float)(Math.pow(1-t,3)*knots.get(index).x+3*Math.pow(1-t,2)*t*fx[index]+3*(1-t)*Math.pow(t,2)*sx[index]+Math.pow(t,3)*knots.get(index+1).x);
        res.y=(float)(Math.pow(1-t,3)*knots.get(index).y+3*Math.pow(1-t,2)*t*fy[index]+3*(1-t)*Math.pow(t,2)*sy[index]+Math.pow(t,3)*knots.get(index+1).y);

        return res;
    }


    public void reset() {
        PointF first=knots.get(0);
        PointF last=knots.get(knots.size()-1);

        knots.clear();

        PointF dir=PointUtils.calculateDirection(first,last);
        float numOfPoints=dir.length()/MIN_DISTANCE;
        dir.x/=numOfPoints;
        dir.y/=numOfPoints;

        knots.add(first);
        for(int i=1;i<(int)numOfPoints;i++){
            knots.add(new PointF(first.x+dir.x*i,first.y+dir.y*i));
        }
        knots.add(last);
        resize();
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

    public static void filterPoints(List<PointF> points) {
        if(points.size()<=2){
            return;
        }

        Iterator<PointF> iter=points.iterator();

        PointF last=iter.next();//Never remove first point

        while(iter.hasNext()){
            PointF curr=iter.next();

            if(!iter.hasNext()){
                break; //Never remove last point
            }

            float dist= PointUtils.distance(curr,last);

            if(dist<MIN_DISTANCE){
                iter.remove();
            }else{
                last=curr;
            }
        }

        if(points.size()==2){//Only the start and end survived
            return;
        }

        PointF end=points.get(points.size()-1);

        float dist=PointUtils.distance(end,last);

        if(dist<RELAXED_MIN_DISTANCE){
            points.remove(last);
        }
    }

    public static void removePointsInOval(RectF oval,List<PointF> points){

        Iterator<PointF> iter=points.iterator();

        while(iter.hasNext()){
            PointF curr=iter.next();

            if(ovalTestValue(oval,curr)<1){ //der Punkt ist im Oval
                iter.remove();
            }
        }
    }


    public static void moveOnOval(RectF oval,PointF point){

        PointF dir=new PointF(oval.centerX()-point.x,oval.centerY()-point.y);
        float length=dir.length()*2;
        dir.x/=length;
        dir.y/=length;

        PointF lastPoint=new PointF(point.x,point.y);

        if(ovalTestValue(oval,point)>1) {
            while (ovalTestValue(oval, point) > 1) {
                lastPoint.set(point);
                point.x += dir.x;
                point.y += dir.y;
            }
        }else{
            while (ovalTestValue(oval, point) < 1) {
                lastPoint.set(point);
                point.x -= dir.x;
                point.y -= dir.y;
            }
        }

        point.set(lastPoint);
    }

    private static float ovalTestValue(RectF oval,PointF point){
        float radiusX=oval.width()/2;
        float radiusY=oval.height()/2;
        float tempX=((point.x-oval.centerX())*(point.x-oval.centerX()))/(radiusX*radiusX);
        float tempY=((point.y-oval.centerY())*(point.y-oval.centerY()))/(radiusY*radiusY);

        return tempX+tempY;
    }
}
