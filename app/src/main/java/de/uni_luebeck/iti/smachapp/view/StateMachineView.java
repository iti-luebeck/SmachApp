package de.uni_luebeck.iti.smachapp.view;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.PointF;
import android.graphics.Rect;
import android.graphics.RectF;
import android.graphics.Typeface;
import android.util.AttributeSet;
import android.view.View;

import de.uni_luebeck.iti.smachapp.model.EditorModel;
import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.Transition;

/**
 * Created by Morten Mey on 27.04.2014.
 */
public class StateMachineView extends View {

    private static final float initialStateOffset = 10f;
    private static final float stateOvalOffset = 25f;

    private static Rect helperRect = new Rect();

    private EditorModel model = null;

    private float scale = 1f;
    private float translationX = 0f;
    private float translationY = 0f;

    private Paint paint;
    private Paint textPaint;
    private Paint highlightePaint;
    private Paint highlighteTextPaint;

    private RectF rect = new RectF();
    private Rect clipBounds=new Rect();

    private Path path=new Path();
    private Path tempPath;

    private Transition highlightedTransition=null;

    private State highlightedState=null;

    public StateMachineView(Context context) {
        super(context);
        createPaint();
    }

    public StateMachineView(Context context, AttributeSet attrs) {
        super(context, attrs);
        createPaint();
    }

    public StateMachineView(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        createPaint();
    }

    private void createPaint() {
        paint = new Paint(Paint.ANTI_ALIAS_FLAG);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(4);

        highlightePaint=new Paint(paint);
        highlightePaint.setColor(Color.BLUE);

        textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        textPaint.setTextAlign(Paint.Align.CENTER);
        textPaint.setTextSize(40);
        textPaint.setTypeface(Typeface.DEFAULT);

        highlighteTextPaint=new Paint(textPaint);
        highlighteTextPaint.setColor(Color.BLUE);
    }

    public void setModel(EditorModel model) {
        this.model = model;
    }

    @Override
    protected void onDraw(Canvas canvas) {

        int width = getWidth() / 2;
        int height = getHeight() / 2;

        canvas.scale(scale, scale, width, height);

        canvas.translate(translationX + width, translationY + height);
        canvas.getClipBounds(clipBounds);

        if (model == null) {
            return;
        }
        for (State state : model.getStateMachine()) {
           Paint oval,text;

            for(Transition trans:state){
                if(trans==highlightedTransition){
                    oval=highlightePaint;
                }else{
                    oval=paint;
                }

                path.reset();
                trans.getPath().fillPath(path);
                canvas.drawPath(path,oval);
            }

            if(state==highlightedState){
                oval=highlightePaint;
                text=highlighteTextPaint;
            }else{
                oval=paint;
                text=textPaint;
            }
            getStateRect(state, rect);
            canvas.drawOval(rect, oval);
            canvas.drawText(state.getName(), state.getX(), state.getY(), text);

            if (state.isInitialState()) {
                rect.left -= initialStateOffset;
                rect.right += initialStateOffset;
                rect.top -= initialStateOffset;
                rect.bottom += initialStateOffset;
                canvas.drawOval(rect, oval);
            }
        }

        if(tempPath!=null) {
            canvas.drawPath(tempPath, highlightePaint);
        }
    }

    public void setScale(float scale) {
        this.scale = scale;
    }

    public float getScale() {
        return scale;
    }

    public void setTranslationX(float x) {
        translationX = x;
    }

    public void setTranslationY(float y) {
        translationY = y;
    }

    public float getTranslationX() {
        return translationX;
    }

    public float getTranslationY() {
        return translationY;
    }

    public void scale(float factor) {
        scale *= factor;
        postInvalidate();
    }

    public void translate(float x, float y) {
        translationX -= x / scale;
        translationY -= y / scale;
        postInvalidate();
    }

    public void reset() {
        scale = 1f;
        translationX = 0;
        translationY = 0;
        postInvalidate();
    }

    public void getStateRect(State state, RectF result) {
        textPaint.getTextBounds(state.getName(), 0, state.getName().length(), helperRect);

        int width = helperRect.width() / 2;
        int height = helperRect.height() / 2;

        result.left = state.getX() - stateOvalOffset - width;
        result.right = state.getX() + stateOvalOffset + width;
        result.top = state.getY() - stateOvalOffset - height;
        result.bottom = state.getY() + stateOvalOffset - height;
    }

    public void translatePoint(PointF point) {
        point.x=point.x/scale+clipBounds.left;
        point.y=point.y/scale+clipBounds.top;
    }

    public void highlighteState(State s){
        highlightedState=s;
        postInvalidate();
    }

    public void highlighteTransition(Transition trans){
        highlightedTransition=trans;
        postInvalidate();
    }

    public void setTempPath(Path p){
        tempPath=p;
        postInvalidate();
    }
}