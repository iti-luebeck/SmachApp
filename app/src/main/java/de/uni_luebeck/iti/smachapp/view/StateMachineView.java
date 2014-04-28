package de.uni_luebeck.iti.smachapp.view;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Point;
import android.graphics.PointF;
import android.graphics.Rect;
import android.graphics.RectF;
import android.graphics.Typeface;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

import de.uni_luebeck.iti.smachapp.controller.ITouchController;
import de.uni_luebeck.iti.smachapp.model.EditorModel;
import de.uni_luebeck.iti.smachapp.model.State;

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

    private RectF rect = new RectF();
    private Rect clipBounds=new Rect();

    private ITouchController controller = null;

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

        textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        textPaint.setTextAlign(Paint.Align.CENTER);
        textPaint.setTextSize(40);
        textPaint.setTypeface(Typeface.DEFAULT);
    }

    public void setModel(EditorModel model) {
        this.model = model;
    }

    public void setController(ITouchController cont) {
        controller = cont;
    }

    public ITouchController getController() {
        return controller;
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
            getStateRect(state, rect);
            canvas.drawOval(rect, paint);
            canvas.drawText(state.getName(), state.getX(), state.getY(), textPaint);

            if (state.isInitialState()) {
                rect.left -= initialStateOffset;
                rect.right += initialStateOffset;
                rect.top -= initialStateOffset;
                rect.bottom += initialStateOffset;
                canvas.drawOval(rect, paint);
            }
        }
    }

    @Override
    public boolean onTouchEvent(MotionEvent e) {
        if (controller == null) {
            return false;
        } else {
            return controller.onTouchEvent(e);
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
}
