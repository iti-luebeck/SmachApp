package de.uni_luebeck.iti.smachapp.view;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;
import android.util.AttributeSet;
import android.widget.ImageView;
import android.widget.LinearLayout;

import de.uni_luebeck.iti.smachapp.app.R;
import de.uni_luebeck.iti.smachapp.model.DebugModel;
import de.uni_luebeck.iti.smachapp.model.DebugModelObserver;

/**
 * Created by Morten Mey on 04.07.2014.
 */
public class SensorView extends ImageView implements DebugModelObserver {

    private DebugModel model;

    private float[] irX = new float[8];
    private float[] irY = new float[8];

    private float[] colorX = new float[3];
    private float[] colorY = new float[3];

    private Paint paint;

    public void setModel(DebugModel model) {
        this.model = model;
        model.addObserver(this);
    }

    public SensorView(Context context) {
        super(context);
        setup();
    }

    public SensorView(Context context, AttributeSet attrs) {
        super(context, attrs);
        setup();
    }

    public SensorView(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        setup();
    }

    private void setup() {
        paint = new Paint(Paint.ANTI_ALIAS_FLAG);
        paint.setColor(Color.WHITE);
        paint.setTextSize(40);
        paint.setTypeface(Typeface.DEFAULT);
        paint.setTextAlign(Paint.Align.CENTER);

        irX[0] = 358 / 400f;
        irX[1] = 249 / 400f;
        irX[2] = 133 / 400f;
        irX[3] = 40 / 400f;
        irX[4] = 40 / 400f;
        irX[5] = 133 / 400f;
        irX[6] = 249 / 400f;
        irX[7] = 358 / 400f;

        irY[0] = 277 / 400f;
        irY[1] = 383 / 400f;
        irY[2] = 383 / 400f;
        irY[3] = 277 / 400f;
        irY[4] = 126 / 400f;
        irY[5] = 33 / 400f;
        irY[6] = 33 / 400f;
        irY[7] = 126 / 400f;

        colorX[0] = 141 / 400f;
        colorY[0] = 135 / 400f;
        colorX[1] = 192 / 400f;
        colorY[1] = 135 / 400f;
        colorX[2] = 243 / 400f;
        colorY[2] = 135 / 400f;

        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.MATCH_PARENT);
        this.setLayoutParams(params);

        this.setImageResource(R.drawable.beep_debug_background);
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        int width = getMeasuredWidth();
        int height = getMeasuredHeight();

        int offsetX = 0;
        int offsetY = 0;
        int size; //the image is square

        if (width > height) {
            offsetX = (width - height) / 2;
            size = height;
        } else {
            offsetY = (height - width) / 2;
            size = width;
        }

        if (model != null) {
            short[] ir = model.getIr();

            for (int i = 0; i < ir.length; i++) {
                canvas.drawText("IR" + i + ":" + Short.toString(ir[i]), irX[i] * size + offsetX, irY[i] * size + offsetY, paint);
            }

            int[] colors = model.getGroundColors();
            for (int i = 0; i < colors.length; i++) {
                canvas.drawText(Integer.toString(colors[i]), colorX[i] * size + offsetX, colorY[i] * size + offsetY, paint);
            }
        }
    }

    @Override
    public void onModelChange(DebugModel model) {
        postInvalidate();
    }
}
