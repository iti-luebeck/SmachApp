package de.uni_luebeck.iti.smachapp.view;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.LinearLayout;

import com.larswerkman.holocolorpicker.ColorPicker;
import com.larswerkman.holocolorpicker.OpacityBar;
import com.larswerkman.holocolorpicker.SaturationBar;
import com.larswerkman.holocolorpicker.ValueBar;

import de.uni_luebeck.iti.smachapp.app.R;
import de.uni_luebeck.iti.smachapp.model.Action;
import de.uni_luebeck.iti.smachapp.model.ColorActuator;

/**
 * Created by Morten Mey on 18.05.2014.
 */
public class ColorSelector extends LinearLayout implements View.OnClickListener,ActuatorUI {

    private ColorActuator actuator;
    private CheckBox check;
    private Button button;

    private int color;

    public ColorSelector(Context context,ColorActuator actuator) {
        super(context);
        this.actuator=actuator;

        check=new CheckBox(context);
        check.setText(actuator.getKey());

        button=new Button(context);
        button.setText(R.string.colorButton);
        button.setBackgroundColor(actuator.getDefaultColor());
        color=actuator.getDefaultColor();
        button.setOnClickListener(this);

        this.addView(check);
        this.addView(button);
    }

    @Override
    public void setToAction(Action action){
        if(!action.getKey().equals(actuator.getKey())){
            throw new IllegalArgumentException("The action is not ment for this actuator.");
        }

        check.setChecked(true);
        color=action.getValue();
        button.setBackgroundColor(color);
        button.postInvalidate();
    }

    @Override
    public Action createAction(){
        return new Action(actuator.getKey(),color);
    }

    @Override
    public boolean isChecked(){
        return  check.isChecked();
    }

    @Override
    public void onClick(View view) {
        AlertDialog.Builder builder=new AlertDialog.Builder(this.getContext());
        builder.setTitle(R.string.colorDialogTitle);

        LayoutInflater factory = LayoutInflater.from(this.getContext());
        View myView = factory.inflate(R.layout.color_picker, null);
        final ColorPicker picker = (ColorPicker) myView.findViewById(R.id.picker);
        picker.addOpacityBar((OpacityBar)myView.findViewById(R.id.opacitybar));
        picker.addSaturationBar((SaturationBar) myView.findViewById(R.id.saturationbar));
        picker.addValueBar((ValueBar) myView.findViewById(R.id.valuebar));
        picker.setColor(color);
        builder.setView(myView);

        builder.setPositiveButton(R.string.accept,new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {
                color=picker.getColor();
                button.setBackgroundColor(color);
                button.postInvalidate();
            }
        });

        builder.setNegativeButton(R.string.disrecard,new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {

            }
        });

        builder.show();
    }
}
