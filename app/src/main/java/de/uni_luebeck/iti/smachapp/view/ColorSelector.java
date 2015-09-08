/*
 * Copyright (c) 2015, Institute of Computer Engineering, University of Lübeck
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
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
import com.larswerkman.holocolorpicker.SaturationBar;
import com.larswerkman.holocolorpicker.ValueBar;

import java.util.List;

import de.uni_luebeck.iti.smachapp.app.R;
import de.uni_luebeck.iti.smachapp.model.Action;
import de.uni_luebeck.iti.smachapp.model.BeepColorRGBActuator;
import de.uni_luebeck.iti.smachapp.model.BeepColorSensor;
import de.uni_luebeck.iti.smachapp.model.Guard;

/**
 * Created by Morten Mey on 18.05.2014.
 */
public class ColorSelector extends LinearLayout implements View.OnClickListener, ActuatorUI, SensorUI {

    private BeepColorRGBActuator actuator;
    private BeepColorSensor sensor;
    private CheckBox check;
    private Button button;

    private int color;

    public ColorSelector(Context context, BeepColorRGBActuator actuator) {
        super(context);
        this.actuator = actuator;
        sensor = null;

        color = actuator.getDefaultColor();
        setup(context, actuator.getName());
    }

    public ColorSelector(Context context, BeepColorSensor sensor) {
        super(context);
        this.sensor = sensor;
        actuator = null;

        color = sensor.getDefaultColor();
        setup(context, sensor.getName());
    }

    private void setup(Context context, String name) {
        check = new CheckBox(context);
        check.setText(name);

        button = new Button(context);
        button.setText(R.string.colorButton);
        button.setBackgroundColor(color);
        button.setOnClickListener(this);

        this.addView(check);
        this.addView(button);
    }

    @Override
    public void setToAction(Action action,boolean checked) {
        if (!action.getActuatorName().equals(actuator.getName())) {
            throw new IllegalArgumentException("The action is not meant for this actuator.");
        }

        check.setChecked(checked);
        color = action.getValue();
        button.setBackgroundColor(color);
        button.postInvalidate();
    }

    @Override
    public Action createAction() {
        return new Action(actuator.getName(), color);
    }

    @Override
    public void setToGuard(Guard guard,boolean checked) {

        List<String> names = guard.getSensorNames();
        for (int i = 0; i < names.size(); i++) {
            if (names.get(i).equals(sensor.getName())) {
                check.setChecked(checked);
                color = guard.getCompValues().get(i);
                button.setBackgroundColor(color);
                button.postInvalidate();
            }
        }
    }

    @Override
    public void fillGuard(Guard guard) {
        guard.getSensorNames().add(sensor.getName());
        guard.getCompValues().add(color);
        guard.getOperators().add("");
    }

    @Override
    public boolean isChecked() {
        return check.isChecked();
    }

    @Override
    public void onClick(View view) {
        AlertDialog.Builder builder = new AlertDialog.Builder(this.getContext());
        builder.setTitle(R.string.colorDialogTitle);

        LayoutInflater factory = LayoutInflater.from(this.getContext());
        View myView = factory.inflate(R.layout.color_picker, null);
        final ColorPicker picker = (ColorPicker) myView.findViewById(R.id.picker);
        picker.addSaturationBar((SaturationBar) myView.findViewById(R.id.saturationbar));
        picker.addValueBar((ValueBar) myView.findViewById(R.id.valuebar));
        picker.setColor(color);
        picker.setOldCenterColor(color);
        builder.setView(myView);

        builder.setPositiveButton(R.string.accept, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {
                color = picker.getColor();
                button.setBackgroundColor(color);
                button.postInvalidate();
            }
        });

        builder.setNegativeButton(R.string.disrecard, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {

            }
        });

        builder.show();
    }
}
