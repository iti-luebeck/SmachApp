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
package de.uni_luebeck.iti.smachapp.model.undo;

import android.content.Context;
import android.widget.Toast;

import java.util.Hashtable;
import java.util.List;
import java.util.Map;

import de.uni_luebeck.iti.smachapp.app.R;
import de.uni_luebeck.iti.smachapp.model.Guard;
import de.uni_luebeck.iti.smachapp.model.Transition;

/**
 * Created by Morten Mey on 29.06.2014.
 */
public class TransitionPropertyChanged implements UndoableAction {

    private class GuardElement {
        public String op;
        public Integer value;

        public GuardElement(String op, Integer value) {
            this.op = op;
            this.value = value;
        }

        @Override
        public boolean equals(Object other) {
            if (!(other instanceof GuardElement)) {
                return false;
            }

            GuardElement elem = (GuardElement) other;

            return op.equals(elem.op) && value.equals(elem.value);
        }

    }

    private Context context;

    private Transition transition;
    private List<Transition> parentList;
    private int oldPriority;
    private int newPriority;

    private String oldName;
    private String newName;

    private Hashtable<String, GuardElement> oldGuard = new Hashtable<String, GuardElement>();
    private Hashtable<String, GuardElement> newGuard = new Hashtable<String, GuardElement>();

    public TransitionPropertyChanged(Transition transition, List<Transition> parent, int priority,Context context) {
        this.context=context;
        this.transition = transition;
        parentList = parent;
        oldPriority = priority;
        oldName = transition.getLabel();

        Guard g = transition.getSmachableGuard();
        for (int i = 0; i < g.getOperators().size(); i++) {
            oldGuard.put(g.getSensorNames().get(i), new GuardElement(g.getOperators().get(i), g.getCompValues().get(i)));
        }
    }

    public boolean operationDone(int priority) {
        newPriority = priority;
        newName = transition.getLabel();

        boolean change = !oldName.equals(newName);

        Guard g = transition.getSmachableGuard();
        for (int i = 0; i < g.getOperators().size(); i++) {
            String key = g.getSensorNames().get(i);
            GuardElement elem = new GuardElement(g.getOperators().get(i), g.getCompValues().get(i));
            newGuard.put(key, elem);

            if (!(oldGuard.containsKey(key) && oldGuard.get(key).equals(elem))) {
                change = true;
            }
        }

        return change || didPriorityChange();
    }

    @Override
    public void undo() {
        if(didPriorityChange()) {
            parentList.remove(transition);
            parentList.add(oldPriority, transition);
        }
        transition.setLabel(oldName);
        Guard g = transition.getSmachableGuard();
        g.clear();

        for (Map.Entry<String, GuardElement> elem : oldGuard.entrySet()) {
            g.add(elem.getKey(), elem.getValue().op, elem.getValue().value);
        }

        Toast.makeText(context, R.string.undid_transition_properties,Toast.LENGTH_SHORT).show();
    }

    @Override
    public void redo() {
        if(didPriorityChange()) {
            parentList.remove(transition);
            parentList.add(newPriority, transition);
        }
        transition.setLabel(newName);
        Guard g = transition.getSmachableGuard();
        g.clear();

        for (Map.Entry<String, GuardElement> elem : newGuard.entrySet()) {
            g.add(elem.getKey(), elem.getValue().op, elem.getValue().value);
        }

        Toast.makeText(context,R.string.redid_transition_properties,Toast.LENGTH_SHORT).show();
    }

    public  boolean didPriorityChange(){
        return newPriority!=oldPriority;
    }

    public Transition getTransition(){
        return transition;
    }
}
