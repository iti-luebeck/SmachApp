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

import java.util.ArrayList;
import java.util.List;

import de.uni_luebeck.iti.smachapp.model.StateMachine;
import de.uni_luebeck.iti.smachapp.model.Transition;

/**
 * Created by Morten Mey on 29.06.2014.
 */
public class DeleteTransitions implements UndoableAction {

    private List<Transition> transitions;
    private List<Integer> indices;
    private StateMachine machine;

    public DeleteTransitions(List<Transition> transitions, StateMachine machine) {
        this.transitions = new ArrayList<Transition>(transitions);
        this.machine = machine;

        indices=new ArrayList<Integer>(transitions.size());

        for(Transition t:transitions){
            indices.add(t.getPreviousState().getTransitions().indexOf(t));
        }
    }

    @Override
    public void undo() {
        for(int i=0;i<transitions.size();i++){
            Transition t=transitions.get(i);
            t.getPreviousState().getTransitions().add(indices.get(i),t);
        }
    }

    @Override
    public void redo() {
        for (Transition t : transitions) {
            machine.removeTransition(t);
        }
    }
}
