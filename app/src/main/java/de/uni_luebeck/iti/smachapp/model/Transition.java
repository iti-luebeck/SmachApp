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
package de.uni_luebeck.iti.smachapp.model;

import de.uni_luebeck.iti.smachGenerator.ISmachableTransition;

/**
 * Created by Morten Mey on 21.04.2014.
 */
public class Transition implements ISmachableTransition {


    private State previous;
    private State next;
    private String name;
    private Guard guard;
    private BezierPath path;

    private Guard disabledGuard = new Guard();

    public Transition(State from, State to, String name, BezierPath path) {
        this(from, to, name, path, new Guard());
    }

    public Transition(State from, State to, String name, BezierPath path, Guard guard) {
        previous = from;
        next = to;
        this.name = name;
        this.path = path;
        this.guard = guard;
    }

    public State getPreviousState() {
        return previous;
    }

    @Override
    public State getFollowerState() {
        return next;
    }

    @Override
    public Guard getSmachableGuard() {
        return guard;
    }

    @Override
    public String getLabel() {
        return name;
    }

    public void setLabel(String name) {
        this.name = name;
    }

    public BezierPath getPath() {
        return path;
    }

    public void setPreviousState(State s) {
        previous = s;
    }

    public void setFollowerState(State s) {
        next = s;
    }

    public Guard getDisabledGuard(){
        return disabledGuard;
    }
}
