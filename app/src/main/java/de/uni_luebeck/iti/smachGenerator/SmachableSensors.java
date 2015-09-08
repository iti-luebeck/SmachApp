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
package de.uni_luebeck.iti.smachGenerator;

import java.util.HashSet;
import java.util.LinkedList;

public class SmachableSensors extends LinkedList<ISmachableSensor> {

    private static final long serialVersionUID = -7460092781520543805L;

    /**
     * Searches for the sensor and returns the first {@link ISmachableSensor}
     * instance with this name.
     *
     * @param sensorName
     * @return first {@link ISmachableSensor} with the specified name or
     * <code>null</code> if there is no sensor with this name
     */
    public ISmachableSensor getSensor(String sensorName) {
        for (ISmachableSensor sensor : this) {
            if (sensor.getName().equals(sensorName))
                return sensor;
        }
        return null;
    }

    /**
     * Creates a list of Strings. Each representing a callback for a topic. The
     * returned LinkedList contains all callbacks, that are needed to receive
     * the data of all {@link ISmachableSensor}s stored in this instance. Every
     * leaf entry will be stored in the global variable with the same name as
     * the the name of the {@link ISmachableSensor}.
     * <p/>
     * Example: the the topic <code>Exampletopic</code> contains two elements
     * <code>e1, e2</code>. <code>e2</code> also contains two elements
     * <code>e3, e4</code>. <code>e1, e3, e4</code> will be stored.
     * <p/>
     * You can access the stored data by writing
     * <code>global <i>sensorname</i></code> in the function that will use
     * <code>sensorname</code> and afterwards simply use <code>sensorname</code>.
     *
     * @return {@link LinkedList} with the callbacks for all
     * {@link ISmachableSensor} stored in this instance
     */
    public HashSet<String> getCallbacks() {
        HashSet<String> results = new HashSet<String>();
        for (ISmachableSensor sensor : this) {
            results.add(sensor.getCallback());
        }
        return results;
    }

    /**
     * Creates Strings representing the definition of all subscribers that are
     * needed to receive the data of all {@link ISmachableSensor} stored in this
     * instance. The name of the callback for a certain topic is "callback_
     * <code>topic</code> ", where all "/" in <code>tobic</code> will be
     * replaced by "_".
     * <p/>
     * Example: the name of the callback function for the topic
     * <code>abc/def</code> will be <code>callback_abc_def</code>.
     *
     * @return {@link HashSet} of subscriber definitions
     */
    public HashSet<String> getSubscriberSetups() {
        HashSet<String> subs = new HashSet<String>();
        for (ISmachableSensor sensor : this) {
            subs.add(sensor.getSubscriberSetup());
        }
        return subs;
    }

    /**
     * Creates Strings representing all message imports that are needed to
     * communicate with all {@link ISmachableSensor} in this instance
     *
     * @return {@link HashSet} of all message imports
     */
    public HashSet<String> getMsgDeps() {
        HashSet<String> deps = new HashSet<String>();
        for (ISmachableSensor sensor : this) {
            deps.addAll(sensor.getImports());
        }
        return deps;
    }

    /**
     * returns statements for each Sensor that initializes the identifier. These
     * "declaration" will be done global after the import statements.
     *
     * @return a HashSet of Identifier initializations
     */
    public HashSet<String> getIdentifierInit() {
        HashSet<String> res = new HashSet<String>();
        for (ISmachableSensor sensor : this) {
            res.add(sensor.getIdentifierInit());
        }
        return res;
    }

    /**
     * returns a HashSet of identifiers of all {@link ISmachableSensor}s stored
     * in this List. These are used to declare the identifiers global in certain
     * functions.
     *
     * @return HashSet of names of the {@link ISmachableSensor}s
     */
    public HashSet<String> getGlobalIdentifiers() {
        HashSet<String> res = new HashSet<String>();
        for (ISmachableSensor sensor : this) {
            res.add(sensor.getGlobalIdentifier());
        }
        return res;
    }

}
