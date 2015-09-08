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
import java.util.NoSuchElementException;

public class SmachableActuators extends LinkedList<ISmachableActuator> {

    private static final long serialVersionUID = 4879265984143961788L;

    /**
     * Searches for the actuator and returns the first
     * {@link ISmachableActuator} instance with this name.
     *
     * @param actuatorName
     * @return first {@link ISmachableActuator} with the specified name
     * @throws NoSuchElementException if there is no matching {@link ISmachableActuator} found in
     *                                this list.
     */
    public ISmachableActuator getActuator(String actuatorName)
            throws NoSuchElementException {
        for (ISmachableActuator actuator : this) {
            if (actuator.getName().equals(actuatorName))
                return actuator;
        }
        throw new NoSuchElementException("Zugriff auf unbekannten Aktor "
                + actuatorName + ".");
    }

    /**
     * Creates Strings representing the definition of all publishers that are
     * needed to communicate with all actuators stored in this instance. Name of
     * the publisher for a certain topic is "pub_<code>topic</code>
     * ", where all "/" in <code>tobic</code> will be replaced by "_".
     * <p/>
     * Example: to publish something on the topic <code>abc/def</code> you will
     * have to use the publisher <code>pub_abc_def</code>.
     *
     * @return {@link HashSet} of publisher definitions
     */
    public HashSet<String> getPublisherSetups() {
        HashSet<String> pubs = new HashSet<String>();
        for (ISmachableActuator actuator : this) {
            pubs.add(actuator.getPublisherSetup());
        }
        return pubs;
    }

    /**
     * Creates Strings representing all message imports that are needed to
     * communicate with all {@link ISmachableActuator} in this instance
     *
     * @return {@link HashSet} of all message imports
     */
    public HashSet<String> getMsgDeps() {
        HashSet<String> deps = new HashSet<String>();
        for (ISmachableActuator actuator : this) {
            deps.addAll(actuator.getImports());
        }
        return deps;
    }

    /**
     * returns a HashSet of publisher names of all {@link ISmachableActuator}s
     * stored in this List. These are used to declare the identifiers global in
     * certain functions.
     *
     * @return HashSet of publisherNames of the {@link ISmachableActuator}s
     */
    public HashSet<String> getGlobalIdentifiers() {
        HashSet<String> res = new HashSet<String>();
        for (ISmachableActuator actuator : this) {
            res.add(actuator.getPublisherName());
        }
        return res;
    }
}