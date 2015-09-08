/*
 * Copyright (c) 2015, Institute of Computer Engineering, University of L�beck
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
package de.uni_luebeck.iti.smachapp.utils;

import android.graphics.PointF;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Morten Mey on 24.05.2014.
 */
public class PointUtils {

    public static float distance(PointF a, PointF b) {
        return calculateDirection(a, b).length();
    }

    public static PointF calculateDirection(PointF from, PointF to) {
        return new PointF(to.x - from.x, to.y - from.y);
    }

    public static void normalize(PointF point) {
        float length = point.length();
        point.x /= length;
        point.y /= length;
    }

    public static List<PointF> copyPoints(List<PointF> points) {
        List<PointF> res = new ArrayList<PointF>(points.size());
        for (PointF p : points) {
            res.add(new PointF(p.x, p.y));
        }
        return res;
    }
}
