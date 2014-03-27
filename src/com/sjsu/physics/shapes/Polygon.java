/*
 * Copyright 1995-2006 Sun Microsystems, Inc.  All Rights Reserved.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 * This code is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 only, as
 * published by the Free Software Foundation.  Sun designates this
 * particular file as subject to the "Classpath" exception as provided
 * by Sun in the LICENSE file that accompanied this code.
 *
 * This code is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * version 2 for more details (a copy is included in the LICENSE file that
 * accompanied this code).
 *
 * You should have received a copy of the GNU General Public License version
 * 2 along with this work; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Please contact Sun Microsystems, Inc., 4150 Network Circle, Santa Clara,
 * CA 95054 USA or visit www.sun.com if you need additional information or
 * have any questions.
 */
package com.sjsu.physics.shapes;

import java.awt.Shape;
import java.awt.geom.Rectangle2D;
import java.util.Arrays;

/**
 * The <code>Polygon</code> class encapsulates a description of a
 * closed, two-dimensional region within a coordinate space. This
 * region is bounded by an arbitrary number of line segments, each of
 * which is one side of the polygon. Internally, a polygon
 * comprises of a list of {@code (x,y)}
 * coordinate pairs, where each pair defines a <i>vertex</i> of the
 * polygon, and two successive pairs are the endpoints of a
 * line that is a side of the polygon. The first and final
 * pairs of {@code (x,y)} points are joined by a line segment
 * that closes the polygon.  This <code>Polygon</code> is defined with
 * an even-odd winding rule.  See
 * {@link java.awt.geom.PathIterator#WIND_EVEN_ODD WIND_EVEN_ODD}
 * for a definition of the even-odd winding rule.
 * This class's hit-testing methods, which include the
 * <code>contains</code>, <code>intersects</code> and <code>inside</code>
 * methods, use the <i>insideness</i> definition described in the
 * {@link Shape} class comments.
 *
 * @author      Sami Shaio
 * @see Shape
 * @author      Herb Jellinek
 * @since       1.0
 */
public class Polygon
{
    public int npoints;
    public int xpoints[];
    public int ypoints[];
    protected Rectangle bounds;
    private static final int MIN_LENGTH = 4;

    /**
     * Constructs and initializes a <code>Polygon</code> from the specified
     * parameters.
     * @param xpoints an array of X coordinates
     * @param ypoints an array of Y coordinates
     * @param npoints the total number of points in the
     *                          <code>Polygon</code>
     * @exception  NegativeArraySizeException if the value of
     *                       <code>npoints</code> is negative.
     * @exception  IndexOutOfBoundsException if <code>npoints</code> is
     *             greater than the length of <code>xpoints</code>
     *             or the length of <code>ypoints</code>.
     * @exception  NullPointerException if <code>xpoints</code> or
     *             <code>ypoints</code> is <code>null</code>.
     * @since 1.0
     */
    public Polygon(int xpoints[], int ypoints[], int npoints) {
        if (npoints > xpoints.length || npoints > ypoints.length) {
            throw new IndexOutOfBoundsException("npoints > xpoints.length || "+
                                                "npoints > ypoints.length");
        }

        if (npoints < 0) {
            throw new NegativeArraySizeException("npoints < 0");
        }

        this.npoints = npoints;
        this.xpoints = Arrays.copyOf(xpoints, npoints);
        this.ypoints = Arrays.copyOf(ypoints, npoints);
    }

    /**
     * Resets this <code>Polygon</code> object to an empty polygon.
     * The coordinate arrays and the data in them are left untouched
     * but the number of points is reset to zero to mark the old
     * vertex data as invalid and to start accumulating new vertex
     * data at the beginning.
     * All internally-cached data relating to the old vertices
     * are discarded.
     * Note that since the coordinate arrays from before the reset
     * are reused, creating a new empty <code>Polygon</code> might
     * be more memory efficient than resetting the current one if
     * the number of vertices in the new polygon data is significantly
     * smaller than the number of vertices in the data from before the
     * reset.
     * @see         java.awt.Polygon#invalidate
     * @since 1.4
     */
    public void reset() {
        npoints = 0;
        bounds = null;
    }

    /**
     * Invalidates or flushes any internally-cached data that depends
     * on the vertex coordinates of this <code>Polygon</code>.
     * This method should be called after any direct manipulation
     * of the coordinates in the <code>xpoints</code> or
     * <code>ypoints</code> arrays to avoid inconsistent results
     * from methods such as <code>getBounds</code> or <code>contains</code>
     * that might cache data from earlier computations relating to
     * the vertex coordinates.
     * @see         java.awt.Polygon#getBounds
     * @since 1.4
     */
    public void invalidate() {
        bounds = null;
    }


    /**
     * Calculates the bounding box of the points passed to the constructor.
     * Sets <code>bounds</code> to the result.
     * @param xpoints[] array of <i>x</i> coordinates
     * @param ypoints[] array of <i>y</i> coordinates
     * @param npoints the total number of points
     */
    void calculateBounds(int xpoints[], int ypoints[], int npoints) {
        int boundsMinX = Integer.MAX_VALUE;
        int boundsMinY = Integer.MAX_VALUE;
        int boundsMaxX = Integer.MIN_VALUE;
        int boundsMaxY = Integer.MIN_VALUE;

        for (int i = 0; i < npoints; i++) {
            int x = xpoints[i];
            boundsMinX = Math.min(boundsMinX, x);
            boundsMaxX = Math.max(boundsMaxX, x);
            int y = ypoints[i];
            boundsMinY = Math.min(boundsMinY, y);
            boundsMaxY = Math.max(boundsMaxY, y);
        }
        bounds = new Rectangle(boundsMinX, boundsMinY,
                               boundsMaxX - boundsMinX,
                               boundsMaxY - boundsMinY);
    }

    
    /**
     * Returns the bounds of this <code>Polygon</code>.
     * @return the bounds of this <code>Polygon</code>.
     */
    public Rectangle getBounds() {
        if (npoints == 0) {
            return null;
        }
        if (bounds == null) {
            calculateBounds(xpoints, ypoints, npoints);
        }
        return bounds.getBounds();
    }

    /**
     * Determines whether the specified coordinates are inside this
     * <code>Polygon</code>.
     * <p>
     * @param x the specified X coordinate to be tested
     * @param y the specified Y coordinate to be tested
     * @return {@code true} if this {@code Polygon} contains
     *         the specified coordinates {@code (x,y)};
     *         {@code false} otherwise.
     * @see #contains(double, double)
     * @since 1.1
     */
    public boolean contains(int x, int y) {
        return contains((double) x, (double) y);
    }

    /**
     * {@inheritDoc}
     * @since 1.2
     */
    public boolean contains(double x, double y) {
        if (npoints <= 2 || !getBounds().contains((int)x, (int)y)) {
            return false;
        }
        int hits = 0;

        int lastx = xpoints[npoints - 1];
        int lasty = ypoints[npoints - 1];
        int curx, cury;

        // Walk the edges of the polygon
        for (int i = 0; i < npoints; lastx = curx, lasty = cury, i++) {
            curx = xpoints[i];
            cury = ypoints[i];

            if (cury == lasty) {
                continue;
            }

            int leftx;
            if (curx < lastx) {
                if (x >= lastx) {
                    continue;
                }
                leftx = curx;
            } else {
                if (x >= curx) {
                    continue;
                }
                leftx = lastx;
            }

            double test1, test2;
            if (cury < lasty) {
                if (y < cury || y >= lasty) {
                    continue;
                }
                if (x < leftx) {
                    hits++;
                    continue;
                }
                test1 = x - curx;
                test2 = y - cury;
            } else {
                if (y < lasty || y >= cury) {
                    continue;
                }
                if (x < leftx) {
                    hits++;
                    continue;
                }
                test1 = x - lastx;
                test2 = y - lasty;
            }

            if (test1 < (test2 / (lasty - cury) * (lastx - curx))) {
                hits++;
            }
        }

        return ((hits & 1) != 0);
    }
}