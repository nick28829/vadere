package org.vadere.util.geometry;

import static  org.junit.jupiter.api.Assertions.assertEquals;
import static  org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Paths;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.vadere.util.geometry.GeometryUtils;
import org.vadere.util.geometry.shapes.VPoint;
import org.vadere.util.geometry.shapes.VPolygon;
import org.vadere.util.geometry.shapes.VTriangle;
import org.vadere.util.io.GeometryPrinter;
import org.vadere.util.io.IOUtils;

/**
 * Basic tests of the {@link VPolygon} class.
 * 
 * 
 */
public class TestPolygon {
	private VPolygon testPolygon;
	private VPolygon copyTestPolygon;
	private VTriangle testTriangle;
	private final double roomSideLen = 100;

	@BeforeEach
	public void setUp() throws Exception {
		testPolygon = GeometryUtils.polygonFromPoints2D(new VPoint(0, 0),
				new VPoint(roomSideLen, 0),
				new VPoint(roomSideLen, roomSideLen),
				new VPoint(0, roomSideLen));

		copyTestPolygon = GeometryUtils.polygonFromPoints2D(new VPoint(0, 0),
				new VPoint(roomSideLen, 0),
				new VPoint(roomSideLen, roomSideLen),
				new VPoint(0, roomSideLen));

		testTriangle = new VTriangle(new VPoint(30, 50), new VPoint(0, 0),
				new VPoint(80, 0));
	}

	@Test
	public void testContainsPoint() {
		assertEquals(true, testPolygon.contains(new VPoint(10, 10)),
				"The polygon should contain this point.");
		assertEquals(true, testPolygon.contains(new VPoint(0, 10)),
				"The polygon should contain this point.");
		assertEquals(true, testPolygon.contains(new VPoint(0, 0)),
				"The polygon should contain this point.");
		assertEquals(false, testPolygon.contains(new VPoint(-5, 0)),
				"The polygon should not contain this point.");

		assertEquals(false, testTriangle.contains(new VPoint(40, 40.000000001)),
				"The triangle should not contain this point.");

		assertEquals(true, testTriangle.contains(new VPoint(40, 39.999999999)),
				"The triangle should not contain this point.");
	}

	@Test
	public void testGetArea() {
		assertEquals(roomSideLen
				* roomSideLen, testPolygon.getArea(), 1e-6, "The area of the polygon is wrong.");
	}

	@Test
	public void testGetHeight() {
		assertEquals(roomSideLen, testPolygon.getBounds2D().getHeight(),
				1e-6, "The height of the polygon is wrong.");
	}

	@Test
	public void testGetWidth() {
		assertEquals(roomSideLen, testPolygon.getBounds2D().getWidth(),
				1e-6, "The width of the polygon is wrong.");
	}

	@Test
	public void testClosestPoint() throws IOException {
		int gridsize = 100;
		double[][] grid = new double[gridsize][gridsize];
		double start = -roomSideLen / 2;
		double end = roomSideLen + roomSideLen / 2;

		for (int row = 0; row < gridsize; row++) {
			for (int col = 0; col < gridsize; col++) {
				double factorX = row / ((double) gridsize - 1);
				double factorY = col / ((double) gridsize - 1);
				VPoint currentPoint = new VPoint(start + (end - start)
						* factorX, start + (end - start) * factorY);

				// VPoint closest = new VPoint(roomSideLen/2,
				// roomSideLen/2);
				VPoint closest = testPolygon.closestPoint(currentPoint);
				grid[row][col] = closest.distance(currentPoint);
			}
		}

		// print evaluated grid
		String g2string = GeometryPrinter.grid2string(grid);
		IOUtils.printDataFile(Paths.get("testreports", "test_polygon2d_closestPoint.txt"),
				g2string);
	}

	@Test
	public void testEquals() {
		assertEquals(testPolygon, copyTestPolygon, "equals() does not work properly.");
	}

	@Test
	public void testOrientation() {
		VPoint p1 = new VPoint(0.0, 0.0);
		VPoint p2 = new VPoint(1.0, 0.0);
		VPoint p3 = new VPoint(1.0, 1.0);
		VPoint p4 = new VPoint(0.5, 2.0);
		VPoint p5 = new VPoint(0.9, 1.0);


		VPolygon ccwPoly = GeometryUtils.polygonFromPoints2D(p1, p2, p3, p4, p5);
		VPolygon cwPoly = GeometryUtils.polygonFromPoints2D(p5, p4, p3, p2, p1);

		assertTrue(ccwPoly.isCCW());
		assertTrue(!cwPoly.isCCW());
	}
}
