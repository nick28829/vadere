package org.vadere.util.geometry.shapes;

import static  org.junit.jupiter.api.Assertions.assertEquals;
import static  org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.vadere.util.geometry.shapes.VPoint;
import org.vadere.util.geometry.shapes.VTriangle;

/**
 * Incomplete tests of the {@link VTriangle} class.
 * 
 * 
 */
public class TestTriangle {

	public static VTriangle testTriangle;

	@BeforeEach
	public void setUp() throws Exception {

		testTriangle = new VTriangle(new VPoint(40, 40), new VPoint(30, 50),
				new VPoint(80, 0));
	}

	@Test
	public void testIsLine() {
		assertEquals(true, testTriangle.isLine(), "Triangle should be a line.");
	}

	@Test
	public void testIncenter() {
		double tolerance = 0.0001;
		VTriangle triangle = new VTriangle(new VPoint(5, 3), new VPoint(10, 5), new VPoint(3, -4.6));
		VTriangle triangle2 = new VTriangle(new VPoint(10, 5), new VPoint(3, -4.6), new VPoint(5, 3));

		VPoint expectedIncenter = new VPoint(6.13526,1.99663); // computed by wolfram-alpha
		VPoint incenter = triangle.getIncenter();

		assertTrue(expectedIncenter.equals(incenter, tolerance), "Incenter should be at " + expectedIncenter + " instead it is at " + incenter);
		assertTrue(incenter.equals(triangle2.getIncenter(), tolerance));
	}

	@Test
	public void testOrthocenter() {
		double tolerance = 0.0001;
		VTriangle triangle = new VTriangle(new VPoint(5, 3), new VPoint(10,5), new VPoint(3,-4.6));
		VPoint expectedOrthocenter = new VPoint(-2.11529,8.18824);
		VPoint orthcenter = triangle.getOrthocenter();
		assertTrue(expectedOrthocenter.equals(orthcenter, tolerance), "Orthocenter should be at " + expectedOrthocenter + " instead it is at " + orthcenter);
	}

	@Test
	public void testIsNonAcute() {
		VTriangle acute = new VTriangle(new VPoint(0,0), new VPoint(1, 0), new VPoint(0.5, 3));
		assertTrue(!acute.isNonAcute());

		VTriangle obscure1 = new VTriangle(new VPoint(1, 0), new VPoint(0,0),  new VPoint(1.5, 3));
		assertTrue(obscure1.isNonAcute());

		VTriangle obscure2 = new VTriangle(new VPoint(0,0), new VPoint(1, 0), new VPoint(1.5, 3));
		assertTrue(obscure2.isNonAcute());

		VTriangle obscure3 = new VTriangle(new VPoint(1.0002, 3), new VPoint(1, 0), new VPoint(0,0));
		assertTrue(obscure3.isNonAcute());

		VTriangle obscure4 = new VTriangle(new VPoint(1, 0), new VPoint(1.5, 3), new VPoint(0,0));
		assertTrue(obscure4.isNonAcute());

		VTriangle obscure5 = new VTriangle(new VPoint(0,0), new VPoint(1.5, 3), new VPoint(1, 0));
		assertTrue(obscure5.isNonAcute());

	}

}
