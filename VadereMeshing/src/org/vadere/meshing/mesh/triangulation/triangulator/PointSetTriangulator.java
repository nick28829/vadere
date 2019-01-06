package org.vadere.meshing.mesh.triangulation.triangulator;

import org.vadere.meshing.mesh.inter.IFace;
import org.vadere.meshing.mesh.inter.IHalfEdge;
import org.vadere.meshing.mesh.inter.IIncrementalTriangulation;
import org.vadere.meshing.mesh.inter.IVertex;
import org.vadere.util.geometry.shapes.IPoint;

import java.util.Collection;

/**
 * <p>A default triangulator: This triangulator takes a set of points P and constructs the Delaunay-Triangulation of P.</p>
 *
 * @author Benedikt Zoennchen
 *
 * @param <P> the type of the points (containers)
 * @param <CE> the type of container of the half-edges
 * @param <CF> the type of the container of the faces
 * @param <V> the type of the vertices
 * @param <E> the type of the half-edges
 * @param <F> the type of the faces
 */
public class PointSetTriangulator<P extends IPoint, CE, CF, V extends IVertex<P>, E extends IHalfEdge<CE>, F extends IFace<CF>> implements ITriangulator<P, CE, CF, V, E, F> {

	/**
	 * the triangulation which determines how points will be inserted.
	 */
    private final IIncrementalTriangulation<P, CE, CF, V, E, F> triangulation;

	/**
	 * the collection of points P.
	 */
	private final Collection<P> points;

	/**
	 * <p>The default constructor.</p>
	 *
	 * @param points        the collection of points P
	 * @param triangulation a triangulation which determines how points will be inserted
	 */
    public PointSetTriangulator(final Collection<P> points, final IIncrementalTriangulation<P, CE, CF, V, E, F> triangulation) {
        this.triangulation = triangulation;
        this.points = points;
    }

    @Override
    public IIncrementalTriangulation<P, CE, CF, V, E, F> generate() {
        triangulation.init();
        triangulation.insert(points);
        triangulation.finish();
        return triangulation;
    }
}
