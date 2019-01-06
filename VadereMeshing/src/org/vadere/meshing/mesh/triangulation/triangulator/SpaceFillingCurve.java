package org.vadere.meshing.mesh.triangulation.triangulator;

import org.jetbrains.annotations.NotNull;
import org.vadere.meshing.mesh.inter.IFace;
import org.vadere.meshing.mesh.inter.IHalfEdge;
import org.vadere.meshing.mesh.inter.IVertex;
import org.vadere.util.geometry.shapes.IPoint;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * <p>A {@link SpaceFillingCurve} is a (linked-)list of nodes {@link SFCNode} in the order in which
 * the curve would travers these nodes. Each node contains a half-edge which refers to
 * a face.</p>
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
public class SpaceFillingCurve<P extends IPoint, CE, CF, V extends IVertex<P>, E extends IHalfEdge<CE>, F extends IFace<CF>> {
	private SFCNode<P, CE, CF, V, E, F> head;

	public SpaceFillingCurve(){
		head = null;
	}

	public void insertFirst(final SFCNode<P, CE, CF, V, E, F> node) {
		assert head == null;

		if(head != null) {
			throw new IllegalArgumentException("the space filling curve has already a first element");
		}

		head = node;
	}

	/**
	 * <p>Replaces the anchor element by two elements i.e. consecutive elements left followed by right.</p>
	 *
	 * @param left      the left element
	 * @param right     the right element
	 * @param anchor    the replaced element
	 */
	public void replace(
			@NotNull final SFCNode<P, CE, CF, V, E, F> left,
			@NotNull final SFCNode<P, CE, CF, V, E, F> right,
			@NotNull SFCNode<P, CE, CF, V, E, F> anchor) {
		assert asList().contains(anchor);
		insertNext(right, anchor);
		insertNext(left, anchor);
		remove(anchor);
	}

	/**
	 * <p>Removes an element from the SFC in O(1).</p>
	 *
	 * @param anchor the element which will be removed
	 */
	public void remove(@NotNull final SFCNode<P, CE, CF, V, E, F> anchor) {
		assert asList().contains(anchor);

		if(anchor == head) {
			head = head.next;
			head.prev = null;
		}
		else {
			anchor.prev.next = anchor.next;
			anchor.next.prev = anchor.prev;
		}

		anchor.destroy();
	}

	/**
	 * <p>Inserts a node after the anchor element in O(1).</p>
	 *
	 * @param node      the node
	 * @param anchor    the anchor element
	 */
	public void insertNext(@NotNull final SFCNode<P, CE, CF, V, E, F> node, @NotNull SFCNode<P, CE, CF, V, E, F> anchor) {
		assert asList().contains(anchor);

		SFCNode<P, CE, CF, V, E, F> anchorNext = anchor.next;
		anchor.next = node;
		node.prev = anchor;
		node.next = anchorNext;

		// tail!
		if(anchorNext != null) {
			anchorNext.prev = node;
		}

	}

	/**
	 * Returns the SFC as ordered <tt>List</tt>.
	 * @return the whole SFC as in an ordered list
	 */
	public List<SFCNode<P, CE, CF, V, E, F>> asList() {
		List<SFCNode<P, CE, CF, V, E, F>> list;
		if(head == null) {
			list = Collections.EMPTY_LIST;
		}
		else {
			list = new ArrayList<>();
			SFCNode<P, CE, CF, V, E, F> node = head;
			while (node != null) {
				list.add(node);
				node = node.next;
			}
		}
		return list;
	}

	@Override
	public String toString() {
		return asList().toString();
	}
}
