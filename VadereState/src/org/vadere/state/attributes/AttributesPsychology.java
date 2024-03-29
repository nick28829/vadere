package org.vadere.state.attributes;

import org.vadere.annotation.helptext.HelpIgnore;
import org.vadere.util.reflection.VadereAttribute;

import java.util.Objects;

public class AttributesPsychology extends Attributes {

	/** Store the members of this class under this key in the JSON file. */
	@HelpIgnore
	public static final String JSON_KEY = "attributesPsychology";

	/** Allows agents to change their behavior (e.g. from TARGET_ORIENTIED to COOPERATIVE if it is too dense) */
	private boolean usePsychologyLayer = false;
	/**
	 * <i>psychologyLayer</i> represents the psychology layer of the simulation.
	 */
	private AttributesPsychologyLayer psychologyLayer = new AttributesPsychologyLayer();

	// Getter
	public boolean isUsePsychologyLayer() {
		return usePsychologyLayer;
	}
	public AttributesPsychologyLayer getPsychologyLayer() { return psychologyLayer; }

	// Setters
	public void setUsePsychologyLayer(boolean usePsychologyLayer) {
		checkSealed();
		this.usePsychologyLayer = usePsychologyLayer;
	}

	public void setPsychologyLayer(AttributesPsychologyLayer attributesPsychologyLayer) {
		checkSealed();
		this.psychologyLayer = attributesPsychologyLayer;
	}

	@Override
	public boolean equals(Object o) {
		if (this == o) return true;
		if (o == null || getClass() != o.getClass()) return false;
		AttributesPsychology that = (AttributesPsychology) o;
		return usePsychologyLayer == that.usePsychologyLayer &&
				psychologyLayer.equals(that.psychologyLayer);
	}

	@Override
	public int hashCode() {
		return Objects.hash(usePsychologyLayer, psychologyLayer);
	}

}
