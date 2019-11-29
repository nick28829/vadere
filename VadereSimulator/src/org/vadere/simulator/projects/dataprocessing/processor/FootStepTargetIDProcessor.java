package org.vadere.simulator.projects.dataprocessing.processor;

import org.vadere.annotation.factories.dataprocessors.DataProcessorClass;
import org.vadere.simulator.control.simulation.SimulationState;
import org.vadere.simulator.projects.dataprocessing.datakey.EventtimePedestrianIdKey;
import org.vadere.simulator.projects.dataprocessing.processor.util.ModelFilter;
import org.vadere.state.scenario.Pedestrian;
import org.vadere.util.logging.Logger;

import java.util.Collection;

@DataProcessorClass()
public class FootStepTargetIDProcessor extends DataProcessor<EventtimePedestrianIdKey, Integer> implements ModelFilter {
	private static Logger logger = Logger.getLogger(FootStepTargetIDProcessor.class);

	public FootStepTargetIDProcessor(){
		super("targetId");
	}

	@Override
	protected void doUpdate(SimulationState state) {
		Collection<Pedestrian> peds = state.getTopography().getElements(Pedestrian.class);
		peds.forEach(p -> p.getTrajectory().getFootSteps().forEach(fs -> {
					this.putValue(new EventtimePedestrianIdKey(fs.getStartTime(), p.getId()), p.getTargets().isEmpty() ? -1 : p.getTargets().getFirst());
		}));
	}
}
