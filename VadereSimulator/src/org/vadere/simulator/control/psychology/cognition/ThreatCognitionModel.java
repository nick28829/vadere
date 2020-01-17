package org.vadere.simulator.control.psychology.cognition;

import org.vadere.state.psychology.cognition.GroupMembership;
import org.vadere.state.psychology.cognition.SelfCategory;
import org.vadere.state.psychology.perception.types.Bang;
import org.vadere.state.psychology.perception.types.ElapsedTime;
import org.vadere.state.psychology.perception.types.Stimulus;
import org.vadere.state.scenario.Pedestrian;
import org.vadere.state.scenario.Topography;
import org.vadere.util.geometry.shapes.VPoint;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Suppose a threat (a {@link Bang}) occurred.
 *
 * Check following conditions for a pedestrian:
 * <ol>
 *     <li>Is pedestrian inside threat area.</li>
 *     <li>Is pedestrian outside threat area.</li>
 *     <li>If pedestrian outside threat area, test if other pedestrians are nearby
 *     who have perceived the threat. If so, imitate their behavior if they are in-group members.</li>
 * </ol>
 */
public class ThreatCognitionModel implements ICognitionModel {

    private Topography topography;

    @Override
    public void initialize(Topography topography) {
        this.topography = topography;
    }

    @Override
    // TODO: Maybe, use also use cooperative behavior from "CooperativeCognitionModel".
    //   Refactor this long if-else cascade so that it is easier to read.
    public void update(Collection<Pedestrian> pedestrians) {
        for (Pedestrian pedestrian : pedestrians) {
            if (pedestrian.getMostImportantStimulus() instanceof Bang) {
                handleThreat(pedestrian, pedestrian.getMostImportantStimulus());
            } else if (pedestrian.getMostImportantStimulus() instanceof ElapsedTime) {
                handleElapsedTime(pedestrian);
            } else {
                throw new IllegalArgumentException("Can only process \"Bang\" and \"ElapsedTime\" stimuli!");
            }
        }
    }

    private void handleThreat(Pedestrian pedestrian, Stimulus stimulus) {
        // Current stimulus is a threat => store it and make clear that pedestrian is inside threat area.
        pedestrian.setPerceivedThreat(stimulus);
        pedestrian.setSelfCategory(SelfCategory.INSIDE_THREAT_AREA);
    }

    private void handleElapsedTime(Pedestrian pedestrian) {
        if (pedestrian.getPerceivedThreat() != null) {
            testIfInsideOrOutsideThreatArea(pedestrian);
        } else { // These agents did not perceive a threat but are aware of other threatened agents.

            // TODO: Pedestrians must be spawned with a random "GroupMembership".
            if (pedestrian.getGroupMembership() == GroupMembership.OUT_GROUP) {
                pedestrian.setSelfCategory(SelfCategory.TARGET_ORIENTED);
            } else if (pedestrian.getGroupMembership() == GroupMembership.IN_GROUP) {
                imitateThreatenedPedestrianIfPresent(pedestrian);

            } else {
                throw new IllegalArgumentException("Can only process \"OUT_GROUP\" and \"IN_GROUP\" group membership!");
            }
        }
    }

    private void testIfInsideOrOutsideThreatArea(Pedestrian pedestrian) {
        Bang threat = (Bang) pedestrian.getPerceivedThreat();

        VPoint threatOrigin = topography.getTarget(threat.getOriginAsTargetId()).getShape().getCentroid();
        double distanceToThreat = threatOrigin.distance(pedestrian.getPosition());

        boolean pedestrianIsInsideThreatArea = (distanceToThreat <= threat.getRadius());

        if (pedestrianIsInsideThreatArea) {
            pedestrian.setSelfCategory(SelfCategory.INSIDE_THREAT_AREA);
        } else {
            pedestrian.setSelfCategory(SelfCategory.OUTSIDE_THREAT_AREA);
        }
    }

    /* If a threatened pedestrian is nearby, use the same reaction as if
     * the current "pedestrian" would have perceived the same threat.
     * I.e., store the perceived threat and use "INSIDE_THREAT_AREA" to
     * accelerate and search for a safe zone.
     */
    private void imitateThreatenedPedestrianIfPresent(Pedestrian pedestrian) {
        List<Pedestrian> threatenedPedestrians = getClosestPedestriansWithSelfCategory(pedestrian, SelfCategory.OUTSIDE_THREAT_AREA);

        if (threatenedPedestrians.isEmpty() == false) {
            Pedestrian threatenedPedestrian = threatenedPedestrians.get(0);

            assert threatenedPedestrian.getPerceivedThreat() != null;

            handleThreat(pedestrian, threatenedPedestrian.getPerceivedThreat());
        } else {
            pedestrian.setSelfCategory(SelfCategory.TARGET_ORIENTED);
        }
    }

    private List<Pedestrian> getClosestPedestriansWithSelfCategory(Pedestrian pedestrian, SelfCategory expectedSelfCategory) {
        VPoint positionOfPedestrian = pedestrian.getPosition();

        List<Pedestrian> closestPedestrians = topography.getSpatialMap(Pedestrian.class)
                .getObjects(positionOfPedestrian, pedestrian.getAttributes().getSearchRadius());

        // Filter out "me" and pedestrians which are further away from target than "me".
        closestPedestrians = closestPedestrians.stream()
                .filter(candidate -> pedestrian.getId() != candidate.getId())
                .filter(candidate -> pedestrian.getSelfCategory() == expectedSelfCategory)
                .collect(Collectors.toList());

        // Sort by distance away from "me".
        closestPedestrians = closestPedestrians.stream()
                .sorted((pedestrian1, pedestrian2) ->
                        Double.compare(
                                positionOfPedestrian.distance(pedestrian1.getPosition()),
                                positionOfPedestrian.distance(pedestrian2.getPosition())
                        ))
                .collect(Collectors.toList());

        return closestPedestrians;
    }
}
