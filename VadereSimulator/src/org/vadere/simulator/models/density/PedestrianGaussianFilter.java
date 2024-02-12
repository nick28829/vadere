package org.vadere.simulator.models.density;

import org.jetbrains.annotations.NotNull;
import org.vadere.simulator.models.potential.timeCostFunction.loading.IPedestrianLoadingStrategy;
import org.vadere.state.scenario.Pedestrian;
import org.vadere.util.geometry.shapes.VPoint;
import org.vadere.util.logging.Logger;

import edu.hm.teamLoewen.ServiceNotRunningException;
import edu.hm.teamLoewen.PredictionService;
import edu.hm.teamLoewen.PotField;


import java.io.IOException;
import java.util.Collection;
import java.util.LinkedList;
import java.util.function.Predicate;

public class PedestrianGaussianFilter<E extends Pedestrian> implements IGaussianFilter {

    private final IGaussianFilter filter;
    private final Collection<E> pedestrians;
    private final Predicate<E> pedestrianPredicate;
    private final IPedestrianLoadingStrategy pedestrianLoadingStrategy;
    private final PredictionService predictionService;
    private int step;
    private static Logger logger = Logger.getLogger(PedestrianGaussianFilter.class);
    private static LinkedList<PotField.Field> fields = new LinkedList<PotField.Field>();

    public PedestrianGaussianFilter(final Collection<E> pedestrians, final IGaussianFilter filter,
                                    final IPedestrianLoadingStrategy pedestrianLoadingStrategy) {
        this(pedestrians, filter, pedestrianLoadingStrategy, p -> true);
    }

    private static PotField.Field filterToField(IGaussianFilter filter) {
        int height = filter.getMatrixHeight();
        int width = filter.getMatrixWidth();
        PotField.Field.Builder fieldBuilder = PotField.Field.newBuilder();
        for (int x=0; x < width; x++) {
            for (int y=0; y < height; y++) {
                double val = filter.getFilteredValue(x, y);
                PotField.Coordinate.Builder coord = PotField.Coordinate.newBuilder();
                coord.setX(x).setY(y).setValue(val);
                fieldBuilder.addCoordinates(coord.build());
            }
        }
        return fieldBuilder.build();
    }

    private boolean usePrediction() {
        if (step < 30) return false; // do not use predictions for first 30 seconds
        return step % 30 > 5;  // every 30 seconds, predict 5 steps
    }

    private void saveFilter(IGaussianFilter filter) {
        PotField.Field field = filterToField(filter);
        fields.add(field);
    }

    private void makePrediction() {
        PotField.FieldSequence.Builder sequenceBuilder = PotField.FieldSequence.newBuilder();
        int fieldLength = fields.size();
        for (int idx = fieldLength - 10; idx < fieldLength; idx++) {
            sequenceBuilder.addFields(fields.get(idx));
        }
        PotField.FieldSequence fieldSequence = sequenceBuilder.build();
        predictionService.fit(fieldSequence);

        PotField.FieldSequence predictions = predictionService.predict(fieldSequence, 30);
        fields.addAll(predictions.getFieldsList());
    }

    public PedestrianGaussianFilter(final Collection<E> pedestrians, final IGaussianFilter filter,
                                    final IPedestrianLoadingStrategy pedestrianLoadingStrategy, final Predicate<E> pedestrianPredicate) {
        this.filter = filter;
        this.pedestrians = pedestrians;
        this.pedestrianPredicate = pedestrianPredicate;
        this.pedestrianLoadingStrategy = pedestrianLoadingStrategy;
        this.predictionService = new PredictionService();
        this.step = 0;
        try {
            predictionService.startService();
        } catch (ServiceNotRunningException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public double getFilteredValue(int x, int y) {

        PotField.Field field = fields.getLast();
        // not very efficient but it avoids converting to another data type and therefore
        // hopefully makes the whole code better understandable
        for (PotField.Coordinate coord : field.getCoordinatesList()) {
            if (coord.getX() == x && coord.getY() == y) return coord.getValue();
        }
        return 0.0; // unknown coordinate, shouldn't happen, for prod use you'd raise an exception
    }

    @Override
    public double getFilteredValue(double x, double y) {
        return getFilteredValue(filter.toXIndex(x), filter.toYIndex(y));
    }

    @Override
    public void  setInputValue(double x, double y, double value) {
        filter.setInputValue(x, y, value);
    }

    @Override
    public void setInputValue(int x, int y, double value) {
        filter.setInputValue(x, y, value);
    }

    @Override
    public void filterImage() {
        step++;
        if (usePrediction()) {
            if (step < fields.size()) return;
            makePrediction();
        }
        setValues();
        filter.filterImage();
        saveFilter(filter);
    }

    @Override
    public int getMatrixWidth() {
        return filter.getMatrixWidth();
    }

    @Override
    public int getMatrixHeight() {
        return filter.getMatrixHeight();
    }

    @Override
    public double getScale() {
        return filter.getScale();
    }

    @Override
    public double getMaxFilteredValue() {
        return filter.getMaxFilteredValue();
    }

    @Override
    public double getMinFilteredValue() {
        return filter.getMinFilteredValue();
    }

	@Override
	public int toXIndex(double x) {
		return filter.toXIndex(x);
	}

	@Override
	public int toYIndex(double y) {
		return filter.toYIndex(y);
	}

	@Override
	public int toFloorXIndex(double x) {
		return filter.toFloorXIndex(x);
	}

	@Override
	public int toFloorYIndex(double y) {
		return filter.toFloorYIndex(y);
	}

	@Override
	public double toXCoord(int xIndex) {
		return filter.toXCoord(xIndex);
	}

	@Override
	public double toYCoord(int yIndex) {
		return filter.toYCoord(yIndex);
	}

	@Override
    public void destroy() {
        this.filter.destroy();
        try {
            predictionService.stopService();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void setValue(@NotNull final E pedestrian) {
        VPoint filteredPosition = pedestrian.getPosition();
        //VPoint filteredPosition = new VPoint(Math.max(0, position.x), Math.max(0, position.y));

        // maybe find a better approximation. Here we extrapolate by the area of 4 rectangles.
	    int lowerLeftX = toFloorXIndex(filteredPosition.x);
	    int lowerLeftY = toFloorYIndex(filteredPosition.y);

	    double coordX = toXCoord(lowerLeftX+1);
	    double coordY = toYCoord(lowerLeftY+1);

	    double dx = Math.abs(coordX - filteredPosition.x) * getScale();
	    double dy = Math.abs(coordY - filteredPosition.y) * getScale();

	    double max = getScale() * getScale();

	    double w1 = dx * dy;
		double w2 = dx * (1.0 - dy);
	    double w3 = (1.0 - dx) * dy;
		double w4 = (1.0 - dx) * (1.0 - dy);

		assert Math.abs((w1+w2+w3+w4) - 1.0) < 0.00001;

	    double value = pedestrianLoadingStrategy.calculateLoading(pedestrian);

	    setInputValue(lowerLeftX + 1, lowerLeftY + 1, value * w1);
	    setInputValue(lowerLeftX + 1, lowerLeftY, value * w2);
	    setInputValue(lowerLeftX, lowerLeftY + 1, value * w3);
	    setInputValue(lowerLeftX, lowerLeftY, value * w4);
    }


    private void setValues() {
        clear();
        pedestrians.stream().filter(pedestrianPredicate).forEach(p -> setValue(p));
    }

    @Override
    public void clear() {
        filter.clear();
    }

    @Override
    public double getInputValue(int x, int y) {
        return filter.getInputValue(x, y);
    }
}
