package org.vadere.simulator.projects.dataprocessing.outputfile;

import org.vadere.annotation.factories.OutputFileClass;
import org.vadere.simulator.projects.dataprocessing.datakey.NoDataKey;

/**
 * @author Mario Teixeira Parente
 *
 */
@OutputFileClass(dataKeyMapping = NoDataKey.class)
public class NoDataKeyOutputFile extends OutputFile<NoDataKey> {
    public NoDataKeyOutputFile() {
        super(new String[] { });
    }

    @Override
    public String[] toStrings(NoDataKey key) {
        return new String[] { };
    }
}
