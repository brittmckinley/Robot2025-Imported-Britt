package frc.robot.Vision;

import com.fasterxml.jackson.annotation.JsonProperty;
import java.util.LinkedHashMap;

/**
 * OLD, used before april tags had the data available on shuffleboard
 * Used to be that we had to mine the raw json dump given by the camera to get data
 * This class was used to convert the json string into a hashmap class (json is a map structure)
 *
 * Custom made by me :) (see git blame -.-)
 */
@Deprecated
public class NetworkResult {

  private LinkedHashMap Results;

  public LinkedHashMap getResults() {
    return this.Results;
  }

  @JsonProperty("Results")
  public void setResults(LinkedHashMap Results) {
    this.Results = Results;
  }
}
