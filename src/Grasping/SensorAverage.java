package Grasping;

/**
 * <p>Average a value for a number of ticks. If you find it necessary to filter
 * your sensor data, you may use this example, or write your own.<\p>
 */
class SensorAverage{
  double [] data;
  int data_idx=0;
  double average=0.0;
  int nTicks;

  public SensorAverage(int n) {
    nTicks=n;
    data = new double[nTicks];
  }

  public double step (double sample) {
    average=average+sample/(double)nTicks;
    average=average-data[data_idx]/(double)nTicks;
    data[data_idx]=sample;
    data_idx++;
    if (data_idx==nTicks) data_idx=0;
    return average;
  }

}
