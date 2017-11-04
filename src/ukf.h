#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  // previous timestamp
  long long previous_timestamp_;

  //create sigma point matrix
  MatrixXd Xsig_aug;

  //Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a = 0.2;

  //Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd = 0.2;

  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred;

  //radar measurement noise standard deviation radius in m
  double std_radr = 0.3;

  //radar measurement noise standard deviation angle in rad
  double std_radphi = 0.0175;

  //radar measurement noise standard deviation radius change in m/s
  double std_radrd = 0.1;

  //create matrix for sigma points in measurement space
  int n_z_radar;
  MatrixXd Zsig_radar;

  VectorXd z_out_radar;
  MatrixXd S_out_radar;

  VectorXd z_out_lidar;
  MatrixXd S_out_lidar;

  int n_z_lidar;
  MatrixXd Zsig_lidar;


  double NIS_radar_;
  double NIS_laser_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param measurement_pack The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage measurement_pack);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param measurement_pack The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage measurement_pack);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param measurement_pack The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage measurement_pack);

  void AugmentedSigmaPoints(void);
  void SigmaPointPrediction(double delta_t);
  void PredictMeanAndCovariance(void);

  void PredictRadarMeasurement(void);

  void PredictLidarMeasurement(void);
};

#endif /* UKF_H */
