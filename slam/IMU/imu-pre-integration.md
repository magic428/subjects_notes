# 

```cpp
std::random_device rd;
std::default_random_engine generator_(rd());
std::normal_distribution<double> noise(0.0, 1.0);

Eigen::Vector3d noise_gyro(noise(generator_),noise(generator_),noise(generator_));
Eigen::Matrix3d gyro_sqrt_cov = param_.gyro_noise_sigma * Eigen::Matrix3d::Identity();
data.imu_gyro = data.imu_gyro + gyro_sqrt_cov * noise_gyro / sqrt( param_.imu_timestep ) + gyro_bias_;

Eigen::Vector3d noise_acc(noise(generator_),noise(generator_),noise(generator_));
Eigen::Matrix3d acc_sqrt_cov = param_.acc_noise_sigma * Eigen::Matrix3d::Identity();
data.imu_acc = data.imu_acc + acc_sqrt_cov * noise_acc / sqrt( param_.imu_timestep ) + acc_bias_;

// gyro_bias update
Eigen::Vector3d noise_gyro_bias(noise(generator_),noise(generator_),noise(generator_));
gyro_bias_ += param_.gyro_bias_sigma * sqrt(param_.imu_timestep ) * noise_gyro_bias;
data.imu_gyro_bias = gyro_bias_;

// acc_bias update
Eigen::Vector3d noise_acc_bias(noise(generator_),noise(generator_),noise(generator_));
acc_bias_ += param_.acc_bias_sigma * sqrt(param_.imu_timestep ) * noise_acc_bias;
data.imu_acc_bias = acc_bias_;
```
