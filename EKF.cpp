#include "EKF.hpp"
#include <stdio.h>
std::mt19937 generator;
double mean = 0.0;
double stddev1 = 0.05, stddev2 = 7;
std::normal_distribution<double> state_noise(mean, stddev1);
std::normal_distribution<double> output_noise(mean, stddev2);

vector EKF::ControlInput()
{
    omega += 0.0605 / dt;
    double v_x = -(omega * sin(omega * dt));
    double v_y = omega * cos(omega * dt);
    vector u;
    u(0) = v_x;
    u(1) = v_y;
    u(2) = 0; //機体自体の回転角速度
    //printf("%f %f\n", v_x, v_y);

    return u;
}

matrix EKF::JacobF(vector x, vector u)
{
    double yaw = x(2); //一つ前の角度
    double v_x = u(0);
    double v_y = u(1);
    matrix jF;
    jF << 1.0, 0, -v_x * dt * sin(yaw) - v_y * dt * cos(yaw), //全方向移動ロボットのヤコビアン
        0, 1.0, v_x * dt * cos(yaw) - v_y * dt * sin(yaw),
        0, 0, 1;
    return jF;
}

matrix EKF::JacobH(vector x)
{
    matrix jH;
    jH << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    return jH;
}
vector EKF::MotionModel(vector x, vector u)
{
    double yaw = x(2);
    this->B << dt * cos(yaw), -dt * sin(yaw), 0,
        dt * sin(yaw), dt * cos(yaw), 0,
        0, 0, dt;
    vector x_c = x + B * u;
    return x_c;
}

vector EKF::ObservationModel(vector x)
{
    vector z = H * x;
    return z;
}

void EKF::StateEquation(vector x, vector u) //ランダムノイズが乗った状態のオドメトリ位置
{
    vector w;
    w(0) = state_noise(generator);
    w(1) = state_noise(generator);            //正規分布に従うランダムノイズを付加
    xDes = MotionModel(this->xDes, u);        //本当の軌道
    this->x = MotionModel(x, u) + w + offset; //実際に使う場合はwとoffsetは不要
    xOddm = this->x;                          //オドメトリの出力位置
}

void EKF::OutputEquation(vector x) //観測程式ではLiDARによる自己位置を計算する ここではオドメトリに分散の大きなノイズを足している
{
    vector v;
    v(0) = output_noise(generator);
    v(1) = output_noise(generator);
    this->z = MotionModel(x, this->u) + v; //LiDARのノイズを足してあげる
    xLidar = this->z;
}

void EKF::Estimation()
{
    u = ControlInput();
    StateEquation(this->x, this->u);
    OutputEquation(this->xDes);        //実際にはzにはLiDARによる自己位置を直接入力でいい 今回はシミュレーションなので動作モデルから導出
    xEst = MotionModel(this->xEst, u); //予測値　
    matrix jF = JacobF(this->xEst, u);
    P = jF * P * jF.transpose() + Q; //事前誤差共分散
}

void EKF::Update()
{

    matrix jH = JacobH(xEst);
    matrix S = jH * P * jH.transpose() + R;
    vector z_est = ObservationModel(xEst); //予測観測値
    matrix K = P * jH.transpose() * S.inverse();
    xEst = xEst + K * (this->z - z_est);
    P = (I - K * jH) * P;
    usleep(100000);
}
