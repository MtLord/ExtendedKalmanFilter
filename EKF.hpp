#ifndef KALMAN_HPP_
#define KALMAN_HPP_
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <random>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
using namespace Eigen;

typedef Vector3d vector;
typedef Matrix<double, 3, 3> matrix;

class EKF
{
private:
    const double dt = 0.005;
    double omega = 0;
    matrix B;
    matrix I;
    matrix H;
    vector w;
    vector v;
    vector p;
    matrix R;      //観測ノイズ分散
    matrix Q;      //状態ノイズ分散
    matrix P;      //誤差共分散行列
    vector offset; //オドメトリの滑り誤差

    vector u;                               //制御入力
    vector x;                               //状態変数
    vector z;                               //観測変数
    void StateEquation(vector x, vector u); //状態方程式
    void OutputEquation(vector x);          //観測方程式
    matrix JacobF(vector x, vector u);      //状態方程式のヤコビアンを求める関数
    matrix JacobH(vector x);                //観測方程式のヤコビアン
    vector ControlInput();                  //制御入力(目標軌道)
    vector MotionModel(vector x, vector u); //機体の動作モデルの関数
    vector ObservationModel(vector x);      //非線形な観測モデル

public:
    vector xDes;  //目標軌道
    vector xOddm; //オドメトリの自己位置
    vector xEst;  //融合後の予測位置
    vector xLidar;

    EKF() : u(0, 0, 0), xDes(1, 0, 0), xOddm(0, 0, 0), xEst(0, 0, 0), xLidar(0, 0, 0)
    {
        H.setIdentity();
        I.setIdentity();
        offset << -0.07, -0.07, 0;
        /********init param**********/
        x << 0, 0, 0;
        z << 0, 0, 0;
        p << 0.1, 0.1, 0.002;
        P = p.asDiagonal();
        w << 0.04, 0.04, 0.01;
        v << 0.00025, 0.00025, 0.03;
        R = w.asDiagonal(); //対角行列を作成
        Q = v.asDiagonal();
        /*****************************/
    }

    void Estimation(); //予測ステップ
    void Update();     //更新ステップ
};

#endif