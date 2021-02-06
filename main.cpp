#include "EKF.hpp"
#include <sys/time.h>
#include <stdio.h>
#include <iostream>
struct timeval t0, t1;
EKF kf;
FILE *gp;
std::vector<float> x_des;
std::vector<float> y_des;
std::vector<float> x_est;
std::vector<float> y_est;
std::vector<float> x_act;
std::vector<float> y_act;
std::vector<float> x_lidar;
std::vector<float> y_lidar;
void *TrajectDrawer(void *arg)
{
}

int main(void)
{
    //gettimeofday(&t0, NULL);
    gp = popen("gnuplot", "w");
    for (int i = 0; i < 500; i++)
    {
        kf.Estimation();
        kf.Update();

        fprintf(gp, "plot '-' w p title 'est' pt 5 ps 2 , '-' w p title 'oddm' pt 5 ps 2,'-' w p title 'actual' pt 5 ps 0.5,'-' w p title 'lidar' pt 5 ps 0.5,'-' w lp pt 5 ps 0.5 lc 9,'-' w lp pt 5 ps 0.5 lc 2,'-' w lp pt 5 ps 1 lc 3,'-' w p pt 5 ps 1 lc 4\n");
        fprintf(gp, "%f %f\n", kf.xEst(0), kf.xEst(1));
        fprintf(gp, "e\n");
        fprintf(gp, "%f %f\n", kf.xOddm(0), kf.xOddm(1));
        fprintf(gp, "e\n");
        fprintf(gp, "%f %f\n", kf.xDes(0), kf.xDes(1)); //目標座標
        fprintf(gp, "e\n");
        fprintf(gp, "%f %f\n", kf.xLidar(0), kf.xLidar(1)); //目標座標
        fprintf(gp, "e\n");
        x_est.push_back(kf.xEst(0));
        y_est.push_back(kf.xEst(1));
        x_act.push_back(kf.xOddm(0));
        y_act.push_back(kf.xOddm(1));
        x_des.push_back(kf.xDes(0));
        y_des.push_back(kf.xDes(1));
        x_lidar.push_back(kf.xLidar(0));
        y_lidar.push_back(kf.xLidar(1));
        for (std::size_t i = 0; i < x_est.size(); i++)
        {
            fprintf(gp, "%f %f\n", x_est[i], y_est[i]);
        }
        fprintf(gp, "e\n");
        for (std::size_t i = 0; i < x_act.size(); i++)
        {
            fprintf(gp, "%f %f\n", x_act[i], y_act[i]);
        }
        fprintf(gp, "e\n");
        for (std::size_t i = 0; i < x_des.size(); i++)
        {
            fprintf(gp, "%f %f\n", x_des[i], y_des[i]);
        } //表示順はプロットと合わせる
        fprintf(gp, "e\n");
        for (std::size_t i = 0; i < x_lidar.size(); i++)
        {
            fprintf(gp, "%f %f\n", x_lidar[i], y_lidar[i]);
        } //表示順はプロットと合わせる
        fprintf(gp, "e\n");
        fflush(gp);

        //printf("estx%lf esty:%lf oddm_x:%lf oddm_y:%lf lidar_x:%lf lidar_y:%lf goal_x:%lf goal_y:%lf \n", kf.xEst(0), kf.xEst(1), kf.xOddm(0), kf.xOddm(1), kf.xLidar(0), kf.xLidar(1), kf.xDes(0), kf.xDes(1));
    }
    //gettimeofday(&t1, NULL);
    //printf("takedtime:%ldusec\n", t1.tv_usec - t0.tv_usec);
    fprintf(gp, "exit\n");
    pclose(gp); //パイプを閉じる

    return 0;
}