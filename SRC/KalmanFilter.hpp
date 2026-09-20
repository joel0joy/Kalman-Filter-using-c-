#pragma once

#include<iostream>

#include<Eigen/dense>



class KalmanFilter{

public:
KalmanFilter(
    double dt,
    const Eigen::MatrixXd& F,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& P,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& H
);

KalmanFilter();

void init();

void init(double dt, const Eigen::VectorXd& X0);

void update(const Eigen::VectorXd& z);

void update(const Eigen::VectorXd& z,double dt,const Eigen::MatrixXd& F);

double time(){ return t;}

Eigen::VectorXd state(){ return X_hat;}


private:

Eigen::MatrixXd F,H,P,Q,R,P0,K,I;

int n,m;

double dt,t,t0;

Eigen::VectorXd X_hat, X_hat_new;

bool initialized;

};


