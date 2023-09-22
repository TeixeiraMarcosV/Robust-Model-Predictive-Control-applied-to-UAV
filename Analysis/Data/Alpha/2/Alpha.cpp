#include "ModelR1.h"

float *alpha;

double cost_function(const Eigen::VectorXf& x){
    
    Eigen::VectorXf relative;
    
    return x(0)*x(0) + (*alpha)*x(1)*x(1);
}



unordered_map<int, Eigen::VectorXf> Goal;

Eigen::VectorXf goal(double current_time){
    // return Goal[round(Ccurrent_time*100)];
    return Goal[round(current_time/0.02)];
}

bool stopping_criterion(const Eigen::VectorXf& x){
//    Eigen::VectorXf e;
//    e = goal(current_time)  - x;
   return false;//e.maxCoeff() < 0.005;
}

int main(){

    float time;
     Eigen::VectorXf axx(2);
     for(time = 0; time<=1; time+=0.02){
          // cout << time << endl;
          axx<< 0,0;
          Goal[round(time/0.02)] = axx;
     }
     for(time = 1+0.02 ; time<=40; time+=0.02){
          // cout << time << endl;
          axx << 1,0;
          Goal[round(time/0.02)] = axx;
     }

    const int n =2;
    const int m = 1;
    const int p = 2;
    Eigen::MatrixXf A(n,n);
    Eigen::MatrixXf B(n,m);
    Eigen::MatrixXf C(p,n);
    Eigen::MatrixXf D(p,m);
    Eigen::MatrixXf discretization_actions(m,3); //cols (Min, max, level of discretization)
    Eigen::VectorXf noise(m);
    
    
    A << 1, 0.02,
         0, 1;
    

    B << 0.001962,
         0.1962;


    C << 1, 0,
         0, 1; 
    
    D << 0,
         0;

    

    noise<< 0;
    int actions_state_possible = 10;
    float discrete_time_step = 0.02;
    discretization_actions << -0.2, 0.2, 0.04;
    Eigen::VectorXf x_0(n);
    x_0 << 0,0;
    Model model1(actions_state_possible,A, B, C, D, discretization_actions, noise, cost_function, goal, 0.02);
    
    
    int horizon =25;
    float a = 0;
    alpha = &a;

    string name = "1DTeste24";
    stringstream ss;
    int pp = 0;

    for (a = 0; a< 0.02; a+=0.002 ){
        cout << a << endl;
        ss.str("");
	    ss.clear();
        ss  <<"Alphaa"<<pp;
        name = ss.str();
        model1.startSimulation(name, 5, x_0, 1, horizon, stopping_criterion);
        pp++;
    }

    return 0;
}