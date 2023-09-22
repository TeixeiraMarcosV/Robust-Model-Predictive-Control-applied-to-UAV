#include "ModelR1.h"

double cost_function(const Eigen::VectorXf& x){
    double a =x(0)*x(0) + 0.0035*x(1)*x(1);
    a += x(2)*x(2) + 0.0035*x(3)*x(3);
 
    return a;
//     return x(0)*x(0) + 0.012*x(1)*x(1);
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
     // inicialize Goal
     float f = 1;
     float time;
     Eigen::VectorXf axx(4);
     for(time = 0 ; time<=40; time+=0.02){
          // cout << time << endl;
          axx << cos(time),-f*sin(time), sin(time), f*cos(time);
          Goal[round(time/0.02)] = axx;
     }

     




    const int n =4;
    const int m = 2;
    const int p = 4;
    Eigen::MatrixXf A(n,n);
    
    Eigen::MatrixXf B(n,m);
    Eigen::MatrixXf C(p,n);
    Eigen::MatrixXf D(p,m);
    Eigen::MatrixXf discretization_actions(m,3); //cols (Min, max, level of discretization)
    Eigen::VectorXf noise(m);
    
    
    A << 1, 0.02,0,0,
         0, 1,0,0,
         0,0,1,0.02,
         0,0,0,1;
    

    B << -0.001962, 0,
         -0.1962, 0,
         0, 0.001962,
         0, 0.1962;


    C << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1; 
    
    D << 0,0,
         0,0,
         0,0,
         0,0;
    
    noise<< 0.03, 0.03;
    int actions_state_possible = 4;
    float discrete_time_step = 0.02;
    cout << "Teste " << endl;
    discretization_actions << -0.2, 0.2, 0.04,
                              -0.2, 0.2, 0.04;
                              
    Eigen::VectorXf x_0(n);
    x_0 << 0,0,0,0;
    
    Model model1(actions_state_possible,A, B, C, D , discretization_actions, noise, cost_function, goal, discrete_time_step);
    
    int horizon = 2;
    model1.startSimulation("2DelpticH22", 20, x_0, 5, horizon, stopping_criterion);
//     for (auto& g: Goal){
//           cout << g.first*0.02 << "- "<< g.second.transpose() << endl;
//     }

    return 0;
}
