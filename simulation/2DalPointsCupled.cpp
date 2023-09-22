
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
     for(time = 0 ; time<2; time+=0.02){
      axx << -1.25672,0, 1.4435, 0;
      Goal[round(time/0.02)] = axx;
    }
    for(time = 2 ; time<4; time+=0.02){
        axx << 0.38731,0, -0.89962, 0;
        Goal[round(time/0.02)] = axx;
    }
    for(time = 4 ; time<6; time+=0.02){
        axx << -1.29906,0, 1.32076, 0;
        Goal[round(time/0.02)] = axx;
    }
    for(time = 6 ; time<8; time+=0.02){
        axx << 1.10249,0, 1.79979, 0;
        Goal[round(time/0.02)] = axx;
    }
    for(time = 8 ; time<10; time+=0.02){
        axx << -1.23527,0, -1.48985, 0;
        Goal[round(time/0.02)] = axx;
    }
    for(time = 10 ; time<12; time+=0.02){
        axx << -1.79865,0, -1.67077, 0;
        Goal[round(time/0.02)] = axx;
    }
    for(time = 12 ; time<14; time+=0.02){
        axx << 1.52187,0, 1.99968, 0;
        Goal[round(time/0.02)] = axx;
    }
    for(time = 14; time<16; time+=0.02){
        axx << -1.81496,0, 1.48434, 0;
        Goal[round(time/0.02)] = axx;
    }
    for(time = 16 ; time<18; time+=0.02){
        axx << 0.67372,0, -1.70463, 0;
        Goal[round(time/0.02)] = axx;
    }
    for(time = 18 ; time<20; time+=0.02){
        axx << 0.21841,0, -0.71277, 0;
        Goal[round(time/0.02)] = axx;
    }
    for(time = 20 ; time<22; time+=0.02){
        axx << -0.17036,0, -1.51138, 0;
        Goal[round(time/0.02)] = axx;
    }
    for(time = 22  ; time<24; time+=0.02){
        axx << -1.9398,0, 0.96539, 0;
        Goal[round(time/0.02)] = axx;
    }
    for(time = 24  ; time<26; time+=0.02){
        axx << -1.38845,0, 0.65388, 0;
        Goal[round(time/0.02)] = axx;
    }
    for(time = 26  ; time<28; time+=0.02){
        axx << -0.85764,0, 0.4996, 0;
        Goal[round(time/0.02)] = axx;
    }
    for(time = 28  ; time<30; time+=0.02){
        axx << -1.58354,0, 0.20549, 0;
        Goal[round(time/0.02)] = axx;
    }
    for(time = 30  ; time<32; time+=0.02){
        axx << 0.69662,0, -1.35393, 0;
        Goal[round(time/0.02)] = axx;
    }
    for(time = 32  ; time<34; time+=0.02){
        axx << -0.12876,0, 1.65833, 0;
        Goal[round(time/0.02)] = axx;
    }
    for(time = 34  ; time<36; time+=0.02){
        axx << 1.32639,0, 0.4623, 0;
        Goal[round(time/0.02)] = axx;
    }
    for(time = 36  ; time<38; time+=0.02){
        axx << 1.77204,0, -1.19621, 0;
        Goal[round(time/0.02)] = axx;
    }
    for(time = 38  ; time<40; time+=0.02){
        axx << -1.17734,0, 1.79872, 0;
        Goal[round(time/0.02)] = axx;
    }
    for(time = 40  ; time<42; time+=0.02){
        axx << -0.08793,0, -0.54533, 0;
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
    model1.startSimulation("2DalCupled", 30, x_0, 10, horizon, stopping_criterion);
//     for (auto& g: Goal){
//           cout << g.first*0.02 << "- "<< g.second.transpose() << endl;
//     }

    return 0;
}

















