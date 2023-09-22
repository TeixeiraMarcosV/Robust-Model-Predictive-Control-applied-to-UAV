#include <iostream>
#include <cmath>
#include <list>
#include <algorithm>
#include <random>
#include <chrono>
#include <vector>
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <functional>
#include "Myhash.h"
using namespace std;

class c2d{
    public:
        Eigen::MatrixXf A_dis, B_dis, C_dis, D_dis;
        double t_dis;
        c2d(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf C, Eigen::MatrixXf D, double t){
            this->t_dis = t;
            this->C_dis = C; 
            this->D_dis = D;
            calculateA_dis(A);
            calculateB_dis(A,B);

        }

        void calculateA_dis(Eigen::MatrixXf A){
            Eigen::MatrixXf Ax(A.rows(), A.rows()), Axx(A.rows(), A.rows());
            Ax.setIdentity(A.rows(), A.rows());
            int n = 1;
            Axx = t_dis* A;
            for(n = 2; n <20; n++){
                Ax += Axx; 
                Axx *= (t_dis/n)* A;
            }
        this->A_dis = Ax;
        }

        void calculateB_dis(Eigen::MatrixXf A, Eigen::MatrixXf B){
            Eigen::MatrixXf Ax(A_dis.rows(), A_dis.rows()), Axx(A_dis.rows(), A_dis.rows());
            Ax.setIdentity(A_dis.rows(), A_dis.rows());
            Ax *= t_dis;
            
            
            int n = 1;
            Axx = (t_dis* t_dis)*A*1/2;
            for(n = 2; n <20; n++){
                Ax -= Axx; 
                Axx *= (-t_dis/(n+1))* A;
            }
        this->B_dis = A_dis*Ax*B;
        }
    
        void display(){
            cout << "A_dis:"<< A_dis<< endl;
            cout << "B_dis:"<< B_dis<< endl;
            cout << "C_dis:"<< C_dis<< endl;
            cout << "D_dis:"<< D_dis<< endl;

        }

};



int main(){
    const int n =2;
    const int m = 1;
    const int p = 2;
    Eigen::MatrixXf A(n,n);
    Eigen::MatrixXf B(n,m);
    Eigen::MatrixXf C(p,n);
    Eigen::MatrixXf D(p,m);
   
    
    
    A << 0, 50,
         0, 0;
    

    B << 0,
         5638;


    C << 1, 0,
         0, 1; 
    
    D << 0,
         0;

    c2d discrete(A, B, C, D, 0.02);
    discrete.display();
    return 0;
}