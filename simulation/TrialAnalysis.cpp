#include "Model.h"
#include <iomanip>

double cost_function(const Eigen::VectorXf& x, const Eigen::VectorXf& goal){
    
    Eigen::VectorXf relative;
    relative = goal-x;
    return relative(0)*relative(0) + 0.0035*relative(1)*relative(1);
}

Eigen::VectorXf goal(double current_time){
    Eigen::VectorXf Goal(2);
    Goal << 1,0;
    // return Goal[round(Ccurrent_time*100)];
    return Goal;
}

bool stopping_criterion(const Eigen::VectorXf& x, double current_time){
//    Eigen::VectorXf e;
//    e = goal(current_time)  - x;
   return false;//e.maxCoeff() < 0.005;
}

void writeDataToCSV(vector<double> a , string filename){
            ofstream file(filename);
            if (file.is_open()) {
                // Write data to CSV file
                for ( auto& time : a) {
                    file  <<std::fixed <<  setprecision(15) << time << "\n";
                }
                
                file.close();
                cout << "Data successfully written to CSV file: " << filename << endl;
            } else {
                cout << "Error opening CSV file: " << filename << endl;
            }
        }
        

int main(){
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

    

    noise<< 0.04;
    int actions_state_possible = 10;
    float discrete_time_step = 0.03;
    discretization_actions << -0.2, 0.2, 0.04;
    Eigen::VectorXf x_0(n);
    x_0 << 0,0;
    
    Model model1(actions_state_possible,A, B, C, D , discretization_actions, noise, cost_function, goal, discrete_time_step);
    
    int horizon;
    
    

    
    int pp = 0;

    auto inicial = chrono::high_resolution_clock::now();
	auto end = chrono::high_resolution_clock::now();
    chrono::duration<double> timeStep = end - inicial;
    vector<double> ax_one_horizon;
    string name ;
    stringstream ss;

    Model::Node f_s(x_0*1/0.000001, &model1, 0);
    unordered_map<Eigen::VectorXf, Model::Node *, MyHash::EigenVectorXfHash, std::equal_to<Eigen::VectorXf>> all_exploited;
    for (horizon = 1; horizon<=24; horizon++ ){
            inicial = chrono::high_resolution_clock::now();
        for(auto i = 0; i<=1000; i++){
            
            model1.trial(0, &f_s, horizon, all_exploited, 1);
            end = chrono::high_resolution_clock::now();
            timeStep = end - inicial;
            ax_one_horizon.push_back(timeStep.count());
        }
        ss.str("");
		ss.clear();
        ss <<"Data"<<horizon<<".csv";
        name  = ss.str();
        writeDataToCSV(ax_one_horizon , name);
        ax_one_horizon.clear();
        model1.clear_all_exploited(all_exploited);

    }
    auto res = chrono::high_resolution_clock::period::den;
    cout << "Resolution: 1/" << res << " of a second" << endl;

    return 0;
}