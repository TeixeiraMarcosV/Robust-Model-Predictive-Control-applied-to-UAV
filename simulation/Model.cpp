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
//#include <Eigen/Dense>
#include "Myhash.h"
using namespace std;
/*
namespace MyHash {
    struct EigenVectorXfHash {
        std::size_t operator()(const Eigen::VectorXf& vector) const {
            std::size_t seed = vector.size();
            for (int i = 0; i < vector.size(); ++i) {
                auto value = std::hash<float>{}(vector(i));
                seed ^= value + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };
}
*/

class ModelAbstract{
    private:
        int n, m, p;
        int actions_possible_global;
        float discretization;
        Eigen::VectorXf noise;
        float discrete_time_step;
        int actions_state_possible;

};

class ModelXf : public ModelAbstract{
    private:Eigen::MatrixXf A;
            Eigen::MatrixXf B;
            Eigen::MatrixXf C;
            Eigen::MatrixXf D;


}; 

class Model{
    private:
    Eigen::MatrixXf A;
    Eigen::MatrixXf B;
    Eigen::MatrixXf C;
    Eigen::MatrixXf D;
    int n,m,p;
    int actions_possible_global;
     // Number of admissible actions per state
    float discretization;
    vector<Eigen::VectorXf> all_actions;
    Eigen::VectorXf noise;
    function<double(const Eigen::VectorXf&, const Eigen::VectorXf&)> cost_function;
    function<Eigen::VectorXf(double)> goal;
    float discrete_time_step;
    int actions_state_possible;
    //Model
        
    public:
        Eigen::VectorXf discretizeVector(const Eigen::VectorXf& x){
            Eigen::VectorXf discret(this->n);
            for(auto i=0; i< this->n; i++){
                discret(i) =  (round(x(i)/ this->discretization))*this->discretization ;
            }
            return discret;
        }
    
        Model( int actions_state_possible,const Eigen::MatrixXf& A, const Eigen::MatrixXf& B, const Eigen::MatrixXf& C, const Eigen::MatrixXf& D, const Eigen::MatrixXf& discretization_actions, const Eigen::VectorXf& noise,function<double(const Eigen::VectorXf&, const Eigen::VectorXf&)> cost_function, function<Eigen::VectorXf( double)> goal , float discrete_time_step){
            bool ax = true;

            //Verificar os ifs depois 
            if ( A.rows() != A.cols()) {
                //throw std::invalid_argument("Matrix A must be square.");
                std::cout << "Matrix A must be square." << std::endl;
                ax = false;

            }
            if ( A.rows() != B.rows()) {
                //throw std::invalid_argument("Matrix A and B must have the same amount of rows.");
                std::cout << "Matrix A and B must have the same amount of rows." << std::endl;
                ax = false;
            }
            if ( C.rows() != D.rows()) {
                //throw std::invalid_argument("Matrix C and D must have the same amount of rows.");
                std::cout << "Matrix C and D must have the same amount of rows."<< std::endl;
                ax = false;
            }
            if ( B.cols() != D.cols()) {
                //throw std::invalid_argument("Matrix B and D must have the same amount of cols.");
                std::cout << "Matrix B and D must have the same amount of cols." << std::endl;
                ax = false;
            }
            if ( B.cols() != discretization_actions.rows()) {
                //throw std::invalid_argument("Matrix B and D must have the same amount of cols.");
                std::cout << "Matrix B and discretization_actions must have the same amount of rows." << std::endl;
                ax = false;
            }

            if(ax){
                std::cout << "Successful model class initialization." << std::endl;
            }else{
                std::cout << "Model class initialization failed." << std::endl;
            }
            this->n = A.rows();
            this->m = B.cols();
            this->p = C.rows();
            //std::cout << "Construtor: " << " p ="<< p << std::endl;
            this->A = A;
            this->B = B;
            this->C = C;
            this->D = D;
            this->discrete_time_step = discrete_time_step;
            this->noise = noise;
            this->discretization = 0.001; //standard discretization
            this->cost_function = cost_function;
            this->goal = goal;
            this->initiation_actions(discretization_actions);
            actions_possible_global = all_actions.size();
            if ( actions_possible_global < actions_state_possible) {
                //throw std::invalid_argument("The number of actions possibles by state is bigger than all actions possibles");
                std::cout << "The number of actions possibles by state is bigger than all actions possibles. by default both will be equal." << std::endl;
                this->actions_state_possible = actions_possible_global;
            }else{
                this->actions_state_possible = actions_state_possible;
            }
            }    
        void initiation_actions(const Eigen::MatrixXf& discretization_actions){
            vector<Eigen::VectorXf> all_discretization;
            all_discretization.clear();
            int  j, n_actions;
            for(j=0; j<  m;  j++){ //m é baseado na inicialisação e tem que bater 
                n_actions =( discretization_actions(j,1) - discretization_actions(j,0))/discretization_actions(j,2) +1;
                Eigen::VectorXf new_vector(n_actions);
                for( auto l = 0; l<n_actions ; l++){
                    new_vector(l) = discretization_actions(j,0) + l* discretization_actions(j,2);
                }
                all_discretization.push_back(new_vector);
            }
            int o = all_discretization.size();
            
            
            int max[o];
            int cont[o];
            int l=0;
            for(const auto& ac: all_discretization){
                max[l] = ac.size();
                cont[l] = 0;
                l++;
                
                }
            Eigen::VectorXf action(o);
            while (cont[0] < max[0]){ 
                
                l = 0;
                for(const auto& ac: all_discretization){
                    action(l) = ac(cont[l]);
                    l++;
                }
                all_actions.push_back(action);
                
                cont[o-1]++;
                for(auto i = o-1 ; i>0; i--){ 
                    if(cont[i] == max[i] ){
                        cont[i] = 0;
                        cont[i-1]++;
                        }
                    }
            
                
            }
            
            
        }
        
        void update_state_discretization(float a){
            this->discretization = a;
        }
         
        void modelInfo(const string& filename){
            ofstream file(filename);
            Eigen::VectorXf xx(4), goal(4);
            xx<< 0.5, 1, 0.8, 0;
            goal << 1,0,1,0;
            double a = cost_function(xx, goal);

            if (file.is_open()) {
                file << "Model Info:\n";
                file << "cust works"<< a <<"\n" ;
                file  << "A:" << "\n"<<A<<"\n"  ;
                file  << "B:" << "\n"<<B<<"\n"  ;
                file  << "C:" << "\n"<<C<<"\n"  ;
                file  << "D:" << "\n"<<D<<"\n"  ;
                file << "(n, m, p): "<<"("<<n<<", "<<m<<", "<<p<<")"<<"\n" ;
                file << "actions_possible_global:" << actions_possible_global<<"\n" ;
                file << "actions_state_possible:" << actions_state_possible<<"\n" ;
                file << "discretization:" << discretization<<"\n" ;
                file << "----------------------------------------------------------" <<"\n" ;
                file << "All actions:" <<"\n" ;
                for(auto& action: all_actions){
                    file << action.transpose() <<"\n";
                }
                file << "----------------------------------------------------------" <<"\n" ;
                file << noise.transpose()<< "\n" ;
                
                file.close();
                cout << "Data successfully written to CSV file: " << filename << endl;


            } else {
                cout << "Error opening CSV file: " << filename << endl;
            }
        }
        
        class Node;
        
         class Action_state_trial{
            public:
            Eigen::VectorXf action;
            double cost;
            list< Node*> childrem;
            unordered_map<Eigen::VectorXf, int, MyHash::EigenVectorXfHash, std::equal_to<Eigen::VectorXf>> childreem;
            Action_state_trial(Eigen::VectorXf action, double cost){
                this->action = action;
                this->cost = cost;
            }

            Action_state_trial(Eigen::VectorXf x, Eigen::VectorXf goal,Eigen::VectorXf action, function<double(const Eigen::VectorXf&, const Eigen::VectorXf&)> cost_function ){
                this->cost = cost_function(x, goal);
                this->action = action;
            } 
            Action_state_trial(){}
            
};
        
        class Node{
            public:
                Eigen::VectorXf x;
                double cost_value; 
                bool exist_childs = false;
                list<Action_state_trial> action_state_trial;
                int  NReached;

                Node(Eigen::VectorXf x, Model* model,double current_time){
                    Eigen::VectorXf G(x.size());
                    G.setZero();
                    this->x = model->discretizeVector(x);	
                    this->NReached = 1;
                    this->cost_value = model->cost_function(x, G);  //cost_function(x, goal);
                    this->exist_childs = false;
                    this->Admissible_Action(model, current_time);
                    }
                
                Node(){}
                
                static bool  compareByCost(Action_state_trial A, Action_state_trial B){
                    return A.cost < B.cost;
                    }
                
                void Admissible_Action(Model *model, double current_time){
                    Eigen::VectorXf next_x;
                        for(auto& action : model->all_actions ){
                            next_x = model->discretizeVector(model->A*(this->x + model->goal(current_time)) + model->B*action);
                            action_state_trial.push_back( Action_state_trial(action,model->cost_function(next_x, model->goal(current_time)) ) );
                        }
                        
                        action_state_trial.sort(compareByCost);	
                        auto it = action_state_trial.begin();
                        advance(it, model->actions_state_possible);
                        action_state_trial.erase(it, action_state_trial.end());
                        
                }
                
               Eigen::VectorXf CreateChildrem(Model *model, unordered_map<Eigen::VectorXf, Node*, MyHash::EigenVectorXfHash, std::equal_to<Eigen::VectorXf>>& all_exploited, double current_time){
                    //std::cout <<"Hello from createchildrem" <<std::endl;
                    //std::cout <<"Hello from createchildrem: state = " <<this->x.transpose()<<std::endl;
                    Eigen::VectorXf f_s;
                    Eigen::VectorXf u_r(model->m);
                    for(auto i = 0; i< (model->m); i++ ){
                            mt19937 gen(std::time(nullptr));
                            normal_distribution<double> distribution(0, model->noise(i));
                            u_r(i) = distribution(gen);
                        }
                    //std::cout <<"Hello from createchildrem: creat noise" <<std::endl;
                    for(auto& actions: action_state_trial){
                        //std::cout <<"Hello from createchildrem: actions" <<std::endl;
                        f_s = model->discretizeVector(model->A*(this->x + model->goal(current_time)) + (model->B)*(actions.action +  u_r) -  model->goal(current_time)); 
                        auto it_exploited = all_exploited.find(f_s);
                        if(it_exploited != all_exploited.end()){
                            it_exploited->second->NReached += 1;
                        }else{                         
                            all_exploited[f_s] = new Node(f_s, model, current_time); 
                            }
                        actions.childreem[f_s] = 0;
                        actions.childrem.push_front(all_exploited[f_s]);    
                            
                        
                        }    
                    this->exist_childs = true;
                    // for(auto& actions: action_state_trial){
                    //     std::cout <<"action "<< actions.action << " :"<<std::endl;
                    //     for(auto& child : actions.childrem){
                    //         std::cout << child->x.transpose() << " :"<<std::endl;
                    //     }
                    // }

                    //std::cout <<"Hello from createchildrem: return "<< (*(action_state_trial.begin()->childrem.begin()))->x.transpose() <<std::endl;
                    return (*(action_state_trial.begin()->childrem.begin()))->x  ;

                    


               }
               
                Eigen::VectorXf ExploitationStep(Model* model,unordered_map<Eigen::VectorXf, Node*, MyHash::EigenVectorXfHash, std::equal_to<Eigen::VectorXf>>& all_exploited, double current_time){
                    Eigen::VectorXf f_s;
                    Eigen::VectorXf u_r(model->m);
                    list<Action_state_trial>::iterator actions = this->action_state_trial.begin();
                    for(auto i = 0; i<model->m; i++ ){
                        mt19937 gen(std::time(nullptr));
                        normal_distribution<double> distribution(0, model->noise(i));
                        u_r(i) = distribution(gen);
                        }
                    f_s = model->discretizeVector(model->A*(this->x + model->goal(current_time)) + (model->B)*( action_state_trial.begin()->action +  u_r) - model->goal(current_time));
                    auto it_exploited = all_exploited.find(f_s);
                    if(it_exploited != all_exploited.end()){
                        it_exploited->second->NReached += 1;
                        
                    }else{                           
                        all_exploited[f_s] = new Node(f_s, model, current_time);
                        
                        }
                    auto it_child = actions->childreem.find(f_s);
                        if(it_child == actions->childreem.end()){
                            actions->childreem[f_s] = 0;
                            actions->childrem.push_front(all_exploited[f_s]);    
                            }
                    
                    return f_s;
                }
               
                void backubQValue(){ ///Fazer teste depois 
                    int  act_total;
                    double newQ =0;
                    for(auto& actions: action_state_trial){
                        act_total = 0;
                        for (auto& child: actions.childrem) {
                             act_total += child->NReached;	
                         }
                        for (auto& child: actions.childrem) {
                             newQ += child->NReached * child->cost_value;
                         } 
                        newQ /= act_total;
                        actions.cost = this->cost_value + newQ;
                    }
                    
                }
                
            };
         
        void trial(double current_time, Node *node, int horizon, unordered_map<Eigen::VectorXf, Node*, MyHash::EigenVectorXfHash, std::equal_to<Eigen::VectorXf>>& all_exploited, int layer){
            //std::cout <<"hello from trial:"<<"state ="<<node->x.transpose() <<std::endl; 
            Eigen::VectorXf f_s;
            list<Action_state_trial>::iterator it_ast = (node->action_state_trial).begin();
            Eigen::VectorXf xkplus1_A = A*node->x;

            if((*node).exist_childs){
                //std::cout <<"hello from if exist childs:" <<std::endl;	
                f_s = node->ExploitationStep(this,all_exploited, current_time);
            }else{
                //std::cout <<"Hello from if no exist childs:" <<std::endl;
                f_s = node->CreateChildrem(this,all_exploited, current_time);
            }
            if(layer==horizon){
                //std::cout <<"Hello from layer == horizon:" <<std::endl;
                return;
            }else{
                //std::cout <<"Hello from layer != horizon:" <<std::endl;
            trial( current_time , all_exploited[f_s], horizon , all_exploited, layer+1);		
            }

            node->backubQValue();
	

}
        void clear_all_exploited(unordered_map<Eigen::VectorXf, Node*, MyHash::EigenVectorXfHash, std::equal_to<Eigen::VectorXf>>& all_exploited){
            for(auto& a: all_exploited){
                delete a.second;
            }
            all_exploited.clear();
            
        }
        Eigen::VectorXf insert_all_exploited(unordered_map<Eigen::VectorXf, Node*, MyHash::EigenVectorXfHash, std::equal_to<Eigen::VectorXf>>& all_exploited,Eigen::VectorXf current_state, double current_time ){
            Eigen::VectorXf f_s;
            Eigen::VectorXf u_r(this->m);
            list<Action_state_trial>::iterator actions = all_exploited[current_state]->action_state_trial.begin();
            for(auto i = 0; i<this->m; i++ ){
                mt19937 gen(std::time(nullptr));
                normal_distribution<double> distribution(0, this->noise(i));
                u_r(i) = distribution(gen);
                }
            //f_s = this->discretizeVector(all_exploited[current_state]->xkplus1_A + (this->B)*( all_exploited[current_state]->action_state_trial.begin()->action + u_r ));
            f_s = this->discretizeVector(this->A*(all_exploited[current_state]->x + this->goal(current_time)) + (this->B)*( all_exploited[current_state]->action_state_trial.begin()->action + u_r ) - this->goal(current_time));
            auto it_exploited = all_exploited.find(f_s);
            if(it_exploited != all_exploited.end()){
                it_exploited->second->NReached += 1;
                actions->childrem.push_front(it_exploited->second);
            }else{                           
                all_exploited[f_s] = new Node(f_s, this, current_time);
                actions->childrem.push_front(all_exploited[f_s]);
                        }
            return f_s;
        }
        void writeDataToCSV(vector<Eigen::VectorXf>& states, const vector<float>& t, const vector<int>& all_trial, const string& filename) {
            ofstream file(filename);
            string a;
            int p, i;
            
            
            if (file.is_open()) {
                file << "State,  Time, Number_of _Trial\n";
                size_t numElements = states.size();
                
                // Write data to CSV file
                for ( i = 0; i < numElements; ++i) {
                    // states[i] = states[i] + this->goal(t[i]);
                    file  <<  states[i].transpose() <<"\t"<< t[i] << "\t" << all_trial[i] << "\n" ;
                    
                ;
                }
                
                file.close();
                cout << "Data successfully written to CSV file: " << filename << endl;
            } else {
                cout << "Error opening CSV file: " << filename << endl;
            }
        }
        
        void writeDataToCSV_all_states(unordered_map<Eigen::VectorXf, Node*, MyHash::EigenVectorXfHash, std::equal_to<Eigen::VectorXf>>& all_exploited, const string& filename) {
            ofstream file(filename);
            if (file.is_open()) {
                for (auto& pair : all_exploited) {
                file << pair.second->x.transpose()<<"\t"<<pair.second->NReached<< "\n";
                }
                file.close();
                cout << "Data successfully written to CSV file: " << filename << endl;
            } else {
                cout << "Error opening CSV file: " << filename << endl;
            }
        }

        void one_simulation(int o, Eigen::VectorXf incial_state, function<bool(Eigen::VectorXf, double)> stop_criterion , double mst, int horizon, string NameBase){
            //inicialization 
            //function<Eigen::VectorXf( double)> goal
            unordered_map<Eigen::VectorXf, Node*, MyHash::EigenVectorXfHash, std::equal_to<Eigen::VectorXf>> all_exploited;
            vector<Eigen::VectorXf> states;
            vector<float> t;
            vector<int> all_trial;
            Eigen::VectorXf current_state = incial_state;
            int n_trial;
            float current_time = 0;
            auto inicial = chrono::high_resolution_clock::now();
	        auto end = chrono::high_resolution_clock::now();
            chrono::duration<double> timeStep = end - inicial;
            all_exploited[current_state] = new Node(current_state, this, current_time);
            states.push_back(current_state - this->goal(0) );
            t.push_back(0);
            all_trial.push_back(0);
            n_trial = 0;
            current_time = 0;
             
            while( !stop_criterion(current_state,current_time) && mst>current_time ){
                //std::cout <<"stop_critérion:" <<stop_criterion(current_state,current_time) <<"\t current_time: "<<current_time <<std::endl;

                current_time += this->discrete_time_step;
                do{
                    //std::cout <<"Helle from do While():"  <<std::endl;
                    // printf("every thing ok befor trial   \n");
                    trial(current_time - this->discrete_time_step, all_exploited[current_state],horizon, all_exploited, 1);
                    // printf("every thing ok after trial   \n");
                    end = chrono::high_resolution_clock::now();
                    timeStep = end - inicial;
                    n_trial++;

                }while(timeStep.count() < current_time );
                //t.push_back(timeStep.count());
                t.push_back(current_time);
                all_trial.push_back(n_trial);
                //next_state
                // printf("every thing ok befor insert   \n");
                current_state = insert_all_exploited(all_exploited, current_state, current_time); 
                // printf("every thing ok after insert   \n");
                states.push_back(current_state);
                n_trial = 0;



            }
            //save in csv file
            stringstream ss;
            ss.str("");
		    ss.clear();
            ss <<NameBase<<"Data" << o <<".csv";


            writeDataToCSV(states, t, all_trial, ss.str());
            states.clear();
            states.shrink_to_fit();
            t.clear();
            t.shrink_to_fit();
            all_trial.clear();
            all_trial.shrink_to_fit();

            ss.str("");
		    ss.clear();
            ss <<NameBase<<"States" << o <<".csv";
            writeDataToCSV_all_states( all_exploited,  ss.str());
            clear_all_exploited(all_exploited);
            
            
        }    
        void startSimulation(string NameBase, float max_time,Eigen::VectorXf incial_state, int number_simulations, int horizon, function<bool(Eigen::VectorXf, double)> stop_criterion){
            //std::cout <<"hello from startSimulation" << std::endl;
             
            for(auto i = 0; i<number_simulations;i++){
                //std::cout <<"Simulation:" << i<< std::endl;
                one_simulation(i, incial_state, stop_criterion,max_time, horizon, NameBase);
            }
                

        }

       
};


double cost_function(const Eigen::VectorXf& x, const Eigen::VectorXf& goal){
    
    Eigen::VectorXf relative;
    relative = goal-x;
    Eigen::VectorXf position(2);
    Eigen::VectorXf velocity(2);
    position(0) = relative(0);
    velocity(0) = relative(1);
    position(1) = relative(2);
    velocity(1) = relative(3);


    return position.dot(position) + 0.02*velocity.dot(velocity);
}

unordered_map<int, Eigen::VectorXf> Goal; 



Eigen::VectorXf goal(double current_time){
    
    return Goal[round(current_time*100)];

}
bool stopping_criterion(const Eigen::VectorXf& x, double current_time){
//    Eigen::VectorXf e;
//    e = goal(current_time)  - x;
   return false;//e.maxCoeff() < 0.005;
}


// class SistemaDiscreto {
// private:
//     Eigen::MatrixXd A_discreto_;
//     Eigen::MatrixXd B_discreto_;
//     Eigen::MatrixXd C_discreto_;
//     Eigen::MatrixXd D_discreto_;

// public:
//     SistemaDiscreto(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& D, double delta_t) {
//         // Calcula a representação discreta do sistema de espaço de estados

//         Eigen::MatrixXd Ad = (A*delta_t).exp();
//         Eigen::MatrixXd Bd;
//         Bd = B * delta_t;

//         // Eigen::MatrixXd temp = Eigen::MatrixXd::Identity(A.rows(), A.cols()) + delta_t * A;
//         // Eigen::MatrixXd Ad = Eigen::MatrixExponential<double>(temp).compute();
//         Eigen::MatrixXd Bd = delta_t * A.inverse() * (Ad - Eigen::MatrixXd::Identity(A.rows(), A.cols())) * B;
//         Eigen::MatrixXd Cd = C;
//         Eigen::MatrixXd Dd = D;

//         A_discreto_ = Ad;
//         B_discreto_ = Bd;
//         C_discreto_ = Cd;
//         D_discreto_ = Dd;
//     }

//     // Métodos para acessar as matrizes da representação discreta
//     Eigen::MatrixXd getAd() const { return A_discreto_; }
//     Eigen::MatrixXd getBd() const { return B_discreto_; }
//     Eigen::MatrixXd getCd() const { return C_discreto_; }
//     Eigen::MatrixXd getDd() const { return D_discreto_; }
// };



int main(){
    
    ///Verificar aleatoriedade
    // g++ -I Eigen Model.cpp -o Model
    // ./Model 
    const int n =4;
    const int m = 2;
    const int p = 4;
    Eigen::Matrix<double, n, n> A;
    Eigen::MatrixXd B(n,m);
    Eigen::MatrixXd C(p,n);
    Eigen::MatrixXd D(p,m);
    Eigen::MatrixXd discretization_actions(m,3); //cols (Min, max, level of discretization)
    Eigen::VectorXd noise(m);
    
    
    A << 1, 0.2, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0.2,
         0, 0, 0, 1;
    

    B << -0.001962, 0,
         -0.1962, 0,
         0, 0.001962,
         0, 0.1962;


    C << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1; 
    
    D << 0, 0,
         0, 0,
         0, 0,
         0, 0;

    //Eigen::MatrixXd Ad = A.exp();
    //std::cout <<"Ad = "<< Ad <<std::endl;
    
    //
    // noise<< 0.03, 0.03; ////Verificar a aleatoriedade disso depois.  


    // int actions_state_possible = 10;
    // float discrete_time_step = 0.02;
    // discretization_actions << -0.2, 0.2, 0.04,
    //                             -0.2, 0.2, 0.04; //Variar 0.005
    // Eigen::VectorXf x_0(4);
    // x_0 << 0,0,0,0;
    
    // Model model1(actions_state_possible,A, B, C, D , discretization_actions, noise, cost_function, goal, discrete_time_step);    
    // //noise<< 0.1; ////Verificar a aleatoriedade disso depois. 
    // model1.update_state_discretization(0.0001);
    
    // int horizon =5;
    // string name = "2D2";
    // float time_simulation = 30;
    // float pi = 3.1415;
    
    // float a =1;
    // float b = 1;
    // float y0 = 0;
    // float x0 = 0;
    // float f = pi/4;

    // Eigen::VectorXf ax(4);
    
    // for(float t = 0; t<=time_simulation+10; t+=discrete_time_step){ //inicialisation of goal 
    //     ax(0) = x0 + a*cos(f*t);
    //     ax(1) = -a*f*sin(f*t);
    //     ax(2) = y0 + b*sin(f*t);
    //     ax(3) = b*f*sin(f*t);
    //     Goal[round(t*100)] = ax;
        
    // }

      
    // model1.startSimulation(name, time_simulation, x_0, 1, horizon, stopping_criterion);
        
    
    return 0;
}