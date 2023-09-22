
#ifndef MODEL_H_INCLUDED
#define MODEL_H_INCLUDED

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
                discret(i) =  (round(x(i)));
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
            this->discretization = 0.000001; //standard discretization
            this->A = A;
            this->B = B;
            this->C = C;
            this->D = D;
            this->discrete_time_step = discrete_time_step;
            this->noise = noise*1/discretization;
            
            this->cost_function = cost_function;
            this->goal = goal;
            this->initiation_actions(discretization_actions*1/discretization); 
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
                // cout << action << endl;
                
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
            noise = noise*discretization/a;
            for( auto& action: all_actions){
                action =  action*discretization/a;
            }
            this->discretization = a;
            

        }
         
        void modelInfo(const string& filename){
            ofstream file(filename);
            Eigen::VectorXf xx(4), goal(4);
           
            if (file.is_open()) {
                file << "Model Info:\n";
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
                    file << action.transpose()*discretization <<"\n";
                }
                file << "----------------------------------------------------------" <<"\n" ;
                file << noise.transpose()*discretization << "\n" ;
                
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
            unordered_map<Eigen::VectorXf, Node*, MyHash::EigenVectorXfHash, std::equal_to<Eigen::VectorXf>> childrem;
            Action_state_trial(Eigen::VectorXf action, double cost){
                this->action = action;
                this->cost = cost;
            }

            
            Action_state_trial(){}
            
};
        
        class Node{
            public:
                Eigen::VectorXf x, x_plus1A;
                double cost_value; 
                bool exist_childs = false;
                list<Action_state_trial> action_state_trial;
                int  NReached;

                Node(Eigen::VectorXf x, Model* model,double current_time){
                    this->x = x;
                    // cout << "x_0 = "<< x.transpose()<< endl;
                    this->x_plus1A = model->A * x;	
                    this->NReached = 1;
                    this->cost_value = model->cost_function(x, (model->goal(current_time) * 1/(model->discretization)));  //cost_function(x, goal);
                    this->exist_childs = false;
                    // cout << "Alta so admissible_Action"<< endl;
                    this->Admissible_Action(model, current_time);
                    }
                
                Node(){}
                
                static bool  compareByCost(Action_state_trial A, Action_state_trial B){
                    return A.cost < B.cost;
                    }
                
                void Admissible_Action(Model *model, double current_time){
                    Eigen::VectorXf next_x;
                        for(auto& action : model->all_actions ){
                            next_x = model->discretizeVector(x_plus1A + model->B*action);
                            action_state_trial.push_back( Action_state_trial(action,model->cost_function(next_x, (model->goal(current_time) * 1/(model->discretization))) ) );
                            // cout << action << " - " << model->cost_function(next_x, (model->goal(current_time) * 1/(model->discretization))  ) << endl;
                        }
                        
                        // cout << "---------------------"<< endl;
                        action_state_trial.sort(compareByCost);	
                        auto it = action_state_trial.begin();
                        advance(it, model->actions_state_possible);
                        // cout << "---------------------"<< endl;
                        action_state_trial.erase(it, action_state_trial.end());
                        // for(auto& action : this->action_state_trial ){
                        //     cout << action.action <<" - " << action.cost << endl;
                            
                        // }
                        // cin.get();

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
                        // cout << actions.action<< endl;
                        // cin.get();
                        f_s = model->discretizeVector(this->x_plus1A+ (model->B)*(actions.action +  u_r) ); 
                        auto it_exploited = all_exploited.find(f_s);
                        if(it_exploited != all_exploited.end()){
                            it_exploited->second->NReached += 1;
                        }else{                         
                            all_exploited[f_s] = new Node(f_s, model, current_time); 
                            }
                        actions.childrem[f_s] = all_exploited[f_s];
                           
                        }    
                    this->exist_childs = true;
                    return action_state_trial.begin()->childrem.begin()->first;

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
                    f_s = model->discretizeVector(x_plus1A + (model->B)*( action_state_trial.begin()->action +  u_r) );
                    auto it_exploited = all_exploited.find(f_s);
                    if(it_exploited != all_exploited.end()){
                        it_exploited->second->NReached += 1;
                        
                    }else{                           
                        all_exploited[f_s] = new Node(f_s, model, current_time);
                        
                        }
                    actions->childrem[f_s] = all_exploited[f_s];
                    return f_s;
                }
               
                void backubQValue(){ ///Analys the case where only the node exploited is recaulated
                    int  act_total;
                    double newQ =0;
                    for(auto& actions: action_state_trial){
                        act_total = 0;
                        for (auto& child: actions.childrem) {
                            
                             act_total += child.second->NReached;	
                         }
                        for (auto& child: actions.childrem) {
                             newQ += child.second->NReached * child.second->cost_value;
                         } 
                        newQ /= act_total;
                        actions.cost = this->cost_value + newQ;
                    }
                    
                }
                
            };
        
        void trial(double current_time, Node *node, int horizon, unordered_map<Eigen::VectorXf, Node*, MyHash::EigenVectorXfHash, std::equal_to<Eigen::VectorXf>>& all_exploited, int layer){
            //std::cout <<"hello from trial:"<<"state ="<<node->x.transpose() <<std::endl; 
            Eigen::VectorXf f_s;
           
            if((*node).exist_childs){
                //std::cout <<"hello from if exist childs:" <<std::endl;	
                f_s = node->ExploitationStep(this,all_exploited, current_time);
            }else{
                //std::cout <<"Hello from if no exist childs:" <<std::endl;
                f_s = node->CreateChildrem(this,all_exploited, current_time);
            }
            if(layer!=horizon){
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
            f_s = this->discretizeVector( all_exploited[current_state]->x_plus1A + (this->B)*( all_exploited[current_state]->action_state_trial.begin()->action + u_r ) );
            auto it_exploited = all_exploited.find(f_s);
            if(it_exploited != all_exploited.end()){
                it_exploited->second->NReached += 1;
            }else{                           
                all_exploited[f_s] = new Node(f_s, this, current_time);
                }

            actions->childrem[f_s] = all_exploited[f_s];
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
                    file  <<  states[i].transpose()*discretization <<"\t"<< t[i] << "\t" << all_trial[i] << "\n" ;
                    
                
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
                file << pair.second->x.transpose()*discretization<<"\t"<<pair.second->NReached<< "\n";
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
            Eigen::VectorXf current_state = discretizeVector(incial_state*1/discretization);
            
            int n_trial;
            float current_time = 0;
            
            all_exploited[current_state] = new Node(current_state, this, current_time);
            // cout << "x_0 = "<< current_state.transpose()<< endl;
            states.push_back(current_state );
            t.push_back(0);
            all_trial.push_back(0);
            n_trial = 0;
            current_time = 0;
            auto inicial = chrono::high_resolution_clock::now();
	        auto end = chrono::high_resolution_clock::now();
            chrono::duration<double> timeStep = end - inicial;

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
            //writeDataToCSV_all_states( all_exploited,  ss.str());
            clear_all_exploited(all_exploited);
            
            
        }    
        void startSimulation(string NameBase, float max_time,Eigen::VectorXf incial_state, int number_simulations, int horizon, function<bool(Eigen::VectorXf, double)> stop_criterion){
            // std::cout <<"hello from startSimulation" << std::endl;
             
            for(auto i = 0; i<number_simulations;i++){
                //std::cout <<"Simulation:" << i<< std::endl;
                one_simulation(i, incial_state, stop_criterion,max_time, horizon, NameBase);
            }
                

        }

       
};



#endif