
#ifndef MODELR3_H_INCLUDED
#define MODELR3_H_INCLUDED

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
#include <unistd.h>
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
    vector<Eigen::VectorXf> B_time_action;
    Eigen::VectorXf noise;
    function<double(const Eigen::VectorXf&)> cost_function;
    function<Eigen::VectorXf( double)> goal1;
    function<Eigen::VectorXf( double)> goal2;
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
        Eigen::VectorXf discretizeVector2(const Eigen::VectorXf& x){
            Eigen::VectorXf discret(this->n*2);
            for(auto i=0; i< this->n*2; i++){
                discret(i) =  (round(x(i)));
            }
            return discret;
        }
    
        Model( int actions_state_possible,const Eigen::MatrixXf& A, const Eigen::MatrixXf& B, const Eigen::MatrixXf& C, const Eigen::MatrixXf& D, const Eigen::MatrixXf& discretization_actions, const Eigen::VectorXf& noise,function<double(const Eigen::VectorXf&)> cost_function, function<Eigen::VectorXf( double)> goal1, function<Eigen::VectorXf( double)> goal2 , float discrete_time_step){
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
            this->discretization = 0.0001; //standard discretization
            this->A = A;
            this->B = B;
            this->C = C;
            this->D = D;
            this->discrete_time_step = discrete_time_step;
            this->noise = noise*1/discretization;
            cout << this->noise.transpose();
            
            this->cost_function = cost_function;
            this->goal = goal1;
            this->goal1 = goal1; 
            this->goal2 = goal2;
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
        Eigen::VectorXf deterministic(Eigen::VectorXf x, Eigen::VectorXf u,double current_time){
            Eigen::VectorXf x_real = A*(goal(current_time) - x)  +B*u;

            return discretizeVector(goal(current_time) - x_real);

        }
        Eigen::VectorXf n_deterministic(Eigen::VectorXf x, Eigen::VectorXf u,double current_time){
            Eigen::VectorXf u_r(m);
            for(auto i = 0; i< (m); i++ ){
                mt19937 gen(std::time(nullptr));
                normal_distribution<double> distribution(0, noise(i));
                u_r(i) = distribution(gen);
            }
            Eigen::VectorXf x_real = A*(goal(current_time) - x)  +B*(u + u_r) ;

            return discretizeVector(goal(current_time) - x_real);

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
                B_time_action.push_back(B*action);
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
            public://This class is used to store the actions of each state and its important information.
            Eigen::VectorXf action;
            double cost;
            int  NReached = 0;
            unordered_map<Eigen::VectorXf, Node*, MyHash::EigenVectorXfHash, std::equal_to<Eigen::VectorXf>> childrem;
            unordered_map<Eigen::VectorXf, int, MyHash::EigenVectorXfHash, std::equal_to<Eigen::VectorXf>> NRchildrem;

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
            public://Stores the state and possible actions
                Eigen::VectorXf x;
                double cost_value; 
                bool exist_childs = false;
                list<Action_state_trial> action_state_trial;
                int  NReached;

                Node(Eigen::VectorXf x, Model* model,double current_time){
                    
                    this->x = x;
                    // cout << "x_0 = "<< x.transpose()<< endl;
                    this->NReached = 1;
                    this->cost_value = model->cost_function(x);  //cost_function(x, goal);
                    this->exist_childs = false;
                    // cout << "Alta so admissible_Action"<< endl;
                    // cout << "Contructor Node " << endl;

                    this->Admissible_Action(model, current_time);
                    
                    }
                
                Node(){}
                
                //used to organize the list of actions
                static bool  compareByCost(Action_state_trial A, Action_state_trial B){
                    return A.cost < B.cost;
                    }
                
                void Admissible_Action(Model *model, double current_time){
                    Eigen::VectorXf next_x, g, next_;
                    // g = model->goal(current_time);
                    // next_ = g - model->A*(g-this->x);
                    next_ =  model->A*(this->x);
                    auto it_B = model->B_time_action.begin();
                        for(auto& action : model->all_actions ){
                            next_x = model->discretizeVector(next_ - (*(it_B)));
                            it_B++;
                            //cout << "Hello from admissible_Action " << endl;
                            action_state_trial.push_back( Action_state_trial(action,model->cost_function(next_x ) ) );
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
                    
                    Eigen::VectorXf f_s;
                    Eigen::VectorXf  g, next_;
                    // g = model->goal(current_time)* 1/(model->discretization);
                    // next_ = g - model->A*(g-this->x);
                    next_ =  model->A*(this->x);
                    Eigen::VectorXf u_r(model->m);
                    for(auto i = 0; i< (model->m); i++ ){
                        mt19937 gen(std::time(nullptr));
                        normal_distribution<double> distribution(0, model->noise(i));
                        u_r(i) = distribution(gen);
                    }

                    for(auto& actions: action_state_trial){
                        actions.NReached = 1;
                        //std::cout <<"Hello from createchildrem: actions" <<std::endl;
                        // cout << actions.action<< endl;
                        // cin.get();

                        f_s = model->discretizeVector(next_ - model->B * actions.action) ;
                        auto it_exploited = all_exploited.find(f_s);
                        if(it_exploited != all_exploited.end()){
                            // it_exploited->second->NReached += 1;
                        }else{                         
                            all_exploited[f_s] = new Node(f_s, model, current_time); 
                            }

                        actions.childrem[f_s] = all_exploited[f_s];
                        actions.NRchildrem[f_s] = 1;
                           
                        }    
                    this->exist_childs = true;
                    return action_state_trial.begin()->childrem.begin()->first;
               }
                
                Eigen::VectorXf ExploitationStep(Model* model,unordered_map<Eigen::VectorXf, Node*, MyHash::EigenVectorXfHash, std::equal_to<Eigen::VectorXf>>& all_exploited, double current_time){
                    Eigen::VectorXf f_s;
                    Eigen::VectorXf  g, next_;
                    // g = model->goal(current_time)* 1/(model->discretization);
                    next_ =  model->A*(this->x);
                    Eigen::VectorXf u_r(model->m);
                    for(auto i = 0; i< (model->m); i++ ){
                        mt19937 gen(std::time(nullptr));
                        normal_distribution<double> distribution(0, model->noise(i));
                        u_r(i) = distribution(gen);
                    }
                   
                    list<Action_state_trial>::iterator actions = this->action_state_trial.begin();
                    f_s = model->discretizeVector(next_ - model->B*actions->action );
                    auto it_exploited = all_exploited.find(f_s);
                    if(it_exploited != all_exploited.end()){
                        // it_exploited->second->NReached += 1;
                        
                    }else{                           
                        all_exploited[f_s] = new Node(f_s, model, current_time);
                        
                        }
                    actions->childrem[f_s] = all_exploited[f_s];
                    
                    auto it_child =   actions->NRchildrem.find(f_s);
                    if(it_child != actions->NRchildrem.end()){
                        // it_exploited->second->NReached += 1;
                        actions->NRchildrem[f_s] +=1;
                        
                    }else{                           
                        actions->NRchildrem[f_s] =1;
                        }
                    return f_s;
                }
                double getMinQ_value(){

                    return  this->action_state_trial.begin()->cost;
                }

               
                void backubQValue(){ ///Analys the case where only the node exploited is recaulated
                    // cout << "hello from backbQvalue"<< endl;
                    int  act_total;
                    
                    for(auto& actions: action_state_trial){
                        double newQ =0;
                        for (auto& child: actions.childrem) {
                             newQ += actions.NRchildrem[child.second->x] * child.second->getMinQ_value();
                         } 
                        newQ /= actions.NReached;
                        // cout << "new Q = "<< newQ<< endl;
                        actions.cost = this->cost_value + newQ;
                        action_state_trial.sort(compareByCost);

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
            Eigen::VectorXf f_s, next_, goal_plus1, goal_n;
            Eigen::VectorXf u_r(this->m);
            goal_n = goal(current_time);
            goal_plus1 = goal(current_time + discrete_time_step );
            next_ = goal_plus1* 1/(discretization)  + A*( current_state - goal_n* 1/(discretization) );

            list<Action_state_trial>::iterator actions = all_exploited[current_state]->action_state_trial.begin();
            for(auto i = 0; i<this->m; i++ ){
                mt19937 gen(std::time(nullptr));
                normal_distribution<double> distribution(0, this->noise(i));
                u_r(i) = distribution(gen);
                }
            //f_s = this->discretizeVector(all_exploited[current_state]->xkplus1_A + (this->B)*( all_exploited[current_state]->action_state_trial.begin()->action + u_r ));
            f_s = this->discretizeVector( next_ - B*(all_exploited[current_state]->action_state_trial.begin()->action + u_r));
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
            
            Eigen::VectorXf x_real(2*n); 
            Eigen::VectorXf gg(2*n); 

            
            
            if (file.is_open()) {
                file << "State,  Time, Number_of _Trial\n";
                size_t numElements = states.size();
                
                // Write data to CSV file
                for ( i = 0; i < numElements; ++i) {
                    // states[i] = states[i] + this->goal(t[i]);
                    // file  <<  states[i].transpose()*discretization    <<"\t"<< t[i] << "\t" << all_trial[i] << "\n" ;
                    gg << goal1(t[i]), goal2(t[i]);
                    x_real = gg -  states[i]*discretization;
                    file  <<  x_real .transpose() <<" "<< gg.transpose() <<"\t"<< t[i] << "\t" << all_trial[i] << "\n" ;
                    
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

        void one_simulation(int o, Eigen::VectorXf incial_state, function<bool(Eigen::VectorXf)> stop_criterion , double mst, int horizon, string NameBase){
            //inicialization 
            
            // std::cout <<"hello from one simulation" << std::endl;
            unordered_map<Eigen::VectorXf, Node*, MyHash::EigenVectorXfHash, std::equal_to<Eigen::VectorXf>> all_exploited;
            vector<Eigen::VectorXf> states;
            vector<float> t;
            vector<int> all_trial;
            // std::cout << "incial  "<< incial_state.transpose() <<std::endl;
            // std::cout << "Goal "<< goal(0).transpose() <<std::endl;
            Eigen::VectorXf current_state(incial_state.size());

            Eigen::VectorXf gg(2*n) ;
            gg << this->goal1(0), this->goal2(0);
            // cout << gg.transpose()<< endl;
            
            current_state = discretizeVector2((  gg - incial_state  )*1/discretization);
            
            
            Eigen::VectorXf X(n);
            Eigen::VectorXf Y(n);
            int pp;
            // std::cout << "current_state"<< current_state.transpose() <<std::endl;
            
            for(pp =0; pp< n; pp+=1 ){
                // std::cout <<"pp =" <<pp<< "  "<< current_state(2*pp)<< "  "<< current_state(2*pp +1) <<std::endl;
                
                X(pp) =  current_state(pp);
                Y(pp) = current_state(2 + pp);
            }
            
            // std::cout <<"hello from one simulation, size = " <<n<< std::endl;
            int n_trial;
            float current_time = 0;
            goal = goal1;
            
            // std::cout <<"hello from one simulation--------------------------------------------------------------------1" << std::endl;
            all_exploited[X] = new Node(X, this, current_time);
             
            // std::cout <<"hello from one simulation--------------------------------------------------------------------2" << std::endl;
            goal = goal2;
            all_exploited[Y] = new Node(Y, this, current_time);
            // std::cout <<"hello from one simulation--------------------------------------------------------------------2" << std::endl;
            // cout << "hello from one simulation  while " << endl;
            // cout << "x_0 = "<< current_state.transpose()<< endl;
            states.push_back(current_state );
            t.push_back(0);
            all_trial.push_back(0);
            n_trial = 0;
            current_time = 0;
            auto inicial = chrono::high_resolution_clock::now();
	        auto end = chrono::high_resolution_clock::now();
            chrono::duration<double> timeStep = end - inicial;
            
            while( !stop_criterion(current_state) && mst>current_time ){
               
                //std::cout <<"stop_critérion:" <<stop_criterion(current_state,current_time) <<"\t current_time: "<<current_time <<std::endl;

                current_time += this->discrete_time_step;
                do{
                    // cout << "custos (x, y) = ("<<all_exploited[X]->action_state_trial.begin()->cost << "," << all_exploited[Y]->action_state_trial.begin()->cost <<endl; 
                    goal = goal1;
                    trial(current_time - this->discrete_time_step, all_exploited[X],horizon, all_exploited, 1);
                    goal = goal2;
                    trial(current_time - this->discrete_time_step, all_exploited[Y],horizon, all_exploited, 1);
                    
                    
                    
                    end = chrono::high_resolution_clock::now();
                    timeStep = end - inicial;
                    n_trial++;

                }while(timeStep.count() < current_time );

               

                //t.push_back(timeStep.count());
                t.push_back(current_time);
                all_trial.push_back(n_trial);

                // Eigen::VectorXf  action_x = - all_exploited[X]->action_state_trial.begin()->action;
                // Eigen::VectorXf action_x = all_exploited[Y]->action_state_trial.begin()->action;
                goal = goal1;
                X = insert_all_exploited(all_exploited, X, current_time - discrete_time_step );
                goal = goal2;
                Y = insert_all_exploited(all_exploited, Y, current_time - discrete_time_step );
                
                // std::cout <<"hello from one simulation" << std::endl;

                current_state(0) = X(0);
                current_state(1) = X(1);
                current_state(2) = Y(0);
                current_state(3) = Y(1);
                
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
        void startSimulation(string NameBase, float max_time,Eigen::VectorXf incial_state, int number_simulations, int horizon, function<bool(Eigen::VectorXf)> stop_criterion){
            // std::cout <<"hello from startSimulation" << std::endl;
             
            for(auto i = 0; i<number_simulations;i++){
                //std::cout <<"Simulation:" << i<< std::endl;
                one_simulation(i, incial_state, stop_criterion,max_time, horizon, NameBase);
            }
                

        }

       
};



#endif