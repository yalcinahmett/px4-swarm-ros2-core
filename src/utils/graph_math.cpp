#include "graph_math.hpp"
   
SwarmGraph::SwarmGraph(){   
    have_desired = false;
}

bool SwarmGraph::updateGraph( const std::vector<Eigen::Vector3d> &swarm){ 

    //Update the vertices
    nodes = swarm;
    
    if(nodes.size() != nodes_des.size()){
        std::cout <<"swarm size : " << nodes.size() << std::endl;
        RCLCPP_WARN(rclcpp::get_logger("SwarmGraph"), "Size of swarm formation vector is incorrect. ");
        return false;
    }

    //Update the feature matrices 
    calcMatrices(nodes, A, D, Lhat);

    if( have_desired ){

        DLhat = Lhat - Lhat_des;

        //Update the gradient vector
        Eigen::Vector3d gradp;
        agent_grad.clear();
        for( size_t idx = 0; idx < nodes.size(); idx++ ){
            calcFGrad(gradp, idx);
            agent_grad.push_back(gradp);
        }
    }else{
        RCLCPP_WARN(rclcpp::get_logger("SwarmGraph"), "Please enter the desired formation!!");
    }
    
    return true;
}


bool SwarmGraph::setDesiredForm( const std::vector<Eigen::Vector3d> &swarm_des ){
    
    //Save the initial desired nodes
    if( !have_desired )
        nodes_des_init = swarm_des;

    nodes_des = swarm_des;
    calcMatrices(swarm_des, A_des, D_des, Lhat_des);
    have_desired = true;
    return have_desired;
}   


bool SwarmGraph::calcMatrices( const std::vector<Eigen::Vector3d> &swarm,
                       Eigen::MatrixXd &Adj, Eigen::VectorXd &Deg,
                       Eigen::MatrixXd &SNL )
{   
    //Init the matrices
    Adj = Eigen::MatrixXd::Zero( swarm.size(), swarm.size() );
    Deg = Eigen::VectorXd::Zero( swarm.size() );
    SNL = Eigen::MatrixXd::Zero( swarm.size(), swarm.size() );

    //Adjacency and Degree
    for( size_t i = 0; i < swarm.size(); i++ ){
        for( size_t j = 0; j < swarm.size(); j++ ){
            Adj(i,j) = calcDist2( swarm[i], swarm[j] );
            Deg(i) += Adj(i,j);
        }
    }

    //Symmetric Normalized Laplacian
    for( size_t i = 0; i < swarm.size(); i++ ){
        for( size_t j = 0; j < swarm.size(); j++ ){
            if( i == j){
                SNL(i,j) = 1;
            }else{
                SNL(i,j) = -Adj(i,j) * pow(Deg(i),-0.5) * pow(Deg(j),-0.5);
            }
        }
    }
    
    return true;
}

double SwarmGraph::calcDist2( const Eigen::Vector3d &v1, const Eigen::Vector3d &v2){
    return (v1-v2).cwiseAbs2().sum();
    
}


bool SwarmGraph::calcFNorm2( double &cost ){
    //Check if have the desired formation
    if( have_desired ){
        cost = DLhat.cwiseAbs2().sum();
        return true;
    }else{
        RCLCPP_WARN(rclcpp::get_logger("SwarmGraph"), "Invalid desired formation.");
        return false;
    }
}

bool SwarmGraph::calcFGrad( Eigen::Vector3d &gradp, int idx ){
    //Check if have the desired formation
    if( have_desired ){
        
        int iter = 0;
        int N = nodes.size();

        Eigen::VectorXd dfde = Eigen::VectorXd::Zero(N - 1);
        Eigen::MatrixXd dedp(N - 1, 3);

        
        //Get D_Fnorm_D_Edge
        double b0 = 0;
        for( int k = 0; k < N; k++ ){
            b0 += A(idx, k) * DLhat(idx, k) / sqrt(D(k));
        }
        b0 = 2 * pow(D(idx), -1.5) * b0;

        for( int i = 0; i < N; i++ ){
            if( i != idx){
                for( int k = 0; k < N; k++ ){
                    dfde(iter) += A(i, k) * DLhat(i, k) / sqrt(D(k));
                }
                dfde(iter) = 2 * pow(D(i), -1.5) * dfde(iter) + b0 + 4 * ( -1/(sqrt(D(i))*sqrt(D(idx)))) * DLhat(i, idx);
                iter++;
            }
        }

        //Get D_Edge_D_Pos
        iter = 0;
        for( int i = 0; i < N; i++ ){
            if( i != idx ){
                for( int k = 0; k < 3; k++ ){
                    dedp(iter, k) = nodes[idx](k) - nodes[i](k);
                }
                iter++;
            }
        }

        // Gerçek gradient - normalize ETMİYORUZ!
        // Böylece uzaktayken büyük, yakındayken küçük gradient elde ediyoruz
        Eigen::Vector3d raw_grad = dfde.transpose() * dedp;
        
        // Çok küçük değerleri sıfırla (sayısal kararlılık için)
        if(raw_grad.norm() < 1e-7){
            gradp = Eigen::Vector3d::Zero();
        } else {
            gradp = raw_grad;
        }
        return true;
    }else{
        RCLCPP_WARN(rclcpp::get_logger("SwarmGraph"), "Invalid desired formation.");
        return false;
    }
}

Eigen::Vector3d SwarmGraph::getGrad(int id){
    if (have_desired && id >= 0 && static_cast<size_t>(id) < nodes_des.size()){
        return agent_grad[id];
    } else {
        Eigen::Vector3d grad = Eigen::Vector3d::Zero();
        RCLCPP_WARN(rclcpp::get_logger("SwarmGraph"), "id is error !!! or desired graph has not been setup !!!");
        return grad;
    }
    
}

bool SwarmGraph::getGrad(std::vector<Eigen::Vector3d> &swarm_grad){
    if (have_desired){
        swarm_grad = agent_grad;
        return true;
    }
    else{
        RCLCPP_WARN(rclcpp::get_logger("SwarmGraph"), "desired graph has not been setup !!!");
        return false;
    }
}

Eigen::Vector3d SwarmGraph::computeVelocity(int agent_id, const std::vector<Eigen::Vector3d> &swarm_positions) {

    // Güvenlik kontrolleri
    if (agent_id < 0 || agent_id >= static_cast<int>(swarm_positions.size())) {
        RCLCPP_WARN(rclcpp::get_logger("SwarmGraph"), "Invalid agent_id: %d", agent_id);
        return Eigen::Vector3d::Zero();
    }
    
    if (!have_desired || agent_id >= static_cast<int>(nodes_des.size())) {
        RCLCPP_WARN(rclcpp::get_logger("SwarmGraph"), "Desired formation not set");
        return Eigen::Vector3d::Zero();
    }

    double k_gain = 100.0;          
    double max_vel = 3.0;         
    double damping = 0.4;         
    double min_gradient = 0.001; 
    
    this->updateGraph(swarm_positions);
    
    Eigen::Vector3d gradient = this->getGrad(agent_id);
    
    // DEBUG LOG
    RCLCPP_INFO(rclcpp::get_logger("SwarmGraph"), 
        "Agent %d | Gradient: [%.4f, %.4f, %.4f] Norm: %.4f", 
        agent_id, gradient.x(), gradient.y(), gradient.z(), gradient.norm());
    
    gradient.z() = 0.0;
    
    if (gradient.norm() < min_gradient) {
        RCLCPP_INFO(rclcpp::get_logger("SwarmGraph"), 
            "Agent %d STOPPING - gradient %.4f < min %.2f", 
            agent_id, gradient.norm(), min_gradient);
        
        previous_velocity_ = previous_velocity_ * 0.5;
        if (previous_velocity_.norm() < 0.05) {
            previous_velocity_ = Eigen::Vector3d::Zero();
        }
        return previous_velocity_;
    }
    
    Eigen::Vector3d velocity = -k_gain * gradient;
    
    velocity.z() = 0.0;
    
    if (velocity.norm() > max_vel) {
        velocity = velocity.normalized() * max_vel;
    }
    velocity = (1.0 - damping) * velocity + damping * previous_velocity_;
    
    // DEBUG LOG
    RCLCPP_INFO(rclcpp::get_logger("SwarmGraph"), 
        "Agent %d | Velocity: [%.2f, %.2f, %.2f]", 
        agent_id, velocity.x(), velocity.y(), velocity.z());
    
    previous_velocity_ = velocity;
    
    return velocity;
}

