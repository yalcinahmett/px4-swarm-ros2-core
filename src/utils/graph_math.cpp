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
        Eigen::Vector3d raw_grad = dfde.transpose() * dedp;
        
        // Zero out very small values (for numerical stability)
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

    // Safety checks
    if (agent_id < 0 || agent_id >= static_cast<int>(swarm_positions.size()) || !have_desired) {
        return Eigen::Vector3d::Zero();
    }

    // Update the graph
    this->updateGraph(swarm_positions);
    
    // Current velocity and Gradient
    Eigen::Vector3d prev_vel = getPreviousVelocity(agent_id);
    Eigen::Vector3d gradient = this->getGrad(agent_id);
    gradient.z() = 0.0; 

    // Find swarm and desired centroids
    Eigen::Vector3d current_centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d desired_centroid = Eigen::Vector3d::Zero();
    
    for (size_t i = 0; i < swarm_positions.size(); i++) {
        current_centroid += swarm_positions[i];
        desired_centroid += nodes_des[i]; // Ideal formation centroid
    }
    current_centroid /= static_cast<double>(swarm_positions.size());
    desired_centroid /= static_cast<double>(nodes_des.size());

    // We shift the target according to the CURRENT centroid of the swarm.
    // Wherever the swarm has moved, my target moves there too.
    Eigen::Vector3d centroid_shift = current_centroid - desired_centroid;
    
    // Flexible target position
    Eigen::Vector3d flexible_target = nodes_des[agent_id] + centroid_shift; 
    
    // Error vector
    Eigen::Vector3d position_error = flexible_target - swarm_positions[agent_id];
    position_error.z() = 0.0;

    double k_graph    = 1.5;  // Graph based velocity gain
    double k_position = 0.5;  // Position correction gain
    double k_drag     = 0.2;  // Drag prevention "Virtual Anchor" (Should be very small)
    
    Eigen::Vector3d vel_graph = -k_graph * gradient;

    Eigen::Vector3d vel_pos = k_position * position_error;

    Eigen::Vector3d anchor_pull = Eigen::Vector3d::Zero();
    if (current_centroid.norm() > 1.0) { // If the centroid has shifted more than 1 meter
        // Apply a small pull towards the centroid
        anchor_pull = -k_drag * (current_centroid - desired_centroid);
    }

    // ========== COLLISION AVOIDANCE - ULTRA AGGRESSIVE ==========
    Eigen::Vector3d collision_vec = Eigen::Vector3d::Zero();
    double collision_radius = 5.0;     // Very early detection
    double safe_distance = 3.0;        // Larger safe zone
    double collision_gain = 25.0;      // Much stronger repulsion
    
    double min_dist_to_others = 999.0;
    int closest_neighbor = -1;
    
    for (size_t i = 0; i < swarm_positions.size(); i++) {
        if (static_cast<int>(i) == agent_id) continue;
        Eigen::Vector3d diff = swarm_positions[agent_id] - swarm_positions[i];
        diff.z() = 0.0;
        double dist = diff.norm();
        
        if (dist < min_dist_to_others) {
            min_dist_to_others = dist;
            closest_neighbor = i;
        }
        
        if (dist < collision_radius && dist > 0.01) {
            double danger = (collision_radius - dist) / (collision_radius - safe_distance);
            danger = std::max(0.0, std::min(danger, 1.0));
            
            // EXTREME exponential repulsion
            double rep = collision_gain * std::pow(danger, 4.0) * (1.0 / (dist * dist));
            
            if (dist < safe_distance) {
                // CRITICAL: Override everything else
                rep = collision_gain * 20.0 / (dist * dist);
            }
            
            collision_vec += rep * diff.normalized();
        }
    }

    // ========== ADAPTIVE TARGET MODIFICATION ==========
    // If we're too close to someone AND they're blocking our path to target
    Eigen::Vector3d modified_target = flexible_target;
    
    if (min_dist_to_others < safe_distance * 1.5 && closest_neighbor >= 0) {
        // Vector from me to my target
        Eigen::Vector3d to_target = flexible_target - swarm_positions[agent_id];
        to_target.z() = 0.0;
        
        // Vector from me to the blocking neighbor
        Eigen::Vector3d to_neighbor = swarm_positions[closest_neighbor] - swarm_positions[agent_id];
        to_neighbor.z() = 0.0;
        
        // Check if neighbor is blocking my path (similar direction)
        if (to_target.norm() > 0.1 && to_neighbor.norm() > 0.1) {
            double angle_cos = to_target.dot(to_neighbor) / (to_target.norm() * to_neighbor.norm());
            
            // If neighbor is in front of me (angle < 90 degrees)
            if (angle_cos > 0.3) {
                // Create a temporary detour target - go around the neighbor
                Eigen::Vector3d perpendicular(-to_neighbor.y(), to_neighbor.x(), 0.0);
                perpendicular.normalize();
                
                // Detour target: move sideways to avoid collision
                modified_target = swarm_positions[agent_id] + perpendicular * safe_distance * 2.0;
                
                RCLCPP_INFO(rclcpp::get_logger("SwarmGraph"), 
                    "Agent %d: Taking detour to avoid agent %d (dist=%.2f)", 
                    agent_id, closest_neighbor, min_dist_to_others);
            }
        }
    }
    
    // Recalculate position error with modified target
    position_error = modified_target - swarm_positions[agent_id];
    position_error.z() = 0.0;
    vel_pos = k_position * position_error;

    // ========== VELOCITY CALCULATION WITH PRIORITIES ==========
    Eigen::Vector3d velocity;
    
    if (min_dist_to_others < safe_distance) {
        // EMERGENCY: 100% collision avoidance
        velocity = collision_vec;
        
        RCLCPP_WARN(rclcpp::get_logger("SwarmGraph"), 
            "Agent %d: EMERGENCY! Distance: %.2fm - IGNORING FORMATION", 
            agent_id, min_dist_to_others);
            
    } else if (min_dist_to_others < safe_distance * 1.5) {
        // DANGER ZONE: 90% collision, 10% formation
        velocity = 0.9 * collision_vec + 0.1 * (vel_graph + vel_pos + anchor_pull);
        
        RCLCPP_WARN(rclcpp::get_logger("SwarmGraph"), 
            "Agent %d: DANGER ZONE! Distance: %.2fm", 
            agent_id, min_dist_to_others);
        
    } else {
        // SAFE: Normal operation with all forces
        velocity = vel_graph + vel_pos + anchor_pull + collision_vec;
    }

    // Limits and Damping
    double max_vel = 1.5;
    if (velocity.norm() > max_vel) velocity = velocity.normalized() * max_vel;
    
    // Reduce damping in emergency situations for faster response
    double damping = (min_dist_to_others < safe_distance) ? 0.05 : 0.2;
    velocity = (1.0 - damping) * velocity + damping * prev_vel;
    setPreviousVelocity(agent_id, velocity);

    return velocity;
}