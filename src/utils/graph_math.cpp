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

    // ===== PARAMETRELER =====
    double k_formation = 0.5;       // Formation gradient kazancı
    double k_position = 0.8;        // Pozisyon hatası kazancı
    double max_vel = 2.0;
    double damping = 0.3;
    double min_error = 0.3;         // Bu mesafenin altında "ulaştı" say (m)
    double collision_radius = 1.5;
    double collision_gain = 2.0;
    
    Eigen::Vector3d prev_vel = getPreviousVelocity(agent_id);
    
    this->updateGraph(swarm_positions);
    
    Eigen::Vector3d gradient = this->getGrad(agent_id);
    
    double formation_cost = 0.0;
    this->calcFNorm2(formation_cost);
    
    // ===== CENTROID HESAPLA =====
    Eigen::Vector3d current_centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d desired_centroid = Eigen::Vector3d::Zero();
    
    for (size_t i = 0; i < swarm_positions.size(); i++) {
        current_centroid += swarm_positions[i];
        desired_centroid += nodes_des[i];
    }
    current_centroid /= static_cast<double>(swarm_positions.size());
    desired_centroid /= static_cast<double>(nodes_des.size());
    
    // ===== POZİSYON HATASI HESAPLA =====
    // Her drone'un hedefine olan pozisyon hatası
    // (desired formation'ı current centroid'e translate et)
    Eigen::Vector3d centroid_offset = current_centroid - desired_centroid;
    Eigen::Vector3d adjusted_desired = nodes_des[agent_id] + centroid_offset;
    Eigen::Vector3d position_error = adjusted_desired - swarm_positions[agent_id];
    position_error.z() = 0.0;  // Z kilitle
    
    double pos_error_norm = position_error.norm();
    
    // ===== DEBUG LOG =====
    RCLCPP_INFO(rclcpp::get_logger("SwarmGraph"), 
        "[Agent %d] Pos:[%.1f,%.1f] AdjDes:[%.1f,%.1f] PosErr:%.2f Grad:[%.4f,%.4f] Cost:%.3f", 
        agent_id,
        swarm_positions[agent_id].x(), swarm_positions[agent_id].y(),
        adjusted_desired.x(), adjusted_desired.y(),
        pos_error_norm,
        gradient.x(), gradient.y(),
        formation_cost);
    
    gradient.z() = 0.0;
    
    // ===== ÇARPIŞMA ÖNLEME =====
    Eigen::Vector3d collision_gradient = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < swarm_positions.size(); i++) {
        if (static_cast<int>(i) == agent_id) continue;
        
        Eigen::Vector3d diff = swarm_positions[agent_id] - swarm_positions[i];
        diff.z() = 0.0;
        double dist = diff.norm();
        
        if (dist < collision_radius && dist > 0.01) {
            double repulsion = collision_gain * (1.0/dist - 1.0/collision_radius) * (1.0/(dist*dist));
            collision_gradient += repulsion * diff.normalized();
            
            RCLCPP_WARN(rclcpp::get_logger("SwarmGraph"), 
                "[Agent %d] COLLISION RISK with %zu, dist=%.2fm", agent_id, i, dist);
        }
    }
    
    // ===== DURMA KONTROLÜ =====
    if (pos_error_norm < min_error && collision_gradient.norm() < 0.01) {
        RCLCPP_INFO(rclcpp::get_logger("SwarmGraph"), 
            "[Agent %d] CONVERGED - pos_error=%.3f < min=%.2f", 
            agent_id, pos_error_norm, min_error);
        
        Eigen::Vector3d decaying_vel = prev_vel * 0.8;
        if (decaying_vel.norm() < 0.05) {
            decaying_vel = Eigen::Vector3d::Zero();
        }
        setPreviousVelocity(agent_id, decaying_vel);
        return decaying_vel;
    }
    
    // ===== HIZ HESAPLA =====
    // 1. Formation gradient: Şekli düzelt (graph-based)
    // 2. Position error: Hedefe doğru çek (centroid-aligned)
    // 3. Collision: Çarpışmadan kaçın
    
    Eigen::Vector3d vel_formation = -k_formation * gradient;
    Eigen::Vector3d vel_position = k_position * position_error.normalized() * std::min(pos_error_norm, max_vel);
    
    // Ağırlıklı toplam:
    // - Formation cost yüksekse: formation ağırlığı artar
    // - Position error yüksekse: position ağırlığı artar
    double formation_weight = std::min(1.0, formation_cost * 10.0);  // 0.1 cost → 1.0 weight
    double position_weight = 1.0;
    
    Eigen::Vector3d velocity = formation_weight * vel_formation 
                              + position_weight * vel_position 
                              + collision_gradient;
    velocity.z() = 0.0;
    
    // ===== HIZ SINIRLA =====
    if (velocity.norm() > max_vel) {
        velocity = velocity.normalized() * max_vel;
    }
    
    // ===== DAMPING =====
    velocity = (1.0 - damping) * velocity + damping * prev_vel;
    
    RCLCPP_INFO(rclcpp::get_logger("SwarmGraph"), 
        "[Agent %d] VelForm:[%.2f,%.2f] VelPos:[%.2f,%.2f] Final:[%.2f,%.2f] |v|=%.2f", 
        agent_id, 
        vel_formation.x(), vel_formation.y(),
        vel_position.x(), vel_position.y(),
        velocity.x(), velocity.y(), 
        velocity.norm());
    
    setPreviousVelocity(agent_id, velocity);
    
    return velocity;
}

