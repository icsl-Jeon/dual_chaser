//
// Created by jbs on 21. 6. 9..
//

#ifndef DUAL_CHASER_COMMON_H
#define DUAL_CHASER_COMMON_H
#include <chasing_utils/Utils.h>


namespace dual_chaser{

    struct GraphParm{
        int T; // horizon step
    };

    struct ChaserGraph{
        GraphParm param;
        Eigen::MatrixXf nodes; // column = [t n x y z i] (n : detector idx in its sphere / i node index )
        Eigen::MatrixXf edges; // column = [t u v w] (u,v : node idx )
        Eigen::MatrixXf optimal_cost_to_nodes; // column = [optimal_cost_to_the_node / optimal_parent]
        int* edge_div_location; // edge_div_location[t] : the first col where the edge from time t-1 to time (t) appears for the first time
        int* node_div_location; // node_div_location[t] : the first col where the node of time t appears for the first time
        int N_node = 0; // number of valid node
        int N_edge = 0;
        int N = 0; // horizon (layer number)
        Eigen::VectorXi optimal_node_idx_seq;
        ChaserGraph() = default;
        ChaserGraph(GraphParm param):param(param) {};
        vector<pcl::PointCloud<pcl::PointXYZ>> getPCLPath(string frame_id,
                                                          int drawUntil = numeric_limits<int>::max());

        void report();
        bool edge_relax();
        bool solve();
    };


}


#endif //DUAL_CHASER_COMMON_H
