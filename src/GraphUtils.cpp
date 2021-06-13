//
// Created by jbs on 21. 6. 9..
//

#include <dual_chaser/GraphUtils.h>


namespace dual_chaser {
    void ChaserGraph::report() {
        printf("N_node : %d / N_edge : %d\n", N_node, N_edge);
    }

    vector<pcl::PointCloud<pcl::PointXYZ>> ChaserGraph::getPCLPath(string frame_id,int drawUntil) {
        int drawTotal = min(drawUntil,N) ;
        vector<pcl::PointCloud<pcl::PointXYZ>> pclPath;

        for (int n = 0; n <= drawTotal; n++) {
            pcl::PointCloud<pcl::PointXYZ> pcl;
            int startIdx = node_div_location[n];
            int endIdx;

            if (n < drawTotal)
                endIdx = node_div_location[n + 1];
            else
                endIdx = N_node;
            for (int i = startIdx; i < endIdx; i++) {
                pcl::PointXYZ pnt;

                pnt.x = nodes(2, i);
                pnt.y = nodes(3, i);
                pnt.z = nodes(4, i);

                pcl.points.push_back(pnt);
            }
            pcl.header.frame_id = frame_id;
            pclPath.push_back(pcl);
        }
        return pclPath;
    }

    bool ChaserGraph::edge_relax() {
        if (N_node) {
            optimal_cost_to_nodes = Eigen::MatrixXf(2, N_node);
            // row 0 = optimal cost
            optimal_cost_to_nodes.block(0, 0, 1, N_node).setConstant(numeric_limits<float>::max());
            optimal_cost_to_nodes(0, 0) = 0;
            // row 1 = optimal parent
            optimal_cost_to_nodes.block(1, 0, 1, N_node).setConstant(-1);
            optimal_cost_to_nodes(1, 0) = -1;
            // relaxation
            for (int e_idx = 0; e_idx < N_edge; e_idx++) {
                int u = edges(1, e_idx), v = edges(2, e_idx);
                float w = edges(3, e_idx);
                if (optimal_cost_to_nodes(0, u) + w < optimal_cost_to_nodes(0, v)) {
                    optimal_cost_to_nodes(0, v) = optimal_cost_to_nodes(0, u) + w;
                    optimal_cost_to_nodes(1, v) = u;
                }
            }

            return true;
        } else
            cout << "[Warning] Edge relaxation was performed before the graph created" << endl;
        return false;
    }

    bool ChaserGraph::solve() {

        optimal_node_idx_seq = Eigen::VectorXi(N + 1); // optimal node sequence. This include 0th node also
        optimal_node_idx_seq.setConstant(-1);

        // 1. edge_relax
        if (edge_relax()) {
            // find the detector at every sequence from backtracing
            Eigen::Index n1 = node_div_location[N];
            Eigen::Index n2 = N_node - 1;
            Eigen::Index min_idx_sub_node; // index of optimal final node in the final sub set of Node matrix
            Eigen::Index opt_goal_idx;

            // First, let's find optimal goal among the last time step
            Eigen::VectorXf cost_last_layer(n2 - n1 + 1);
            for (int i = 0; i < cost_last_layer.size(); i++)
                cost_last_layer(i) = optimal_cost_to_nodes(0, n1 + i);

            cost_last_layer.minCoeff(&min_idx_sub_node);
            opt_goal_idx = n1 + min_idx_sub_node;
            optimal_node_idx_seq(N) = opt_goal_idx;
            int parent_idx, child_idx = opt_goal_idx;

            for (int t = N - 1; t >= 0; t--) {
                parent_idx = optimal_cost_to_nodes(1, child_idx);
                optimal_node_idx_seq(t) = parent_idx;
                child_idx = parent_idx;
            }
        }
            // if not, unsolved
        else {
            cout << "[DetectorGraph] no solution." << endl;
            return false;
        }
        cout << "optimal node" << endl;
        cout << optimal_node_idx_seq.transpose() << endl;
        return true;
    }

}