#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <stdio.h>

#include "utils/Logger.hpp"

#ifndef __VSTK_VIS_GRAPH_H
#define __VSTK_VIS_GRAPH_H

namespace vstk {

    class VisibilityGraph {
        private:
            std::unordered_map<std::string, std::unordered_set<std::string>> adj_list;

        public:
            VisibilityGraph();
            void insert_node(std::string image_id);
            void insert_edge(std::string src_image_id, std::string target_image_id);
            void delete_edge(std::string src_image_id, std::string target_image_id);
            bool is_node_present(std::string image_id);
            std::unordered_set<std::string> get_neighbours(std::string parent_image_id);

    };

}

#endif