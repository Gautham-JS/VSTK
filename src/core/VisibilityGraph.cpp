#include "core/VisibilityGraph.hpp"

vstk::VisibilityGraph::VisibilityGraph() {};

void vstk::VisibilityGraph::insert_node(std::string image_id) {
    //DBGLOG("Inserting node %s to visibility-graph", image_id);
    if(!is_node_present(image_id)) this->adj_list[image_id] = std::unordered_set<std::string>();
}

void vstk::VisibilityGraph::insert_edge(std::string src_image_id, std::string target_image_id) {
    DBGLOG("Adding unweighted edge between nodes %s and %s", src_image_id, target_image_id);
    insert_node(src_image_id);
    insert_node(target_image_id);
    this->adj_list[src_image_id].insert(target_image_id);
}

void vstk::VisibilityGraph::delete_edge(std::string src_image_id, std::string target_image_id) {
    DBGLOG("Removing unweighted edge between nodes %s and %s", src_image_id, target_image_id);
    this->adj_list[src_image_id].erase(target_image_id);
}


bool vstk::VisibilityGraph::is_node_present(std::string image_id) {
    return (this->adj_list.find(image_id) != this->adj_list.end());
}

std::unordered_set<std::string> vstk::VisibilityGraph::get_neighbours(std::string parent_image_id) {
    if(!is_node_present(parent_image_id)) return std::unordered_set<std::string>();
    return (this->adj_list.find(parent_image_id))->second;
}