//
// Created by usuario on 13/12/23.
//

#include "graph.h"

Graph::Graph()
{
    nodes.push_back(0);
    currentNode = 0;
}

int Graph::addNode()
{
    nodes.push_back(nodes.size());
    currentNode++;
    return nodes.size();
}

int Graph::addEdge(int n1, int n2)
{
    qInfo() << "n1: " << n1 << "n2: " << n2;
    if( std::ranges::find(nodes, n1) != nodes.end() and
        std::ranges::find(nodes, n2) != nodes.end() and
        std::ranges::find(edges, std::make_pair(n1, n2)) == edges.end())
            edges.emplace_back(n1,n2);
}
void Graph::print()
{
    qInfo() << "Node: ";
    for(const auto &n : nodes)
        qInfo() << n;
    qInfo() << "Edges: ";
    qInfo() << edges.size();
    for(const auto &n : edges)
        qInfo() << n.first << n.second;
}

int Graph::getCurrentNode() {
    return currentNode;
}

void Graph::drawGraph(AbstractGraphicViewer *viewer)
{

}
