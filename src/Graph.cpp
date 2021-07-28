#include "Graph.h"
#include "Node.h"
#include "Edge.h"
#include <iostream>
#include <fstream>
#include <stack>
#include <queue>
#include <list>
#include <math.h>
#include <cstdlib>
#include <ctime>
#include <float.h>
#include <iomanip>

using namespace std;

/**************************************************************************************************
 * Defining the Graph's methods
**************************************************************************************************/

// Constructor
Graph::Graph(int order, bool directed, bool weighted_edge, bool weighted_node)
{

    this->order = order;
    this->directed = directed;
    this->weighted_edge = weighted_edge;
    this->weighted_node = weighted_node;
    this->first_node = this->last_node = nullptr;
    this->number_edges = 0;
}

void Graph::printarGrafo(){

        cout << "printando grafo" << endl;

      if(this->first_node != nullptr){

        for(Node* aux = this->first_node; aux != nullptr; aux = aux->getNextNode()){
                 if(aux->getFirstEdge() != nullptr){
                for(Edge* aux2 = aux->getFirstEdge(); aux2 != nullptr; aux2 = aux2->getNextEdge()){
                    cout << endl;
                    cout << aux->getId() <<  " -- "<< aux2->getTargetId() << "|" << aux2->getWeight();
                }
            }
        }
        cout << endl;
    }else{

        cout << "primeiro no eh nulo" << endl;
    }

}
// Destructor
Graph::~Graph()
{

    Node *next_node = this->first_node;

    while (next_node != nullptr)
    {

        next_node->removeAllEdges();
        Node *aux_node = next_node->getNextNode();
        delete next_node;
        next_node = aux_node;
    }
}

// Getters
int Graph::getOrder()
{

    return this->order;
}
int Graph::getNumberEdges()
{

    return this->number_edges;
}
//Function that verifies if the graph is directed
bool Graph::getDirected()
{

    return this->directed;
}
//Function that verifies if the graph is weighted at the edges
bool Graph::getWeightedEdge()
{

    return this->weighted_edge;
}

//Function that verifies if the graph is weighted at the nodes
bool Graph::getWeightedNode()
{

    return this->weighted_node;
}


Node *Graph::getFirstNode()
{

    return this->first_node;
}

Node *Graph::getLastNode()
{

    return this->last_node;
}

// Other methods
/*
    The outdegree attribute of nodes is used as a counter for the number of edges in the graph.
    This allows the correct updating of the numbers of edges in the graph being directed or not.
*/
void Graph::insertNode(int id)
{
    Node* no = new Node(id);
    no->setNextNode(nullptr);
    //no->setAnteriorNo(nullptr);

    if(this->getLastNode() == nullptr){
            this->first_node= no;
            this->last_node = no;
    }else{
        //no->setAnteriorNo(this->ultimo_no);
        this->last_node->setNextNode(no);
        this->last_node = no;
    }
    this->order++;
    //this->numeroNos++;
}

Node *Graph::getNode(int id)
{
   if(this->first_node != nullptr){
        for(Node* aux = this->first_node; aux != nullptr; aux = aux->getNextNode()){
            if(aux->getId() == id){
                return aux;}
        }
    }

    return nullptr;
}

void Graph::insertEdge(int id, int target_id, float weight)
{
    Node* aux = this->getNode(id);
    if(id == target_id){

    }else{

    bool flagErroEntrada = false;
    if(aux == nullptr){
        this->insertNode(id);
        aux = this->getNode(id);
        aux->insertEdge(target_id,weight);
        this->number_edges++;
    }else{
         if(aux->searchEdge(target_id)){
            bool flagErroEntrada = true;
         }else{
             Node* aux2 = this->getNode(target_id);
             if(aux2 != nullptr){
                if(aux2->searchEdge(id)){
                }else{
                  aux->insertEdge(target_id,weight);
                  this->number_edges++;
                }
             }else{
              aux->insertEdge(target_id,weight);
              this->number_edges++;
             }
         }
    }

    if(getNode(target_id) == nullptr){
            this->insertNode(target_id);
    }


    }
}

void Graph::removeNode(int id){

}

bool Graph::searchNode(int id)
{

}
//Function that prints a set of edges belongs breadth tree

void Graph::breadthFirstSearch(ofstream &output_file){

}



float Graph::floydMarshall(int idSource, int idTarget){

}



float Graph::dijkstra(int idSource, int idTarget){

}

//function that prints a topological sorting
void topologicalSorting(){

}

void breadthFirstSearch(ofstream& output_file){

}
Graph* getVertexInduced(int* listIdNodes){

}

Graph* agmKuskal(){

}
Graph* agmPrim(){

}
