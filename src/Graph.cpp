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


void Graph::bubbleSort()
{
   int i, j;

   bool swapped;
   Node* primeiro = this->first_node;
   Node* aux = this->first_node;

   for (i = 0; i < this->order; i++){
     swapped = false;

     primeiro = this->first_node;
     for (j = 0; j < this->order; j++)
     {
        if(primeiro->getNextNode() != nullptr){
        if (primeiro->getId() > primeiro->getNextNode()->getId())
        {
           swapNos(primeiro, primeiro->getNextNode());
           swapped = true;
        }
        if(primeiro->getNextNode() != nullptr){
       primeiro = primeiro->getNextNode();
       }
       }
     }

     if (swapped == false){
        break;}

   aux = aux->getNextNode();
   }

}


void Graph::swapNos(Node* primeiro, Node* segundo)
{
    Node* temp = primeiro;
    Node* anterior = primeiro->getgetAnteriorNo();

   if(segundo->getProximoNo() == nullptr && primeiro->getAnteriorNo() == nullptr){
        segundo->setAnteriorNo(nullptr);
        segundo->setProximoNo(primeiro);

        primeiro->setAnteriorNo(segundo);
        primeiro->setProximoNo(nullptr);

        this->ultimo_no = primeiro;
        this->primeiroNo = segundo;
    }else{
    if(segundo->getProximoNo() == nullptr){

    anterior->setProximoNo(segundo);

    segundo->setAnteriorNo(anterior);
    segundo->setProximoNo(primeiro);

    primeiro->setAnteriorNo(segundo);
    primeiro->setProximoNo(nullptr);

    this->ultimo_no = primeiro;

    }else{
       if(primeiro->getAnteriorNo() == nullptr){
        No* proximo = segundo->getProximoNo();

        segundo->setAnteriorNo(nullptr);
        segundo->setProximoNo(primeiro);

        primeiro->setAnteriorNo(segundo);
        primeiro->setProximoNo(proximo);

        proximo->setAnteriorNo(primeiro);

        this->primeiroNo = segundo;

       }else{
    No* proximo = segundo->getProximoNo();

    anterior->setProximoNo(segundo);

    segundo->setAnteriorNo(anterior);
    segundo->setProximoNo(primeiro);

    primeiro->setAnteriorNo(segundo);
    primeiro->setProximoNo(proximo);

    proximo->setAnteriorNo(primeiro);
       }
    }}

}

float Graph::dijkstra(int idSource, int idTarget, ofstream& arquivo_saida){
    arquivo_saida << "---------Algoritmo de Dijkstra---------" << endl;
    arquivo_saida << "No -- No  |  Distancia" << endl;
    arquivo_saida << "--------------------------" << endl;

    this->bubbleSort();
        int V = this->numeroNos+1;
        int dist[V];

    bool sptSet[V];

    for (int i = 0; i < V; i++){
        dist[i] = 999, sptSet[i] = false;
    }

    dist[idSource] = 0;
    int graph[V][V];


      for (int i = 0; i < V; i++)
        {
            for (int j = 0; j < V; j++)
            {
                    if( i == j ){
                        graph[i][j] = 0;
                    }else{
                    graph[i][j] = 999;
                    }
            }
        }
    No* no;
    Aresta* aresta;
  for (no = this->primeiroNo ; no != nullptr ; no = no->getProximoNo())
    {
        aresta = no->getPrimeiraAresta();
        while ( aresta != nullptr){

            graph[no->getId()][aresta->getTargetId()] = aresta->getPeso();
            if(!this->directed){
              graph[aresta->getTargetId()][no->getId()] = aresta->getPeso();
            }
            aresta = aresta->getProximaAresta();
        }
    }

    int saltos[V];
    for (int count = 0; count < V - 1; count++) {

         int u = this->minDistance(dist, sptSet);

        sptSet[u] = true;

         for (int v = 0; v < V; v++){
            if (!sptSet[v] && graph[u][v] && dist[u] != 999
                && dist[u] + graph[u][v] < dist[v]){
                dist[v] = dist[u] + graph[u][v];
                saltos[v] = u;
                }
        }
    }
    arquivo_saida << idSource <<" -- " << idTarget << "  |      " <<dist[idTarget]<< endl;

    cout << idTarget;
    arquivo_saida << idTarget;
    for(int x = idTarget; x != idSource ; x = saltos[x] ){
             cout  << " salto->" <<  saltos[x];
             arquivo_saida <<  " salto->" <<  saltos[x];
    }
    cout << endl;
    arquivo_saida << endl;
    arquivo_saida << "Distancia entre os vertices " << dist[idTarget] << endl;

    arquivo_saida << "--------------------------------------------------------------------------------------------------------" << endl;

    return dist[idTarget];
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

void Graph::printarGrafoGraphviz(ofstream& output_file){
        output_file << "graph Grafo {";

       if(this->directed){
       if(this->first_node != nullptr){
       if(this->weighted_edge{
        for(Node* aux = this->first_node; aux != nullptr; aux = aux->getNextNode()){
                 if(aux->getFirstEdge() != nullptr){
                for(Edge* aux2 = aux->getFirstEdge(); aux2 != nullptr; aux2 = aux2->getNextEdge()){
                    output_file << endl;
                    output_file << aux->getId() <<  " -- "<< aux2->getTargetId() << "[label="  << aux2->getWeight() << "];" ;
                }
            }
        }
    }else{
         for(Node* aux = this->first_node; aux != nullptr; aux = aux->getNextNode()){
                 if(aux->getFirstEdge() != nullptr){
                for(Edge* aux2 = aux->getFirstEdge(); aux2 != nullptr; aux2 = aux2->getNextEdge()){
                    output_file << endl;
                    output_file << aux->getId() <<  " -- "<< aux2->getTargetId() << ";" ;
                }
            }
        }
    }
    }
    output_file << '}' << endl ;
    }else{
        cout << "O grafo não é direcionado" << endl;
    }


    }
