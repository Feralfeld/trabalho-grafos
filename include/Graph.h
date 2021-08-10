/**************************************************************************************************
 * Implementation of the TAD Graph
**************************************************************************************************/

#ifndef GRAPH_H_INCLUDED
#define GRAPH_H_INCLUDED
#include "Node.h"
#include <fstream>
#include <stack>
#include <list>

using namespace std;

class Graph{

    //Atributes
    private:
        int order;
        int number_edges;
        bool directed;
        bool weighted_edge;
        bool weighted_node;
        Node* first_node;
        Node* last_node;

    public:
        //Constructor
        Graph(int order, bool directed, bool weighted_edge, bool weighted_node);
        //Destructor
        ~Graph();
        //Getters
        int getOrder();
        int getNumberEdges();
        bool getDirected();
        bool getWeightedEdge();
        bool getWeightedNode();
        Node* getFirstNode();
        Node* getLastNode();
        //Other methods
        void insertNode(int id);
        void insertEdge(int id, int target_id, float weight);
        void removeNode(int id);
        bool searchNode(int id);
        Node* getNode(int id);
        void bubbleSort();
        void swapNos(Node* primeiro, Node* segundo);

        //methods phase1
        void topologicalSorting();
        void fechoTransitivoDireto(int idNode);
        void fechoTransitivoIndireto(int idNode);
        void breadthFirstSearch(ofstream& output_file);
        Graph* getVertexInduced(int* listIdNodes);
        Graph* agmKuskal(ofstream &arquivo_saida);
        Graph* agmPrim(ofstream& arquivo_saida);
        float floydMarshall(int idSource, int idTarget, ofstream& arquivo_saida);
        float floydMarshallNoOutPut(int idSource, int idTarget);
        float dijkstra(int idSource, int idTarget, ofstream& arquivo_saida);
        void printarGrafo();
        void printarInfos();
        void buscaEmProfundidade(int idNode);
        void printarGrafoGraphviz();
        void ordenacaoTopologica();


        //methods phase1
        float greed();
        float greedRandom();
        float greedRactiveRandom();


        int minDistance(int dist[], bool sptSet[]);
    private:
        //Auxiliar methods

};

#endif // GRAPH_H_INCLUDED
