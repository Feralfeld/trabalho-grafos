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
#include <stack>
#define INFINITO 99999999


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

void Graph::printarInfos(){

    cout << "Ordem do grafo " << order << endl;
    cout << "Numero de arestas " << number_edges << endl;
    cout << "Direcionado " << directed << endl;
    cout << "Peso nas arestas " << weighted_edge<< endl;
    cout << "Peso nos nos " << weighted_node << endl;

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
    //this->order++;
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


void Graph::buscaEmProfundidade(int v){
    this->printarGrafo();
    int flag = 1;

    if(this->getNode(0) != nullptr){
            flag = 0;
    }

    Graph* novo = new Graph(0,this->directed, this->weighted_edge, this->weighted_node);
    novo->insertNode(v);

	stack<int> pilha;
	bool visitados[this->order]; // vetor de visitados

	// marca todos como não visitados
	for(int i = 0; i < this->order; i++)
		visitados[i] = false;

	while(true)
	{

		if(!visitados[v-flag])
		{
			visitados[v-flag] = true; // marca como visitado
			pilha.push(v-flag); // insere "v" na pilha
		}

		bool achou = false;

        int it = 0;

        // busca por um vizinho não visitado
		Node* no = this->getNode(v);


		for(Edge* aresta = no->getFirstEdge(); aresta != nullptr; aresta = aresta->getNextEdge()){


            if(!visitados[aresta->getTargetId()-flag]){
                achou = true;
                it = aresta->getTargetId();
                novo->insertEdge(v, aresta->getTargetId(), aresta->getWeight());
				break;
            }
		}

		if(achou){
			v = it; // atualiza o "v"
		} else {
      	// se todos os vizinhos estão visitados ou não existem vizinhos
			// remove da pilha
			pilha.pop();
     		// se a pilha ficar vazia, então terminou a busca

			if(pilha.empty()){
     			break;
			}

			// se chegou aqui, é porque pode pegar elemento do topo
			v = pilha.top();

		}
	}

	novo->printarGrafo();
}


void Graph::removeNode(int id){
if(searchNode(id)){
for(Node* aux = this->first_node; aux != nullptr; aux = aux->getNextNode()){
if(aux->getNextNode()->getId() == id){
Node* aux2 = aux->getNextNode(); // selected id
aux2->removeAllEdges();
aux->setNextNode(aux2->getNextNode());
delete aux2;
}
}
} else {
cout << "Node not found!" << endl;
}
}

bool Graph::searchNode(int id)
{
if(this->first_node != nullptr){
for(Node* aux = this->first_node; aux != nullptr; aux = aux->getNextNode()){
if(aux->getId() == id)
return true;
}
}

return false;
}


void Graph::breadthFirstSearch(ofstream &output_file){

}

void Graph::fechoTransitivoDireto(int idNode){

    Graph* novo = new Graph(0,this->directed, this->weighted_edge, this->weighted_node);
    novo->insertNode(idNode);
    Node* no;
    Edge* aresta;


    for (no = novo->getNode(idNode); no != nullptr; no = no->getNextNode())
    {
        aresta = this->getNode(no->getId())->getFirstEdge();

        while ( aresta != nullptr){

            novo->insertEdge(no->getId(),aresta->getTargetId(), aresta->getWeight());
            aresta = aresta->getNextEdge();
        }
    }

    novo->printarGrafo();
}



void Graph::fechoTransitivoIndireto(int idNode){

    Graph* novo = new Graph(0,this->directed, this->weighted_edge, this->weighted_node);
    novo->insertNode(idNode);
    Node* no;
    Edge* aresta;


    for (no = this->getNode(idNode); no != nullptr; no = no->getNextNode())
    {
        aresta = this->getNode(no->getId())->getFirstEdge();

        while ( aresta != nullptr){

            if(aresta->getTargetId() == no->getId()){

            novo->insertEdge(no->getId(),aresta->getTargetId(), aresta->getWeight());
            aresta = aresta->getNextEdge();
            }
        }
    }

    novo->printarGrafo();
}

float Graph::floydMarshall(int idSource, int idTarget, ofstream& arquivo_saida){
    arquivo_saida << "---------Algoritmo de Floyd Marshall---------" << endl;
    arquivo_saida << "No -- No  |  Distancia" << endl;
    arquivo_saida << "--------------------------" << endl;


    int V = this->order+1;
    int dist[V][V], i, j, k;
    Edge* aresta;
    Node* no;
     int anterior[V][V];

      for (i = 0; i < V; i++)
        {
            for (j = 0; j < V; j++)
            {
                 anterior[i][j] = idSource;
                    if( i == j ){
                        dist[i][j] = 0;
                    }else{
                    dist[i][j] = 9999;
                    }
            }
        }

    for (no = this->first_node; no != nullptr ; no = no->getNextNode())
    {
        aresta = no->getFirstEdge();
        while ( aresta != nullptr){

            dist[no->getId()][aresta->getTargetId()] = aresta->getWeight();
            if(!this->directed){
              dist[aresta->getTargetId()][no->getId()] = aresta->getWeight();
            }
            aresta = aresta->getNextEdge();

        }
    }

    for (k = 0; k < V; k++)
    {
        for (i = 0; i < V; i++)
        {
            for (j = 0; j < V; j++)
            {
                if (dist[i][k] + dist[k][j] < dist[i][j]){
                    dist[i][j] = dist[i][k] + dist[k][j];
                    anterior[i][j] = k;

                }
            }
        }
    }
 arquivo_saida << idSource <<" -- " << idTarget << "  |      " <<dist[idSource][idTarget]<< endl;

      cout << idTarget ;
      arquivo_saida << idTarget;
    int y = idTarget;
    for(int x = idTarget; x != idSource ; x = anterior[idSource][x]){
        cout << " salto-->" << anterior[idSource][x];
        arquivo_saida << " salto-->" << anterior[idSource][x];
    }
    cout <<endl;
    arquivo_saida << endl;
    arquivo_saida <<  "Distancia entre os vertices " << dist[idSource][idTarget] << endl;
    arquivo_saida << "--------------------------------------------------------------------------------------------------------" << endl;

return dist[idSource][idTarget];

}


void Graph::bubbleSort()
{


}


void Graph::swapNos(Node* primeiro, Node* segundo)
{


}

float Graph::dijkstra(int idSource, int idTarget, ofstream& arquivo_saida){

    arquivo_saida << "---------Algoritmo de Dijkstra---------" << endl;
    arquivo_saida << "No -- No  |  Distancia" << endl;
    arquivo_saida << "--------------------------" << endl;

        int V = this->order+1;
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
    Node* no;
    Edge* aresta;
  for (no = this->first_node ; no != nullptr ; no = no->getNextNode())
    {
        aresta = no->getFirstEdge();
        while ( aresta != nullptr){

            graph[no->getId()][aresta->getTargetId()] = aresta->getWeight();
            if(!this->directed){
              graph[aresta->getTargetId()][no->getId()] = aresta->getWeight();
            }
            aresta = aresta->getNextEdge();
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


int Graph::minDistance(int dist[], bool sptSet[])
{     int V = this->order+1;
    int min = 999, min_index;

    for (int v = 0; v < V; v++)
        if (sptSet[v] == false && dist[v] <= min)
            min = dist[v], min_index = v;

    return min_index;
}

//function that prints a topological sorting
void topologicalSorting(){

}

void breadthFirstSearch(ofstream& output_file){

}

Graph* getVertexInduced(int* listIdNodes){

}

class ArestaSimples
{
public:
    int origem;
    int destino;
    int peso;
};

Graph* Graph::agmKuskal(ofstream &arquivo_saida){

    int i = 0, quantNos = 0; int quant_aresta = this-> number_edges;

    ArestaSimples* listaAresta = new ArestaSimples[this->number_edges];

      for(Node* aux = this->getFirstNode(); aux != nullptr; aux = aux->getNextNode()){
                for(Edge* aux2 = aux->getFirstEdge(); aux2 != nullptr; aux2 = aux2->getNextEdge()){
                listaAresta[i].origem = aux->getId();
                listaAresta[i].destino = aux2->getTargetId();
                listaAresta[i].peso = aux2->getWeight();
                i++;
                 }
        }

   ArestaSimples aux;
    for (int j = 0; j < quant_aresta; j++)
    {
        for (int k = j + 1; k < quant_aresta; k++)
        {
            if (listaAresta[j].peso > listaAresta[k].peso)
            {
                aux = listaAresta[j];
                listaAresta[j] = listaAresta[k];
                listaAresta[k] = aux;
            }
        }
    }

    ArestaSimples *listaArestasSolucao = new ArestaSimples[quant_aresta];

    int quantArestasSolucao = 0;
    int atual = 0;
    int arvore[quant_aresta];

    for (int i = 0; i < quant_aresta; i++)
    {
        arvore[i] = i;
    }

    for (int i = 0; i < quant_aresta; i++)
    {
        if (arvore[listaAresta[i].origem] != arvore[listaAresta[i].destino])
        {
            listaArestasSolucao[atual] = listaAresta[i];
            atual++;
            quantArestasSolucao++;
            int verticeAntigo = arvore[listaAresta[i].origem];
            int novoVertice = arvore[listaAresta[i].destino];
            for (int j = 0; j < quant_aresta; j++)
            {
                if (arvore[j] == verticeAntigo)
                    arvore[j] = novoVertice;
            }
        }
    }

    unsigned long long int somatorioPesos = 0;
    arquivo_saida << "---------AGM KRUSKAL---------" << endl;
    arquivo_saida << "[No_Origem -- No_Destino] - Peso" << endl;
    arquivo_saida << "-----------------------------" << endl;
    for (int l = 0; l < quantArestasSolucao; l++)
    {
        if (true)
        {arquivo_saida << "[" << listaArestasSolucao[l].origem << " -> " << listaArestasSolucao[l].destino << "] - " << listaArestasSolucao[l].peso << endl;
            cout << "[" << listaArestasSolucao[l].origem << " -> " << listaArestasSolucao[l].destino << "] - " << listaArestasSolucao[l].peso << endl;

        }
        else
        {if (listaArestasSolucao[l].origem == 0)
            {
            arquivo_saida << "[" << this->order << " -> " << listaArestasSolucao[l].destino << "] - " << listaArestasSolucao[l].peso << endl;
            cout  << "[" << this->order << " -> " << listaArestasSolucao[l].destino << "] - " << listaArestasSolucao[l].peso << endl;
            }else if (listaArestasSolucao[l].destino == 0)
            {
                arquivo_saida << "[" << listaArestasSolucao[l].origem << " -> " << this->order << "] - " << listaArestasSolucao[l].peso << endl;
                cout  << "[" << listaArestasSolucao[l].origem << " -> " << this->order << "] - " << listaArestasSolucao[l].peso << endl;
              }else{
                arquivo_saida << "[" << listaArestasSolucao[l].origem << " -> " << listaArestasSolucao[l].destino << "] - " << listaArestasSolucao[l].peso << endl;
                cout << "[" << listaArestasSolucao[l].origem << " -> " << listaArestasSolucao[l].destino << "] - " << listaArestasSolucao[l].peso << endl;
                }
        }
        somatorioPesos += listaArestasSolucao[l].peso;
    }
    arquivo_saida << "Somatorio dos Pesos: " << somatorioPesos << endl;
    arquivo_saida << "Quantidade de arestas: " << quantArestasSolucao << endl;
    arquivo_saida << "--------------------------------------------------------------------------------------------------------" << endl
                  << endl;
    cout << "Somatorio dos Pesos: " << somatorioPesos << endl;
    cout << "Quantidade de arestas: " << quantArestasSolucao << endl;
    delete[] listaAresta;
    delete[] listaArestasSolucao;


}

Graph* Graph::agmPrim(ofstream& arquivo_saida){


    arquivo_saida << "---------Algoritmo de Prim---------" << endl;
    arquivo_saida << "No -- No  |  Peso Aresta" << endl;
    arquivo_saida << "--------------------------" << endl;

    Graph* arvore = new Graph(this->order,0,1,0);


    int **matrizDeAdjacencias;
    int *menoresCustos = new int[this->order];
    int *maisProximo = new int[this->order];
    bool *visitados = new bool[this->order];

    matrizDeAdjacencias = new int *[this->order];

    for (int i = 0; i < this->order; i++)
    {
        matrizDeAdjacencias[i] = new int[this->order];
        for (int j = 0; j < this->order; j++)
        {
            matrizDeAdjacencias[i][j] = INFINITO;
        }
    }

    int bug = 0;
    int bugReverso = 0;
    if(this->first_node->getId() == 0){
            bugReverso = 1;
    }else{
            bug = 1;
    }


          for(Node* aux = this->first_node; aux != nullptr; aux = aux->getNextNode()){
                 if(aux->getFirstEdge() != nullptr){
                for(Edge* aux2 = aux->getFirstEdge(); aux2 != nullptr; aux2 = aux2->getNextEdge()){
                   matrizDeAdjacencias[aux->getId()-bug][aux2->getTargetId()-bug] = aux2->getWeight();
                   matrizDeAdjacencias[aux2->getTargetId()-bug][aux->getId()-bug] = aux2->getWeight();
                }
            }
        }

    visitados[0] = true;
    for (int i = 1; i < this->order; i++)
    {
        menoresCustos[i] = matrizDeAdjacencias[0][i];
        maisProximo[i] = 0;
        visitados[i] = false;
    }

    int peso = 0;
    int atualArestas = 0;
    int verticeAnterior = 0;

    for (int i = 0; i < this->order; i++)
    {
        int min = INFINITO;
        int indice = 1;

        for (int j = 0; j < this->order; j++)
        {
            if (menoresCustos[j] < min && !visitados[j])
            {   min = menoresCustos[j];
                indice = j;
            }
        }

        if (indice == 1 && min == INFINITO)
        {break;
        }
        else
        {peso += min;
        }

        if (false)
            {
                arquivo_saida << verticeAnterior-bugReverso << " -- " << indice-bugReverso << " | " << min << endl;
                   cout << verticeAnterior-bugReverso << " -- " << indice-bugReverso << " | " << min << endl;
        }
        else
        {if (verticeAnterior == 0)
            {
                arquivo_saida << this->order-bugReverso << " -- " << indice-bugReverso << " | " << min << endl;
                cout << this->order-bugReverso << " -- " << indice-bugReverso << " | " << min << endl;
            }
            else if (indice == 0)
            {
                arquivo_saida << verticeAnterior-bugReverso << " -- " << this->order << " | " << min << endl;
                   cout << verticeAnterior-bugReverso << " -- " << this->order << " | " << min << endl;
            }else{
                arquivo_saida << verticeAnterior-bugReverso << " -- " << indice-bugReverso << " | " << min << endl;
                cout << verticeAnterior-bugReverso << " -- " << indice-bugReverso << " | " << min << endl;
            }
        }

        verticeAnterior = indice;
        visitados[indice] = true;
        atualArestas++;
        for (int j = 1; j < this->order; j++)
        {
            if ((matrizDeAdjacencias[indice][j] < menoresCustos[j]) && (!visitados[j]))
            {
                menoresCustos[j] = matrizDeAdjacencias[indice][j];
                maisProximo[j] = indice;
            }
        }
    }

    arquivo_saida << "Somatorio dos Pesos: " << peso << endl;
    arquivo_saida << "Quantidade de arestas: " << atualArestas << endl;
    arquivo_saida << "--------------------------------------------------------------------------------------------------------" << endl
                  << endl;

    cout << "Somatorio dos Pesos: " << peso << endl;
    cout << "Quantidade de arestas: " << atualArestas << endl;

    for (int i = 0; i < this->order; i++)
    {
        delete[] matrizDeAdjacencias[i];
    }
    delete[] matrizDeAdjacencias;
}

void Graph::printarGrafoGraphviz(){

       ofstream output_file;

       output_file.open ("resultado/teste1.dot", ios::out | ios::trunc);

       if(!this->directed){
       output_file << "graph Grafo {";
       if(this->first_node != nullptr){
       if(this->weighted_edge){
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
        cout << "O grafo é direcionado" << endl;
        output_file << "digraph Grafo {";

        if(this->weighted_edge){
        for(Node* aux = this->first_node; aux != nullptr; aux = aux->getNextNode()){
                 if(aux->getFirstEdge() != nullptr){
                for(Edge* aux2 = aux->getFirstEdge(); aux2 != nullptr; aux2 = aux2->getNextEdge()){
                    output_file << endl;
                    output_file << aux->getId() <<  " -> "<< aux2->getTargetId() << "[label="  << aux2->getWeight() << "];" ;
                }
            }
        }
    }else{
         for(Node* aux = this->first_node; aux != nullptr; aux = aux->getNextNode()){
                 if(aux->getFirstEdge() != nullptr){
                for(Edge* aux2 = aux->getFirstEdge(); aux2 != nullptr; aux2 = aux2->getNextEdge()){
                    output_file << endl;
                    output_file << aux->getId() <<  " -> "<< aux2->getTargetId() << ";" ;
                }
            }
        }
    }
    }
     output_file << endl;
    output_file << "}";
    }



void Graph::ordenacaoTopologica(){





}


