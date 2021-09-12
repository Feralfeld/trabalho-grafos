#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <utility>
#include <tuple>
#include <iomanip>
#include <stdlib.h>
#include <chrono>
#include "./include/Graph.h"
#include "./include/Node.h"

using namespace std;


Graph* leituraInstanciaPAGMG(ifstream& input_file, int lineCount){

    int idNodeSource;
    int idNodeTarget;
    int pesoAresta;
    int grupo;
    int order;
    bool directed = false;
    bool weightedEdge = true;
    bool weightedNode = false;

    //Criando objeto grafo
    Graph* graph = new Graph(order, directed, weightedEdge, weightedNode);
    int totalCountar = 0;
    int noCounter = 0;
    while(noCounter < lineCount && input_file >> grupo) {
        graph->insertNodeGroup(noCounter,grupo);
        noCounter++;
        totalCountar++;
    }

    //Leitura de arquivo
    while(input_file >> idNodeSource >> idNodeTarget >> pesoAresta) {
        graph->insertEdge(idNodeSource, idNodeTarget, pesoAresta);
        totalCountar++;
    }

    //cout << "QUANTIDADE DE GRUPOS" << graph->getQuantidadeGrupos() << endl;
    return graph;
}




Graph* leitura(ifstream& input_file, int directed, int weightedEdge, int weightedNode){

    //Variáveis para auxiliar na criação dos nós no Grafo
    int idNodeSource;
    int idNodeTarget;
    int order;

    //Pegando a ordem do grafo
    input_file >> order;

    //Criando objeto grafo
    Graph* graph = new Graph(order, directed, weightedEdge, weightedNode);

    //Leitura de arquivo

    if(!graph->getWeightedEdge() && !graph->getWeightedNode()){

        while(input_file >> idNodeSource >> idNodeTarget) {

            graph->insertEdge(idNodeSource, idNodeTarget, 0);

        }

    }else if(graph->getWeightedEdge() && !graph->getWeightedNode() ){

        float edgeWeight;

        while(input_file >> idNodeSource >> idNodeTarget >> edgeWeight) {

            graph->insertEdge(idNodeSource, idNodeTarget, edgeWeight);

        }

    }else if(graph->getWeightedNode() && !graph->getWeightedEdge()){

        float nodeSourceWeight, nodeTargetWeight;

        while(input_file >> idNodeSource >> nodeSourceWeight >> idNodeTarget >> nodeTargetWeight) {

            graph->insertEdge(idNodeSource, idNodeTarget, 0);
            graph->getNode(idNodeSource)->setWeight(nodeSourceWeight);
            graph->getNode(idNodeTarget)->setWeight(nodeTargetWeight);

        }

    }else if(graph->getWeightedNode() && graph->getWeightedEdge()){

        float nodeSourceWeight, nodeTargetWeight, edgeWeight;

        while(input_file >> idNodeSource >> nodeSourceWeight >> idNodeTarget >> nodeTargetWeight) {

            graph->insertEdge(idNodeSource, idNodeTarget, edgeWeight);
            graph->getNode(idNodeSource)->setWeight(nodeSourceWeight);
            graph->getNode(idNodeTarget)->setWeight(nodeTargetWeight);

        }

    }

    return graph;
}

Graph* leituraInstancia(ifstream& input_file, int directed, int weightedEdge, int weightedNode){

    //Vari�veis para auxiliar na criação dos nós no Grafo
    int idNodeSource;
    int idNodeTarget;
    int order;
    int numEdges;

    //Pegando a ordem do grafo
    input_file >> order >> numEdges;

    //Criando objeto grafo
    Graph* graph = new Graph(order, directed, weightedEdge, weightedNode);

    cout << "Lendo instancia" << endl;
    //Leitura de arquivo
    while(input_file >> idNodeSource >> idNodeTarget) {

        graph->insertEdge(idNodeSource, idNodeTarget, 0);

    }
    cout << "Lendo instancia OK" << endl;
    return graph;
}

int menu(){

    int selecao;

    cout << "MENU" << endl;
    cout << "----" << endl;
    cout << "[1] Fecho transitivo direto de um vertice." << endl;
    cout << "[2] Fecho transitivo indireto de um vertice." << endl;
    cout << "[3] Caminho Mínimo entre dois vertices - Dijkstra" << endl;
    cout << "[4] Caminho Mínimo entre dois vertices - Floyd" << endl;
    cout << "[5] Arvore Geradora Minima de Prim" << endl;
    cout << "[6] Arvore Geradora Minima de Kruskal" << endl;
    cout << "[7] Imprimir caminhamento em profundidade" << endl;
    cout << "[8] Imprimir ordenacao topologica" << endl;
    cout << "[9] Algoritmo Guloso (PAGMG)" << endl;
    cout << "[10] Algoritmo Guloso Randomizado (PAGMG)" << endl;
    cout << "[11] Algoritmo Guloso Randomizado Reativo (PAGMG)" << endl;
    cout << "[12] Imprimir informacoes do grafo" << endl;

    cin >> selecao;

    return selecao;

}

void selecionar(int selecao, Graph* graph, ofstream& output_file){

    switch (selecao) {

            //Fecho transitivo direto de um vértice;
        case 1:{
            int idNo;
            cout << "Digite o primeiro vertice" << endl;
            cin >> idNo;

            graph->fechoTransitivoDireto(idNo);
            break;
        }

            //Fecho transitivo indireto de um vértice;
        case 2:{
        int idNo;
            cout << "Digite o primeiro vertice" << endl;
            cin >> idNo;

            graph->fechoTransitivoIndireto(idNo);
            break;
        }

            //Caminho mínimo entre dois vértices usando Dijkstra;
        case 3:{
            int primeiro, segundo;
            cout << "Digite o primeiro vertice" << endl;
            cin >> primeiro;
            cout << "Digite o segundo vertice" << endl;
            cin >> segundo;


            cout << "distancia entre os vertices " <<  graph->dijkstra(primeiro,segundo,output_file) << endl;
            break;

        }

            //Caminho mínimo entre dois vértices usando Floyd;
        case 4:{

            int primeiro, segundo;
            cout << "Digite o primeiro vertice" << endl;
            cin >> primeiro;
            cout << "Digite o segundo vertice" << endl;
            cin >> segundo;


            cout << "Distancia entre os vertices " <<  graph->floydMarshall(primeiro,segundo,output_file) << endl;
            break;
            break;
        }

            //AGM Prim;
        case 5:{
            cout << "Arvore Geradora Minima de Prim" << endl;
            graph->agmPrim(output_file);
            break;
        }

            //AGM - Kruscal;
        case 6:{
       cout << "Árvore Geradora Minima de Kruskal" << endl;
            graph->agmKuskal(output_file);
            break;
        }
            //Busca em Profundidade;
        case 7:{
            int idNo;
            cout << "Digite o ID do vertice" << endl;
            cin >> idNo;

            graph->buscaEmProfundidade(idNo);
            break;
        }
            //Ordenação Topologica;
        case 8:{
                cout << "PRINTANDO ORDENACAO TOPOLOGICA" << endl;
                if(graph->getDirected() == 0){
                    cout << "O grafo deve ser direcionado!" << endl;
                } else {
                    graph->ordenacaoTopologica();
                }
            break;
        }

        // Algoritmo guloso;
        case 9:{
            if(graph->getDirected() == 1){
                cout << "O grafo nao pode ser direcionado...";
            } else {
                cout << "PRINTANDO ALGORITMO GULOSO" << endl;
                graph->guloso(output_file);
            }
            break;
        }

        // Algoritmo guloso randomizado;
        case 10:{
            if(graph->getDirected() == 1){
                cout << "O grafo nao pode ser direcionado...";
            } else {
                cout << "PRINTANDO ALGORITMO GULOSO RANDOMIZADO" << endl;
                graph->gulosoRandomizado(output_file,1000);
            }
             break;
        }

        // Algoritmo guloso randomizado reativo;
        case 11:{
            if(graph->getDirected() == 1){
                cout << "O grafo nao pode ser direcionado...";
            } else {
                cout << "PRINTANDO ALGORITMO GULOSO RANDOMIZADO REATIVO" << endl;
                float a[5] = {0.6, 0.7, 0.8, 0.9, 1};
                graph->gulosoRandomizadoReativo(output_file,a,2500,10);
            }
             break;
        }

        case 12:{
                cout << "PRINTANDO informacoes" << endl;
                graph->printarGrafoGraphviz();
                graph->printarInfos();
                graph->printarGrafo();
            break;
        }

        default:
        {
            cout << " Error!!! invalid option!!" << endl;
        }

    }
}

int mainMenu(ofstream& output_file, Graph* graph){

    int selecao = 1;

    while(selecao != 0){
         //system("cls");//system("clear");
        selecao = menu();

        if(output_file.is_open())
            selecionar(selecao, graph, output_file);

        else
            cout << "Unable to open the output_file" << endl;

        output_file << endl;

    }

    return 0;
}



int main(int argc, char const *argv[]) {

    //Verificação se todos os parâmetros do programa foram entrados

      for(int valor =0 ; valor < argc ; valor++ ){
        cout << "Valor args [" << valor<<"] = " << argv[valor] << endl;
      }

    if (argc != 6) {

        cout << "ERROR: Expecting: ./<program_name> <input_file> <output_file> <directed> <weighted_edge> <weighted_node> " << endl;
        return 1;

    }

    string program_name(argv[0]);
    string input_file_name(argv[1]);

    //string instance;

   // if(input_file_name.find("v") <= input_file_name.size()){
        string instance = input_file_name.substr(0,input_file_name.find("."));
        cout << "Running " << program_name << " with instance " << instance << " ... " << endl;
   // }


    int auxGrupos = 0;
    int auxNodes = 0;
    char ch;
    bool flag = false;
    bool flag2 = false;
    string cstring = "0";
    string cstringaux = "0";
    //string teste =   input_file_name.split(".")[0];

    int l = instance.length();


    // Analisa fileName para saber se é instancia PAGGM
    for (int i = 0; i < l; i++) {
        ch = instance.at(i);
        if(flag){
         if(isdigit(ch)){
            if(flag2){
        cstring = cstring + ch;
            }else{
        cstringaux = cstringaux + ch;
            }
        }else{
        flag2 = true;
        }
        }else{
            if(ch == '/'){
                flag = true;
            }
        }
    }

    auxGrupos = stoi(cstringaux);
    auxNodes = stoi(cstring);

    cout << "printando grupos " << auxGrupos << endl;
    cout << "printando nodes " << auxNodes << endl;


    //Abrindo arquivo de entrada
    ifstream input_file;
    ofstream output_file;
    input_file.open(argv[1], ios::in);
    output_file.open(argv[2], ios::out | ios::trunc);



    Graph* graph;

    if(input_file.is_open()){

        if(auxGrupos < 1){
         cout << "Leitura grafo normal" << endl;
        graph = leitura(input_file, atoi(argv[3]), atoi(argv[4]), atoi(argv[5]));
        }else{
         cout << "Leitura grafo PAGGM" << endl;
         graph = leituraInstanciaPAGMG(input_file, auxNodes);
        }
    }else
        cout << "Unable to open " << argv[1];

    mainMenu(output_file, graph);



    //Fechando arquivo de entrada
    input_file.close();

    //Fechando arquivo de saída
    output_file.close();

    return 0;
}

