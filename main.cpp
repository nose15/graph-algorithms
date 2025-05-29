#include <cstdio>
#include <memory>
#include <cstring>
#include <filesystem>
#include <vector>
#include <list>

struct Edge {
  int target;
  int weight;
};

class AdjList {
 private:
  std::list<Edge> * adj_list;
 public:
  AdjList() {
    adj_list = new std::list<Edge>();
  }
  ~AdjList() {
    delete adj_list;
  }

  void add(int target, int weight) {
    adj_list->push_back({target, weight});
  }

  std::list<Edge>::iterator get(int index) {
    for (auto it = adj_list->begin(); it != adj_list->end(); it++) {
      if (index == 0) {
        return it;
      }

      index--;
    }

    throw std::range_error("Index out of range");
  }
};

struct Graph {
  std::vector<std::vector<int>> adj_matrix;

  // each vertex has a list that stores its edges
  // each edge is made out of target vertex and weight
  std::vector<AdjList> adj_list;
};

struct Paths {
  // TODO: Define structure for path finding results
};

Graph prim_list(std::vector<AdjList> adj_lists) {
  // TODO: Implement Prim for list
}

Graph prim_matrix(std::vector<std::vector<int>> adj_matrix) {
  // TODO: Implement Prim for matrix
}

Graph kruskal_list(std::vector<AdjList> adj_lists) {
  // TODO: Implement Kruskal for list
}

Graph kruskal_matrix(std::vector<std::vector<int>> adj_matrix) {
  // TODO: Implement Kruskal for matrix
}

Paths dijkstra_list(std::vector<AdjList> adj_lists, int starting_vertex) {
  // TODO: Implement Dijkstra's for list
}

Paths dijkstra_matrix(std::vector<std::vector<int>> adj_matrix, int starting_vertex) {
  // TODO: Implement Dijkstra's for matrix
}

Paths bellman_ford_list(std::vector<AdjList> adj_lists, int starting_vertex) {
  // TODO: Implement Bellman-Ford for list
}

Paths bellman_ford_matrix(std::vector<std::vector<int>> adj_matrix, int starting_vertex) {
  // TODO: Implement Bellman-Ford for matrix
}

void run_bellman_ford(std::shared_ptr<Graph> graph, int starting_vertex) {
  printf("Running Bellman Ford for starting vertex %d\n", starting_vertex);

  Paths paths;

  printf("Adjacency matrix:\n");
  // TODO: Start time measure
  paths = bellman_ford_matrix(graph->adj_matrix, starting_vertex);
  // TODO: Stop time measure
  // TODO: Display time and results

  printf("Adjacency list:\n");
  // TODO: Start time measure
  paths = bellman_ford_list(graph->adj_list, starting_vertex);
  // TODO: Stop time measure
  // TODO: Display time and results
}

void run_dijkstra(std::shared_ptr<Graph> graph, int starting_vertex) {
  printf("Running Dijkstra for starting vertex %d\n", starting_vertex);

  Paths paths;

  printf("Adjacency matrix:\n");
  // TODO: Start time measure
  paths = dijkstra_matrix(graph->adj_matrix, starting_vertex);
  // TODO: Stop time measure
  // TODO: Display time and results

  printf("Adjacency list:\n");
  // TODO: Start time measure
  paths = dijkstra_list(graph->adj_list, starting_vertex);
  // TODO: Stop time measure
  // TODO: Display time and results
}

void run_kruskal(std::shared_ptr<Graph> graph) {
  printf("Running Kruskal\n");

  Graph mst;

  printf("Adjacency matrix:\n");
  // TODO: Start time measure
  mst = kruskal_matrix(graph->adj_matrix);
  // TODO: Stop time measure
  // TODO: Display time and results

  printf("Adjacency list:\n");
  // TODO: Start time measure
  mst = kruskal_list(graph->adj_list);
  // TODO: Stop time measure
  // TODO: Display time and results
}

void run_prim(std::shared_ptr<Graph> graph) {
  printf("Running Prim\n");

  Graph mst;

  printf("Adjacency matrix:\n");
  // TODO: Start time measure
  mst = prim_matrix(graph->adj_matrix);
  // TODO: Stop time measure
  // TODO: Display time and results

  printf("Adjacency list:\n");
  // TODO: Start time measure
  mst = prim_list(graph->adj_list);
  // TODO: Stop time measure
  // TODO: Display time and results
}

void display_graph(std::shared_ptr<Graph> graph) {
  printf("Displaying graph\n");

  // TODO: Implement graph display for each method
}

std::string file_path_dialog() {
  char file_name[PATH_MAX];
  std::string file_path;

  while (true) {
    printf("Podaj sciezke do pliku: ");
    scanf("%s", file_name);
    file_path = std::string(__FILE__).substr(0, strlen(__FILE__) - strlen("main.cpp")) + std::string(file_name);

    if (std::filesystem::exists(file_path) && !std::filesystem::is_directory(file_path)) {
      break;
    }

    printf("Niepoprawna sciezka pliku.\n");
  }

  return file_path;
}

Graph generate_graph(int vertex_count, int density) {
  printf("Generating graph\n");

  // TODO: Implement graph generation

  return Graph();
}

int read_path_from_file(std::shared_ptr<Graph>& graph, const std::string& file_path) {
  graph = std::make_shared<Graph>();
  printf("Reading path graph from file %s\n", file_path.c_str());

  // TODO: Implement graph construction from file and reading the starting vertex

  return 1;
}

void read_mst_from_file(std::shared_ptr<Graph>& graph, const std::string& file_path) {
  graph = std::make_shared<Graph>();
  printf("Reading mst graph from file %s\n", file_path.c_str());

  // TODO: Implement graph construction from file
}

int path_gen_dialog(std::shared_ptr<Graph> graph) {
  int vertex_count = -1;
  int density = -1;
  int starting_vertex = -1;

  while (true) {
    printf("Podaj liczbe wierzcholkow: ");
    scanf("%d", &vertex_count);

    if (vertex_count > 0) {
      break;
    }

    printf("Niepoprawna liczba wierzcholkow - powinna byc wieksza od 0\n");
  }

  while (true) {
    printf("Podaj gestosc: ");
    scanf("%d", &density);

    if (density > 0 && density < 100) {
      break;
    }

    printf("Niepoprawna gestosc - powinna byc z przedzialu <0;100>\n");
  }

  while (true) {
    printf("Podaj poczatkowy wierzcholek: ");
    scanf("%d", &starting_vertex);

    if (starting_vertex >= 0 && starting_vertex < vertex_count) {
      break;
    }

    printf("Niepoprawny wierzcholek startowy - powinien byc z przedzialu <0;%d>\n", vertex_count - 1);
  }

  *graph = generate_graph(vertex_count, density);
  return starting_vertex;
}

int path_file_dialog(std::shared_ptr<Graph>& graph) {
  std::string file_path = file_path_dialog();
  int starting_vertex = read_path_from_file(graph, file_path);
  return starting_vertex;
}

void mst_gen_dialog(std::shared_ptr<Graph> graph) {
  int vertex_count = -1;
  int density = -1;

  while (true) {
    printf("Podaj liczbe wierzcholkow: ");
    scanf("%d", &vertex_count);

    if (vertex_count > 0) {
      break;
    }

    printf("Niepoprawna liczba wierzcholkow - powinna byc wieksza od 0\n");
  }


  while (true) {
    printf("Podaj gestosc: ");
    scanf("%d", &density);

    if (density > 0 && density < 100) {
      break;
    }

    printf("Niepoprawna gestosc - powinna byc z przedzialu <0;100>\n");
  }

  *graph = generate_graph(vertex_count, density);
}

void mst_file_dialog(std::shared_ptr<Graph>& graph) {
  std::string file_path = file_path_dialog();
  read_mst_from_file(graph, file_path);
}

void mst_menu() {
  std::shared_ptr<Graph> graph;

  int choice;
  while (true) {
    printf("Wybierz opcje:\n"
           "1. Wczytaj z pliku\n"
           "2. Wygeneruj graf losowo\n"
           "3. Wyświetl reprezentacje grafu (lista i macierz)\n"
           "4. Algorytm Prima\n"
           "5. Algorytm Kruskala\n"
           "0. Powrot do glownego menu\n"
           "> ");
    scanf("%d", &choice);

    switch (choice) {
      case 1:
        mst_file_dialog(graph);
        break;
      case 2:
        mst_gen_dialog(graph);
        break;
      case 3:
        if (graph) display_graph(graph);
        else printf("Brak grafu, najpierw wczytaj graf (1.)\n");
        break;
      case 4:
        if (graph) run_prim(graph);
        else printf("Brak grafu, najpierw wczytaj graf (1.)\n");
        break;
      case 5:
        if (graph) run_kruskal(graph);
        else printf("Brak grafu, najpierw wczytaj graf (1.)\n");
        break;
      case 0:
        printf("Wracanie do glownego menu\n");
        break;
      default:
        printf("Niepoprawny wybór, spróbuj ponownie.\n");
        break;
    }

    if (choice == 0) {
      break;
    }
  }
}

void path_menu() {
  int starting_vertex;
  std::shared_ptr<Graph> graph;

  int choice;
  while (true) {
    printf("Wybierz opcje:\n"
           "1. Wczytaj z pliku\n"
           "2. Wygeneruj graf losowo\n"
           "3. Wyświetl reprezentacje grafu (lista i macierz)\n"
           "4. Algorytm Dijkstry\n"
           "5. Algorytm Bellmanna-Forda\n"
           "0. Powrot do glownego menu\n"
           "> ");
    scanf("%d", &choice);

    switch (choice) {
      case 1:
        starting_vertex = path_file_dialog(graph);
        break;
      case 2:
        starting_vertex = path_gen_dialog(graph);
        break;
      case 3:
        if (graph) display_graph(graph);
        else printf("Brak grafu, najpierw wczytaj graf (1.)\n");
        break;
      case 4:
        if (graph) run_dijkstra(graph, starting_vertex);
        else printf("Brak grafu, najpierw wczytaj graf (1.)\n");
        break;
      case 5:
        if (graph) run_bellman_ford(graph, starting_vertex);
        else printf("Brak grafu, najpierw wczytaj graf (1.)\n");
        break;
      case 0:
        printf("Wracanie do glownego menu\n");
        break;
      default:
        printf("Niepoprawny wybór, spróbuj ponownie.\n");
        break;
    }

    if (choice == 0) {
      break;
    }
  }
}

void main_menu() {
  int choice;

  while (true) {
    printf("Wybierz problem:\n"
           "1. Wyznaczenie minimalnego drzewa rozpinającego (MST)\n"
           "2. Wyznaczenie najkrótszych ścieżek w grafie\n"
           "> ");
    scanf("%d", &choice);

    switch (choice) {
      case 1:
        mst_menu();
        break;
      case 2:
        path_menu();
        break;
      case 0:
        printf("Zamykanie...\n");
      default: {
        printf("Niepoprawny wybór, spróbuj ponownie.\n");
        break;
      }

    }

    if (choice == 0) {
      break;
    }
  }
}

int main() {
  main_menu();
  return 0;
}
