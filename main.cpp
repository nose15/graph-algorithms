#include <cstdio>
#include <memory>
#include <cstring>
#include <filesystem>
#include <vector>
#include <list>
#include <fstream>
#include <iostream>
#include <random>
#include <set>
#include <queue>

struct Edge {
  int target;
  int weight;
};

class AdjList {
 private:
  std::list<Edge> * adj_list;
  int size;
 public:
  AdjList() {
    size = 0;
    adj_list = new std::list<Edge>();
  }
  ~AdjList() {
    adj_list->clear();
    delete adj_list;
  }

  int get_size() const {
    return size;
  }

  void add(int target, int weight) {
    adj_list->push_back({target, weight});
    size++;
  }

  Edge get(int index) const {
    for (auto it = adj_list->begin(); it != adj_list->end(); it++) {
      if (index == 0) {
        return {it->target, it->weight};
      }

      index--;
    }

    throw std::range_error("Index out of range");
  }
};

struct Graph {
  Graph() {
    adj_matrix = new std::vector<std::vector<int>>();
    adj_list = new std::vector<AdjList>();
  }

  std::vector<std::vector<int>> * adj_matrix;

  // each vertex has a list that stores its edges
  // each edge is made out of target vertex and weight
  std::vector<AdjList> * adj_list;
};

struct Path {
  int cost;
  std::vector<int> path;
};

struct DisjointSet {
  std::vector<int> parent, rank;

  DisjointSet(int n) {
    parent.resize(n);
    rank.resize(n, 0);
    for (int i = 0; i < n; ++i)
      parent[i] = i;
  }

  int find(int u) {
    if (parent[u] != u)
      parent[u] = find(parent[u]);
    return parent[u];
  }

  void unite(int u, int v) {
    int ru = find(u), rv = find(v);
    if (ru == rv) return;

    if (rank[ru] < rank[rv])
      parent[ru] = rv;
    else if (rank[ru] > rank[rv])
      parent[rv] = ru;
    else {
      parent[rv] = ru;
      rank[ru]++;
    }
  }
};

Graph prim_list(const std::vector<AdjList>* adj_lists) {
  int V = adj_lists->size();
  std::vector<bool> visited(V, false);
  std::vector<int> key(V, INT_MAX);
  std::vector<int> parent(V, -1);
  key[0] = 0;

  using pii = std::pair<int, int>; // (weight, vertex)
  std::priority_queue<pii, std::vector<pii>, std::greater<>> pq;
  pq.push({0, 0});

  while (!pq.empty()) {
    int u = pq.top().second;
    pq.pop();

    if (visited[u]) continue;
    visited[u] = true;

    const AdjList& neighbors = adj_lists->at(u);
    for (int i = 0; i < neighbors.get_size(); i++) {
      Edge edge = neighbors.get(i);
      int v = edge.target;
      int w = edge.weight;

      if (!visited[v] && w < key[v]) {
        key[v] = w;
        parent[v] = u;
        pq.push({w, v});
      }
    }
  }

  // Build the MST graph result
  Graph mst;
  mst.adj_list = new std::vector<AdjList>(V);

  for (int v = 1; v < V; ++v) {
    int u = parent[v];
    int w = key[v];
    if (u != -1) {
      mst.adj_list->at(u).add(v, w);
      mst.adj_list->at(v).add(u, w); // Assuming undirected MST
    }
  }

  return mst;
}

Graph prim_matrix(const std::vector<std::vector<int>>* adj_matrix) {
  size_t n = adj_matrix->size();
  std::vector<bool> in_mst(n, false);
  std::vector<int> key(n, std::numeric_limits<int>::max());
  std::vector<int> parent(n, -1);

  key[0] = 0;

  for (int count = 0; count < n - 1; ++count) {
    int u = -1;
    int min_key = std::numeric_limits<int>::max();

    for (int v = 0; v < n; ++v) {
      if (!in_mst[v] && key[v] < min_key) {
        min_key = key[v];
        u = v;
      }
    }

    if (u == -1) break;
    in_mst[u] = true;

    for (int v = 0; v < n; ++v) {
      int weight = adj_matrix->at(u)[v];
      if (weight && !in_mst[v] && weight < key[v]) {
        key[v] = weight;
        parent[v] = u;
      }
    }
  }

  auto mst = std::make_shared<Graph>();
  mst->adj_list->resize(n);
  mst->adj_matrix->resize(n, std::vector<int>(n, 0));

  for (int v = 1; v < n; ++v) {
    int u = parent[v];
    int weight = adj_matrix->at(u)[v];

    mst->adj_list->at(u).add(v, weight);
    mst->adj_list->at(v).add(u, weight);

    mst->adj_matrix->at(u)[v] = weight;
    mst->adj_matrix->at(v)[u] = weight;
  }

  return *mst;
}

Graph kruskal_list(const std::vector<AdjList>* adj_lists) {
  int V = adj_lists->size();
  std::vector<std::tuple<int, int, int>> edges; // (weight, u, v)

  std::set<std::pair<int, int>> seen;
  for (int u = 0; u < V; ++u) {
    const AdjList& list = adj_lists->at(u);
    for (int i = 0; i < list.get_size(); ++i) {
      Edge e = list.get(i);
      if (seen.count({e.target, u}) == 0) {
        edges.push_back({e.weight, u, e.target});
        seen.insert({u, e.target});
      }
    }
  }

  std::sort(edges.begin(), edges.end());

  DisjointSet ds(V);
  Graph mst;
  mst.adj_list = new std::vector<AdjList>(V);

  for (const auto& [w, u, v] : edges) {
    if (ds.find(u) != ds.find(v)) {
      ds.unite(u, v);
      mst.adj_list->at(u).add(v, w);
      mst.adj_list->at(v).add(u, w);
    }
  }

  return mst;
}

Graph kruskal_matrix(const std::vector<std::vector<int>>* adj_matrix) {
  int V = adj_matrix->size();
  std::vector<std::tuple<int, int, int>> edges; // (weight, u, v)

  for (int u = 0; u < V; ++u) {
    for (int v = u + 1; v < V; ++v) {
      int w = adj_matrix->at(u)[v];
      if (w > 0)
        edges.push_back({w, u, v});
    }
  }

  std::sort(edges.begin(), edges.end());
  DisjointSet ds(V);
  Graph mst;

  mst.adj_matrix->resize(V, std::vector<int>(V, 0));
  mst.adj_list->resize(V);

  for (const auto& [w, u, v] : edges) {
    if (ds.find(u) != ds.find(v)) {
      ds.unite(u, v);
      mst.adj_matrix->at(u)[v] = w;
      mst.adj_list->at(u).add(v, w);
      mst.adj_matrix->at(v)[u] = w;
      mst.adj_list->at(v).add(u, w);
    }
  }

  return mst;
}

std::vector<Path> dijkstra_list(const std::vector<AdjList>* adj_lists, int starting_vertex) {
  int n = adj_lists->size();
  std::vector<int> dist(n, std::numeric_limits<int>::max());
  std::vector<int> prev(n, -1);
  std::vector<bool> visited(n, false);

  dist[starting_vertex] = 0;

  using P = std::pair<int, int>; // (distance, vertex)
  std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
  pq.push({0, starting_vertex});

  while (!pq.empty()) {
    int u = pq.top().second;
    pq.pop();

    if (visited[u]) continue;
    visited[u] = true;

    const AdjList& neighbors = adj_lists->at(u);
    for (int i = 0; i < neighbors.get_size(); ++i) {
      Edge e = neighbors.get(i);
      int v = e.target;
      int weight = e.weight;
      if (dist[u] + weight < dist[v]) {
        dist[v] = dist[u] + weight;
        prev[v] = u;
        pq.push({dist[v], v});
      }
    }
  }

  std::vector<Path> paths(n);
  for (int v = 0; v < n; ++v) {
    paths[v].cost = dist[v];
    if (dist[v] == std::numeric_limits<int>::max()) continue; // unreachable

    for (int at = v; at != -1; at = prev[at]) {
      paths[v].path.insert(paths[v].path.begin(), at);
    }
  }

  return paths;
}

std::vector<Path> dijkstra_matrix(const std::vector<std::vector<int>>* adj_matrix, int starting_vertex) {
  int n = adj_matrix->size();
  std::vector<int> dist(n, std::numeric_limits<int>::max());
  std::vector<int> prev(n, -1);
  std::vector<bool> visited(n, false);

  dist[starting_vertex] = 0;

  using P = std::pair<int, int>; // (distance, vertex)
  std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
  pq.push({0, starting_vertex});

  while (!pq.empty()) {
    int u = pq.top().second;
    pq.pop();

    if (visited[u]) continue;
    visited[u] = true;

    for (int v = 0; v < n; ++v) {
      int weight = (*adj_matrix)[u][v];
      if (weight > 0 && dist[u] + weight < dist[v]) {
        dist[v] = dist[u] + weight;
        prev[v] = u;
        pq.push({dist[v], v});
      }
    }
  }

  std::vector<Path> paths(n);
  for (int v = 0; v < n; ++v) {
    paths[v].cost = dist[v];
    if (dist[v] == std::numeric_limits<int>::max()) continue; // unreachable

    for (int at = v; at != -1; at = prev[at]) {
      paths[v].path.insert(paths[v].path.begin(), at);
    }
  }

  return paths;
}

std::vector<Path> bellman_ford_list(const std::vector<AdjList>* adj_lists, int starting_vertex) {
  // TODO: Implement Bellman-Ford for list
  return {};
}

std::vector<Path> bellman_ford_matrix(const std::vector<std::vector<int>>* adj_matrix, int starting_vertex) {
  // TODO: Implement Bellman-Ford for matrix
  return {};
}

void display_mst_results(Graph& mst_res, double time_) {
  int total_weight = 0;
  std::set<std::pair<int, int>> printed_edges;

  for (int i = 0; i < mst_res.adj_list->size(); i++) {
    printf("%d: ", i);
    auto adj_list = mst_res.adj_list->at(i);

    for (int j = 0; j < adj_list.get_size(); j++) {
      auto edge = adj_list.get(j);
      printf("(%d, %d)", edge.target, edge.weight);

      if (printed_edges.count({i, edge.target}) == 0) {
        printed_edges.insert({std::min(i, edge.target), std::max(i, edge.target)});
        total_weight += edge.weight;
      }

      if (j != adj_list.get_size() - 1) {
        printf(" -> ");
      } else {
        printf(";\n");
      }
    }
  }
}

void display_path_finding_results(const std::vector<Path>& path_res, double time_) {
  std::cout << "Pathfinding Results:\n";
  std::cout << "Computation Time: " << time_ << " seconds\n";

  for (size_t v = 0; v < path_res.size(); ++v) {
    const Path& p = path_res[v];

    std::cout << "Vertex " << v << ": ";

    if (p.cost == std::numeric_limits<int>::max()) {
      std::cout << "Unreachable\n";
      continue;
    }

    std::cout << "Cost = " << p.cost << ", Path = ";

    for (size_t i = 0; i < p.path.size(); ++i) {
      std::cout << p.path[i];
      if (i < p.path.size() - 1) std::cout << " -> ";
    }

    std::cout << "\n";
  }
}

void run_bellman_ford(std::shared_ptr<Graph> graph, int starting_vertex) {
  printf("Running Bellman Ford for starting vertex %d\n", starting_vertex);

  std::vector<Path> paths;

  printf("Adjacency matrix:\n");
  auto start_matrix = std::chrono::high_resolution_clock::now();
  paths = bellman_ford_matrix(graph->adj_matrix, starting_vertex);
  auto end_matrix = std::chrono::high_resolution_clock::now();
  auto duration_matrix = end_matrix - start_matrix;
  display_path_finding_results(paths, duration_matrix.count());

  printf("Adjacency list:\n");
  auto start_list = std::chrono::high_resolution_clock::now();
  paths = bellman_ford_list(graph->adj_list, starting_vertex);
  auto end_list = std::chrono::high_resolution_clock::now();
  auto duration_list = end_list - start_list;
  display_path_finding_results(paths, duration_list.count());
}

void run_dijkstra(std::shared_ptr<Graph> graph, int starting_vertex) {
  printf("Running Dijkstra for starting vertex %d\n", starting_vertex);

  std::vector<Path> paths;

  printf("Adjacency matrix:\n");
  auto start_matrix = std::chrono::high_resolution_clock::now();
  paths = dijkstra_matrix(graph->adj_matrix, starting_vertex);
  auto end_matrix = std::chrono::high_resolution_clock::now();
  auto duration_matrix = end_matrix - start_matrix;
  display_path_finding_results(paths, duration_matrix.count());

  printf("Adjacency list:\n");
  auto start_list = std::chrono::high_resolution_clock::now();
  paths = dijkstra_list(graph->adj_list, starting_vertex);
  auto end_list = std::chrono::high_resolution_clock::now();
  auto duration_list = end_list - start_list;
  display_path_finding_results(paths, duration_list.count());
}

void run_kruskal(std::shared_ptr<Graph> graph) {
  printf("Wykonywanie algorytmu Kruskala\n");

  Graph mst;

  printf("Na macierzy:\n");
  auto start_matrix = std::chrono::high_resolution_clock::now();
  mst = kruskal_matrix(graph->adj_matrix);
  auto end_matrix = std::chrono::high_resolution_clock::now();
  auto duration_matrix = end_matrix - start_matrix;
  display_mst_results(mst, duration_matrix.count());

  printf("Na liscie:\n");
  auto start_list = std::chrono::high_resolution_clock::now();
  mst = kruskal_list(graph->adj_list);
  auto end_list = std::chrono::high_resolution_clock::now();
  auto duration_list = end_list - start_list;
  display_mst_results(mst, duration_list.count());
}

void run_prim(std::shared_ptr<Graph> graph) {
  printf("Wykonywanie algorytmu Prima\n");

  Graph mst;

  printf("Na macierzy:\n");
  auto start_matrix = std::chrono::high_resolution_clock::now();
  mst = prim_matrix(graph->adj_matrix);
  auto end_matrix = std::chrono::high_resolution_clock::now();
  auto duration_matrix = end_matrix - start_matrix;
  display_mst_results(mst, duration_matrix.count());

  printf("Na liście:\n");
  auto start_list = std::chrono::high_resolution_clock::now();
  mst = prim_list(graph->adj_list);
  auto end_list = std::chrono::high_resolution_clock::now();
  auto duration_list = end_list - start_list;
  display_mst_results(mst, duration_list.count());
}

void display_graph(std::shared_ptr<Graph> graph) {
  printf("Wyświetlanie grafu\nLista sąsiedztwa:\n");

  for (int i = 0; i < graph->adj_list->size(); i++) {
    printf("%d: ", i);
    auto adj_list = graph->adj_list->at(i);

    for (int j = 0; j < adj_list.get_size(); j++) {
      auto edge = adj_list.get(j);
      printf("(%d, %d)", edge.target, edge.weight);

      if (j != adj_list.get_size() - 1) {
        printf(" -> ");
      } else {
        printf(";\n");
      }
    }
  }

  printf("Macierz sąsiedztwa:\n");

  for (int i = 0; i < graph->adj_matrix->size(); i++) {
    for (int j = 0; j < graph->adj_matrix->at(i).size(); j++) {
      printf("%d ", graph->adj_matrix->at(i)[j]);
    }
    printf("\n");
  }
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

std::shared_ptr<Graph> generate_graph(int vertex_count, int density) {
  auto graph = std::make_shared<Graph>();
  graph->adj_list->resize(vertex_count);
  graph->adj_matrix->resize(vertex_count, std::vector<int>(vertex_count, 0));

  std::mt19937 rng(static_cast<unsigned>(time(nullptr)));
  std::uniform_int_distribution<int> weight_dist(1, 10); // weights between 1 and 10
  std::uniform_int_distribution<int> vertex_dist(0, vertex_count - 1);

  std::set<std::pair<int, int>> edges;

  std::vector<int> nodes(vertex_count);
  for (int i = 0; i < vertex_count; ++i) nodes[i] = i;
  std::shuffle(nodes.begin(), nodes.end(), rng);

  for (int i = 1; i < vertex_count; ++i) {
    int u = nodes[i];
    int v = nodes[vertex_dist(rng) % i];
    int weight = weight_dist(rng);
    edges.insert({std::min(u, v), std::max(u, v)});
    graph->adj_list->at(u).add(v, weight);
    graph->adj_list->at(v).add(u, weight);
    graph->adj_matrix->at(u)[v] = weight;
    graph->adj_matrix->at(v)[u] = weight;
  }

  int max_edges = vertex_count * (vertex_count - 1) / 2;
  int target_edges = max_edges * density / 100;
  int current_edges = static_cast<int>(edges.size());

  while (current_edges < target_edges) {
    int u = vertex_dist(rng);
    int v = vertex_dist(rng);
    if (u == v) continue;

    auto edge = std::make_pair(std::min(u, v), std::max(u, v));
    if (edges.count(edge)) continue;

    int weight = weight_dist(rng);
    edges.insert(edge);
    graph->adj_list->at(u).add(v, weight);
    graph->adj_list->at(v).add(u, weight);
    graph->adj_matrix->at(u)[v] = weight;
    graph->adj_matrix->at(v)[u] = weight;
    current_edges++;
  }

  return graph;
}

int read_path_from_file(std::shared_ptr<Graph>& graph, const std::string& file_path) {
  std::ifstream file(file_path);
  if (!file.is_open()) {
    std::cerr << "Failed to open file: " << file_path << std::endl;
    return -1;
  }

  int edge_count, vertex_count, starting_vertex;
  file >> edge_count >> vertex_count >> starting_vertex;

  graph = std::make_shared<Graph>();

  graph->adj_list->resize(vertex_count);
  graph->adj_matrix->resize(vertex_count, std::vector<int>(vertex_count, 0));

  for (int i = 0; i < edge_count; ++i) {
    int src, dest, weight;
    file >> src >> dest >> weight;

    graph->adj_list->at(src).add(dest, weight);
    graph->adj_list->at(dest).add(src, weight);

    graph->adj_matrix->at(src)[dest] = weight;
    graph->adj_matrix->at(dest)[src] = weight;
  }

  file.close();

  return starting_vertex;
}

void read_mst_from_file(std::shared_ptr<Graph>& graph, const std::string& file_path) {
  std::ifstream file(file_path);
  if (!file.is_open()) {
    std::cerr << "Failed to open file: " << file_path << std::endl;
    return;
  }

  int edge_count, vertex_count;
  file >> edge_count >> vertex_count;

  graph = std::make_shared<Graph>();

  graph->adj_list->resize(vertex_count);
  graph->adj_matrix->resize(vertex_count, std::vector<int>(vertex_count, 0));

  for (int i = 0; i < edge_count; ++i) {
    int src, dest, weight;
    file >> src >> dest >> weight;

    graph->adj_list->at(src).add(dest, weight);
    graph->adj_list->at(dest).add(src, weight);

    graph->adj_matrix->at(src)[dest] = weight;
    graph->adj_matrix->at(dest)[src] = weight;
  }

  file.close();
}


int path_gen_dialog(std::shared_ptr<Graph>& graph) {
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

  graph = generate_graph(vertex_count, density);
  return starting_vertex;
}

int path_file_dialog(std::shared_ptr<Graph>& graph) {
  std::string file_path = file_path_dialog();
  int starting_vertex = read_path_from_file(graph, file_path);
  return starting_vertex;
}

void mst_gen_dialog(std::shared_ptr<Graph>& graph) {
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

  graph = generate_graph(vertex_count, density);
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
