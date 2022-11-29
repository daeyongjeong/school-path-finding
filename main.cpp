#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <iostream>
#include <queue>
#include <set>

using namespace std;

struct Coord3D
{
    double x, y, z;
    Coord3D(double x, double y, double z) : x(x), y(y), z(z){};
};

struct Vertex
{
    int id;
    string name;
    Coord3D coord;
};

struct Edge
{
    int vid1, vid2;
    double cost;
};

struct Graph
{
    vector<Vertex> vertices;
    vector<vector<Edge>> edges;

    void addVertex(string name, Coord3D coord);
    void addEdge(int vid1, int vid2, double cost);
    vector<int> getNeighbors(int vid);
    int getCost(int vid1, int vid2);
};

void Graph::addVertex(string name, Coord3D coord)
{
    int new_id = vertices.size();
    Vertex v = {new_id, name, coord};
    vector<Edge> e;
    vertices.push_back(v);
    edges.push_back(e);
}

void Graph::addEdge(int vid1, int vid2, double cost)
{
    Edge e1 = {vid1, vid2, cost};
    Edge e2 = {vid2, vid1, cost};
    edges[vid1].push_back(e1);
    edges[vid2].push_back(e2);
}

vector<int> Graph::getNeighbors(int vid)
{
    vector<int> neighbors;
    for (Edge e : edges[vid])
        neighbors.push_back(e.vid2);
    return neighbors;
}

int Graph::getCost(int vid1, int vid2)
{
    for (Edge e : edges[vid1])
        if (e.vid2 == vid2)
            return e.cost;
    return -1; // failure
}

class A_star
{
private:
    Graph g;
    int sid;
    int gid;

    double heuristic(int current);
    void reconstruct_path(int came_from[], int current);

public:
    deque<int> path;
    A_star(Graph g, int sid, int gid);
    void print_path();
};

double A_star::heuristic(int current)
{
    Coord3D scoord = g.vertices[current].coord;
    Coord3D gcoord = g.vertices[gid].coord;
    return abs(scoord.x - gcoord.x) + abs(scoord.y - gcoord.y) + abs(scoord.z - gcoord.z);
}

void A_star::reconstruct_path(int came_from[], int current)
{
    path.push_back(current);
    int pid = current;
    while (came_from[pid] != -1)
    {
        pid = came_from[pid];
        path.push_front(pid);
    }
}

// A* finds a path from start to goal.
// h is the heuristic function. h(n) estimates the cost to reach goal from node n.
A_star::A_star(Graph g, int sid, int gid) : g(g), sid(sid), gid(gid)
{
    // The set of discovered nodes that may need to be (re-)expanded.
    // Initially, only the start node is known.
    // This is usually implemented as a min-heap or priority queue rather than a hash-set.
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> open_queue;
    set<int> open_set;
    open_queue.push(pair<int, int>(0, sid));
    open_set.insert(sid);

    // For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start
    // to n currently known.
    const int vertices_count = g.vertices.size();
    cout << "vertices_count: " << vertices_count << endl; // DEBUG
    int came_from[vertices_count];
    for (int i = 0; i < vertices_count; ++i)
        came_from[i] = -1;

    // For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
    double g_score[vertices_count];
    for (int i = 0; i < vertices_count; ++i)
        g_score[i] = MAXFLOAT;
    g_score[sid] = 0;

    // For node n, fScore[n] := gScore[n] + h(n). fScore[n] represents our current best guess as to
    // how cheap a path could be from start to finish if it goes through n.
    double f_score[vertices_count];
    for (int i = 0; i < vertices_count; ++i)
        f_score[i] = MAXFLOAT;
    f_score[sid] = heuristic(sid);
    cout << "f_score[sid]: " << f_score[sid] << endl; // DEBUG

    while (!open_set.empty())
    {
        int current = open_queue.top().second;
        cout << "current: " << current << endl; // DEBUG
        if (current == gid)
            reconstruct_path(came_from, current);

        open_queue.pop();
        open_set.erase(current);
        vector<int> current_neighbors = g.getNeighbors(current);
        // DEBUG
        cout << "current_neighbors: " << endl;
        for (int i = 0; i < current_neighbors.size(); ++i)
            cout << current_neighbors[i] << " ";
        cout << endl;
        // DEBUG ENDS
        for (int neighbor : current_neighbors)
        {
            double tentative_g_score = g_score[current] + g.getCost(current, neighbor);
            cout << "tentative_g_score: " << tentative_g_score << endl; // DEBUG
            if (tentative_g_score < g_score[neighbor])
            {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g_score;
                f_score[neighbor] = tentative_g_score + heuristic(neighbor);
                if (open_set.find(neighbor) == open_set.end())
                {
                    open_queue.push(pair<int, int>(f_score[neighbor], neighbor));
                    open_set.insert(neighbor);
                }
            }
        }
    }
}

void A_star::print_path()
{
    cout << "path: " << endl;
    for (int i = 0; i < path.size(); ++i)
        cout << g.vertices[path[i]].name << endl;
}

// TODO: CSV -> 구조체 변환 코드

int main()
{
    Graph g;
    g.addVertex("신공학관 3층 북쪽 계단", Coord3D(0, 0, 19.5));
    g.addVertex("신공학관 3층 3101", Coord3D(-2.15, 0, 19.5));
    g.addEdge(0, 1, 2.15);

    A_star a(g, 0, 1);
    a.print_path();

    return 0;
}
