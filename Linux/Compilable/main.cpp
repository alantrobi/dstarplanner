#include <SFML/Graphics.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cstdlib>

using namespace std;

const double INF = numeric_limits<double>::infinity();

struct Node {
    uint64_t id;
    string label;
    sf::Vector2f pos;
    bool available = true; // Added field
};

struct Edge {
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;
    bool available;
    double reliability = 0.9; // Added field, default to 0.9 (range 0.0 to 1.0)
};

class DStarLite {
    struct Key { double a, b; };
    struct Item { Key key; uint64_t u; };

    struct Compare {
        bool operator()(const Item& x, const Item& y) const {
            if (x.key.a != y.key.a) return x.key.a > y.key.a;
            if (x.key.b != y.key.b) return x.key.b > y.key.b;
            return x.u > y.u;
        }
    };

    vector<Node>& nodes;
    vector<Edge>& edges;
    unordered_map<uint64_t, double> g, rhs;
    unordered_set<uint64_t> bad;
    unordered_set<uint64_t> unavailableNodes;
    priority_queue<Item, vector<Item>, Compare> open;
    uint64_t start, goal;

    const Node* node(uint64_t id) const {
        for (const auto& n : nodes) if (n.id == id) return &n;
        return nullptr;
    }

    double h(uint64_t a, uint64_t b) const {
        // Return 0.0 to ensure the heuristic is always admissible.
        // Screen pixel coordinates (e.g. 400px) are orders of magnitude larger 
        // than edge costs (e.g. 3.0), which causes D* Lite to terminate prematurely.
        return 0.0;
    }

    bool isNodeTraversable(uint64_t id) const {
        if (bad.count(id) != 0 || unavailableNodes.count(id) != 0) return false;
        if (id != start && id != goal) {
            double s = getSafety(id);
            if (s < safetyThreshold) return false;
        }
        return true;
    }

    vector<const Edge*> succ(uint64_t u) const {
        vector<const Edge*> r;
        if (!isNodeTraversable(u)) return r;
        for (const auto& e : edges)
            if (e.from == u && e.available && isNodeTraversable(e.to))
                r.push_back(&e);
        return r;
    }

    vector<const Edge*> pred(uint64_t u) const {
        vector<const Edge*> r;
        if (!isNodeTraversable(u)) return r;
        for (const auto& e : edges)
            if (e.to == u && e.available && isNodeTraversable(e.from))
                r.push_back(&e);
        return r;
    }

    Key key(uint64_t u) const {
        double gv = getG(u), rv = getRhs(u);
        double m = min(gv, rv);
        return { m + h(start, u), m };
    }

    static bool lessKey(const Key& x, const Key& y) {
        if (x.a != y.a) return x.a < y.a;
        return x.b < y.b;
    }

    static bool sameKey(const Key& x, const Key& y) {
        return !lessKey(x, y) && !lessKey(y, x);
    }

    void compute() {
        while (!open.empty()) {
            Item cur = open.top();
            open.pop();

            Key fresh = key(cur.u);
            if (lessKey(cur.key, fresh)) {
                open.push({ fresh, cur.u });
                continue;
            }
            if (!sameKey(cur.key, fresh)) continue;

            Key sk = key(start);
            if (!lessKey(cur.key, sk) && getG(start) == getRhs(start))
                break;

            double gVal = getG(cur.u);
            double rhsVal = getRhs(cur.u);

            if (gVal > rhsVal) {
                g[cur.u] = rhsVal;
                for (const Edge* e : pred(cur.u))
                    updateVertex(e->from);
            }
            else {
                g[cur.u] = INF;
                updateVertex(cur.u);
                for (const Edge* e : pred(cur.u))
                    updateVertex(e->from);
            }
        }
    }

public:
    double alpha = 1000.0;
    double beta = 1.0;
    double gamma = 2.0;
    double delta = 1.0;
    double safetyThreshold = 80.0; // Added for hard safety constraint

    DStarLite(vector<Node>& n, vector<Edge>& e, uint64_t s, uint64_t t)
        : nodes(n), edges(e), start(s), goal(t) {
        reset();
    }

    DStarLite(const DStarLite&) = delete;
    DStarLite& operator=(const DStarLite&) = delete;

    double getG(uint64_t id) const {
        auto it = g.find(id);
        return it != g.end() ? it->second : INF;
    }

    double getRhs(uint64_t id) const {
        auto it = rhs.find(id);
        return it != rhs.end() ? it->second : INF;
    }

    double getSafety(uint64_t u) const {
        if (bad.empty()) return 1000.0;
        const Node* curr = node(u);
        if (!curr) return 0.0;
        double minD = INF;
        for (const auto& n : nodes) {
            if (bad.count(n.id)) {
                double dx = curr->pos.x - n.pos.x;
                double dy = curr->pos.y - n.pos.y;
                double dist = sqrt(dx * dx + dy * dy);
                if (dist < minD) minD = dist;
            }
        }
        return minD;
    }

    double getEdgeCost(const Edge& e) const {
        if (!e.available) return INF;
        if (!isNodeTraversable(e.from) || !isNodeTraversable(e.to)) return INF;
        double s = getSafety(e.to);
        double c = beta * e.cost + gamma * (1000.0 - s) + delta * (1.0 - e.reliability);
        return max(0.0, c);
    }

    void updateVertex(uint64_t u) {
        if (u != goal) {
            rhs[u] = INF;
            if (isNodeTraversable(u)) {
                for (const Edge* e : succ(u)) {
                    double edgeC = getEdgeCost(*e);
                    double gTo = getG(e->to);
                    if (edgeC != INF && gTo != INF) {
                        rhs[u] = min(rhs[u], edgeC + gTo);
                    }
                }
            }
        }
        if (getG(u) != getRhs(u))
            open.push({ key(u), u });
    }

    void reset() {
        start = 0;
        goal = 6;
        bad.clear();
        unavailableNodes.clear();
        resetStructuresOnly();
    }

    void resetStructuresOnly() {
        while (!open.empty()) open.pop();
        g.clear(); rhs.clear();

        for (const auto& n : nodes) {
            g[n.id] = INF;
            rhs[n.id] = INF;
        }
        rhs[goal] = 0.0;
        open.push({ key(goal), goal });
        compute();
    }

    void replan() { compute(); }

    void onWeightChanged() {
        for (const auto& n : nodes) {
            updateVertex(n.id);
        }
        compute();
    }

    void toggleEdge(uint64_t id) {
        for (auto& e : edges) {
            if (e.id == id) {
                e.available = !e.available;
                updateVertex(e.from);
                compute();
                return;
            }
        }
    }

    void changeCost(uint64_t id, double amount) {
        for (auto& e : edges) {
            if (e.id == id) {
                e.cost = max(1.0, e.cost + amount);
                updateVertex(e.from);
                compute();
                return;
            }
        }
    }

    void changeReliability(uint64_t id, double amount) {
        for (auto& e : edges) {
            if (e.id == id) {
                e.reliability = max(0.0, min(1.0, e.reliability + amount));
                updateVertex(e.from);
                compute();
                return;
            }
        }
    }

    void toggleAvailable(uint64_t id) {
        if (id == start || id == goal) return;
        if (unavailableNodes.count(id)) unavailableNodes.erase(id);
        else unavailableNodes.insert(id);

        updateVertex(id);
        for (const auto& e : edges) {
            if (e.to == id) {
                updateVertex(e.from);
            }
        }
        compute();
    }

    void toggleBad(uint64_t id) {
        if (id == start || id == goal) return;

        if (bad.count(id)) bad.erase(id);
        else bad.insert(id);

        for (const auto& n : nodes) {
            updateVertex(n.id);
        }
        compute();
    }

    bool isBad(uint64_t id) const { return bad.count(id) != 0; }
    bool isUnavailable(uint64_t id) const { return unavailableNodes.count(id) != 0; }
    uint64_t getStart() const { return start; }
    uint64_t getGoal() const { return goal; }

    void setStart(uint64_t s) {
        start = s;
        if (bad.count(start)) bad.erase(start);
        if (unavailableNodes.count(start)) unavailableNodes.erase(start);
        resetStructuresOnly();
    }

    void setGoal(uint64_t gVal) {
        goal = gVal;
        if (bad.count(goal)) bad.erase(goal);
        if (unavailableNodes.count(goal)) unavailableNodes.erase(goal);
        resetStructuresOnly();
    }

    vector<uint64_t> path() const {
        vector<uint64_t> p;
        if (bad.count(start) || bad.count(goal) || unavailableNodes.count(start) || unavailableNodes.count(goal) || getG(start) == INF)
            return p;

        uint64_t u = start;
        unordered_set<uint64_t> seen;
        seen.insert(u);
        p.push_back(u);

        while (u != goal) {
            const Edge* best = nullptr;
            double value = INF;

            for (const Edge* e : succ(u)) {
                double edgeC = getEdgeCost(*e);
                double gTo = getG(e->to);
                if (gTo == INF || edgeC == INF) continue;
                double v = edgeC + gTo;
                if (v < value) {
                    value = v;
                    best = e;
                }
            }

            if (!best) return {};
            u = best->to;
            if (seen.count(u)) return {};
            seen.insert(u);
            p.push_back(u);
        }
        return p;
    }

    double pathCost() const {
        auto p = path();
        if (p.empty()) return INF;
        double total = 0.0;
        for (size_t i = 0; i + 1 < p.size(); ++i) {
            bool found = false;
            for (const auto& e : edges) {
                if (e.from == p[i] && e.to == p[i + 1] && e.available) {
                    total += e.cost;
                    found = true;
                    break;
                }
            }
            if (!found) return INF;
        }
        return total;
    }

    double pathReliability() const {
        auto p = path();
        if (p.empty()) return 0.0;
        double total = 0.0;
        for (size_t i = 0; i + 1 < p.size(); ++i) {
            for (const auto& e : edges) {
                if (e.from == p[i] && e.to == p[i + 1] && e.available) {
                    total += e.reliability;
                    break;
                }
            }
        }
        return total;
    }

    double pathMinSafety() const {
        auto p = path();
        if (p.empty()) return 0.0;
        double minS = INF;
        for (uint64_t nodeId : p) {
            double s = getSafety(nodeId);
            if (s < minS) minS = s;
        }
        return minS == INF ? 0.0 : minS;
    }

    double getPathScore() const {
        auto p = path();
        if (p.empty()) return -INF;
        double G = (p.back() == goal) ? 1.0 : 0.0;
        double C = pathCost();
        double D = pathMinSafety();
        if (bad.empty()) D = 0.0;
        double R = pathReliability();
        return alpha * G - beta * C + gamma * D + delta * R;
    }

    const unordered_set<uint64_t>& getBadNodes() const { return bad; }
    const unordered_set<uint64_t>& getUnavailableNodes() const { return unavailableNodes; }
};

const Node* findNode(const vector<Node>& nodes, uint64_t id) {
    for (const auto& n : nodes) if (n.id == id) return &n;
    return nullptr;
}

sf::Vector2f posOf(const vector<Node>& nodes, uint64_t id) {
    const Node* n = findNode(nodes, id);
    return n ? n->pos : sf::Vector2f();
}

string nameOf(const vector<Node>& nodes, uint64_t id) {
    const Node* n = findNode(nodes, id);
    return n ? n->label + " (" + to_string(id) + ")" : "?";
}

double pointSegment(sf::Vector2f p, sf::Vector2f a, sf::Vector2f b) {
    sf::Vector2f d = b - a;
    sf::Vector2f q = p - a;
    double dd = d.x * d.x + d.y * d.y;
    if (dd == 0) return hypot(p.x - a.x, p.y - a.y);
    double t = (q.x * d.x + q.y * d.y) / dd;
    t = max(0.0, min(1.0, t));
    sf::Vector2f c = a + d * static_cast<float>(t);
    return hypot(p.x - c.x, p.y - c.y);
}

bool hasNode(const vector<uint64_t>& p, uint64_t id) {
    return find(p.begin(), p.end(), id) != p.end();
}

bool hasEdge(const vector<uint64_t>& p, uint64_t a, uint64_t b) {
    for (size_t i = 0; i + 1 < p.size(); ++i)
        if (p[i] == a && p[i + 1] == b) return true;
    return false;
}

string num(double x) {
    if (x == INF) return "INF";
    if (x == -INF) return "-INF";
    ostringstream s;
    s << fixed << setprecision(2) << x;
    return s.str();
}

string pathText(const vector<Node>& nodes, const vector<uint64_t>& p) {
    if (p.empty()) return "NO PATH";
    string s;
    for (size_t i = 0; i < p.size(); ++i) {
        if (i) s += " -> ";
        s += nameOf(nodes, p[i]);
    }
    return s;
}

string compactPathText(const vector<Node>& nodes, const vector<uint64_t>& p) {
    if (p.empty()) return "NO PATH";
    string s;
    for (size_t i = 0; i < p.size(); ++i) {
        if (i) s += "->";
        const Node* n = findNode(nodes, p[i]);
        s += n ? n->label : "?";
    }
    return s;
}

void text(sf::RenderWindow& w, const sf::Font& f, const string& s,
    unsigned size, sf::Color c, float x, float y, bool center = false) {
    sf::Text t;
    t.setFont(f); t.setString(s); t.setCharacterSize(size); t.setFillColor(c);
    if (center) {
        auto b = t.getLocalBounds();
        t.setOrigin(b.left + b.width / 2.f, b.top + b.height / 2.f);
    }
    t.setPosition(x, y);
    w.draw(t);
}

class Log {
    vector<string> lines;
    int offset = 0;
public:
    void add(const string& s) { lines.push_back(s); offset = 0; }
    void scroll(int d) {
        int maxOff = max(0, (int)lines.size() - 13);
        offset = max(0, min(maxOff, offset + d));
    }
    void draw(sf::RenderWindow& w, const sf::Font& f, float x, float y, float width, float height) const {
        sf::RectangleShape r({ width, height });
        r.setPosition(x, y);
        r.setFillColor(sf::Color(17, 20, 28)); // Slightly darker panel background for contrast
        r.setOutlineThickness(1.5f);
        r.setOutlineColor(sf::Color(55, 65, 81));
        w.draw(r);

        text(w, f, "EVENT LOG", 16, sf::Color(96, 165, 250), x + 15, y + 12); // Bright sky blue header

        const int visible = 12;
        int end = (int)lines.size() - offset;
        int begin = max(0, end - visible);

        for (int i = begin; i < end; ++i) {
            sf::Color lineC = sf::Color(243, 244, 246); // Default bright white
            const string& line = lines[i];
            if (line.rfind("SYSTEM:", 0) == 0) {
                lineC = sf::Color(147, 197, 253); // Soft blue
            } else if (line.rfind("PLAN:", 0) == 0) {
                lineC = sf::Color(52, 211, 153); // Bright green
            } else if (line.rfind("D* LITE:", 0) == 0) {
                lineC = sf::Color(192, 132, 252); // Soft purple
            } else if (line.rfind("NODE:", 0) == 0 || line.rfind("EDGE:", 0) == 0) {
                lineC = sf::Color(251, 146, 60); // Orange
            } else if (line.rfind("SETUP:", 0) == 0 || line.rfind("RESET:", 0) == 0) {
                lineC = sf::Color(248, 113, 113); // Soft red
            }
            
            // Truncate line if it's too long for the box (prevent overflow)
            string drawLine = line;
            if (drawLine.size() > 58) {
                drawLine = drawLine.substr(0, 55) + "...";
            }
            
            text(w, f, drawLine, 12, lineC, x + 15, y + 42 + (i - begin) * 26);
        }

        text(w, f, "Mouse wheel: scroll history", 11,
            sf::Color(156, 163, 175), x + 15, y + height - 20);
    }
};

bool clickedBox(sf::Vector2f m, float x, float y, float w, float h) {
    return m.x >= x && m.x <= x + w && m.y >= y && m.y <= y + h;
}

void drawButton(sf::RenderWindow& w, const sf::Font& f, const string& textStr,
                float x, float y, float width, float height,
                sf::Color bgNormal, sf::Color bgHover, sf::Color textColor,
                sf::Vector2f mousePos, bool active = true) {
    sf::RectangleShape r({ width, height });
    r.setPosition(x, y);
    bool hovered = active && r.getGlobalBounds().contains(mousePos);
    r.setFillColor(hovered ? bgHover : bgNormal);
    r.setOutlineThickness(1.5f);
    r.setOutlineColor(sf::Color(55, 65, 81));
    w.draw(r);

    sf::Text t;
    t.setFont(f);
    t.setString(textStr);
    t.setCharacterSize(13);
    t.setFillColor(active ? textColor : sf::Color(100, 105, 120));
    auto b = t.getLocalBounds();
    t.setOrigin(b.left + b.width / 2.f, b.top + b.height / 2.f);
    t.setPosition(x + width / 2.f, y + height / 2.f);
    w.draw(t);
}

enum Tab { DASHBOARD, SETUP };
enum SelectedType { NONE, NODE, EDGE };

int main() {
    // Enable high-quality random seeding
    srand(static_cast<unsigned>(time(nullptr)));

    const unsigned W = 1400, H = 820;
    const float GRAPH = 900.f, RIGHT = 920.f, RW = 460.f;

    sf::RenderWindow window(
        sf::VideoMode(1400, 820),
        "D* Lite - Multi-Objective Path Planner",
        sf::Style::Default
    );

    window.setSize(sf::Vector2u(
		sf::VideoMode::getDesktopMode().width,
		sf::VideoMode::getDesktopMode().height
	));
	
    window.setFramerateLimit(60);

    vector<Node> nodes = {
        {0,"A",{110,390}, true},
        {1,"B",{300,180}, true},
        {2,"C",{300,600}, true},
        {4,"D",{510,180}, true},
        {5,"E",{510,680}, true}, // Moved E down from 600 to 680 to avoid overlapping straight line C->F
        {7,"F",{700,600}, true},
        {8,"G",{510,390}, true},
        {6,"H",{800,390}, true}
    };

    vector<Edge> edges = {
        {0,0,1,2,true,0.95}, {1,1,4,2,true,0.90}, {2,4,6,2,true,0.85},
        {3,0,2,4,true,0.80}, {4,2,5,4,true,0.95}, {5,5,6,4,true,0.75},
        {6,2,7,5,true,0.90}, {7,7,6,5,true,0.85},
        {8,0,8,3,true,0.99}, {9,8,6,3,true,0.90}
    };

    DStarLite planner(nodes, edges, 0, 6);

    sf::Font font;
    if (!font.loadFromFile("arial.ttf"))
        return 1;

    Log log;
    log.add("SYSTEM: D* Lite planner loaded.");
    log.add("SYSTEM: Score(P) = aG - bC + gD + dR");
    log.add("SYSTEM: Left-click Node: toggle AVAILABLE.");
    log.add("SYSTEM: Right-click Node: toggle GOOD / BAD.");
    log.add("SYSTEM: Left-click Edge: toggle AVAILABLE.");
    log.add("SYSTEM: Shift+Edge: Cost +1 | Ctrl+Edge: Cost -1.");
    log.add("SYSTEM: Switch tabs above to customize weights and setup.");

    vector<uint64_t> previous = planner.path();
    string status = "Initial path planned successfully.";
    
    Tab currentTab = DASHBOARD;
    int selectedType = NONE;
    int64_t selectedId = -1;
    int64_t edgeSourceNodeId = -1;

    int64_t draggedNodeId = -1;
    sf::Vector2f dragStartMousePos;
    sf::Vector2f dragStartNodePos;
    bool hasMovedNode = false;

    int draggedSliderIdx = -1; // Index of slider being dragged (0-4), -1 if none
    bool wasNodeAlreadySelected = false; // Track selection state before click

    auto report = [&](const string& reason) {
        vector<uint64_t> now = planner.path();
        double c = planner.pathCost();
        double s = planner.pathMinSafety();
        double r = planner.pathReliability();
        double sc = planner.getPathScore();

        log.add("PLAN: " + reason +
            " | Path: " + compactPathText(nodes, now) +
            " | Scr: " + num(sc));

        if (now != previous)
            log.add("D* LITE: Path changed due to update.");
        else
            log.add("D* LITE: Path unchanged.");

        previous = now;
    };

    log.add("PLAN: initial = " + compactPathText(nodes, previous) + " | Scr: " + num(planner.getPathScore()));

    while (window.isOpen()) {
        sf::Event ev;
        sf::Vector2i mPixel = sf::Mouse::getPosition(window);
        sf::Vector2f m = window.mapPixelToCoords(mPixel);

        while (window.pollEvent(ev)) {
            if (ev.type == sf::Event::Closed)
                window.close();

            if (ev.type == sf::Event::MouseWheelScrolled) {
                if (currentTab == DASHBOARD &&
                    ev.mouseWheelScroll.x >= (int)RIGHT &&
                    ev.mouseWheelScroll.y >= 390 &&
                    ev.mouseWheelScroll.y <= 790) {
                    log.scroll(ev.mouseWheelScroll.delta > 0 ? 3 : -3);
                }
            }

            if (ev.type == sf::Event::KeyPressed) {
                if (ev.key.code == sf::Keyboard::R) {
                    // Quick reset shortcut
                    nodes = {
                        {0,"A",{110,390}, true},
                        {1,"B",{300,180}, true},
                        {2,"C",{300,600}, true},
                        {4,"D",{510,180}, true},
                        {5,"E",{510,680}, true},
                        {7,"F",{700,600}, true},
                        {8,"G",{510,390}, true},
                        {6,"H",{800,390}, true}
                    };
                    edges = {
                        {0,0,1,2,true,0.95}, {1,1,4,2,true,0.90}, {2,4,6,2,true,0.85},
                        {3,0,2,4,true,0.80}, {4,2,5,4,true,0.95}, {5,5,6,4,true,0.75},
                        {6,2,7,5,true,0.90}, {7,7,6,5,true,0.85},
                        {8,0,8,3,true,0.99}, {9,8,6,3,true,0.90}
                    };
                    planner.alpha = 1000.0;
                    planner.beta = 1.0;
                    planner.gamma = 2.0;
                    planner.delta = 1.0;
                    planner.safetyThreshold = 80.0;
                    planner.reset();
                    selectedType = NONE;
                    selectedId = -1;
                    edgeSourceNodeId = -1;
                    status = "Visualizer and environment reset.";
                    log.add("RESET: environment restored.");
                    report("reset");
                }
                else if (ev.key.code == sf::Keyboard::Space) {
                    planner.replan();
                    status = "Forced D* Lite path planning computation.";
                    log.add("D* LITE: manual replanning triggered.");
                    report("manual replan");
                }
            }

            if (ev.type == sf::Event::MouseButtonPressed) {
                // Determine modifier keys accurately using Keyboard state
                bool shiftPressed = sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift);
                bool ctrlPressed = sf::Keyboard::isKeyPressed(sf::Keyboard::LControl) || sf::Keyboard::isKeyPressed(sf::Keyboard::RControl);

                if (ev.mouseButton.button == sf::Mouse::Left) {
                    // 1. Check tab bar clicks
                    if (clickedBox(m, RIGHT, 15, 215, 35)) {
                        currentTab = DASHBOARD;
                    }
                    else if (clickedBox(m, RIGHT + 225, 15, 215, 35)) {
                        currentTab = SETUP;
                    }
                    // 2. Check Graph click
                    else if (m.x < GRAPH) {
                        bool clickedNode = false;
                        for (const auto& n : nodes) {
                            double dist = hypot(m.x - n.pos.x, m.y - n.pos.y);
                            if (dist <= 31.0) {
                                // Start node dragging
                                wasNodeAlreadySelected = (selectedType == NODE && selectedId == static_cast<int64_t>(n.id));
                                draggedNodeId = static_cast<int64_t>(n.id);
                                dragStartMousePos = m;
                                dragStartNodePos = n.pos;
                                hasMovedNode = false;

                                selectedId = static_cast<int64_t>(n.id);
                                selectedType = NODE;
                                clickedNode = true;
                                break;
                            }
                        }

                        if (!clickedNode) {
                            // Check click on edges
                            double bestDist = 20.0;
                            int bestEdgeIdx = -1;
                            for (size_t i = 0; i < edges.size(); ++i) {
                                auto a = posOf(nodes, edges[i].from);
                                auto b = posOf(nodes, edges[i].to);
                                double d = pointSegment(m, a, b);
                                if (d < bestDist) {
                                    bestDist = d;
                                    bestEdgeIdx = (int)i;
                                }
                            }

                            if (bestEdgeIdx != -1) {
                                Edge& e = edges[bestEdgeIdx];
                                selectedId = static_cast<int64_t>(e.id);
                                selectedType = EDGE;

                                if (shiftPressed) {
                                    planner.changeCost(e.id, 1.0);
                                    log.add("EDGE: " + nameOf(nodes, e.from) + " -> " + nameOf(nodes, e.to) + " cost: " + num(e.cost));
                                    status = "Edge cost increased; replanned.";
                                    report("edge cost increase");
                                }
                                else if (ctrlPressed) {
                                    planner.changeCost(e.id, -1.0);
                                    log.add("EDGE: " + nameOf(nodes, e.from) + " -> " + nameOf(nodes, e.to) + " cost: " + num(e.cost));
                                    status = "Edge cost decreased; replanned.";
                                    report("edge cost decrease");
                                }
                                else {
                                    planner.toggleEdge(e.id);
                                    log.add("EDGE: " + nameOf(nodes, e.from) + " -> " + nameOf(nodes, e.to) + (e.available ? " set AVAILABLE." : " set BLOCKED."));
                                    status = "Edge availability toggled; replanned.";
                                    report("edge availability toggle");
                                }
                            }
                        }
                    }
                    // 3. Check Setup tab interface buttons
                    else if (currentTab == SETUP) {
                        // Slider click detection
                        bool clickedSlider = false;
                        for (int i = 0; i < 5; ++i) {
                            float sliderY = 100.f + static_cast<float>(i) * 36.f;
                            if (clickedBox(m, RIGHT + 265.f, sliderY, 170.f, 24.f)) {
                                draggedSliderIdx = i;
                                double pct = (m.x - (RIGHT + 270.f)) / 160.f;
                                if (pct < 0.0) pct = 0.0;
                                if (pct > 1.0) pct = 1.0;
                                
                                if (i == 0) planner.alpha = pct * 2000.f;
                                else if (i == 1) planner.beta = pct * 10.f;
                                else if (i == 2) planner.gamma = pct * 10.f;
                                else if (i == 3) planner.delta = pct * 10.f;
                                else if (i == 4) planner.safetyThreshold = pct * 300.f;
                                
                                planner.onWeightChanged();
                                clickedSlider = true;
                                break;
                            }
                        }

                        if (clickedSlider) {
                            // Captured by slider click
                        }
                        // --- Selected Element Actions ---
                        else if (selectedType == NODE && selectedId != -1) {
                            // Toggle Node Availability (Y = 385)
                            if (clickedBox(m, RIGHT + 20, 385, 180, 28)) {
                                if (selectedId == planner.getStart() || selectedId == planner.getGoal()) {
                                    status = "Start and Goal nodes cannot be made unavailable.";
                                    log.add("ERROR: Start and Goal nodes must remain available.");
                                } else {
                                    planner.toggleAvailable(selectedId);
                                    bool nowAvail = !planner.isUnavailable(selectedId);
                                    log.add("NODE: " + nameOf(nodes, selectedId) + (nowAvail ? " set AVAILABLE." : " set UNAVAILABLE."));
                                    status = "Node availability toggled; replanned.";
                                    report("node availability toggle");
                                }
                            }
                            // Toggle Node GOOD/BAD (Y = 385)
                            else if (clickedBox(m, RIGHT + 220, 385, 180, 28)) {
                                if (selectedId == planner.getStart() || selectedId == planner.getGoal()) {
                                    status = "Start and Goal nodes cannot be marked BAD.";
                                    log.add("ERROR: Start and Goal nodes cannot be marked BAD.");
                                } else {
                                    planner.toggleBad(selectedId);
                                    bool nowBad = planner.isBad(selectedId);
                                    log.add("NODE: " + nameOf(nodes, selectedId) + (nowBad ? " set BAD." : " set GOOD."));
                                    status = "Node state toggled (GOOD/BAD); replanned.";
                                    report("node state toggle");
                                }
                            }
                            // Set as Goal Node (Y = 422)
                            else if (clickedBox(m, RIGHT + 20, 422, 180, 28)) {
                                if (selectedId == planner.getGoal()) {
                                    status = "Node " + nameOf(nodes, selectedId) + " is already the Goal.";
                                }
                                else if (selectedId == planner.getStart()) {
                                    status = "Node " + nameOf(nodes, selectedId) + " is the Start node. Change start first.";
                                    log.add("SETUP: Start and Goal cannot be the same node.");
                                }
                                else {
                                    planner.setGoal(selectedId);
                                    status = "Goal node changed to " + nameOf(nodes, selectedId) + "; replanned.";
                                    log.add("SETUP: Set Goal to " + nameOf(nodes, selectedId));
                                    report("goal node changed");
                                }
                            }
                            // Set as Start Node (Y = 422)
                            else if (clickedBox(m, RIGHT + 220, 422, 180, 28)) {
                                if (selectedId == planner.getStart()) {
                                    status = "Node " + nameOf(nodes, selectedId) + " is already the Start.";
                                }
                                else if (selectedId == planner.getGoal()) {
                                    status = "Node " + nameOf(nodes, selectedId) + " is the Goal node. Change goal first.";
                                    log.add("SETUP: Start and Goal cannot be the same node.");
                                }
                                else {
                                    planner.setStart(selectedId);
                                    status = "Start node changed to " + nameOf(nodes, selectedId) + "; replanned.";
                                    log.add("SETUP: Set Start to " + nameOf(nodes, selectedId));
                                    report("start node changed");
                                }
                            }
                            // Delete Node (Y = 485)
                            else if (clickedBox(m, RIGHT + 20, 485, 170, 28)) {
                                uint64_t targetId = selectedId;
                                // Can't delete start or goal
                                if (targetId == planner.getStart() || targetId == planner.getGoal()) {
                                    status = "Start and Goal nodes cannot be deleted.";
                                    log.add("ERROR: Start/Goal nodes are protected.");
                                }
                                else {
                                    // Remove node
                                    nodes.erase(remove_if(nodes.begin(), nodes.end(), [targetId](const Node& n) { return n.id == targetId; }), nodes.end());
                                    // Remove connected edges
                                    edges.erase(remove_if(edges.begin(), edges.end(), [targetId](const Edge& e) { return e.from == targetId || e.to == targetId; }), edges.end());
                                    
                                    planner.resetStructuresOnly();
                                    selectedType = NONE;
                                    selectedId = -1;
                                    edgeSourceNodeId = -1;
                                    status = "Node and its edges deleted; replanned.";
                                    log.add("NODE: Deleted node ID " + to_string(targetId));
                                    report("node deleted");
                                }
                            }
                            // Set as Edge Source (Y = 485)
                            else if (clickedBox(m, RIGHT + 210, 485, 190, 28)) {
                                edgeSourceNodeId = selectedId;
                                status = "Selected node set as edge source. Click target node to create edge.";
                                log.add("SETUP: Edge source node set to " + nameOf(nodes, edgeSourceNodeId));
                            }
                        }
                        else if (selectedType == EDGE && selectedId != -1) {
                            // Edge Cost [-] and [+] (Y = 380)
                            if (clickedBox(m, RIGHT + 280, 380, 30, 25)) {
                                planner.changeCost(selectedId, -1.0);
                                status = "Selected edge cost decreased; replanned.";
                                report("edge cost decrease");
                            }
                            else if (clickedBox(m, RIGHT + 320, 380, 30, 25)) {
                                planner.changeCost(selectedId, 1.0);
                                status = "Selected edge cost increased; replanned.";
                                report("edge cost increase");
                            }
                            // Edge Reliability [-] and [+] (Y = 420)
                            else if (clickedBox(m, RIGHT + 280, 420, 30, 25)) {
                                planner.changeReliability(selectedId, -0.05);
                                status = "Selected edge reliability decreased; replanned.";
                                report("edge reliability decrease");
                            }
                            else if (clickedBox(m, RIGHT + 320, 420, 30, 25)) {
                                planner.changeReliability(selectedId, 0.05);
                                status = "Selected edge reliability increased; replanned.";
                                report("edge reliability increase");
                            }
                            // Toggle Edge Availability (Y = 465)
                            else if (clickedBox(m, RIGHT + 20, 465, 180, 30)) {
                                planner.toggleEdge(selectedId);
                                status = "Edge availability toggled; replanned.";
                                report("edge availability toggle");
                            }
                            // Delete Edge (Y = 465)
                            else if (clickedBox(m, RIGHT + 220, 465, 180, 30)) {
                                uint64_t targetId = selectedId;
                                edges.erase(remove_if(edges.begin(), edges.end(), [targetId](const Edge& e) { return e.id == targetId; }), edges.end());
                                planner.resetStructuresOnly();
                                selectedType = NONE;
                                selectedId = -1;
                                status = "Edge deleted; replanned.";
                                log.add("EDGE: Deleted edge ID " + to_string(targetId));
                                report("edge deleted");
                            }
                        }

                        // --- Graph Operations Buttons (Y = 560 to 780) ---
                        // Add Node (Y = 590)
                        if (clickedBox(m, RIGHT + 20, 590, 180, 35)) {
                            uint64_t newId = 0;
                            for (const auto& n : nodes) {
                                if (n.id >= newId) newId = n.id + 1;
                            }
                            string labelStr = "";
                            for (char c = 'A'; c <= 'Z'; ++c) {
                                string candidate(1, c);
                                bool taken = false;
                                for (const auto& n : nodes) {
                                    if (n.label == candidate) {
                                        taken = true;
                                        break;
                                    }
                                }
                                if (!taken) {
                                    labelStr = candidate;
                                    break;
                                }
                            }
                            if (labelStr.empty()) {
                                labelStr = "N" + to_string(newId);
                            }

                            // Find a free visual spot on the canvas
                            float rx = 100.f + (rand() % 650);
                            float ry = 100.f + (rand() % 450);
                            bool spotFree = false;
                            int retries = 0;
                            while (!spotFree && retries < 100) {
                                spotFree = true;
                                for (const auto& n : nodes) {
                                    if (hypot(rx - n.pos.x, ry - n.pos.y) < 80.f) {
                                        spotFree = false;
                                        rx = 100.f + (rand() % 650);
                                        ry = 100.f + (rand() % 450);
                                        break;
                                    }
                                }
                                retries++;
                            }

                            nodes.push_back({ newId, labelStr, {rx, ry}, true });
                            planner.resetStructuresOnly();
                            status = "Node " + labelStr + " added to graph; replanned.";
                            log.add("NODE: Added node " + labelStr + " at (" + num(rx) + ", " + num(ry) + ")");
                            report("node added");
                        }
                        // Add Edge (Y = 590)
                        else if (clickedBox(m, RIGHT + 220, 590, 180, 35) && edgeSourceNodeId != -1 && selectedType == NODE && selectedId != edgeSourceNodeId) {
                            uint64_t targetId = selectedId;
                            // Check if edge already exists
                            bool edgeExists = false;
                            for (const auto& e : edges) {
                                if (e.from == edgeSourceNodeId && e.to == targetId) {
                                    edgeExists = true;
                                    break;
                                }
                            }

                            if (edgeExists) {
                                status = "Edge already exists between selected nodes.";
                                log.add("ERROR: Duplicate edge creation blocked.");
                            }
                            else {
                                uint64_t nextEdgeId = 0;
                                for (const auto& e : edges) {
                                    if (e.id >= nextEdgeId) nextEdgeId = e.id + 1;
                                }
                                edges.push_back({ nextEdgeId, (uint64_t)edgeSourceNodeId, targetId, 2.0, true, 0.9 });
                                planner.resetStructuresOnly();
                                status = "Edge created from " + nameOf(nodes, edgeSourceNodeId) + " to " + nameOf(nodes, targetId);
                                log.add("EDGE: Added transition " + nameOf(nodes, edgeSourceNodeId) + " -> " + nameOf(nodes, targetId));
                                edgeSourceNodeId = -1;
                                report("edge added");
                            }
                        }
                        // Reset Setup (Y = 650)
                        else if (clickedBox(m, RIGHT + 20, 650, 180, 35)) {
                            nodes = {
                                {0,"A",{110,390}, true},
                                {1,"B",{300,180}, true},
                                {2,"C",{300,600}, true},
                                {4,"D",{510,180}, true},
                                {5,"E",{510,680}, true},
                                {7,"F",{700,600}, true},
                                {8,"G",{510,390}, true},
                                {6,"H",{800,390}, true}
                            };
                            edges = {
                                {0,0,1,2,true,0.95}, {1,1,4,2,true,0.90}, {2,4,6,2,true,0.85},
                                {3,0,2,4,true,0.80}, {4,2,5,4,true,0.95}, {5,5,6,4,true,0.75},
                                {6,2,7,5,true,0.90}, {7,7,6,5,true,0.85},
                                {8,0,8,3,true,0.99}, {9,8,6,3,true,0.90}
                            };
                            planner.alpha = 1000.0;
                            planner.beta = 1.0;
                            planner.gamma = 2.0;
                            planner.delta = 1.0;
                            planner.safetyThreshold = 80.0;
                            planner.reset();
                            selectedType = NONE;
                            selectedId = -1;
                            edgeSourceNodeId = -1;
                            status = "Graph, weights and path reset to defaults.";
                            log.add("RESET: environment restored.");
                            report("reset");
                        }
                        // Force Replan (Y = 650)
                        else if (clickedBox(m, RIGHT + 220, 650, 180, 35)) {
                            planner.replan();
                            status = "Manual computation triggered successfully.";
                            log.add("D* LITE: recalculation forced.");
                            report("manual replan");
                        }
                    }
                }
                else if (ev.mouseButton.button == sf::Mouse::Right) {
                    // Right click node inside canvas to toggle GOOD/BAD
                    if (m.x < GRAPH) {
                        for (const auto& n : nodes) {
                            double dist = hypot(m.x - n.pos.x, m.y - n.pos.y);
                            if (dist <= 31.0) {
                                if (n.id == planner.getStart() || n.id == planner.getGoal()) {
                                    status = "Start and Goal nodes cannot be marked BAD.";
                                    log.add("NODE: " + n.label + " is protected.");
                                }
                                else {
                                    planner.toggleBad(n.id);
                                    bool isNowBad = planner.isBad(n.id);
                                    selectedId = static_cast<int64_t>(n.id);
                                    selectedType = NODE;
                                    log.add("NODE: " + n.label + (isNowBad ? " marked BAD." : " restored to GOOD."));
                                    status = "Node " + n.label + (isNowBad ? " marked BAD; " : " restored; ") + "replanned.";
                                    report("node state toggle");
                                }
                                break;
                            }
                        }
                    }
                }
            }

            if (ev.type == sf::Event::MouseMoved) {
                sf::Vector2f curMousePos = window.mapPixelToCoords({ ev.mouseMove.x, ev.mouseMove.y });
                if (draggedNodeId != -1) {
                    sf::Vector2f diff = curMousePos - dragStartMousePos;
                    if (hypot(diff.x, diff.y) > 5.f) {
                        hasMovedNode = true;
                    }
                    for (auto& n : nodes) {
                        if (static_cast<int64_t>(n.id) == draggedNodeId) {
                            n.pos = dragStartNodePos + diff;
                            if (n.pos.x < 35.f) n.pos.x = 35.f;
                            if (n.pos.x > GRAPH - 35.f) n.pos.x = GRAPH - 35.f;
                            if (n.pos.y < 35.f) n.pos.y = 35.f;
                            if (n.pos.y > 820.f - 35.f) n.pos.y = 820.f - 35.f;
                            break;
                        }
                    }
                    planner.onWeightChanged();
                }

                if (draggedSliderIdx != -1) {
                    double pct = (curMousePos.x - (RIGHT + 270.f)) / 160.f;
                    if (pct < 0.0) pct = 0.0;
                    if (pct > 1.0) pct = 1.0;
                    
                    if (draggedSliderIdx == 0) planner.alpha = pct * 2000.f;
                    else if (draggedSliderIdx == 1) planner.beta = pct * 10.f;
                    else if (draggedSliderIdx == 2) planner.gamma = pct * 10.f;
                    else if (draggedSliderIdx == 3) planner.delta = pct * 10.f;
                    else if (draggedSliderIdx == 4) planner.safetyThreshold = pct * 300.f;
                    
                    planner.onWeightChanged();
                }
            }

            if (ev.type == sf::Event::MouseButtonReleased && ev.mouseButton.button == sf::Mouse::Left) {
                if (draggedSliderIdx != -1) {
                    string sname = "?";
                    string reportReason = "?";
                    double val = 0.0;
                    if (draggedSliderIdx == 0) { sname = "Alpha (Goal)"; reportReason = "Alpha change"; val = planner.alpha; }
                    else if (draggedSliderIdx == 1) { sname = "Beta (Cost)"; reportReason = "Beta change"; val = planner.beta; }
                    else if (draggedSliderIdx == 2) { sname = "Gamma (Safety)"; reportReason = "Gamma change"; val = planner.gamma; }
                    else if (draggedSliderIdx == 3) { sname = "Delta (Reliability)"; reportReason = "Delta change"; val = planner.delta; }
                    else if (draggedSliderIdx == 4) { sname = "Safety Threshold"; reportReason = "Safety Threshold change"; val = planner.safetyThreshold; }
                    
                    log.add("SETUP: " + sname + " changed to " + num(val));
                    status = sname + " updated to " + num(val) + "; replanned.";
                    report(reportReason);
                    draggedSliderIdx = -1;
                }

                if (draggedNodeId != -1) {
                    if (!hasMovedNode) {
                        if (wasNodeAlreadySelected) {
                            planner.toggleAvailable(draggedNodeId);
                            bool nowAvail = !planner.isUnavailable(draggedNodeId);
                            string label = "?";
                            for (const auto& n : nodes) {
                                if (static_cast<int64_t>(n.id) == draggedNodeId) {
                                    label = n.label;
                                    break;
                                }
                            }
                            log.add("NODE: " + label + (nowAvail ? " set AVAILABLE." : " set UNAVAILABLE."));
                            status = "Node " + label + (nowAvail ? " available; " : " unavailable; ") + "replanned.";
                            report("node availability toggle");
                        } else {
                            string label = "?";
                            for (const auto& n : nodes) {
                                if (static_cast<int64_t>(n.id) == draggedNodeId) {
                                    label = n.label;
                                    break;
                                }
                            }
                            status = "Selected Node " + label + ".";
                            log.add("SETUP: Selected Node " + label);
                        }
                    }
                    else {
                        string label = "?";
                        for (const auto& n : nodes) {
                            if (static_cast<int64_t>(n.id) == draggedNodeId) {
                                label = n.label;
                                break;
                            }
                        }
                        log.add("NODE: Moved node " + label + " to new position.");
                        status = "Node " + label + " repositioned; replanned.";
                        report("node move");
                    }
                    draggedNodeId = -1;
                    hasMovedNode = false;
                }
            }
        }

        vector<uint64_t> p = planner.path();
        double cost = planner.pathCost();
        double minSafety = planner.pathMinSafety();
        double pathRel = planner.pathReliability();
        double pathScr = planner.getPathScore();

        // 1. Draw Canvas background
        window.clear(sf::Color(10, 12, 18));

        sf::RectangleShape graphBg({ GRAPH, 820 });
        graphBg.setFillColor(sf::Color(14, 16, 23));
        window.draw(graphBg);

        // Sidebar Divider
        sf::RectangleShape divider({ 2.f, 820.f });
        divider.setPosition(GRAPH, 0);
        divider.setFillColor(sf::Color(55, 65, 81));
        window.draw(divider);

        // Canvas Header
        text(window, font, "D* LITE PLANNER", 24, sf::Color(243, 244, 246), 30, 20);
        text(window, font, "Multi-Objective Planning & Incremental Replanning Visualizer", 13,
            sf::Color(156, 163, 175), 31, 52);

        // Render Canvas Connections (Directed Edges with Arrowheads)
        for (size_t i = 0; i < edges.size(); ++i) {
            const auto& e = edges[i];
            auto a = posOf(nodes, e.from);
            auto b = posOf(nodes, e.to);
            auto d = b - a;
            float len = sqrt(d.x * d.x + d.y * d.y);
            if (len < 1e-3f) continue;

            sf::Vector2f unitD = d / len;
            sf::Vector2f perpD(-unitD.y, unitD.x);

            bool isPath = hasEdge(p, e.from, e.to);
            float thick = isPath ? 7.f : 3.f;

            // Determine Edge Color
            sf::Color edgeColor = sf::Color(75, 85, 99); // Idle gray
            if (!e.available)
                edgeColor = sf::Color(239, 68, 68); // Red unavailable
            else if (isPath)
                edgeColor = sf::Color(16, 185, 129); // Green path
            if (selectedType == EDGE && static_cast<int64_t>(e.id) == selectedId)
                edgeColor = sf::Color(245, 158, 11); // Selected highlight orange

            // Main Line Body
            sf::RectangleShape line({ len - 31.f, thick });
            line.setOrigin(0, thick / 2.f);
            line.setPosition(a);
            line.setRotation(atan2(d.y, d.x) * 180.f / 3.14159265f);
            line.setFillColor(edgeColor);
            window.draw(line);

            // Directed Arrowhead pointing towards target node (radius 30)
            float arrowLen = isPath ? 14.f : 11.f;
            float arrowHalfW = isPath ? 7.f : 5.5f;
            sf::Vector2f tip = b - unitD * 31.f;
            sf::Vector2f leftWing = tip - unitD * arrowLen + perpD * arrowHalfW;
            sf::Vector2f rightWing = tip - unitD * arrowLen - perpD * arrowHalfW;

            sf::ConvexShape arrow(3);
            arrow.setPoint(0, tip);
            arrow.setPoint(1, leftWing);
            arrow.setPoint(2, rightWing);
            arrow.setFillColor(edgeColor);
            window.draw(arrow);

            // Display cost and reliability on the transition midpoint
            ostringstream cs;
            cs << fixed << setprecision(1) << e.cost << " (R:" << setprecision(2) << e.reliability << ")";
            auto mid = (a + b) / 2.f;

            text(window, font, cs.str(), 11,
                e.available ? sf::Color(209, 213, 219) : sf::Color(248, 113, 113),
                mid.x + 8, mid.y - 18);
        }

        // Render Canvas States (Nodes)
        for (const auto& n : nodes) {
            sf::Color c;
            sf::Color borderC = sf::Color(243, 244, 246);

            if (n.id == planner.getStart())
                c = sf::Color(37, 99, 235); // Blue start
            else if (n.id == planner.getGoal())
                c = sf::Color(217, 119, 6); // Orange/Amber goal
            else if (planner.isBad(n.id)) {
                c = sf::Color(220, 38, 38); // Red BAD state
                borderC = sf::Color(254, 202, 202);
            }
            else if (planner.isUnavailable(n.id)) {
                c = sf::Color(75, 85, 99); // Dark Gray unavailable
                borderC = sf::Color(107, 114, 128);
            }
            else if (hasNode(p, n.id))
                c = sf::Color(5, 150, 105); // Green path node
            else
                c = sf::Color(31, 41, 55); // Standard node color

            if (selectedType == NODE && static_cast<int64_t>(n.id) == selectedId)
                borderC = sf::Color(245, 158, 11); // Highlight selected orange

            sf::CircleShape circle(30);
            circle.setOrigin(30, 30);
            circle.setPosition(n.pos);
            circle.setFillColor(c);
            circle.setOutlineThickness(2.5f);
            circle.setOutlineColor(borderC);
            window.draw(circle);

            // Print Label and ID
            text(window, font, n.label, 20, sf::Color::White, n.pos.x, n.pos.y - 12, true);
            text(window, font, "ID " + to_string(n.id), 10, sf::Color(229, 231, 235), n.pos.x, n.pos.y + 12, true);

            // Print D* Lite algorithm state and safety distance below the node
            string gValStr = num(planner.getG(n.id));
            string rhsValStr = num(planner.getRhs(n.id));
            string sValStr = num(planner.getSafety(n.id));

            text(window, font, "g:" + gValStr + " / r:" + rhsValStr, 10, sf::Color(156, 163, 175), n.pos.x, n.pos.y + 35, true);
            text(window, font, "Safety Dist: " + sValStr, 10, sf::Color(156, 163, 175), n.pos.x, n.pos.y + 48, true);
        }

        // Render Canvas Quick Guide panel
        sf::RectangleShape guidePanel({ 845.f, 90.f });
        guidePanel.setPosition(25, 705);
        guidePanel.setFillColor(sf::Color(17, 24, 39));
        guidePanel.setOutlineThickness(1.5f);
        guidePanel.setOutlineColor(sf::Color(55, 65, 81));
        window.draw(guidePanel);

        text(window, font, "CONTROLS QUICK REFERENCE", 13, sf::Color(243, 244, 246), 40, 715);
        text(window, font, "Left-click Node: toggle AVAILABLE/UNAVAILABLE", 11, sf::Color(209, 213, 219), 40, 740);
        text(window, font, "Right-click Node: toggle GOOD / BAD state", 11, sf::Color(209, 213, 219), 40, 765);
        text(window, font, "Left-click Edge: toggle AVAILABLE/BLOCKED", 11, sf::Color(209, 213, 219), 420, 740);
        text(window, font, "Shift + Edge click: cost +1.0  |  Ctrl + Edge click: cost -1.0", 11, sf::Color(209, 213, 219), 420, 765);

        // Draw Tab Headers
        drawButton(window, font, "DASHBOARD", RIGHT, 15, 215, 35,
            currentTab == DASHBOARD ? sf::Color(37, 99, 235) : sf::Color(31, 41, 55),
            sf::Color(29, 78, 216), sf::Color::White, m, true);

        drawButton(window, font, "SETUP SCREEN", RIGHT + 225, 15, 215, 35,
            currentTab == SETUP ? sf::Color(37, 99, 235) : sf::Color(31, 41, 55),
            sf::Color(29, 78, 216), sf::Color::White, m, true);

        // Sidebar content based on selected tab
        if (currentTab == DASHBOARD) {
            // Live status card
            sf::RectangleShape card({ RW, 180 });
            card.setPosition(RIGHT, 70);
            card.setFillColor(sf::Color(17, 24, 39));
            card.setOutlineThickness(1.5f);
            card.setOutlineColor(sf::Color(55, 65, 81));
            window.draw(card);

            text(window, font, "CURRENT OPTIMAL PATH", 13, sf::Color(156, 163, 175), RIGHT + 15, 82);
            
            // Limit text size for optimal paths that are long
            string pathString = pathText(nodes, p);
            unsigned pathTextSize = 16;
            if (pathString.size() > 40) pathTextSize = 13;
            if (pathString.size() > 60) pathTextSize = 11;

            text(window, font, pathString, pathTextSize,
                p.empty() ? sf::Color(239, 68, 68) : sf::Color(16, 185, 129),
                RIGHT + 15, 108);

            text(window, font, "PATH SCORE METRICS BREAKDOWN:", 11, sf::Color(156, 163, 175), RIGHT + 15, 142);
            text(window, font, "Score: " + num(pathScr), 14, sf::Color(243, 244, 246), RIGHT + 15, 162);
            text(window, font, "Cost (C): " + num(cost), 12, sf::Color(209, 213, 219), RIGHT + 15, 187);
            text(window, font, "Min Safety (D): " + num(minSafety), 12, sf::Color(209, 213, 219), RIGHT + 160, 187);
            text(window, font, "Reliability (R): " + num(pathRel), 12, sf::Color(209, 213, 219), RIGHT + 310, 187);
            text(window, font, "Goal Reached (G): " + string(p.empty() ? "No (0)" : "Yes (1)"), 11, sf::Color(156, 163, 175), RIGHT + 15, 215);

            // "What just happened" Card
            sf::RectangleShape statusCard({ RW, 110 });
            statusCard.setPosition(RIGHT, 265);
            statusCard.setFillColor(sf::Color(17, 24, 39));
            statusCard.setOutlineThickness(1.5f);
            statusCard.setOutlineColor(sf::Color(55, 65, 81));
            window.draw(statusCard);

            text(window, font, "PLANNER STATUS UPDATE", 13, sf::Color(156, 163, 175), RIGHT + 15, 277);

            // Text wrap status
            string s = status;
            vector<string> wrapped;
            while (s.size() > 50) {
                size_t cut = s.rfind(' ', 50);
                if (cut == string::npos) cut = 50;
                wrapped.push_back(s.substr(0, cut));
                s.erase(0, cut);
                while (!s.empty() && s[0] == ' ') s.erase(0, 1);
            }
            wrapped.push_back(s);

            for (size_t i = 0; i < wrapped.size() && i < 2; ++i)
                text(window, font, wrapped[i], 13, sf::Color(243, 244, 246),
                    RIGHT + 15.f, 305.f + static_cast<float>(i) * 22.f);

            log.draw(window, font, RIGHT, 390, RW, 410);
        }
        else {
            // SETUP TAB ACTIVE
            // 1. Weighting Factors Card
            sf::RectangleShape wCard({ RW, 250 });
            wCard.setPosition(RIGHT, 65);
            wCard.setFillColor(sf::Color(17, 24, 39));
            wCard.setOutlineThickness(1.5f);
            wCard.setOutlineColor(sf::Color(55, 65, 81));
            window.draw(wCard);

            text(window, font, "MULTI-OBJECTIVE CONFIGURATION", 13, sf::Color(156, 163, 175), RIGHT + 15, 77);

            string labels[5] = { 
                "Alpha (Goal Completion):", 
                "Beta (Transition Cost):", 
                "Gamma (Safety Distance):", 
                "Delta (Reliability):",
                "Safety Threshold (Min S):"
            };
            double vals[5] = { 
                planner.alpha, 
                planner.beta, 
                planner.gamma, 
                planner.delta,
                planner.safetyThreshold
            };

            for (int i = 0; i < 5; ++i) {
                float yOffset = static_cast<float>(i) * 36.f;
                float sliderCenterY = 112.f + yOffset;
                
                // Draw Label & Value
                text(window, font, labels[i], 12, sf::Color(209, 213, 219), RIGHT + 15.f, 105.f + yOffset);
                text(window, font, num(vals[i]), 13, sf::Color(243, 244, 246), RIGHT + 210.f, 105.f + yOffset);

                // Draw Slider Track Line
                sf::RectangleShape track({ 160.f, 4.f });
                track.setOrigin(0.f, 2.f);
                track.setPosition(RIGHT + 270.f, sliderCenterY);
                track.setFillColor(sf::Color(75, 85, 99));
                window.draw(track);

                // Calculate Handle Position
                double range = 1.0;
                if (i == 0) range = 2000.0;
                else if (i == 1 || i == 2 || i == 3) range = 10.0;
                else if (i == 4) range = 300.0;
                
                double pct = vals[i] / range;
                if (pct < 0.0) pct = 0.0;
                if (pct > 1.0) pct = 1.0;
                
                float handleX = RIGHT + 270.f + static_cast<float>(pct) * 160.f;

                // Draw Handle Circle
                sf::CircleShape handle(7.f);
                handle.setOrigin(7.f, 7.f);
                handle.setPosition(handleX, sliderCenterY);
                
                // Coloring handle on drag/hover
                bool isDragged = (draggedSliderIdx == i);
                sf::RectangleShape boundsRect({ 170.f, 24.f });
                boundsRect.setPosition(RIGHT + 265.f, 100.f + yOffset);
                bool isHovered = boundsRect.getGlobalBounds().contains(m);
                
                if (isDragged) {
                    handle.setFillColor(sf::Color(245, 158, 11)); // Selected/dragged amber
                } else if (isHovered) {
                    handle.setFillColor(sf::Color(96, 165, 250)); // Hovered light blue
                } else {
                    handle.setFillColor(sf::Color(37, 99, 235)); // Regular blue
                }
                
                window.draw(handle);
            }

            // 2. Selected Element Details
            sf::RectangleShape selCard({ RW, 220 });
            selCard.setPosition(RIGHT, 325);
            selCard.setFillColor(sf::Color(17, 24, 39));
            selCard.setOutlineThickness(1.5f);
            selCard.setOutlineColor(sf::Color(55, 65, 81));
            window.draw(selCard);

            text(window, font, "SELECTED GRAPH ELEMENT CONFIGURATOR", 13, sf::Color(156, 163, 175), RIGHT + 15, 337);

            if (selectedType == NONE || selectedId == -1) {
                text(window, font, "No element selected.", 12, sf::Color(156, 163, 175), RIGHT + 15, 370);
                text(window, font, "Click a state (Node) or connection (Edge) in the", 12, sf::Color(156, 163, 175), RIGHT + 15, 395);
                text(window, font, "left canvas to edit properties, delete, or link elements.", 12, sf::Color(156, 163, 175), RIGHT + 15, 420);
            }
            else if (selectedType == NODE) {
                const Node* n = findNode(nodes, selectedId);
                if (n) {
                    string roleStr = "";
                    if (selectedId == planner.getStart()) roleStr = " [START NODE]";
                    else if (selectedId == planner.getGoal()) roleStr = " [GOAL NODE]";
                    text(window, font, "Node: " + n->label + " (ID " + to_string(n->id) + ")" + roleStr, 13, sf::Color(243, 244, 246), RIGHT + 15, 355);
                    
                    bool isAvail = !planner.isUnavailable(selectedId);
                    bool isB = planner.isBad(selectedId);

                    // Row 1: Availability & State (Y = 385, height 28)
                    drawButton(window, font, isAvail ? "Set UNAVAILABLE" : "Set AVAILABLE", RIGHT + 20, 385, 180, 28,
                        isAvail ? sf::Color(5, 150, 105) : sf::Color(55, 65, 81), sf::Color(4, 120, 87), sf::Color::White, m, (selectedId != planner.getStart() && selectedId != planner.getGoal()));

                    drawButton(window, font, isB ? "Set GOOD State" : "Set BAD State", RIGHT + 220, 385, 180, 28,
                        isB ? sf::Color(220, 38, 38) : sf::Color(5, 150, 105), sf::Color(185, 28, 28), sf::Color::White, m, (selectedId != planner.getStart() && selectedId != planner.getGoal()));

                    // Row 2: Set Goal / Set Start (Y = 422, height 28)
                    bool isGoal = (selectedId == planner.getGoal());
                    bool isStart = (selectedId == planner.getStart());

                    drawButton(window, font, isGoal ? "Current Goal" : "Set as Goal State", RIGHT + 20, 422, 180, 28,
                        isGoal ? sf::Color(180, 83, 9) : sf::Color(217, 119, 6), sf::Color(245, 158, 11), sf::Color::White, m, !isGoal);

                    drawButton(window, font, isStart ? "Current Start" : "Set as Start State", RIGHT + 220, 422, 180, 28,
                        isStart ? sf::Color(30, 64, 175) : sf::Color(37, 99, 235), sf::Color(59, 130, 246), sf::Color::White, m, !isStart);

                    // Row 3: Safety Distance (Y = 460)
                    text(window, font, "Safety Dist to BAD states: " + num(planner.getSafety(selectedId)), 11, sf::Color(209, 213, 219), RIGHT + 15, 460);

                    // Row 4: Delete Node & Set Edge Source (Y = 485, height 28)
                    drawButton(window, font, "Delete Node", RIGHT + 20, 485, 170, 28,
                        sf::Color(185, 28, 28), sf::Color(220, 38, 38), sf::Color::White, m, (!isStart && !isGoal));

                    drawButton(window, font, "Set as Edge Source", RIGHT + 210, 485, 190, 28,
                        sf::Color(75, 85, 99), sf::Color(107, 114, 128), sf::Color::White, m, true);
                }
            }
            else if (selectedType == EDGE) {
                // Find selected edge
                Edge* selectedEdgePtr = nullptr;
                for (auto& e : edges) {
                    if (e.id == (uint64_t)selectedId) {
                        selectedEdgePtr = &e;
                        break;
                    }
                }

                if (selectedEdgePtr) {
                    string fromLabel = nameOf(nodes, selectedEdgePtr->from);
                    string toLabel = nameOf(nodes, selectedEdgePtr->to);
                    text(window, font, "Edge: " + fromLabel + " -> " + toLabel, 13, sf::Color(243, 244, 246), RIGHT + 15, 355);

                    // Cost configure
                    text(window, font, "Transition Cost: " + num(selectedEdgePtr->cost), 12, sf::Color(209, 213, 219), RIGHT + 15, 385);
                    drawButton(window, font, "-", RIGHT + 280, 380, 30, 25, sf::Color(31, 41, 55), sf::Color(55, 65, 81), sf::Color::White, m, true);
                    drawButton(window, font, "+", RIGHT + 320, 380, 30, 25, sf::Color(31, 41, 55), sf::Color(55, 65, 81), sf::Color::White, m, true);

                    // Reliability configure
                    text(window, font, "Reliability: " + num(selectedEdgePtr->reliability), 12, sf::Color(209, 213, 219), RIGHT + 15, 425);
                    drawButton(window, font, "-", RIGHT + 280, 420, 30, 25, sf::Color(31, 41, 55), sf::Color(55, 65, 81), sf::Color::White, m, true);
                    drawButton(window, font, "+", RIGHT + 320, 420, 30, 25, sf::Color(31, 41, 55), sf::Color(55, 65, 81), sf::Color::White, m, true);

                    // Availability toggle
                    bool isAvail = selectedEdgePtr->available;
                    drawButton(window, font, isAvail ? "Block Edge" : "Enable Edge", RIGHT + 20, 465, 180, 30,
                        isAvail ? sf::Color(220, 38, 38) : sf::Color(5, 150, 105), sf::Color(185, 28, 28), sf::Color::White, m, true);

                    // Delete Edge
                    drawButton(window, font, "Delete Edge", RIGHT + 220, 465, 180, 30,
                        sf::Color(185, 28, 28), sf::Color(220, 38, 38), sf::Color::White, m, true);
                }
            }

            // 3. Graph Operations Card
            sf::RectangleShape opsCard({ RW, 240 });
            opsCard.setPosition(RIGHT, 560);
            opsCard.setFillColor(sf::Color(17, 24, 39));
            opsCard.setOutlineThickness(1.5f);
            opsCard.setOutlineColor(sf::Color(55, 65, 81));
            window.draw(opsCard);

            text(window, font, "ENVIRONMENT & PLANNING OPERATIONS", 13, sf::Color(156, 163, 175), RIGHT + 15, 572);

            // Add Node
            drawButton(window, font, "Add Node", RIGHT + 20, 595, 180, 35,
                sf::Color(5, 150, 105), sf::Color(4, 120, 87), sf::Color::White, m, true);

            // Add Edge (Requires source to be set, and a target selected that is a different Node)
            bool canAddEdge = (edgeSourceNodeId != -1 && selectedType == NODE && selectedId != edgeSourceNodeId);
            string addEdgeText = "Add Edge";
            if (edgeSourceNodeId != -1) {
                const Node* srcNode = findNode(nodes, edgeSourceNodeId);
                if (srcNode) {
                    if (selectedType == NODE && selectedId != edgeSourceNodeId) {
                        const Node* dstNode = findNode(nodes, selectedId);
                        if (dstNode) {
                            addEdgeText = "Link " + srcNode->label + " -> " + dstNode->label;
                        }
                    } else {
                        addEdgeText = "Link from " + srcNode->label + "...";
                    }
                }
            }
            
            drawButton(window, font, addEdgeText, RIGHT + 220, 595, 180, 35,
                canAddEdge ? sf::Color(37, 99, 235) : sf::Color(31, 41, 55),
                sf::Color(29, 78, 216), sf::Color::White, m, canAddEdge);

            // Reset Setup
            drawButton(window, font, "Reset Setup", RIGHT + 20, 645, 180, 35,
                sf::Color(217, 119, 6), sf::Color(180, 83, 9), sf::Color::White, m, true);

            // Force Replan
            drawButton(window, font, "Force Replan", RIGHT + 220, 645, 180, 35,
                sf::Color(107, 114, 128), sf::Color(75, 85, 99), sf::Color::White, m, true);

            if (edgeSourceNodeId != -1) {
                text(window, font, "Click target Node on canvas to enable 'Link' button.", 11, sf::Color(156, 163, 175), RIGHT + 20, 695);
            }
            else {
                text(window, font, "Select a Node -> click 'Set as Edge Source' to draw edges.", 11, sf::Color(156, 163, 175), RIGHT + 20, 695);
            }

            text(window, font, "Reset restores original nodes, default weights and optimal path.", 10, sf::Color(156, 163, 175), RIGHT + 20, 715);
            
            // Draw status text inside Setup panel
            text(window, font, "Status: " + status.substr(0, min((size_t)60, status.size())), 11, sf::Color(243, 244, 246), RIGHT + 20, 735);
        }

        window.display();
    }

    return 0;
}
