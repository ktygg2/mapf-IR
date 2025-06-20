#include "../include/problem.hpp"

#include <fstream>
#include <regex>
#include <sstream>

#include "../include/util.hpp"

Problem::Problem(const std::string& _instance)
    : instance(_instance), instance_initialized(true)
{
  std::ifstream file(instance);
  if (!file) halt("file " + instance + " is not found.");

  std::string line;
  std::regex r_map(R"(map_file=(.+))");
  std::regex r_agents(R"(agents=(\d+))");
  
  bool map_loaded = false;
  bool agent_section = false;

  while (getline(file, line)) {
    if (*(line.end() - 1) == 0x0d) line.pop_back(); // remove \r for Windows

    std::smatch results;
    if (std::regex_match(line, results, r_map)) {
      G = new Grid(results[1].str());
      map_loaded = true;
      std::cout << "Map file loaded: " << results[1].str() << std::endl;
    } else if (std::regex_match(line, results, r_agents)) {
      num_agents = std::stoi(results[1]);
      agent_section = true;
      std::cout << "Number of agents: " << num_agents << std::endl;
    } else if (agent_section && !line.empty()) {
      std::stringstream ss(line);
      int sx, sy, sz, gx, gy, gz;
      if (ss >> sx >> sy >> sz >> gx >> gy >> gz) {
        Node* start = G->getNode(sx, sy, sz);
        Node* goal = G->getNode(gx, gy, gz);

        if (!start || !goal) {
          std::cout << "Invalid node - start: (" << sx << "," << sy << "," << sz << ") or goal: (" << gx << "," << gy << "," << gz << ")" << std::endl;
          halt("Start or goal node not found on the map. Check coordinates.");
        }

        config_s.push_back(start);
        config_g.push_back(goal);
        std::cout << "Parsed Start: (" << sx << "," << sy << "," << sz << ") -> Goal: (" << gx << "," << gy << "," << gz << ")" << std::endl;
      }
    }
  }

  if (!map_loaded || num_agents == 0 || config_s.size() != (size_t)num_agents) {
    halt("Failed to initialize problem from instance file. Check format.");
  }

  MT = new std::mt19937(DEFAULT_SEED);
  max_timestep = DEFAULT_MAX_TIMESTEP;
  max_comp_time = DEFAULT_MAX_COMP_TIME;
  std::cout << "Problem initialized with " << num_agents << " agents." << std::endl;
}

Problem::Problem(Problem* P, Config _config_s, Config _config_g,
                 int _max_comp_time, int _max_timestep)
    : G(P->getG()),
      MT(P->getMT()),
      config_s(_config_s),
      config_g(_config_g),
      num_agents(P->getNum()),
      max_timestep(_max_timestep),
      max_comp_time(_max_comp_time),
      instance_initialized(false)
{
}

Problem::Problem(Problem* P, int _max_comp_time)
    : G(P->getG()),
      MT(P->getMT()),
      config_s(P->getConfigStart()),
      config_g(P->getConfigGoal()),
      num_agents(P->getNum()),
      max_timestep(P->getMaxTimestep()),
      max_comp_time(_max_comp_time),
      instance_initialized(false)
{
}

Problem::~Problem()
{
  if (instance_initialized) {
    if (G != nullptr) delete G;
    if (MT != nullptr) delete MT;
  }
}

Node* Problem::getStart(int i) const
{
  if (!(0 <= i && i < (int)config_s.size())) halt("invalid index");
  return config_s[i];
}

Node* Problem::getGoal(int i) const
{
  if (!(0 <= i && i < (int)config_g.size())) halt("invalid index");
  return config_g[i];
}

void Problem::setRandomStartsGoals()
{
  config_s.clear();
  config_g.clear();
  Grid* grid = reinterpret_cast<Grid*>(G);
  const int N = grid->getWidth() * grid->getHeight() * grid->getDepth();
  std::vector<int> starts(N);
  std::iota(starts.begin(), starts.end(), 0);
  std::shuffle(starts.begin(), starts.end(), *MT);
  int i = 0;
  while (true) {
    while (G->getNode(starts[i]) == nullptr) {
      ++i;
      if (i >= N) halt("number of agents is too large.");
    }
    config_s.push_back(G->getNode(starts[i]));
    if ((int)config_s.size() == num_agents) break;
    ++i;
  }
  std::vector<int> goals(N);
  std::iota(goals.begin(), goals.end(), 0);
  std::shuffle(goals.begin(), goals.end(), *MT);
  int j = 0;
  while (true) {
    while (G->getNode(goals[j]) == nullptr) {
      ++j;
      if (j >= N) halt("set goal, number of agents is too large.");
    }
    if (G->getNode(goals[j]) == config_s[config_g.size()]) {
      config_g.clear();
      std::shuffle(goals.begin(), goals.end(), *MT);
      j = 0;
      continue;
    }
    config_g.push_back(G->getNode(goals[j]));
    if ((int)config_g.size() == num_agents) break;
    ++j;
  }
  std::cout << "Random starts and goals set." << std::endl;
}

void Problem::setWellFormedInstance()
{
  config_s.clear();
  config_g.clear();
  const int N = G->getNodesSize();
  Nodes prohibited, starts_goals;
  while ((int)config_g.size() < getNum()) {
    while (true) {
      Node* s;
      do {
        s = G->getNode(getRandomInt(0, N - 1, MT));
      } while (s == nullptr || inArray(s, prohibited));
      Node* g;
      do {
        g = G->getNode(getRandomInt(0, N - 1, MT));
      } while (g == nullptr || g == s || inArray(g, prohibited));
      auto path = G->getPath(s, g, starts_goals);
      if (!path.empty()) {
        config_s.push_back(s);
        config_g.push_back(g);
        starts_goals.push_back(s);
        starts_goals.push_back(g);
        for (auto v : path) {
          if (!inArray(v, prohibited)) prohibited.push_back(v);
        }
        break;
      }
    }
  }
  std::cout << "Well-formed instance set." << std::endl;
}

void Problem::halt(const std::string& msg) const
{
  std::cout << "error@Problem: " << msg << std::endl;
  this->~Problem();
  std::exit(1);
}

void Problem::warn(const std::string& msg) const
{
  std::cout << "warn@Problem: " << msg << std::endl;
}

void Problem::makeScenFile(const std::string& output_file)
{
  Grid* grid = reinterpret_cast<Grid*>(G);
  std::ofstream log;
  log.open(output_file, std::ios::out);
  log << "map_file=" << grid->getMapFileName() << "\n";
  log << "agents=" << num_agents << "\n";
  log << "seed=0\n";
  log << "random_problem=0\n";
  log << "max_timestep=" << max_timestep << "\n";
  log << "max_comp_time=" << max_comp_time << "\n";
  for (int i = 0; i < num_agents; ++i) {
    log << config_s[i]->pos.x << "," << config_s[i]->pos.y << "," << config_s[i]->pos.z << ","
        << config_g[i]->pos.x << "," << config_g[i]->pos.y << "," << config_g[i]->pos.z << "\n";
  }
  log.close();
  std::cout << "Scenario file created: " << output_file << std::endl;
}

