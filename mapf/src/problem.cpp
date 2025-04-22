#include "../include/problem.hpp"

#include <fstream>
#include <regex>

#include "../include/util.hpp"

Problem::Problem(const std::string& _instance)
    : instance(_instance), instance_initialized(true)
{
  // read map from instance file
  std::ifstream file(instance);
  if (!file) halt("file " + instance + " is not found.");

  std::string line;
  std::smatch results;
  std::regex r_map = std::regex(R"(map_file=(.+))");

  while (getline(file, line)) {
    if (*(line.end() - 1) == 0x0d) line.pop_back();
    if (std::regex_match(line, results, r_map)) {
      G = new Grid(results[1].str());
      break;
    }
  }

  // 하드코딩 설정
  num_agents = 2;

  // 하드코딩된 시작/목표 위치 (빈 공간 10x10x10)
  Node* s1 = G->getNode(0, 0, 0);     // Agent 0 start
  Node* g1 = G->getNode(9, 9, 9);     // Agent 0 goal
  Node* s2 = G->getNode(9, 0, 0);     // Agent 1 start
  Node* g2 = G->getNode(0, 9, 9);     // Agent 1 goal

  if (!s1 || !g1 || !s2 || !g2) {
    halt("Hardcoded node(s) do not exist. Check map size and coordinates.");
  }

  config_s.push_back(s1);
  config_g.push_back(g1);
  config_s.push_back(s2);
  config_g.push_back(g2);

  // 기본값 설정
  MT = new std::mt19937(DEFAULT_SEED);
  max_timestep = DEFAULT_MAX_TIMESTEP;
  max_comp_time = DEFAULT_MAX_COMP_TIME;

  // trimming (여기선 사실 필요 없음)
  config_s.resize(num_agents);
  config_g.resize(num_agents);
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
  // initialize
  config_s.clear();
  config_g.clear();

  // get grid size
  Grid* grid = reinterpret_cast<Grid*>(G);
  const int N = grid->getWidth() * grid->getHeight() * grid->getDepth();  // 3D 환경에 맞게 수정

  // set starts
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

  // set goals
  std::vector<int> goals(N);
  std::iota(goals.begin(), goals.end(), 0);
  std::shuffle(goals.begin(), goals.end(), *MT);
  int j = 0;
  while (true) {
    while (G->getNode(goals[j]) == nullptr) {
      ++j;
      if (j >= N) halt("set goal, number of agents is too large.");
    }
    // retry if goal is same as start
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
}

void Problem::setWellFormedInstance()
{
  // initialize
  config_s.clear();
  config_g.clear();

  // get grid size
  const int N = G->getNodesSize();
  Nodes prohibited, starts_goals;

  while ((int)config_g.size() < getNum()) {
    while (true) {
      // determine start (3D 환경에 맞게 수정)
      Node* s;
      do {
        s = G->getNode(getRandomInt(0, N - 1, MT));
      } while (s == nullptr || inArray(s, prohibited));

      // determine goal (3D 환경에 맞게 수정)
      Node* g;
      do {
        g = G->getNode(getRandomInt(0, N - 1, MT));
      } while (g == nullptr || g == s || inArray(g, prohibited));

      // ensure well formed property
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
}

// I know that using "const" is something wired...
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
}
