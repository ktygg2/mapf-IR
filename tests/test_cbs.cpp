#include <cbs.hpp>

#include "gtest/gtest.h"

TEST(CBS, solve)
{
  Problem P = Problem("../tests/instances/example.txt");
  std::unique_ptr<Solver> solver = std::make_unique<CBS>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));

  // solution 출력 시 x, y, z 좌표를 포함
  const auto& solution = solver->getSolution();  // Plan 객체가 반환됨
for (int i = 0; i < solution.size(); ++i) {
  std::cout << "Agent " << i << " path: ";
  Path path = solution.getPath(i);  // 각 에이전트의 경로 가져오기
  for (const auto& node : path) {
    std::cout << "(" << node->pos.x << ", " << node->pos.y << ", " << node->pos.z << ") ";
  }
  std::cout << "\n";
}

  ASSERT_EQ(solver->getSolution().getSOC(), 635);
}
