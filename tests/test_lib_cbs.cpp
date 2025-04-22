#include <graph.hpp>
#include <lib_cbs.hpp>

#include "gtest/gtest.h"

TEST(LibCBS, getFirstConstraints)
{
  // 3D 환경에서 테스트를 위해 적절한 .map3d 파일을 사용
  auto G = Grid("arena_3d.map3d");

  // 좌표 기반으로 노드를 얻는다고 가정 (z=0층)
  Node* n0 = G.getNode(0, 0, 0);
  Node* n1 = G.getNode(0, 1, 0);
  Node* n2 = G.getNode(0, 2, 0);

  // 두 에이전트의 경로 삽입
  auto paths = Paths(2);
  paths.insert(0, {n0, n1, n2});
  paths.insert(1, {n1, n1, n2});  // vertex conflict at time=1, node=n1

  auto constraints = LibCBS::getFirstConstraints(paths);
  ASSERT_EQ((int)constraints.size(), 2);  // 양쪽 에이전트 모두에게 제약

  auto c0 = constraints[0];
  ASSERT_EQ(c0->id, 0);      // agent 0
  ASSERT_EQ(c0->t, 1);       // time step
  ASSERT_EQ(c0->v->id, n1->id); // node ID

  auto c1 = constraints[1];
  ASSERT_EQ(c1->id, 1);      
  ASSERT_EQ(c1->t, 1);       
  ASSERT_EQ(c1->v->id, n1->id);
}
