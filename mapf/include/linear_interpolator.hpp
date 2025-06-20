#pragma once

#include <vector>
#include <memory>
#include "node.hpp" // 상대 경로로 맞추세요 (예: "../include/node.hpp" → "node.hpp")

// 그래프 기반 보간을 위해 포워드 선언
class Graph;

namespace interpolator {

class LinearInterpolator {
public:
// 보간 단위 해상도 (예: 0.1 = 10Hz)
LinearInterpolator(double resolution = 0.1);

// 각 agent별 원래 경로를 받아 보간된 경로 반환 (그래프 필요)
std::vector<std::vector<std::shared_ptr<Node>>> interpolate(
const std::vector<std::vector<Node*>>& original_paths, Graph* graph);

// 경로 균등 분할 - 주어진 세그먼트당 divisions_per_segment개 노드 생성 (그래프 필요)
std::vector<std::shared_ptr<Node>> dividePathEvenly(
const std::vector<Node*>& original_path, int divisions_per_segment, Graph* graph);

private:
double resolution_; // 타임스텝 당 세분화 해상도
};

} // namespace interpolator