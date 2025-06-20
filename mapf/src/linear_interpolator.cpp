#include "linear_interpolator.hpp"
#include "graph.hpp" // Graph 정의를 위해 필요
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>

namespace interpolator {

LinearInterpolator::LinearInterpolator(double resolution)
    : resolution_(resolution) {}

std::vector<std::vector<std::shared_ptr<Node>>> LinearInterpolator::interpolate(
    const std::vector<std::vector<Node*>>& original_paths, Graph* graph) {
    
    std::vector<std::vector<std::shared_ptr<Node>>> interpolated_paths;

    std::cout << "[Debug] 처리 중인 경로 수: " << original_paths.size() << "\n";

    for (const auto& path : original_paths) {
        std::vector<std::shared_ptr<Node>> new_path;
        if (path.empty()) continue;

        std::cout << "\n[Debug] Original Path 시작: (" 
                  << path[0]->pos.x << "," << path[0]->pos.y << "," << path[0]->pos.z << ")"
                  << " → 종료: (" 
                  << path.back()->pos.x << "," << path.back()->pos.y << "," << path.back()->pos.z << ")\n";

        for (size_t i = 0; i + 1 < path.size(); ++i) {
            Node* from = path[i];
            Node* to = path[i + 1];

            double dx = to->pos.x - from->pos.x;
            double dy = to->pos.y - from->pos.y;
            double dz = to->pos.z - from->pos.z;

            double max_step = std::max({std::abs(dx), std::abs(dy), std::abs(dz), 1.0});
            int steps = static_cast<int>(std::ceil(max_step / resolution_));

            std::cout << "[Debug] From (" << from->pos.x << "," << from->pos.y << "," << from->pos.z 
                      << ") → To (" << to->pos.x << "," << to->pos.y << "," << to->pos.z << ")"
                      << " | steps=" << steps << "\n";

            for (int step = 0; step <= steps; ++step) {
                double t = (steps > 0) ? static_cast<double>(step) / steps : 0.0;

                // 반드시 정수 좌표로 반올림
                int x = static_cast<int>(std::round(from->pos.x + t * dx));
                int y = static_cast<int>(std::round(from->pos.y + t * dy));
                int z = static_cast<int>(std::round(from->pos.z + t * dz));

                // 그래프에 실제 존재하는 노드만 추가
                Node* interp_node = graph->getNode(x, y, z);
                if (!interp_node) continue;

                // 중복 노드 방지
                if (new_path.empty() || new_path.back().get() != interp_node) {
                    new_path.push_back(std::shared_ptr<Node>(interp_node, [](Node*){}));
                    std::cout << "[Debug] 보간 노드: (" << x << "," << y << "," << z << ")\n";
                }
            }
        }

        if (!new_path.empty()) {
            std::cout << "[Debug] 최종 보간 경로 길이: " << new_path.size() << "\n";
            std::cout << "[Debug] 보간 경로 시작: (" 
                      << new_path.front()->pos.x << "," 
                      << new_path.front()->pos.y << "," 
                      << new_path.front()->pos.z << ")\n";
            std::cout << "[Debug] 보간 경로 종료: (" 
                      << new_path.back()->pos.x << "," 
                      << new_path.back()->pos.y << "," 
                      << new_path.back()->pos.z << ")\n\n";
        }

        interpolated_paths.push_back(new_path);
    }
    return interpolated_paths;
}

std::vector<std::shared_ptr<Node>> LinearInterpolator::dividePathEvenly(
    const std::vector<Node*>& original_path, int divisions_per_segment, Graph* graph) {
    
    std::vector<std::shared_ptr<Node>> divided_path;
    if (original_path.empty()) return divided_path;

    // 첫 노드: 원본 포인터를 소유권 없이 shared_ptr로 감쌈
    divided_path.push_back(std::shared_ptr<Node>(original_path[0], [](Node*){}));

    for (size_t i = 0; i + 1 < original_path.size(); ++i) {
        Node* from = original_path[i];
        Node* to = original_path[i + 1];

        double dx = to->pos.x - from->pos.x;
        double dy = to->pos.y - from->pos.y;
        double dz = to->pos.z - from->pos.z;

        std::cout << "[Debug] 세그먼트 분할: (" 
                  << from->pos.x << "," << from->pos.y << "," << from->pos.z 
                  << ") → (" << to->pos.x << "," << to->pos.y << "," << to->pos.z 
                  << ") | " << divisions_per_segment << "개로 분할\n";

        // divisions_per_segment만큼 노드 생성 (from, to 제외)
        for (int step = 1; step < divisions_per_segment; ++step) {
            float t = static_cast<float>(step) / divisions_per_segment;

            int x = static_cast<int>(std::round(from->pos.x + t * dx));
            int y = static_cast<int>(std::round(from->pos.y + t * dy));
            int z = static_cast<int>(std::round(from->pos.z + t * dz));

            Node* new_node = graph->getNode(x, y, z);
            if (!new_node) continue;

            // 중복 노드 방지
            if (divided_path.empty() || divided_path.back().get() != new_node) {
                divided_path.push_back(std::shared_ptr<Node>(new_node, [](Node*){}));
                std::cout << "[Debug] 분할 노드: (" << x << "," << y << "," << z << ")\n";
            }
        }

        // 끝 노드: 마지막 세그먼트가 아니면 추가
        if (i != original_path.size() - 2) {
            divided_path.push_back(std::shared_ptr<Node>(to, [](Node*){}));
        }
    }

    // 마지막 노드: 원본 포인터를 소유권 없이 shared_ptr로 감쌈
    divided_path.push_back(std::shared_ptr<Node>(original_path.back(), [](Node*){}));

    return divided_path;
}

} // namespace interpolator
