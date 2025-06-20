#pragma once
#include "problem.hpp"
#include "linear_interpolator.hpp"

// 플랜: 시간에 따른 에이전트들의 위치(config)의 배열
struct Plan {
private:
  Configs configs;  // 시간별 에이전트 위치 (Node* 리스트)

public:
  ~Plan() {}

  // 특정 시간(t)의 전체 configuration 반환
  Config get(const int t) const;

  // 특정 시간(t)에서 특정 에이전트(i)의 위치 반환
  Node* get(const int t, const int i) const;

  // 특정 에이전트(i)의 전체 경로(Path) 반환
  Path getPath(const int i) const;

  // 특정 에이전트(i)의 이동에 소요된 cost (정지 시간 제외)
  int getPathCost(const int i) const;

  // 마지막 configuration 반환
  Config last() const;

  // 특정 에이전트(i)의 마지막 위치 반환
  Node* last(const int i) const;

  // 계획 초기화
  void clear();

  // 새로운 configuration 추가
  void add(const Config& c);

  // 계획이 비어있는지 여부
  bool empty() const;

  // 전체 시간 길이 반환
  int size() const;

  // 마지막 시간 (makespan)
  int getMakespan() const;

  // 전체 에이전트 경로 cost의 합 (Sum of Costs)
  int getSOC() const;

  // 계획을 다른 계획과 연결 (두 계획 사이의 처음/끝 config 일치해야 함)
  Plan operator+(const Plan& other) const;
  void operator+=(const Plan& other);

  // 계획이 문제에 대해 유효한지 검사
  bool validate(Problem* P) const;
  bool validate(const Config& starts, const Config& goals) const;

  // 경로를 갱신할 때, 다른 에이전트와의 충돌을 피하기 위한 최소 시작 시간 계산
  int getMaxConstraintTime(const int id, Node* s, Node* g, Graph* G) const;
  int getMaxConstraintTime(const int id, Problem* P) const;

  // 에러 처리 함수
  void halt(const std::string& msg) const;
  void warn(const std::string& msg) const;
};


