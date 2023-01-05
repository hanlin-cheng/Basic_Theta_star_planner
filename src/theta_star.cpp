#include <vector>
#include "nav2_theta_star_planner/theta_star.hpp"

namespace theta_star
{

ThetaStar::ThetaStar()
: w_traversal_cost_(1.0),
  w_euc_cost_(2.0),
  w_heuristic_cost_(1.0),
  how_many_corners_(8),
  allow_unknown_(true),
  size_x_(0),
  size_y_(0),
  index_generated_(0)
{
  exp_node = new tree_node;
}

void ThetaStar::setStartAndGoal(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  unsigned int s[2], d[2];
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, s[0], s[1]);
  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, d[0], d[1]);

  src_ = {static_cast<int>(s[0]), static_cast<int>(s[1])};
  dst_ = {static_cast<int>(d[0]), static_cast<int>(d[1])};
}

// 生成路径入口
bool ThetaStar::generatePath(std::vector<coordsW> & raw_path)
{
  // 初始化全局变量的值
  resetContainers();
  // 初始化节点存储容器
  addToNodesData(index_generated_);
  // 计算成本g，对于起始点来说就是计算遍历成本
  double src_g_cost = getTraversalCost(src_.x, src_.y);
  // 计算启发式H成本
  double src_h_cost = getHCost(src_.x, src_.y);
  // 起始节点
  nodes_data_[index_generated_] =
  {src_.x, src_.y, src_g_cost, src_h_cost, &nodes_data_[index_generated_], true,
    src_g_cost + src_h_cost};
  queue_.push({&nodes_data_[index_generated_]});
  // 存储id
  addIndex(src_.x, src_.y, &nodes_data_[index_generated_]);
  tree_node * curr_data = &nodes_data_[index_generated_];
  index_generated_++;
  nodes_opened = 0;

  // 主循环搜索路径，广度优先搜索
  while (!queue_.empty()) {
    nodes_opened++;

    // 判断是否已经接触到目标点
    if (isGoal(*curr_data)) {
      break;
    }

    // 检查视线（lineofsight）更新父节点
    resetParent(curr_data);
    // 展开当前节点
    setNeighbors(curr_data);

    curr_data = queue_.top();
    queue_.pop();
  }

  if (queue_.empty()) {
    raw_path.clear();
    return false;
  }

  // 依据父节点回溯获取路径
  backtrace(raw_path, curr_data);
  // 每次执行完毕之后清空优先队列
  clearQueue();

  return true;
}

// 检查视线（lineofsight）更新父节点
void ThetaStar::resetParent(tree_node * curr_data)
{
  double g_cost, los_cost = 0;
  // 将当前节点移除开放队列
  curr_data->is_in_queue = false;
  // 当前节点的父节点
  const tree_node * curr_par = curr_data->parent_id;
  // 当前节点父节点的父节点
  const tree_node * maybe_par = curr_par->parent_id;

  // 检查从当前节点到当前节点父节点的父节点视线是否被遮挡
  if (losCheck(curr_data->x, curr_data->y, maybe_par->x, maybe_par->y, los_cost)) {
    // 计算成本距离g
    // 成本g除了包含分段距离，还包含每个节点的遍历成本los_cost
    g_cost = maybe_par->g +
      getEuclideanCost(curr_data->x, curr_data->y, maybe_par->x, maybe_par->y) + los_cost;

    // 更新父节点
    if (g_cost < curr_data->g) {
      curr_data->parent_id = maybe_par;
      curr_data->g = g_cost;
      curr_data->f = g_cost + curr_data->h;
    }
  }
}

// 展开当前节点
void ThetaStar::setNeighbors(const tree_node * curr_data)
{
  int mx, my;
  tree_node * m_id = nullptr;
  double g_cost, h_cost, cal_cost;

  for (int i = 0; i < how_many_corners_; i++) {
    mx = curr_data->x + moves[i].x;
    my = curr_data->y + moves[i].y;

    // 检查是否越界
    if (withinLimits(mx, my)) {
      // 检查是都是可通行的节点
      if (!isSafe(mx, my)) {
        continue;
      }
    } else {
      continue;
    }

    // 成本g除了包含分段距离，还包含每个节点的遍历成本
    g_cost = curr_data->g + getEuclideanCost(curr_data->x, curr_data->y, mx, my) + getTraversalCost(mx, my);

    // 取得节点指针
    m_id = getIndex(mx, my);

    if (m_id == nullptr) {
      // 插入节点数据
      addToNodesData(index_generated_);
      m_id = &nodes_data_[index_generated_];
      // 插入节点指针索引
      addIndex(mx, my, m_id);
      index_generated_++;
    }

    exp_node = m_id;

    h_cost = getHCost(mx, my);
    cal_cost = g_cost + h_cost;
    // 比较f值判断是否需要更改父节点
    if (exp_node->f > cal_cost) {
      exp_node->g = g_cost;
      exp_node->h = h_cost;
      exp_node->f = cal_cost;
      exp_node->parent_id = curr_data;
      // 判断节点是否已经在队列中
      if (!exp_node->is_in_queue) {
        exp_node->x = mx;
        exp_node->y = my;
        exp_node->is_in_queue = true;
        queue_.push({m_id});
      }
    }
  }
}

// 依据父节点回溯获取路径
void ThetaStar::backtrace(std::vector<coordsW> & raw_points, const tree_node * curr_n) const
{
  std::vector<coordsW> path_rev;
  coordsW world{};
  do {
    costmap_->mapToWorld(curr_n->x, curr_n->y, world.x, world.y);
    path_rev.push_back(world);
    if (path_rev.size() > 1) {
      curr_n = curr_n->parent_id;
    }
  } while (curr_n->parent_id != curr_n);
  costmap_->mapToWorld(curr_n->x, curr_n->y, world.x, world.y);
  path_rev.push_back(world);

  raw_points.reserve(path_rev.size());
  for (int i = static_cast<int>(path_rev.size()) - 1; i >= 0; i--) {
    raw_points.push_back(path_rev[i]);
  }
}

// 视线检查是否被遮挡
bool ThetaStar::losCheck(
  const int & x0, const int & y0, const int & x1, const int & y1,
  double & sl_cost) const
{
  sl_cost = 0;

  int cx, cy;
  int dy = abs(y1 - y0), dx = abs(x1 - x0), f = 0;
  int sx, sy;
  sx = x1 > x0 ? 1 : -1;
  sy = y1 > y0 ? 1 : -1;

  int u_x = (sx - 1) / 2;
  int u_y = (sy - 1) / 2;
  cx = x0;
  cy = y0;

  if (dx >= dy) {
    while (cx != x1) {
      f += dy;
      if (f >= dx) {
        if (!isSafe(cx + u_x, cy + u_y, sl_cost)) {
          return false;
        }
        cy += sy;
        f -= dx;
      }
      if (f != 0 && !isSafe(cx + u_x, cy + u_y, sl_cost)) {
        return false;
      }
      if (dy == 0 && !isSafe(cx + u_x, cy, sl_cost) && !isSafe(cx + u_x, cy - 1, sl_cost)) {
        return false;
      }
      cx += sx;
    }
  } else {
    while (cy != y1) {
      f = f + dx;
      if (f >= dy) {
        if (!isSafe(cx + u_x, cy + u_y, sl_cost)) {
          return false;
        }
        cx += sx;
        f -= dy;
      }
      if (f != 0 && !isSafe(cx + u_x, cy + u_y, sl_cost)) {
        return false;
      }
      if (dx == 0 && !isSafe(cx, cy + u_y, sl_cost) && !isSafe(cx - 1, cy + u_y, sl_cost)) {
        return false;
      }
      cy += sy;
    }
  }
  return true;
}

// 初始化全局变量的值
void ThetaStar::resetContainers()
{
  index_generated_ = 0;
  int last_size_x = size_x_;
  int last_size_y = size_y_;
  int curr_size_x = static_cast<int>(costmap_->getSizeInCellsX());
  int curr_size_y = static_cast<int>(costmap_->getSizeInCellsY());
  if (((last_size_x != curr_size_x) || (last_size_y != curr_size_y)) &&
    static_cast<int>(node_position_.size()) < (curr_size_x * curr_size_y))
  {
    // 初始化索引容器
    initializePosn(curr_size_y * curr_size_x - last_size_y * last_size_x);
    nodes_data_.reserve(curr_size_x * curr_size_y);
  } else {
    initializePosn();
  }
  size_x_ = curr_size_x;
  size_y_ = curr_size_y;
}

// 初始化索引容器
void ThetaStar::initializePosn(int size_inc)
{
  int i = 0;

  if (!node_position_.empty()) {
    for (; i < size_x_ * size_y_; i++) {
      node_position_[i] = nullptr;
    }
  }

  for (; i < size_inc; i++) {
    node_position_.push_back(nullptr);
  }
}
}  //  namespace theta_star
