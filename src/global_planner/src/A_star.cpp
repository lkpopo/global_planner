#include "A_star.h"

using namespace std;
using namespace Eigen;

namespace global_planner
{

  Astar::~Astar()
  {
    for (int i = 0; i < max_search_num; i++)
    {
      // delete表示释放堆内存
      delete path_node_pool_[i];
    }
  }

  void Astar::init()
  {
    lambda_heu_ = 2.0;
    lambda_cost_ = 300.0;
    max_search_num = 100000;
    resolution_ = 0.5;

    tie_breaker_ = 1.0 + 1.0 / max_search_num;

    this->inv_resolution_ = 1.0 / resolution_;

    has_global_point = false;
    path_node_pool_.resize(max_search_num);

    // 新建
    for (int i = 0; i < max_search_num; i++)
    {
      path_node_pool_[i] = new Node;
    }

    use_node_num_ = 0;
    iter_num_ = 0;

    // 初始化占据地图
    Occupy_map_ptr.reset(new Occupy_map);

    Occupy_map_ptr->resolution_ = resolution_;
    Occupy_map_ptr->init();

    // 读取地图参数
    origin_ = Occupy_map_ptr->min_range_;
    map_size_3d_ = Occupy_map_ptr->max_range_ - Occupy_map_ptr->min_range_;
  }

  void Astar::reset()
  {
    // 重置与搜索相关的变量
    expanded_nodes_.clear();
    path_nodes_.clear();

    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
    open_set_.swap(empty_queue);

    for (int i = 0; i < use_node_num_; i++)
    {
      NodePtr node = path_node_pool_[i];
      node->parent = NULL;
      node->node_state = NOT_EXPAND;
    }

    use_node_num_ = 0;
    iter_num_ = 0;
  }

  // 搜索函数，输入为：起始点及终点
  // 将传输的数组通通变为指针！！！！ 以后改
  bool Astar::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
  {
    // 首先检查目标点是否可到达
    if (Occupy_map_ptr->getOccupancy(end_pt))
    {
      cout << "Astar search: [ Astar can't find path: goal point is occupied ]" << endl;
      return false;
    }

    goal_pos = end_pt;
    Eigen::Vector3i end_index = posToIndex(end_pt);

    // 初始化,将起始点设为第一个路径点
    NodePtr cur_node = path_node_pool_[0];
    cur_node->parent = NULL;
    cur_node->position = start_pt;
    cur_node->index = posToIndex(start_pt);
    cur_node->g_score = 0.0;
    cur_node->f_score = lambda_heu_ * getManhHeu(cur_node->position, end_pt);
    cur_node->node_state = IN_OPEN_SET;

    // 将当前点推入open set
    open_set_.push(cur_node);
    // 迭代次数+1
    use_node_num_ += 1;
    // 记录当前为已扩展
    expanded_nodes_.insert(cur_node->index, cur_node);

    NodePtr terminate_node = NULL;

    // 搜索主循环
    while (!open_set_.empty())
    {
      // 获取f_score最低的点
      cur_node = open_set_.top();

      // 判断终止条件(当前点与终点距离小于等于一个单位时)
      bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
                       abs(cur_node->index(1) - end_index(1)) <= 1 &&
                       abs(cur_node->index(2) - end_index(2)) <= 1;

      if (reach_end)
      {
        // 将当前点设为终止点，并往回形成路径
        terminate_node = cur_node;
        retrievePath(terminate_node);

        return true;
      }

      /* ---------- pop node and add to close set ---------- */
      open_set_.pop();
      // 将当前点推入close set
      cur_node->node_state = IN_CLOSE_SET; // in expand set
      iter_num_ += 1;

      /* ---------- init neighbor expansion ---------- */
      Eigen::Vector3d cur_pos = cur_node->position;
      Eigen::Vector3d expand_node_pos;

      vector<Eigen::Vector3d> inputs;
      Eigen::Vector3d d_pos;

      /* ---------- expansion loop ---------- */
      // 扩展： 3*3*3 - 1 = 26种可能
      for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
      {
        for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
        {
          for (double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_)
          {

            d_pos << dx, dy, dz;

            // 跳过自己那个格子
            if (d_pos.norm() < 1e-3)
            {
              continue;
            }

            // 扩展节点的位置
            expand_node_pos = cur_pos + d_pos;

            // 确认该点在地图范围内
            if (!Occupy_map_ptr->isInMap(expand_node_pos))
            {
              continue;
            }

            // 计算扩展节点的index
            Eigen::Vector3i d_pos_id;
            d_pos_id << int(dx / resolution_), int(dy / resolution_), int(dz / resolution_);
            Eigen::Vector3i expand_node_id = d_pos_id + cur_node->index;

            // 检查当前扩展的点是否在close set中，如果是则跳过
            NodePtr expand_node = expanded_nodes_.find(expand_node_id);
            if (expand_node != NULL && expand_node->node_state == IN_CLOSE_SET)
            {
              continue;
            }

            // 检查当前扩展点是否被占据,如果是则跳过
            bool is_occupy = Occupy_map_ptr->getOccupancy(expand_node_pos);
            if (is_occupy)
            {
              continue;
            }

            // 如果能通过上述检查则
            double tmp_g_score, tmp_f_score;
            tmp_g_score = d_pos.squaredNorm() + cur_node->g_score;
            tmp_f_score = tmp_g_score + lambda_heu_ * getManhHeu(expand_node_pos, end_pt) + lambda_cost_ * Occupy_map_ptr->getCost(expand_node_pos);

            // 如果扩展的当前节点为NULL，即未扩展过
            if (expand_node == NULL)
            {
              expand_node = path_node_pool_[use_node_num_];
              expand_node->index = expand_node_id;
              expand_node->position = expand_node_pos;
              expand_node->f_score = tmp_f_score;
              expand_node->g_score = tmp_g_score;
              expand_node->parent = cur_node;
              expand_node->node_state = IN_OPEN_SET;

              open_set_.push(expand_node);
              expanded_nodes_.insert(expand_node_id, expand_node);

              use_node_num_ += 1;
              // 超过最大搜索次数
              if (use_node_num_ == max_search_num)
              {
                cout << "Astar search: [ Astar can't find path: reach the max_search_num ]" << endl;
                return false;
              }
            }
            // 如果当前节点已被扩展过，则更新其状态
            else if (expand_node->node_state == IN_OPEN_SET)
            {
              if (tmp_g_score < expand_node->g_score)
              {
                // expand_node->index = expand_node_id;
                expand_node->position = expand_node_pos;
                expand_node->f_score = tmp_f_score;
                expand_node->g_score = tmp_g_score;
                expand_node->parent = cur_node;
              }
            }
          }
        }
      }
    }

    // 搜索完所有可行点，即使没达到最大搜索次数，也没有找到路径
    // 这种一般是因为无人机周围被占据，或者无人机与目标点之间无可通行路径造成的
    cout << "Astar search: [ Astar can't find path: max_search_num: open set empty ]" << endl;
    return false;
  }

  // 由最终点往回生成路径
  void Astar::retrievePath(NodePtr end_node)
  {
    NodePtr cur_node = end_node;
    path_nodes_.push_back(cur_node);

    while (cur_node->parent != NULL)
    {
      cur_node = cur_node->parent;
      path_nodes_.push_back(cur_node);
    }

    // 反转顺序
    reverse(path_nodes_.begin(), path_nodes_.end());

    // 直接在这里生成路径？
    path_nodes_ = prunePathLineOfSight(path_nodes_, Occupy_map_ptr);
  }

  Path Astar::getPath()
  {
    Path path;
    path.poses.reserve(path_nodes_.size());
    
    for (uint i = 0; i < path_nodes_.size(); ++i)
    {
      Eigen::Vector3d v= path_nodes_[i]->position;
      path.poses.emplace_back(Pose(v.x(),v.y(),v.z()));
    }
    path.poses.emplace_back(goal_pos.x(),goal_pos.y(),goal_pos.z());
    return path;
  }

  double Astar::getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2)
  {
    double dx = fabs(x1(0) - x2(0));
    double dy = fabs(x1(1) - x2(1));
    double dz = fabs(x1(2) - x2(2));

    double h = 0;

    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx < 1e-4)
    {
      h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy < 1e-4)
    {
      h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz < 1e-4)
    {
      h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }

    return tie_breaker_ * h;
  }

  double Astar::getManhHeu(Eigen::Vector3d x1, Eigen::Vector3d x2)
  {
    double dx = fabs(x1(0) - x2(0));
    double dy = fabs(x1(1) - x2(1));
    double dz = fabs(x1(2) - x2(2));

    return tie_breaker_ * (dx + dy + dz);
  }

  double Astar::getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2)
  {
    return tie_breaker_ * (x2 - x1).norm();
  }

  std::vector<NodePtr> Astar::getVisitedNodes()
  {
    vector<NodePtr> visited;
    visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
    return visited;
  }

  Eigen::Vector3i Astar::posToIndex(Eigen::Vector3d pt)
  {
    Vector3i idx;
    idx << floor((pt(0) - origin_(0)) * inv_resolution_), floor((pt(1) - origin_(1)) * inv_resolution_),
        floor((pt(2) - origin_(2)) * inv_resolution_);

    return idx;
  }

  void Astar::indexToPos(Eigen::Vector3i id, Eigen::Vector3d &pos)
  {
    for (int i = 0; i < 3; ++i)
      pos(i) = (id(i) + 0.5) * resolution_ + origin_(i);
  }

  // 检查碰撞
  bool Astar::lineCollisionFree(const Occupy_map::Ptr &map_ptr,
                                const Eigen::Vector3d &p0,
                                const Eigen::Vector3d &p1,
                                double step)
  {
    Eigen::Vector3d dir = p1 - p0;
    double len = dir.norm();
    if (len < 1e-6)
      return true;
    dir /= len;
    int n = std::max(1, static_cast<int>(std::ceil(len / step)));
    for (int i = 0; i <= n; ++i)
    {
      double t = static_cast<double>(i) / n;
      Eigen::Vector3d p = p0 + dir * (t * len);
      if (!map_ptr->isInMap(p))
        return false; // out of map -> treat as collision
      if (map_ptr->getOccupancy(p) != 0)
        return false; // occupied
    }
    return true;
  }

  // 优化规划好的路径，尽量走直线
  std::vector<NodePtr> Astar::prunePathLineOfSight(const std::vector<NodePtr> &raw_path,
                                                   const Occupy_map::Ptr &map_ptr)
  {
    std::vector<NodePtr> out;
    if (raw_path.empty())
      return out;

    const double step = std::max(map_ptr->resolution_ * 0.5, 0.05); // 采样步长

    size_t i = 0;
    while (i < raw_path.size())
    {
      // 尝试把 i 直接连到最远的 j
      size_t j = raw_path.size() - 1;
      bool found = false;
      for (; j > i; --j)
      {
        if (lineCollisionFree(map_ptr, raw_path[i]->position, raw_path[j]->position, step))
        {
          out.push_back(raw_path[i]);
          i = j;
          found = true;
          break;
        }
      }
      if (!found)
      {
        // 兜底情况
        out.push_back(raw_path[i]);
        ++i;
      }
    }
    // 保证最后点是目标
    if (!out.empty() && (out.back()->position - raw_path.back()->position).norm() > 1e-6)
    {
      out.push_back(raw_path.back());
    }
    return out;
  }
}