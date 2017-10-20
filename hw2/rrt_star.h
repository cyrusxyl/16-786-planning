#ifndef RRT_STAR_H_
#define RRT_STAR_H_

#include "utilities.h"
#include <cmath>

struct RRT_Star_Planner {
  std::vector<Vertex *> tree_;
  int numofDOFs_;
  double *map_;
  int x_size_;
  int y_size_;
  int numofsamples_;
  double extend_distance_;
  double gamma_;

  RRT_Star_Planner(int numofDOFs, double *map, int x_size, int y_size,
                   int numofsamples, double extend_distance, double gamma) {
    tree_.clear();
    numofDOFs_ = numofDOFs;
    map_ = map;
    x_size_ = x_size;
    y_size_ = y_size;
    numofsamples_ = numofsamples;
    extend_distance_ = extend_distance;
    gamma_ = gamma;
  }

  void init() { tree_.clear(); }

  Vertex *getNear(Vertex *q_rand) {
    // find exit from graph and compute heuristic
    Vertex *q_near = NULL;
    double min_dist = HUGE_VAL;
    for (auto v : tree_) {
      double curr_dist = getDistance(v, q_rand);
      if (curr_dist < min_dist) {
        q_near = v;
        min_dist = curr_dist;
      }
    }

    return q_near;
  }

  Vertex *getTreeEscape(Vertex *goal) {
    // find exit from graph and compute heuristic
    Vertex *escape = NULL;
    for (auto v : tree_) {
      double dist = getDistance(v, goal);
      if (connect(v, goal, numofDOFs_, map_, x_size_, y_size_)) {
        if (v->g_value_ + dist < goal->g_value_) {
          escape = v;
          goal->g_value_ = v->g_value_ + dist;
        }
      }
    }

    return escape;
  }

  bool newConfig(Vertex* q_near, Vertex* q_rand, Vertex* q_new) {
    // maximum extendable config
    double norm = std::min(1.0, extend_distance_/getDistance(q_near, q_rand));
    for(int d=0; d<numofDOFs_; d++) {
      q_new->angles_[d] = q_near->angles_[d] + norm * (q_rand->angles_[d] - q_near->angles_[d]);
    }

    int numsteps = getDistance(q_near, q_new) / (PI/20) + 1;
    // step towards maximum extendable config
    std::vector<double> step(numofDOFs_);
    for(int d=0; d<numofDOFs_; d++) {
      step[d] = (q_new->angles_[d] - q_near->angles_[d]) / numsteps ;
    }

    for(int i=1; i<=numsteps; i++) {
      std::vector<double> interpolated_angles(numofDOFs_);
      for(int d=0; d<numofDOFs_; d++) {
        interpolated_angles[d] = q_near->angles_[d] + step[d] * i;
      }
      if(!IsValidArmConfiguration(&interpolated_angles[0], numofDOFs_, map_, x_size_, y_size_)) {
        if(i==1) {
          return false;
        } else {
          // blocked, update config
          for(int d=0; d<numofDOFs_; d++) {
            q_new->angles_[d] = q_near->angles_[d] + step[d] * (i-1);
          }
          break;
        }
      }
    }
    return true;
  }

  std::vector<Vertex *> getNeighborhood(Vertex *q_new) {
    std::vector<Vertex *> neighborhood;
    double c = gamma_*pow(log(tree_.size())/double(tree_.size()),1.0/numofDOFs_);
    double neighborhood_radius = std::min(c, extend_distance_);

    for (auto v : tree_) {
      double dist = getDistance(v, q_new);
      if (dist < neighborhood_radius) {
        // connectable
        if (connect(v, q_new, numofDOFs_, map_, x_size_, y_size_)) {
          neighborhood.push_back(v);
        }
      }
    }
    return neighborhood;
  }

  void removeEdge(Vertex *parent, Vertex *child) {
    parent->connected_.erase(std::remove(parent->connected_.begin(),
                                         parent->connected_.end(), child),
                             parent->connected_.end());
  }

  void updateCost(Vertex* q_near) {
    double dist = getDistance(q_near, q_near->parent_);
    q_near->g_value_ = q_near->parent_->g_value_ + dist;
    for(auto child : q_near->connected_) {
      updateCost(child);
    }
  }

  void printCost(Vertex* q) {
    if(q->parent_ == NULL) {
      return;
    }
    printCost(q->parent_);
    std::cout << q->g_value_ << "->";
  }

  int extend(Vertex *q_rand) {
    int status = 0;
    Vertex *q_nearest = getNear(q_rand);
    Vertex *q_new = new Vertex(numofDOFs_);
    if (newConfig(q_nearest, q_rand, q_new)) {
      Vertex *q_min = q_nearest;
      std::vector<Vertex *> neighborhood = getNeighborhood(q_new);
      for (auto q_near : neighborhood) {
        double dist = getDistance(q_near, q_new);
        if (q_near->g_value_ + dist < q_new->g_value_) {
          q_min = q_near;
        }
      }
      q_new->parent_ = q_min;
      q_min->connected_.push_back(q_new);
      double dist = getDistance(q_min, q_new);
      q_new->g_value_ = q_min->g_value_ + dist;

      for (auto q_near : neighborhood) {
        double dist = getDistance(q_near, q_new);
        if (q_new->g_value_ + dist < q_near->g_value_) {
          removeEdge(q_near->parent_, q_near);
          q_near->parent_ = q_new;
          updateCost(q_near);
        }
      }

      // add vertex
      tree_.push_back(q_new);

      if (getDistance(q_new, q_rand) < 1e-3) {
        // reached
        status = 2;
      } else {
        // advanced
        status = 1;
      }
    }
    return status;
  }

  void buildRRT(Vertex *start) {
    init();
    start->g_value_ = 0;
    tree_.push_back(start);
    int i = 0;
    while (i < numofsamples_) {
      Vertex *q_rand = new Vertex(numofDOFs_);
      getRandomVertex(numofDOFs_, q_rand);
      if (IsValidArmConfiguration(q_rand->getAnglesPtr(), numofDOFs_, map_,
                                  x_size_, y_size_)) {
        extend(q_rand);
        i++;
      }
    }
  }

  // void expandRRT(int numofsamples) {
  //   int i = 0;
  //   while (i < numofsamples) {
  //     Vertex *q_rand = new Vertex(numofDOFs_);
  //     getRandomVertex(numofDOFs_, q_rand);
  //     if (IsValidArmConfiguration(q_rand->getAnglesPtr(), numofDOFs_, map_,
  //                                 x_size_, y_size_)) {
  //       extend(q_rand);
  //       i++;
  //     }
  //   }
  // }

  void retrievePath(Vertex *goal, std::vector<std::vector<double>> &plan) {
    plan.clear();
    Vertex *escape = getTreeEscape(goal);
    if (escape != NULL) {
      // std::cout << "tree exit: " << escape << std::endl;
      goal->parent_ = escape;
    } else {
      std::cout << "cannot find exit from tree" << std::endl;
    }

    for (Vertex *v = goal; v != NULL; v = v->parent_) {
      // std::cout << "[ ";
      // for (int i = 0; i < numofDOFs_; i++) {
      //   std::cout << v->angles_[i] << " ";
      // }
      // std::cout << "]\n";
      plan.push_back(v->angles_);
    }
    std::cout << goal->g_value_  << '\n';
  }

  void query(double *start_angles, double *goal_angles,
             std::vector<std::vector<double>> &plan) {
     time_t t1, t2;
     t1 = clock();
    // reverse search
    Vertex *start = new Vertex(goal_angles, numofDOFs_);
    Vertex *goal = new Vertex(start_angles, numofDOFs_);

    // connect start and goal to graph
    buildRRT(start);
    // retrieve path, backward result of reversed search, path is start to goal
    retrievePath(goal, plan);
    //
    // for(int i=0; i<100; i++) {
    //   expandRRT(10);
    //   retrievePath(goal, plan);
    // }
    t2 = clock();
    std::cout << ((float)t2-(float)t1)/CLOCKS_PER_SEC << '\n';
  }
};

#endif
