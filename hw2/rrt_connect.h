#ifndef RRT_CONNECT_H_
#define RRT_CONNECT_H_

#include "utilities.h"

struct RRT_Connect_Planner{
  std::vector<Vertex*> tree_a_;
  std::vector<Vertex*> tree_b_;
  int numofDOFs_;
  double* map_;
  int x_size_;
  int y_size_;
  int numofsamples_;
  double extend_distance_;

  RRT_Connect_Planner(int numofDOFs, double*	map, int x_size, int y_size, int numofsamples, double extend_distance) {
    tree_a_.clear();
    tree_b_.clear();
    numofDOFs_ = numofDOFs;
    map_ = map;
    x_size_ = x_size;
    y_size_ = y_size;
    numofsamples_ = numofsamples;
    extend_distance_ = extend_distance;
  }

  void init() {
    tree_a_.clear();
    tree_b_.clear();
  }

  Vertex* getNear(Vertex* q_rand, std::vector<Vertex*> tree) {
    // find exit from graph and compute heuristic
    Vertex* q_near = NULL;
    double min_dist = HUGE_VAL;
    for(auto v : tree) {
      double curr_dist = getDistance(v, q_rand);
      if(curr_dist < min_dist) {
        q_near = v;
        min_dist = curr_dist;
      }
    }

    return q_near;
  }

  bool newConfig(Vertex* q_near, Vertex* q_rand, Vertex* q_new) {
    // maximum extendable config
    double norm = std::min(1.0, extend_distance_/getDistance(q_near, q_rand));
    for(int d=0; d<numofDOFs_; d++) {
      q_new->angles_[d] = q_near->angles_[d] + norm * (q_rand->angles_[d] - q_near->angles_[d]);
    }

    // step towards maximum extendable config
    std::vector<double> step(numofDOFs_);
    for(int d=0; d<numofDOFs_; d++) {
      step[d] = (q_new->angles_[d] - q_near->angles_[d]) / 10 ;
    }

    for(int i=1; i<=10; i++) {
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

  int extend(std::vector<Vertex*>& tree, Vertex* q_rand, double extend_distance) {
    int status = 0;
    Vertex* q_near = getNear(q_rand, tree);
    Vertex* q_new = new Vertex(numofDOFs_);
    if(newConfig(q_near, q_rand, q_new)) {
      // add edge
      q_new->parent_ = q_near;
      // add vertex
      tree.push_back(q_new);

      if(getDistance(q_new, q_rand) < 1e-3) {
        // reached
        status = 2;
      } else {
        // advanced
        status = 1;
      }
    }
    return status;
  }

  int connect(std::vector<Vertex*>& tree, Vertex* q) {
    int S;
    do {
      S = extend(tree, q, extend_distance_);
    }
    while (S==1);
    return S;
  }

  void buildRRT(Vertex* start, Vertex* goal) {
    init();
    tree_a_.push_back(start);
    tree_b_.push_back(goal);
    std::vector<Vertex*>* tree_a = &tree_a_;
    std::vector<Vertex*>* tree_b = &tree_b_;

    int i = 0;
    while(i < numofsamples_) {
      Vertex* q_rand = new Vertex(numofDOFs_);
      getRandomVertex(numofDOFs_, q_rand);
      if(IsValidArmConfiguration(q_rand->getAnglesPtr(), numofDOFs_, map_, x_size_, y_size_)) {
        if(extend(*tree_a, q_rand, extend_distance_)!=0) {
          if(connect(*tree_b, tree_a->back())==2) {
            std::cout << "found path" << '\n';
            break;
          }
        }
        std::swap(tree_a, tree_b);
        i++;
      }
    }
  }

  void retrievePath(std::vector<std::vector<double> >& plan) {
    plan.clear();
    //check if connected
    if(getDistance(tree_a_.back(), tree_b_.back()) > 1e-3) {
      std::cout << "not connected" << '\n';
      return;
    }

    for(Vertex* v=tree_a_.back(); v!=NULL; v=v->parent_) {
      plan.push_back(v->angles_);
    }
    std::reverse(plan.begin(),plan.end());

    for(Vertex* v=tree_b_.back(); v!=NULL; v=v->parent_) {
      plan.push_back(v->angles_);
    }
  }

  void query(double* start_angles, double* goal_angles, std::vector<std::vector<double> >& plan) {
    // reverse search
    Vertex* start = new Vertex(start_angles, numofDOFs_);
    Vertex* goal = new Vertex(goal_angles, numofDOFs_);

    // connect start and goal to graph
    buildRRT(start, goal);

    // retrieve path, backward result of reversed search, path is start to goal
    retrievePath(plan);
  }
};

#endif
