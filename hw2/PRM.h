#ifndef PRM_H_
#define PRM_H_

#include "utilities.h"

struct PRM_Planner{
  std::vector<Vertex*> vertices_;
  int numofDOFs_;
  double* map_;
  int x_size_;
  int y_size_;

  PRM_Planner(int numofDOFs, double*	map, int x_size, int y_size) {
    vertices_.clear();
    numofDOFs_ = numofDOFs;
    map_ = map;
    x_size_ = x_size;
    y_size_ = y_size;
  }

  void init() {
    vertices_.clear();
  }

  void resetGraphStatus() {
    for(auto v : vertices_) {
      v->resetStatus();
    }
  }

  void buildRoadMap(int numofsamples, double neighborhood_radius) {
    init();
    int i = 0;
    while(i < numofsamples) {
      Vertex* new_v = new Vertex(numofDOFs_);
      getRandomVertex(numofDOFs_, new_v);
      if(IsValidArmConfiguration(new_v->getAnglesPtr(), numofDOFs_, map_, x_size_, y_size_)) {
        for(auto v : vertices_) {
          // same neighborhood
          double dist = getDistance(v, new_v);
          if(dist < neighborhood_radius) {
            // connectable
            if(connect(v, new_v, numofDOFs_, map_, x_size_, y_size_)) {
              // update connectivity
              v->connected_.push_back(new_v);
              v->costs_to_connected_.push_back(dist);
              new_v->connected_.push_back(v);
              new_v->costs_to_connected_.push_back(dist);
            }
          }
        }
        // add new vertex
        vertices_.push_back(new_v);
        i++;
      }
    }
  }

  Vertex* getGraphEntrance(Vertex* start) {
    // find entrance to graph
    Vertex* entrance = NULL;
    double min_dist = HUGE_VAL;
    for(auto v : vertices_) {
      if(connect(start, v, numofDOFs_, map_, x_size_, y_size_)) {
        double curr_dist = getDistance(start, v);
        if(curr_dist < min_dist) {
          entrance = v;
          min_dist = curr_dist;
        }
      }
    }
    if(entrance!=NULL) {
      entrance->g_value_ = min_dist;
      entrance->parent_ = start;
      std::cout << "graph entrace: " << entrance << std::endl;
    } else {
      std::cout << "cannot find entrance to graph" << std::endl;
    }

    return entrance;
  }

  Vertex* getGraphEscape(Vertex* goal) {
    // find exit from graph and compute heuristic
    Vertex* escape = NULL;
    double min_dist = HUGE_VAL;
    for(auto v : vertices_) {
      v->h_value_ = getDistance(v, goal);
      if(connect(v, goal, numofDOFs_, map_, x_size_, y_size_)) {
        if(v->h_value_ < min_dist) {
          escape = v;
          min_dist = v->h_value_;
        }
      }
    }

    if(escape != NULL) {
      std::cout << "graph exit: " << escape << std::endl;
      goal->parent_ = escape;
    } else {
      std::cout << "cannot find entrance to graph" << std::endl;
    }
    return escape;
  }

  void computePath(Vertex* entrance, Vertex* escape) {
    // A-star search
    bool found_path = false;
    std::vector<Vertex*> OPEN;
    entrance->open_listed_ = true;
    OPEN.push_back(entrance);
    while(!OPEN.empty()) {
      Vertex* v = OPEN[0];
      OPEN.erase(OPEN.begin());
      if(v == escape) {
        found_path = true;
        break;
      }
      v->closed_ = true;
      for(int i=0; i<v->connected_.size(); i++) {
        Vertex* v_next = v->connected_[i];
        double cost = v->costs_to_connected_[i];
        if(!v_next->closed_ && v->g_value_+cost < v_next->g_value_) {
          v_next->g_value_ = v->g_value_+cost;
          v_next->parent_ = v;
          if(!v_next->open_listed_) {
            v_next->open_listed_ = true;
            OPEN.push_back(v_next);
          }
        }
      }
      std::sort(OPEN.begin(), OPEN.end(), compareVertexPtr);
    }

    if(found_path) {
      std::cout << "Route Found\n" << std::endl;
    } else {
      std::cout << "Find Path Failed\n" << std::endl;
    }
  }

  void retrievePath(Vertex* start, Vertex* goal, std::vector<std::vector<double> >& plan) {
    plan.clear();
    for(Vertex* v=goal; v!=NULL; v=v->parent_) {
      std::cout << "[ ";
      for(int i=0; i<numofDOFs_; i++) {
        std::cout << v->angles_[i] << " ";
      }
      std::cout << "]\n";
      plan.push_back(v->angles_);
    }
  }

  void query(double* start_angles, double* goal_angles, std::vector<std::vector<double> >& plan) {
    // reverse search
    Vertex* start = new Vertex(goal_angles, numofDOFs_);
    Vertex* goal = new Vertex(start_angles, numofDOFs_);

    // connect start and goal to graph
    Vertex* entrance = getGraphEntrance(start);
    Vertex* escape = getGraphEscape(goal);
    // A-star search
    computePath(entrance, escape);
    // retrieve path, backward result of reversed search, path is start to goal
    retrievePath(start, goal, plan);
    // reset A-star search values
    resetGraphStatus();
  }
};

#endif
