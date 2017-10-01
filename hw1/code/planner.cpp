/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include "mex.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <math.h>
#include <queue>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vector>

/* Input Arguments */
#define MAP_IN prhs[0]
#define ROBOT_IN prhs[1]
#define GOAL_IN prhs[2]

/* Output Arguments */
#define ACTION_OUT plhs[0]

/*access to the map is shifted to account for 0-based indexing in the map,
whereas 1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)*/
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y - 1) * XSIZE + (X - 1))

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

/* Primitives Information */
#define NUMOFDIRS 8
#define NUMOFPRIMS 5
#define NUMOFINTERSTATES 10
#define NUMOFDIM 3

#define RES 0.1

typedef float PrimArray[NUMOFDIRS][NUMOFPRIMS][NUMOFINTERSTATES][NUMOFDIM];

int temp = 0;

bool applyaction(double *map, int x_size, int y_size, float robotposeX,
                 float robotposeY, float robotposeTheta, float *newx,
                 float *newy, float *newtheta, PrimArray mprim, int dir,
                 int prim) {
  int i;
  for (i = 0; i < NUMOFINTERSTATES; i++) {
    *newx = robotposeX + mprim[dir][prim][i][0];
    *newy = robotposeY + mprim[dir][prim][i][1];
    *newtheta = mprim[dir][prim][i][2];

    int gridposx = (int)(*newx / RES + 0.5);
    int gridposy = (int)(*newy / RES + 0.5);

    /* check validity */
    if (gridposx <= 1 || gridposx >= x_size || gridposy <= 1 ||
        gridposy >= y_size) {
      return false;
    }
    if ((int)map[GETMAPINDEX(gridposx, gridposy, x_size, y_size)] != 0) {
      return false;
    }
  }

  return true;
}

int getPrimitiveDirectionforRobotPose(float angle) {
  /* returns the direction index with respect to the PrimArray */
  /* normalize bw 0 to 2pi */
  if (angle < 0.0) {
    angle += 2 * M_PI;
  }
  int dir = (int)(angle / (2 * M_PI / NUMOFDIRS) + 0.5);
  if (dir == 8) {
    dir = 0;
  }
  return dir;
}

static void planner(double *map, int x_size, int y_size, float robotposeX,
                    float robotposeY, float robotposeTheta, float goalposeX,
                    float goalposeY, PrimArray mprim, int *prim_id) {
  // printf("temp=%d\n", temp);
  temp = temp + 1;

  *prim_id = 0; /* arbitrary action */

  /*printf("robot: %d %d; ", robotposeX, robotposeY);*/
  /*printf("goal: %d %d;", goalposeX, goalposeY);*/

  /*for now greedily move towards the target, */
  /*but this is where you can put your planner */
  double mindisttotarget = 1000000;
  int dir;
  int prim;
  dir = getPrimitiveDirectionforRobotPose(robotposeTheta);

  for (prim = 0; prim < NUMOFPRIMS; prim++) {
    float newx, newy, newtheta;
    bool ret;
    ret =
        applyaction(map, x_size, y_size, robotposeX, robotposeY, robotposeTheta,
                    &newx, &newy, &newtheta, mprim, dir, prim);
    /* skip action that leads to collision */
    if (ret) {
      double disttotarget =
          (double)sqrt(((newx - goalposeX) * (newx - goalposeX) +
                        (newy - goalposeY) * (newy - goalposeY)));
      if (disttotarget < mindisttotarget) {
        mindisttotarget = disttotarget;

        *prim_id = prim;
      }
    }
  }
  // printf("action %d\n", *prim_id);
  return;
}

struct state {
  // pose
  float poseX_ = 0;
  float poseY_ = 0;
  float poseTheta_ = 0;
  // state variables
  float g_value_ = HUGE_VAL;
  float h_value_ = HUGE_VAL;
  int generated_iteration_ = 0;
  bool expanded_ = false;
  bool open_ = false;

  struct state *parent_ = NULL;

  int prim_to_this_ = 0;

  std::vector<struct state *> succ_;
  std::vector<int> prims_to_succ_;

  state(float X, float Y, float theta) {
    poseX_ = X;
    poseY_ = Y;
    poseTheta_ = theta;
  }

  bool operator==(const struct state &b) const {
    return std::abs(poseX_ - b.poseX_) < 1e-3 &&
           std::abs(poseY_ - b.poseY_) < 1e-3 &&
           std::abs(poseTheta_ - b.poseTheta_) < 1e-3;
  }
};

struct ComparePtr {
  bool operator()(const struct state *a, const struct state *b) const {
    return a->g_value_ + a->h_value_ > b->g_value_ + b->h_value_;
  }
};

struct A_STAR {
  std::priority_queue<struct state *, std::vector<struct state *>, ComparePtr>
      OPEN_;
  std::vector<struct state *> S_;

  struct state *s_start_;
  struct state *s_goal_;

  double *map_;
  int x_size_;
  int y_size_;
  float sigma_;
  PrimArray mprim_;

  std::vector<int> path_;

  A_STAR(double *map, int x_size, int y_size, float sigma, PrimArray mprim) {
    map_ = map;
    x_size_ = x_size;
    y_size_ = y_size;
    sigma_ = sigma;
    int i, j, k;
    for (i = 0; i < NUMOFDIRS; ++i) {
      for (j = 0; j < NUMOFPRIMS; ++j) {
        for (k = 0; k < NUMOFINTERSTATES; ++k) {
          mprim_[i][j][k][0] = mprim[i][j][k][0];
          mprim_[i][j][k][1] = mprim[i][j][k][1];
          mprim_[i][j][k][2] = mprim[i][j][k][2];
        }
      }
    }
  }

  float h(struct state *s) {
    float dx = s_goal_->poseX_ - s->poseX_;
    float dy = s_goal_->poseY_ - s->poseY_;
    return sigma_ * sqrtf(pow(dx, 2) + pow(dy, 2));
  }

  bool testClosedList(struct state *s) {
    return (s_start_ == s) || (s->expanded_ && s->parent_ != NULL);
  }

  int in(struct state *s, std::vector<struct state *> V) {
    for (int i = V.size() - 1; i >= 0; i--) {
      if (*s == *V[i]) {
        return i;
      }
    }
    return -1;
  }

  bool in(struct state *s,
          std::priority_queue<struct state *, std::vector<struct state *>,
                              ComparePtr>
              V) {
    for (unsigned int i = 0; i < V.size(); i++) {
      if (*s == *V.top()) {
        return true;
      }
      V.pop();
    }
    return false;
  }

  bool in_open(struct state *s) { return s->open_; }

  void getSucc(struct state *s) {
    int dir;
    dir = getPrimitiveDirectionforRobotPose(s->poseTheta_);

    for (int prim = 0; prim < NUMOFPRIMS; prim++) {
      float newx, newy, newtheta;
      bool ret;
      ret = applyaction(map_, x_size_, y_size_, s->poseX_, s->poseY_,
                        s->poseTheta_, &newx, &newy, &newtheta, mprim_, dir,
                        prim);
      /* skip action that leads to collision */
      if (ret) {
        struct state *s_next = new struct state(newx, newy, newtheta);
        int index = in(s_next, S_);
        if (index != -1) {
          s->succ_.push_back(S_[index]);
          s->prims_to_succ_.push_back(prim);
        } else {
          s->succ_.push_back(s_next);
          s->prims_to_succ_.push_back(prim);
          S_.push_back(s_next);
        }
      }
    }
  }

  bool computeCostMinimalPath(float runtime) {
    int count = 0;
    clock_t start_time = clock();
    while (!OPEN_.empty() &&
           float(clock() - start_time) / CLOCKS_PER_SEC < (0.95 - runtime)) {
      struct state *s = OPEN_.top();
      count++;
      OPEN_.pop();
      s->expanded_ = true;
      getSucc(s);
      // printf("expanding state %d: %f, %f, %f, f=%f+%f\n", count, s->poseX_,
      //        s->poseY_, s->poseTheta_, s->g_value_, s->h_value_);
      for (unsigned int i = 0; i < s->succ_.size(); i++) {
        struct state *s_prime = s->succ_[i];
        if (!testClosedList(s_prime)) {
          s_prime->h_value_ = h(s_prime);
          if (s_prime->g_value_ > s->g_value_ + 1) {
            s_prime->g_value_ = s->g_value_ + 1;
            s_prime->parent_ = s;
            s_prime->prim_to_this_ = s->prims_to_succ_[i];
            if (!in_open(s_prime)) {
              // printf("succ: %f, %f, %f, f=%f+%f, prim=%d\n", s_prime->poseX_,
              // s_prime->poseY_,
              //  s_prime->poseTheta_, s_prime->g_value_, s_prime->h_value_,
              //  s_prime->prim_to_this_);
              s_prime->open_ = true;
              OPEN_.push(s_prime);
            }
          }
        }
      }
      if (std::abs(s_goal_->poseX_ - s->poseX_) < 0.5 &&
          std::abs(s_goal_->poseY_ - s->poseY_) < 0.5) {
        s_goal_ = s;
        return true;
      }
    }
    return false;
  }

  int retrievePath(float robotposeX, float robotposeY, float robotposeTheta,
                   float goalposeX, float goalposeY, float runtime) {
    s_start_ = new struct state(robotposeX, robotposeY, robotposeTheta);
    s_start_->g_value_ = 0;
    s_goal_ = new struct state(goalposeX, goalposeY, 0);
    s_start_->h_value_ = h(s_start_);
    s_start_->open_ = true;
    OPEN_.push(s_start_);
    S_.push_back(s_start_);
    bool ret = computeCostMinimalPath(runtime);
    if (ret) {
      for (struct state *s = s_goal_; s != s_start_; s = s->parent_) {
        path_.push_back(s->prim_to_this_);
      }
      return path_.back();
    } else {
      return -1;
    }
  }
};

static void A_Star_Planner(double *map, int x_size, int y_size,
                           float robotposeX, float robotposeY,
                           float robotposeTheta, float goalposeX,
                           float goalposeY, PrimArray mprim, int *prim_id) {
  // printf("temp=%d\n", temp);
  temp = temp + 1;
  float weights[6] = {5, 3, 2, 1.8, 1.5, 1.2};
  int i = 0;
  float sigma;
  clock_t start_time = clock();
  float runtime = float(clock() - start_time) / CLOCKS_PER_SEC;
  while (runtime < 0.8 && i < 6) {
    sigma = weights[i];
    struct A_STAR a_star_planner(map, x_size, y_size, sigma, mprim);
    int ret = a_star_planner.retrievePath(
        robotposeX, robotposeY, robotposeTheta, goalposeX, goalposeY, runtime);
    if (ret != -1) {
      *prim_id = ret;
      i++;
      runtime = float(clock() - start_time) / CLOCKS_PER_SEC;
      printf("Using weight = %f, runtime = %f\n", sigma, runtime);
    } else {
      printf("Tried weight = %f, cannot finish in 1s\n", sigma);
      break;
    }
  }

  printf("Finish Planning\n\n");
  return;
}

/*prhs contains input parameters (3):
1st is matrix with all the obstacles
2nd is a row vector <x,y> for the robot pose
3rd is a row vector <x,y> for the target pose
plhs should contain output parameters (1):
1st is a row vector <dx,dy> which corresponds to the action that the robot
should make*/

void parseMotionPrimitives(PrimArray mprim) {
  FILE *fp;
  fp = fopen("unicycle_8angles.mprim", "r+");
  char skip_c[100];
  int skip_f;
  float resolution;
  int num_angles;
  int num_mprims;
  fscanf(fp, "%s %f", skip_c, &resolution);
  fscanf(fp, "%s %d", skip_c, &num_angles);
  fscanf(fp, "%s %d", skip_c, &num_mprims);

  int i, j, k;
  for (i = 0; i < NUMOFDIRS; ++i) {
    for (j = 0; j < NUMOFPRIMS; ++j) {
      fscanf(fp, "%s %d", skip_c, &skip_f);
      for (k = 0; k < NUMOFINTERSTATES; ++k) {
        fscanf(fp, "%f %f %f", &mprim[i][j][k][0], &mprim[i][j][k][1],
               &mprim[i][j][k][2]);
      }
    }
  }
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])

{

  /* Read motion primtives */
  PrimArray motion_primitives;
  parseMotionPrimitives(motion_primitives);

  /* Check for proper number of arguments */
  if (nrhs != 3) {
    mexErrMsgIdAndTxt("MATLAB:planner:invalidNumInputs",
                      "Three input arguments required.");
  } else if (nlhs != 1) {
    mexErrMsgIdAndTxt("MATLAB:planner:maxlhs", "One output argument required.");
  }

  /* get the dimensions of the map and the map matrix itself*/
  int x_size = mxGetM(MAP_IN);
  int y_size = mxGetN(MAP_IN);
  double *map = mxGetPr(MAP_IN);

  /* get the dimensions of the robotpose and the robotpose itself*/
  int robotpose_M = mxGetM(ROBOT_IN);
  int robotpose_N = mxGetN(ROBOT_IN);
  if (robotpose_M != 1 || robotpose_N != 3) {
    mexErrMsgIdAndTxt("MATLAB:planner:invalidrobotpose",
                      "robotpose vector should be 1 by 3.");
  }
  double *robotposeV = mxGetPr(ROBOT_IN);
  float robotposeX = (float)robotposeV[0];
  float robotposeY = (float)robotposeV[1];
  float robotposeTheta = (float)robotposeV[2];

  /* get the dimensions of the goalpose and the goalpose itself*/
  int goalpose_M = mxGetM(GOAL_IN);
  int goalpose_N = mxGetN(GOAL_IN);
  if (goalpose_M != 1 || goalpose_N != 3) {
    mexErrMsgIdAndTxt("MATLAB:planner:invalidgoalpose",
                      "goalpose vector should be 1 by 3.");
  }
  double *goalposeV = mxGetPr(GOAL_IN);
  float goalposeX = (float)goalposeV[0];
  float goalposeY = (float)goalposeV[1];

  /* Create a matrix for the return action */
  ACTION_OUT = mxCreateNumericMatrix(1, 1, mxINT8_CLASS, mxREAL);
  int *action_ptr = (int *)mxGetData(ACTION_OUT);

  /* Do the actual planning in a subroutine */
  A_Star_Planner(map, x_size, y_size, robotposeX, robotposeY, robotposeTheta,
                 goalposeX, goalposeY, motion_primitives, &action_ptr[0]);
  return;
}
