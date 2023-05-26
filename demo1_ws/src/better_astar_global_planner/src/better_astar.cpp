/* 
The improved Astar algorithm(planner) applied in ROS
Author：Grizi-ju
Tutorial：https://www.bilibili.com/video/BV1z94y1d7p3?spm_id_from=333.999.0.0
Reference：http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS
*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>

#include "better_astar.h"

#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(BAstar_planner::BAstarPlannerROS, nav_core::BaseGlobalPlanner)

int value;
int mapSize;
bool *OGM;
static const float INFINIT_COST = INT_MAX; //!< cost of non connected nodes
float infinity = std::numeric_limits<float>::infinity();
float tBreak; // coefficient for breaking ties
ofstream MyExcelFile("BA_result.xlsx", ios::trunc);

int clock_gettime(clockid_t clk_id, struct timespect *tp);

/*
这段代码实现了计算两个timespec类型时间变量之间的时间差，并返回一个新的timespec类型时间变量。

首先，在函数的参数中传入了两个timespec类型时间变量start和end。

接下来，判断end.tv_nsec减去start.tv_nsec是否小于0，如果小于0，说明end.tv_nsec跨越了一个秒，需要计算进去，即temp.tv_sec减1，temp.tv_nsec加上end.tv_nsec - start.tv_nsec + 1000000000。

如果大于等于0，说明end.tv_nsec没有跨越秒，直接计算temp.tv_sec为end.tv_sec减去start.tv_sec，temp.tv_nsec为end.tv_nsec减去start.tv_nsec。

最后，返回计算出的时间差temp。
*/
timespec diff(timespec start, timespec end)
{
  timespec temp;
  if ((end.tv_nsec - start.tv_nsec) < 0)
  {
    temp.tv_sec = end.tv_sec - start.tv_sec - 1;
    temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
  }
  else
  {
    temp.tv_sec = end.tv_sec - start.tv_sec;
    temp.tv_nsec = end.tv_nsec - start.tv_nsec;
  }
  return temp;
}

inline vector<int> findFreeNeighborCell(int CellID);

//Default Constructor  这些构造函数用于初始化 BAstarPlannerROS 类的对象，并实现不同的初始化方式。
//namespace BAstar_planner { ... } 定义了命名空间 BAstar_planner，这个命名空间用于避免不同模块之间的命名冲突
namespace BAstar_planner
{
   //BAstarPlannerROS::BAstarPlannerROS() 是类 BAstarPlannerROS 的默认构造函数，该函数没有任何参数并且在函数体中没有执行任何操作
  BAstarPlannerROS::BAstarPlannerROS()
  {
  }
  // /BAstarPlannerROS::BAstarPlannerROS(ros::NodeHandle &nh) 
  //是带有 ros::NodeHandle 引用参数的构造函数，它将传递进来的 ros::NodeHandle 对象赋值给类中定义的 ROSNodeHandle 属性。
  BAstarPlannerROS::BAstarPlannerROS(ros::NodeHandle &nh)  
  {
    ROSNodeHandle = nh;
  }
  //BAstarPlannerROS::BAstarPlannerROS(std::string name, costmap_2d::Costmap2DROS *costmap_ros) 
  //是带有 std::string 和 costmap_2d::Costmap2DROS * 参数的构造函数，用于初始化 BAstarPlannerROS 类的对象。该函数调用了 initialize 函数对类的属性进行初始化。
  BAstarPlannerROS::BAstarPlannerROS(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    initialize(name, costmap_ros);
  }
/*
这段代码是一个基于ROS的BAStar路径规划器的初始化函数。其主要功能是初始化路径规划器的各项参数，包括代价地图、节点间距、起点终点等。
*/
  void BAstarPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    initialized_ = false;//表示路径规划器的初始化状态为未完成
    if (!initialized_)//判断路径规划器是否已经完成初始化，如果没有，则执行以下代码
    {
      costmap_ros_ = costmap_ros;//用以获取代价地图和其它相关信息
      costmap_ = costmap_ros_->getCostmap();//获取代价地图
      
      ros::NodeHandle private_nh("~/" + name);//以 "~/" + name 的方式创建 ROS 节点句柄。
      _plan_pub = private_nh.advertise<nav_msgs::Path>("global_plan", 1);//创建 ROS 订阅者，用于发布全局路径。 
      _frame_id = costmap_ros->getGlobalFrameID();// 获取当前代价地图的坐标系名称。
      
      originX = costmap_->getOriginX();
      originY = costmap_->getOriginY();//获取代价地图的原点位置坐标。

      width = costmap_->getSizeInCellsX();
      height = costmap_->getSizeInCellsY();//获取代价地图的宽和高。
      resolution = costmap_->getResolution();//获取代价地图的分辨率。
      mapSize = width * height;//计算代价地图的大小（即总像素数）。
      tBreak = 1 + 1 / (mapSize);//计算每个节点之间的距离，用以控制节点数量和运行时间。
      value = 0; //初始化节点的代价值。

      OGM = new bool[mapSize];//动态分配内存用于记录每个像素是否可通过，其中 OGM 为一个 bool 型指针。
      for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)//遍历代价地图的每一个像素。
      {
        for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
        {
          unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));//获取当前像素的代价值。
          //cout<<cost;
          if (cost == 0)
            OGM[iy * width + ix] = true;//将该像素是否可通过的信息存储到数组 OGM 中，如当前像素的代价值为 0，则将其标记为可通过
          else
            OGM[iy * width + ix] = false;
        }
      }
      //这段代码是在使用 C++ 的文件输出流对象 MyExcelFile 将一行文本写入到文件中。该行文本包括了多个字段，用制表符 \t 分隔。
      MyExcelFile << "StartID\tStartX\tStartY\tGoalID\tGoalX\tGoalY\tPlannertime(ms)\tpathLength\tnumberOfCells\t" << endl;

      ROS_INFO("BAstar planner initialized successfully");//
      initialized_ = true;//表示路径规划器的初始化状态完成
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }
   //这是一个ROS机器人路径规划模块中的makePlan函数，用于根据起点和终点规划路径。
  bool BAstarPlannerROS::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                  std::vector<geometry_msgs::PoseStamped> &plan)
  {

    if (!initialized_)//判断模块是否已经初始化
    {
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }
    //使用ROS_DEBUG()输出调试信息，以便于后期跟踪问题
    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
              goal.pose.position.x, goal.pose.position.y);
    //清空存储路径规划结果的plan变量，
    plan.clear();
    //检查目标点goal是否在规划地图的全局坐标系中。如果不在，则返回错误信息。
    if (goal.header.frame_id != costmap_ros_->getGlobalFrameID())
    {
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
                costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }
    
    /*
    将 ROS 消息中的姿态信息 goal 和 start 转换为 tf::Stamped<tf::Pose> 类型，并将转换后的结果分别存储在 goal_tf 和 start_tf 中
    */
    tf::Stamped<tf::Pose> goal_tf;
    tf::Stamped<tf::Pose> start_tf;

    poseStampedMsgToTF(goal, goal_tf);
    poseStampedMsgToTF(start, start_tf);

    // convert the start and goal positions
    //将起点和终点的位置转换为地图上的格子索引。
    //这里使用了convertToCellIndex()和convertToCoordinate()函数完成坐标和格子索引之间的转换。
    //这两个函数分别用于将坐标转换为格子索引，以及将格子索引转换为坐标。
    float startX = start.pose.position.x;
    float startY = start.pose.position.y;

    float goalX = goal.pose.position.x;
    float goalY = goal.pose.position.y;

    getCorrdinate(startX, startY);
    getCorrdinate(goalX, goalY);

    int startCell;
    int goalCell;
    //判断起点和终点是否在地图内部。如果不在，则返回错误信息。这里使用了isCellInsideMap()函数对起点和终点是否在地图内部进行判断。
    if (isCellInsideMap(startX, startY) && isCellInsideMap(goalX, goalY))
    {
      startCell = convertToCellIndex(startX, startY);

      goalCell = convertToCellIndex(goalX, goalY);

      MyExcelFile << startCell << "\t" << start.pose.position.x << "\t" << start.pose.position.y << "\t" << goalCell << "\t" << goal.pose.position.x << "\t" << goal.pose.position.y;
    }
    else
    {
      ROS_WARN("the start or goal is out of the map");
      return false;
    }

    /////////////////////////////////////////////////////////

    // call global planner
    //如果起点和终点都合法，则调用BAstarPlanner函数进行路径规划
    if (isStartAndGoalCellsValid(startCell, goalCell))
    {

      vector<int> bestPath;
      bestPath.clear();
    /*
    BAstarPlanner函数会返回一组最优路径的格子索引序列bestPath。
    如果能够得到一条有效的路径，则将其转换为具体的坐标，并记录路径长度和路径数量，同时发布出去。
    */
      bestPath = BAstarPlanner(startCell, goalCell);

      //if the global planner find a path
      if (bestPath.size() > 0)
      {
       
        // convert the path

        for (int i = 0; i < bestPath.size(); i++)
        {

          float x = 0.0;
          float y = 0.0;

          int index = bestPath[i];

          convertToCoordinate(index, x, y);
          
          geometry_msgs::PoseStamped pose = goal;
          /*
          PoseStamped表示一个具有时间戳（stamp）的位姿信息，包括位置（position）和方向（orientation），
          而geometry_msgs则提供了一组常见的几何原语（如点、向量和姿态）。这些消息类型被设计为提供一个通用的数据类型，并在整个系统中促进互操作性。
          */
          //x,y,z的坐标
          pose.pose.position.x = x;
          pose.pose.position.y = y;
          pose.pose.position.z = 0.0;
          //四元数位姿
          pose.pose.orientation.x = 0.0;
          pose.pose.orientation.y = 0.0;
          pose.pose.orientation.z = 0.0;
          pose.pose.orientation.w = 1.0;

          plan.push_back(pose);
        }
         //如果能够得到一条有效的路径，则将其转换为具体的坐标，并记录路径长度和路径数量，同时发布出去。
        float path_length = 0.0;
        
        std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
        
        geometry_msgs::PoseStamped last_pose;
        last_pose = *it;//last_pose记录前一个点的数据
        it++;
        //然后使用迭代器遍历plan，计算每两个相邻的点之间的直线距离，将其累加到path_length变量中
        for (; it != plan.end(); ++it)
        {
          path_length += hypot((*it).pose.position.x - last_pose.pose.position.x,
                               (*it).pose.position.y - last_pose.pose.position.y);
            /*
            hypot是一个计算两个数的欧几里得距离的函数。在C++和MATLAB等编程语言中都有现成的实现，可以直接调用
            */
          last_pose = *it;//更新上一个点数据
        }
        cout << "The global path length: " << path_length << " meters" << endl;
        MyExcelFile << "\t" << path_length << "\t" << plan.size() << endl;
        //publish the plan
        /*
        将路径规划结果发布出去。这里发布的消息类型是nav_msgs::Path，表示一条由一系列位姿点构成的路径。
        nav_msgs::Path中的位姿点是geometry_msgs::PoseStamped类型的，
        包含位置和时间戳等信息。在发布之前，需要将plan中的位姿点转换为path.poses，并设置路径的帧ID和时间戳。
        */
        nav_msgs::Path path;
        path.poses.resize(plan.size());
        
        if (plan.empty())
        {
          //still set a valid frame so visualization won't hit transform issues
          path.header.frame_id = _frame_id;
          path.header.stamp = ros::Time::now();
        }
        else
        {
          //path.header.frame_id表示路径规划结果所在的坐标系名称。
          path.header.frame_id = plan[0].header.frame_id;
          path.header.stamp = plan[0].header.stamp;
        }

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        /*
        这段代码是将路径规划结果转化为ROS可识别的nav_msgs::Path消息，并使用ROS话题发布机制将其发布出去。
        首先，使用一个循环遍历plan中的每个位姿点，并将它们存放在一个path.poses数组中，其中path.poses的数据类型是std::vector<geometry_msgs::PoseStamped>。 
        然后，使用_plan_pub.publish(path)的语句将path变量的内容发布到与该publisher相关联的话题上。这里的_plan_pub是一个ros::Publisher对象，表示发布路径规划结果的ROS话题。具体来说，这个条件语句会把机器人的路径规划结果发布到话题中，以便其他机器人或节点可以订阅这个话题，从而接收到机器人的路径规划信息。
        最后，函数返回true，表示路径规划成功完成并已经将规划结果发布出去。
        */
        for (unsigned int i = 0; i < plan.size(); i++)
        {
          path.poses[i] = plan[i];
        }

        _plan_pub.publish(path);
        return true;
      }
      //最后，如果找不到有效的路径，则返回错误信息。
      else
      {
        ROS_WARN("The planner failed to find a path, choose other goal position");
        return false;
      }
    }

    else
    {
      ROS_WARN("Not valid start or goal");
      return false;
    }
  }
  void BAstarPlannerROS::getCorrdinate(float &x, float &y)
  {
    //originX,originY是原点的偏移量
    x = x - originX;
    y = y - originY;
  }
   
   /*
   该函数输入参数为栅格地图中的某个点的横纵坐标，输出参数为该点所在网格的索引。
   因此，convertToCellIndex函数的作用就是，将机器人当前实际坐标转换为其在栅格地图中的网格坐标，并返回机器人当前所处网格的索引。
   */
  int BAstarPlannerROS::convertToCellIndex(float x, float y)
  {

    int cellIndex;

    float newX = x / resolution;//resolution是分辨率
    float newY = y / resolution;

    cellIndex = getCellIndex(newY, newX);

    return cellIndex;
  }
   
   /*
   这段代码是将在栅格地图中获取的网格坐标转换为机器人在实际世界中的坐标(x, y)。
   */
  void BAstarPlannerROS::convertToCoordinate(int index, float &x, float &y)
  {

    x = getCellColID(index) * resolution;

    y = getCellRowID(index) * resolution;

    x = x + originX;
    y = y + originY;
  }
   
   //判断点是否在地图内
  bool BAstarPlannerROS::isCellInsideMap(float x, float y)
  {
    bool valid = true;

    if (x > (width * resolution) || y > (height * resolution))
      valid = false;

    return valid;
  }
  /*
  这段代码用于将栅格地图中的坐标(mx, my)转换为实际坐标系下的坐标(wx, wy)。
  函数mapToWorld将输入的栅格地图坐标(mx, my)转换为实际坐标系下的坐标，并将结果存储在输出变量wx和wy中。
  */
  void BAstarPlannerROS::mapToWorld(double mx, double my, double &wx, double &wy)
  {
    costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
    wx = costmap->getOriginX() + mx * resolution;
    wy = costmap->getOriginY() + my * resolution;
  }
  /*
  这段代码是基于BA*算法实现的全局路径规划器。
  函数BAstarPlanner接收机器人当前的栅格地图中的起始位置(startCell)和目标位置(goalCell)，并返回一条最佳路径。
  */
  vector<int> BAstarPlannerROS::BAstarPlanner(int startCell, int goalCell)
  {
    
    vector<int> bestPath;
   
    //float g_score [mapSize][2];
    float g_score[mapSize];
    
    for (uint i = 0; i < mapSize; i++)
      g_score[i] = infinity;//在开始时，将g_score的所有值都设置为无穷大(infinity)。
    
    timespec time1, time2;
    /* take current time here */

    //记录程序开始时间为time1，调用findPath函数来寻找起始位置到目标位置的最佳路径，并将g_score传递给该函数进行更新。
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
    
    bestPath = findPath(startCell, goalCell, g_score);
    //函数执行完后，记录程序结束时间为time2，计算出寻找路径所花费的时间，并将最佳路径存储到bestPath中。
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);

    cout << "time to generate best global path by the new planner A* = " << (diff(time1, time2).tv_sec) * 1e3 + (diff(time1, time2).tv_nsec) * 1e-6 << " microseconds" << endl;
    //最后，输出路径生成所花费的时间，并将其写入到一个Excel文件中。
    MyExcelFile << "\t" << (diff(time1, time2).tv_sec) * 1e3 + (diff(time1, time2).tv_nsec) * 1e-6;

    return bestPath;
  }

  /*******************************************************************************/
  //Function Name: findPath
  //Inputs: the map layout, the start and the goal Cells and a boolean to indicate if we will use break ties or not
  //Output: the best path
  //Description: it is used to generate the robot free path
  /*********************************************************************************/
  /*被调用时传入了起始位置startCell，目标位置goalCell以及代价值数组g_score[]*/
  //A*算法实现的主要代码

  //怎么感觉像是Dijkstra算法
  vector<int> BAstarPlannerROS::findPath(int startCell, int goalCell, float g_score[])
  {
    value++;
    vector<int> bestPath;
    vector<int> emptyPath;
    cells CP;
    /*
    multiset是C++ STL中的一个关联容器，实现了自动排序功能，可以存储多个相同值的元素。
    */
    //cells
    multiset<cells> OPL;
    int currentCell;
    
    //calculate g_score and f_score of the start position
    g_score[startCell] = 0;
    CP.currentCell = startCell;
    //F=G+H A*算法的公式
    CP.fCost = g_score[startCell] + calculateHCost(startCell, goalCell);
    
    //add the start cell to the open list
    //
    OPL.insert(CP);
    currentCell = startCell;
    
    //while the open list is not empty continuie the search or g_score(goalCell) is equal to infinity
    //当openlist列表不为空和还没搜索到终点时
    while (!OPL.empty() && g_score[goalCell] == infinity)
    {
      //choose the cell that has the lowest cost fCost in the open set which is the begin of the multiset
      currentCell = OPL.begin()->currentCell;
      //remove the currentCell from the openList
      OPL.erase(OPL.begin());
      //search the neighbors of the current Cell
      vector<int> neighborCells;
      neighborCells = findFreeNeighborCell(currentCell);
      for (uint i = 0; i < neighborCells.size(); i++) //for each neighbor v of current cell
      {
        // if the g_score of the neighbor is equal to INF: unvisited cell
        if (g_score[neighborCells[i]] == infinity)
        {
          g_score[neighborCells[i]] = g_score[currentCell] + getMoveCost(currentCell, neighborCells[i]);//更新G值
          addNeighborCellToOpenList(OPL, neighborCells[i], goalCell, g_score);//把邻近结点添加到openlist列表中
        } //end if
      }   //end for
    }     //end while

    if (g_score[goalCell] != infinity) // if g_score(goalcell)==INF : construct path
    //终点的G值不是无穷大时，就是找到了最好路径
    {
      bestPath = constructPath(startCell, goalCell, g_score);
      return bestPath;
    }
    else                                                                                                                                                                                                                                                                                                                                                                                                        
    {
      cout << "Failure to find a path !" << endl;
      return emptyPath;
    }
  }

  /*******************************************************************************/
  //Function Name: constructPath
  //Inputs: the start and the goal Cells
  //Output: the best path                                                                                                                            
  //Description: it is used to construct the robot path
  /*********************************************************************************/
  /*
  这段代码是用于构建路径的，主要包括以下内容：
根据起点和终点，利用A*算法等搜索最优路径，并获取每个节点的代价值g_score。
根据g_score数组，逆向生成可行的最优路径bestPath和原始路径path。
  在进行路径构建时，首先会使用A*算法等搜索最优路径，并记录每个节点的代价值g_score。然后，·  
  */
  vector<int> BAstarPlannerROS::constructPath(int startCell, int goalCell, float g_score[])
  {
    vector<int> bestPath;
    vector<int> path;
    //path.begin() + bestPath.size()表示一个迭代器，它指向的是bestPath列表的末尾位置
    //bestPath.size()表示bestPath列表当前的长度，二者相加就是bestPath列表的末尾位置
    
    path.insert(path.begin() + bestPath.size(), goalCell);//vector容器中插入终点
    int currentCell = goalCell;
    
    while (currentCell != startCell)
    {
      vector<int> neighborCells;
      neighborCells = findFreeNeighborCell(currentCell);
      
      vector<float> gScoresNeighbors;
      for (uint i = 0; i < neighborCells.size(); i++)
      gScoresNeighbors.push_back(g_score[neighborCells[i]]);
      //每次循环中，寻找当前currentCell的自由相邻单元格(neighborCells)，并计算出它们的代价值(gScoresNeighbors)，将代价值最小的单元格作为当前位置。
      int posMinGScore = distance(gScoresNeighbors.begin(), min_element(gScoresNeighbors.begin(), gScoresNeighbors.end()));
      currentCell = neighborCells[posMinGScore];//找到代价值(gScoresNeighbors)最小的点，更新当前结点
      
      //insert the neighbor in the path
      //把节点插入到路径中
      path.insert(path.begin() + path.size(), currentCell);
    }
    //这段代码是将路径path反向生成可行的最优路径bestPath的过程。
    for (uint i = 0; i < path.size(); i++)
      bestPath.insert(bestPath.begin() + bestPath.size(), path[path.size() - (i + 1)]);

    return bestPath;
  }

  /*******************************************************************************/
  //Function Name: calculateHCost
  //Inputs:the cellID and the goalCell
  //Output: the distance between the current cell and the goal cell
  //Description: it is used to calculate the hCost
  /*********************************************************************************/
  float BAstarPlannerROS::calculateHCost(int cellID, int goalCell)
  {    
    double h;    //heuristic(n)
    double distance;  //distance
    double w;    //weight(n)

    int x1=getCellRowID(goalCell);
    int y1=getCellColID(goalCell);
    int x2=getCellRowID(cellID);
    int y2=getCellColID(cellID);

    w = 3.0;
    //distance = abs(x1-x2)+abs(y1-y2);         //Manhatten Distance   曼哈顿距离
    //distance = sqrt(pow(x1,x2)+pow(y1,y2));       //Euclidean Distance 欧几里得吗距离
    distance = min(abs(x1-x2),abs(y1-y2))  *sqrt(2)  + max(abs(x1-x2),abs(y1-y2)) - min(abs(x1-x2),abs(y1-y2));   //Diagonal Distance  对角线距离
    
    h = w * distance;
    
    return h;    //return h to return 0    meaning that improved A* algorithm to Dijkstra
    //返回的h=0,就是Dijkstra算法
    //返回是h，就是A*算法
  }


  /*******************************************************************************/
  //Function Name: addNeighborCellToOpenList
  //Inputs: the open list, the neighbors Cell, the g_score matrix, the goal cell
  //Output:
  //Description: it is used to add a neighbor Cell to the open list
  /*********************************************************************************/
  //把邻近结点添加到openlist列表中，openlist的结点包含F值信息
  void BAstarPlannerROS::addNeighborCellToOpenList(multiset<cells> &OPL, int neighborCell, int goalCell, float g_score[])
  {
    cells CP;
    CP.currentCell = neighborCell; //insert the neighbor cell
    CP.fCost = g_score[neighborCell] + calculateHCost(neighborCell, goalCell);
    OPL.insert(CP);
    //multiset<cells>::iterator it = OPL.lower_bound(CP);
    //multiset<cells>::iterator it = OPL.upper_bound(CP);
    //OPL.insert( it, CP  );
  }

  /*******************************************************************************
 * Function Name: findFreeNeighborCell
 * Inputs: the row and columun of the current Cell
 * Output: a vector of free neighbor cells of the current cell
 * Description:it is used to find the free neighbors Cells of a the current Cell in the grid
 * Check Status: Checked by Anis, Imen and Sahar
*********************************************************************************/

  vector<int> BAstarPlannerROS::findFreeNeighborCell(int CellID)
  {
    int rowID = getCellRowID(CellID);
    int colID = getCellColID(CellID);
    int neighborIndex;
    vector<int> freeNeighborCells;
  /*
  函数使用两个变量 rowID 和 colID 来存储该单元格的行列坐标，然后通过循环遍历以该单元格为中心的 
  3×3的矩阵中的所有单元格，寻找空闲邻居单元格。
  */
    for (int i = -1; i <= 1; i++)
      for (int j = -1; j <= 1; j++)
      {
        //check whether the index is valid
        //检查索引是否有效
        //第一部分用来检查邻居单元格是否在地图的边界之内，确保计算出的行列坐标不会越界
        if ((rowID + i >= 0) && (rowID + i < height) && (colID + j >= 0) && (colID + j < width) && (!(i == 0 && j == 0)))
        {
          neighborIndex = getCellIndex(rowID + i, colID + j);
          if (isFree(neighborIndex))
            freeNeighborCells.push_back(neighborIndex);
        }
      }
    return freeNeighborCells;
  }

  /*******************************************************************************/
  //Function Name: isStartAndGoalCellsValid
  //Inputs: the start and Goal cells
  //Output: true if the start and the goal cells are valid
  //Description: check if the start and goal cells are valid
  /*********************************************************************************/
  //检查开始节点和终点是否有效
  bool BAstarPlannerROS::isStartAndGoalCellsValid(int startCell, int goalCell)
  {
    bool isvalid = true;
    bool isFreeStartCell = isFree(startCell);
    bool isFreeGoalCell = isFree(goalCell);
    if (startCell == goalCell)
    {
      //cout << "The Start and the Goal cells are the same..." << endl;
      isvalid = false;
    }
    else
    {
      if (!isFreeStartCell && !isFreeGoalCell)
      {
        //cout << "The start and the goal cells are obstacle positions..." << endl;
        isvalid = false;
      }
      else
      {
        if (!isFreeStartCell)
        {
          //cout << "The start is an obstacle..." << endl;
          isvalid = false;
        }
        else
        {
          if (!isFreeGoalCell)
          {
            //cout << "The goal cell is an obstacle..." << endl;
            isvalid = false;
          }
          else
          {
            if (findFreeNeighborCell(goalCell).size() == 0)
            {
              //cout << "The goal cell is encountred by obstacles... "<< endl;
              isvalid = false;
            }
            else
            {
              if (findFreeNeighborCell(startCell).size() == 0)
              {
                //cout << "The start cell is encountred by obstacles... "<< endl;
                isvalid = false;
              }
            }
          }
        }
      }
    }
    return isvalid;
  }
  //用于计算网格地图中两个单元格之间的移动代价（cost）
  float BAstarPlannerROS::getMoveCost(int i1, int j1, int i2, int j2)
  {
    float moveCost = INFINIT_COST; //start cost with maximum value. Change it to real cost of cells are connected
    //if cell2(i2,j2) exists in the diagonal of cell1(i1,j1)
    if ((j2 == j1 + 1 && i2 == i1 + 1) || (i2 == i1 - 1 && j2 == j1 + 1) || (i2 == i1 - 1 && j2 == j1 - 1) || (j2 == j1 - 1 && i2 == i1 + 1))
    {
      //如果相对位置是对角线位置，则将 moveCost 设为 DIAGNOL_MOVE_COST 或 1.4
      //moveCost = DIAGONAL_MOVE_COST;
      moveCost = 1.4;
    }
    //if cell 2(i2,j2) exists in the horizontal or vertical line with cell1(i1,j1)
    else
    {
      if ((j2 == j1 && i2 == i1 - 1) || (i2 == i1 && j2 == j1 - 1) || (i2 == i1 + 1 && j2 == j1) || (i1 == i2 && j2 == j1 + 1))
      {
        //如果相对位置是在同一行或同一列，即垂直或水平位置，则将 moveCost 设为常量 MOVE_COST 或 1。
        //moveCost = MOVE_COST;
        moveCost = 1;
      }
    }
    return moveCost;
  }
  
  //计算两点之间的移动代价
  float BAstarPlannerROS::getMoveCost(int CellID1, int CellID2)
  {
    int i1 = 0, i2 = 0, j1 = 0, j2 = 0;

    i1 = getCellRowID(CellID1);
    j1 = getCellColID(CellID1);
    i2 = getCellRowID(CellID2);
    j2 = getCellColID(CellID2);

    return getMoveCost(i1, j1, i2, j2);
  }

  //verify if the cell(i,j) is free
  bool BAstarPlannerROS::isFree(int i, int j)
  {
    int CellID = getCellIndex(i, j);
    return OGM[CellID];
  }

  //verify if the cell(i,j) is free
  bool BAstarPlannerROS::isFree(int CellID)
  {
    return OGM[CellID];
  }
};

bool operator<(cells const &c1, cells const &c2) { return c1.fCost < c2.fCost; }