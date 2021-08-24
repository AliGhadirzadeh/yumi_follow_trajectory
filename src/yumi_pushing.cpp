#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <boost/lexical_cast.hpp>

using namespace std;

vector<double> right_arm_joint_positions;
vector<double> left_arm_joint_positions;
vector<double> right_arm_joint_velocity;
vector<double> left_arm_joint_velocity;
vector< vector<double> > trajectory_command;
long int trajectory_time_index = -1;
double trajectory_start_time;

# define TIME_INDEX       0
# define JOINT_POS_INDEX  1
# define JOINT_VEL_INDEX  8

// function declarations
void print_joint_values();
vector < vector <double> > load_data(string filename);
vector < vector <double> > str2vector(string str);
double limit_joint_acceleration(double new_vel, double curr_vel);

void joint_state_callback(const sensor_msgs::JointState & msg)
{
    int right_arm_indecis[7] = {1,3,13,5,7,9,11};
    int left_arm_indecis[7] = {0,2,12,4,6,8,10};
    for(int i = 0;i < 7; i++)
    {
        right_arm_joint_positions[i] = msg.position[right_arm_indecis[i]];
        left_arm_joint_positions[i] = msg.position[left_arm_indecis[i]];
        right_arm_joint_velocity[i] = msg.velocity[right_arm_indecis[i]];
        left_arm_joint_velocity[i] = msg.velocity[left_arm_indecis[i]];
    }
}

void update_traj_callback(const std_msgs::String & msg)
{
  string traj_str = msg.data;
  trajectory_command = str2vector(traj_str);
  trajectory_time_index = 0;
  trajectory_start_time = ros::Time::now().toSec();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "yumi_pushing_controller_node"); // init the ROS node

  double right_arm_initial_positions[7] = {1.89, -0.996, -1.75, 0.211, -2.15, 1.24, 1.26};
  right_arm_joint_positions.resize(7);
  right_arm_joint_velocity.resize(7);
  left_arm_joint_positions.resize(7);
  left_arm_joint_velocity.resize(7);

  ros::NodeHandle joint_node;
  ros::Subscriber joint_subscriber = joint_node.subscribe("/yumi/joint_states", 1, joint_state_callback);

  vector<ros::NodeHandle> r_velocity_command_node(7);
  vector<ros::Publisher> r_velocity_command_pub(7);
  vector<ros::NodeHandle> l_velocity_command_node(7);
  vector<ros::Publisher> l_velocity_command_pub(7);
  string command_topic;
  int urdf_order[7] = {1,2,7,3,4,5,6};
  for(int i = 0; i < 7; i++)
  {
    command_topic = "yumi/joint_vel_controller_" + to_string(urdf_order[i]) + "_r/command";
    r_velocity_command_pub[i] = r_velocity_command_node[i].advertise<std_msgs::Float64>(command_topic.c_str(), 10);
    command_topic = "yumi/joint_vel_controller_" + to_string(urdf_order[i]) + "_l/command";
    l_velocity_command_pub[i] = l_velocity_command_node[i].advertise<std_msgs::Float64>(command_topic.c_str(), 10);
  }
  ros::Publisher r_gripper_pub, l_gripper_pub;
  ros::NodeHandle r_gripper_node, l_gripper_node;
  r_gripper_pub = r_gripper_node.advertise<std_msgs::Float64>("/yumi/gripper_r_effort_cmd", 10);
  l_gripper_pub = l_gripper_node.advertise<std_msgs::Float64>("/yumi/gripper_l_effort_cmd", 10);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  srand( time(NULL) );

  sleep(1);

  std_msgs::Float64 cmd;
  // move to the initial joint position
  bool all_fine = false;
  while(all_fine == false)
  {
    all_fine = true;
    for (int i = 0; i < 7; i++)
    {
      cmd.data = 2.0*(right_arm_initial_positions[i]-right_arm_joint_positions[i]);
      cmd.data = limit_joint_acceleration(cmd.data, right_arm_joint_velocity[i]);
      r_velocity_command_pub[i].publish(cmd);
      if(abs(right_arm_initial_positions[i]-right_arm_joint_positions[i])>0.005)
        all_fine = false;
    }
    usleep(50000);
  }
  print_joint_values();

  cout << "waiting for the trajectories to be sent" << endl;
  while(trajectory_time_index < 0)
    usleep(100);

  cout << endl;
  for (int i = 0; i < trajectory_command.size(); i++)
  {
    for (int j = 0; j < trajectory_command[i].size(); j++)
    {
        cout << trajectory_command[i][j] << ", ";
    }
    cout << endl;
  }
  return 0;
  
  while (ros::ok()) {
    double current_time = ros::Time::now().toSec() - trajectory_start_time;
    if (current_time > trajectory_command[TIME_INDEX][trajectory_time_index])
    {
      for (int i = 0; i < 7; i++)
      {
        cmd.data = 0.1*(trajectory_command[i+JOINT_POS_INDEX][trajectory_time_index]-right_arm_joint_positions[i]);  // feedback term
        cmd.data += 1.0*trajectory_command[i+JOINT_VEL_INDEX][trajectory_time_index];                                // feedforward term
        cmd.data = limit_joint_acceleration(cmd.data, right_arm_joint_velocity[i]);
        r_velocity_command_pub[i].publish(cmd);
      }
      trajectory_time_index++;
      if (trajectory_time_index >= traj.size())
      {
        cout << "program completed :)" << endl;
        break;
      }
    }
  }
  cmd.data = 0.0;
  for (int i = 0; i < 7; i++)
  {
    l_velocity_command_pub[i].publish(cmd);
    r_velocity_command_pub[i].publish(cmd);
  }
  std::cout << "The node terminated successfully" << std::endl;
  return 0;
}


void print_joint_values()
{
  std::cout << std::setprecision(3);
  std::cout << "RIGHT ARM JOINTS: ";
  for(int i = 0; i < 7; i++)
  {
    if (right_arm_joint_positions[i] >= 0)
      std::cout << " ";
    std::cout << right_arm_joint_positions[i] << ", ";
  }
  std::cout << std::endl;
  std::cout << "LEFT ARM JOINTS:  ";
  for(int i = 0; i < 7; i++)
  {
    if (left_arm_joint_positions[i] >= 0)
      std::cout << " ";
    std::cout << left_arm_joint_positions[i] << ", ";
  }
  std::cout << std::endl;
}

vector < vector <double> > load_data(string filename)
{
  vector< vector<double> > data;
  vector <double> row;
  ifstream reader;
  reader.open(filename.c_str());
  string line;
  while (getline (reader,line) )
  {
    std::size_t p = line.find(" ");
    while(p != std::string::npos )
    {
      string value = line.substr(0, p);
      line = line.substr(p+1);
      p = line.find(" ");
      row.push_back(boost::lexical_cast<double>(value));
    }
    row.push_back(boost::lexical_cast<double>(line));
    data.push_back(row);
    row.clear();
  }
  reader.close();
  return data;
}

vector < vector <double> > str2vector(string str)
{
  vector< vector<double> > data;
  vector <double> row;
  std::size_t p_line = str.find('\n');
  while (p_line != std::string::npos)
  {
    string line = str.substr(0, p_line);
    std::size_t p = line.find(" ");
    while(p != std::string::npos)
    {
      string value = line.substr(0, p);
      line = line.substr(p + 1);
      p = line.find(" ");
      row.push_back(boost::lexical_cast<double>(value));
    }
    row.push_back(boost::lexical_cast<double>(line));
    data.push_back(row);
    row.clear();
    str = str.substr(p_line+1)
  }
  return data;
}

double limit_joint_acceleration(double new_vel, double curr_vel)
{
  double cmd = new_vel;
  const double acc_threshold = 0.06;
  if((new_vel - curr_vel) > acc_threshold)
    cmd = acc_threshold;
  else if((new_vel - curr_vel) < (-acc_threshold))
    cmd = -acc_threshold;
  return cmd;
}
