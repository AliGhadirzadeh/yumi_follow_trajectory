#include <ros/ros.h>
// KDL stuff
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

# define TIME_INDEX       0
# define JOINT_POS_INDEX  1
# define JOINT_VEL_INDEX  8
# define GRIPPER_INDEX    15


// function declarations
void print_joint_values();
vector < vector <double> > load_data(string filename);
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

int main(int argc, char **argv) {
  ros::init(argc, argv, "replay_joint_traj"); // init the ROS node

  if (argc < 2)
  {
    fprintf(stderr,"ERROR, filename is not given\n");
    exit(1);
  }
  string traj_filename = argv[1];

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

  // loading the trajectory from the given file
  vector< vector<double> > traj = load_data(traj_filename);

  // initialize the gripper open/close
  if (traj[0][GRIPPER_INDEX]>0)
    cmd.data = 10.0;
  else
    cmd.data = 0.0;
  r_gripper_pub.publish(cmd);

  cout << "initial gripper command sent" << endl;

  // move to the initial joint position
  bool all_fine = false;
  while(all_fine == false)
  {
    all_fine = true;
    for (int i = 0; i < 7; i++)
    {
      cmd.data = 2.0*(traj[0][i+JOINT_POS_INDEX]-right_arm_joint_positions[i]);
      cmd.data = limit_joint_acceleration(cmd.data, right_arm_joint_velocity[i]);
      r_velocity_command_pub[i].publish(cmd);
      if(abs(traj[0][i+JOINT_POS_INDEX]-right_arm_joint_positions[i])>0.005)
        all_fine = false;
    }
    usleep(50000);
  }
  print_joint_values();


  double max_run_duration = 2000;
  double current_time, start_time;
  start_time = ros::Time::now().toSec();
  long int timestep_counter = 1;

  // follow the given trajectory
  while (ros::ok()) {
    current_time = ros::Time::now().toSec() - start_time;
    if (current_time > max_run_duration)
    {
      cout << "program run time exceeds the maximum run duration" << endl;
      break;
    }
    if (current_time > traj[timestep_counter][TIME_INDEX])
    {
      for (int i = 0; i < 7; i++)
      {
        cmd.data = 0.1*(traj[timestep_counter][i+JOINT_POS_INDEX]-right_arm_joint_positions[i]);  // feedback term
        cmd.data += 1.0*traj[timestep_counter][i+JOINT_VEL_INDEX];                                // feedforward term
        cmd.data = limit_joint_acceleration(cmd.data, right_arm_joint_velocity[i]);
        r_velocity_command_pub[i].publish(cmd);
      }
      if (traj[timestep_counter][GRIPPER_INDEX]>0)
        cmd.data = 10.0;
      else
        cmd.data = 0.0;
      r_gripper_pub.publish(cmd);
      
      timestep_counter++;
      if (timestep_counter >= traj.size())
      {
        cout << "program completed the given trajectory" << endl;
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
  r_gripper_pub.publish(cmd);
  l_gripper_pub.publish(cmd);

  std::cout << "Program done!!!" << std::endl;
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
