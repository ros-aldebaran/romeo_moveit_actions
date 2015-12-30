#include <boost/program_options.hpp>
#include <simplepickplace.hpp>

void parse_command_line(int argc, char ** argv,
                        std::string &robot_name_,
                        double &test_step_,
                        double &x_min_,
                        double &x_max_,
                        double &y_min_,
                        double &y_max_,
                        double &z_min_,
                        double &z_max_,
                        std::string &left_arm_name_,
                        std::string &right_arm_name_,
                        bool &verbose_)
{
  std::string robot_name;
  double test_step(0.0);
  double x_min(0.0);
  double x_max(0.0);
  double y_min(0.0);
  double y_max(0.0);
  double z_min(0.0);
  double z_max(0.0);
  std::string left_arm_name("");
  std::string right_arm_name("");
  bool verbose;
  boost::program_options::options_description desc("Configuration");
  desc.add_options()
    ("help", "show this help message")
    ("robot_name", boost::program_options::value<std::string>(&robot_name)->default_value(robot_name_),
     "robot_name")
    ("test_step", boost::program_options::value<double>(&test_step)->default_value(test_step_),
     "test_step")
    ("x_min", boost::program_options::value<double>(&x_min)->default_value(x_min_),
     "x_min")
    ("x_max", boost::program_options::value<double>(&x_max)->default_value(x_max_),
     "x_max")
    ("y_min", boost::program_options::value<double>(&y_min)->default_value(y_min_),
     "y_min")
    ("y_max", boost::program_options::value<double>(&y_max)->default_value(y_max_),
     "y_max")
    ("z_min", boost::program_options::value<double>(&z_min)->default_value(z_min_),
     "z_min")
    ("z_max", boost::program_options::value<double>(&z_max)->default_value(z_max_),
     "z_max")
    ("left_arm_name", boost::program_options::value<std::string>(&left_arm_name)->default_value(left_arm_name),
     "left_arm_name")
    ("right_arm_name", boost::program_options::value<std::string>(&right_arm_name)->default_value(right_arm_name),
     "right_arm_name")
    ("verbose", boost::program_options::value<bool>(&verbose)->default_value(verbose_),
     "verbose")
    /*("depth_frame_id", boost::program_options::value<std::string>(&depth_frame_id)->default_value(m_depth_frame_id),
     "depth_frame_id")*/
    ;
  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);
  robot_name_ = vm["robot_name"].as<std::string>();
  test_step_ = vm["test_step"].as<double>();
  x_min_ = vm["x_min"].as<double>();
  x_max_ = vm["x_max"].as<double>();
  y_min_ = vm["y_min"].as<double>();
  y_max_ = vm["y_max"].as<double>();
  z_min_ = vm["z_min"].as<double>();
  z_max_ = vm["z_max"].as<double>();
  left_arm_name_ = vm["left_arm_name"].as<std::string>();
  right_arm_name_ = vm["right_arm_name"].as<std::string>();
  verbose_ = vm["verbose"].as<bool>();

  ROS_INFO_STREAM("robot name = " << robot_name_);
  /*ROS_INFO_STREAM("test_step= " << test_step_);
  ROS_INFO_STREAM("x_min= " << x_min_);
  ROS_INFO_STREAM("x_max= " << x_max_);
  ROS_INFO_STREAM("y_min= " << y_min_);
  ROS_INFO_STREAM("y_max= " << y_max_);
  ROS_INFO_STREAM("z_min= " << z_min_);
  ROS_INFO_STREAM("z_max= " << z_max_);
  ROS_INFO_STREAM("left_arm_name = " << left_arm_name);
  ROS_INFO_STREAM("right_arm_name = " << right_arm_name);*/

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return ;
  }
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "moveit_simple_action");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Check for verbose flag
  bool verbose = false;
  /*if (argc > 1)
  {
    for (std::size_t i = 0; i < argc; ++i)
    {
      if (strcmp(argv[i], "--verbose") == 0)
      {
        ROS_INFO_STREAM_NAMED("main","Running in VERBOSE mode (slower)");
        verbose = true;
      }
    }
  }*/
  //verbose = true;

  std::string robot_name("romeo");
  double test_step(0.0);
  double x_min(0.0);
  double x_max(0.0);
  double y_min(0.0);
  double y_max(0.0);
  double z_min(0.0);
  double z_max(0.0);
  std::string left_arm_name("left");
  std::string right_arm_name("right");
  parse_command_line(argc, argv, robot_name, test_step,
                     x_min, x_max, y_min, y_max, z_min, z_max,
                     left_arm_name, right_arm_name,
                     verbose);

  srand (time(NULL));

  // Start the pick place node
  moveit_simple_actions::SimplePickPlace server_pickplace(robot_name, test_step,
                                           x_min, x_max, y_min, y_max, z_min, z_max,
                                           left_arm_name, right_arm_name,
                                           verbose);

  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}
