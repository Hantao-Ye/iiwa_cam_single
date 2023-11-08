
// iiwa_cam pkg defined srvs
#include <iiwa_cam/EndEffectorPose.h>
#include <iiwa_cam/EndEffectorState.h>
#include <iiwa_cam/PathRecorder.h>

// iiwa_stack_cam defined msgs srvs acts
#include <iiwa_msgs/CartesianPose.h>
#include <iiwa_msgs/CartesianWrench.h>

// ROS build-in headers
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>

// C++ STL
#include <atomic>
#include <csv2.hpp>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>

namespace cam
{
  constexpr int KUKA_WD_TIMEOUT_MIN = 5; // minimum threshold time of watchdog

  /**
   * @brief Class to store cartesian position in Kuka
   *
   */
  class KukaCartesianPose
  {
  public:
    double cart[7];
    int status;
    uint64_t time_stamp_ns;

    KukaCartesianPose() = default;
    KukaCartesianPose(const iiwa_msgs::CartesianPose &msg)
    {
      cart[0] = msg.poseStamped.pose.position.x;
      cart[1] = msg.poseStamped.pose.position.y;
      cart[2] = msg.poseStamped.pose.position.z;
      cart[3] = msg.poseStamped.pose.orientation.w;
      cart[4] = msg.poseStamped.pose.orientation.x;
      cart[5] = msg.poseStamped.pose.orientation.y;
      cart[6] = msg.poseStamped.pose.orientation.z;

      status = msg.redundancy.status;

      time_stamp_ns = msg.poseStamped.header.stamp.toNSec();
    }

    std::vector<std::string> to_vector() const
    {
      std::vector<std::string> res;
      res.reserve(9);
      for (const auto &cart_pos : cart)
      {
        res.emplace_back(std::to_string(cart_pos));
      }
      res.emplace_back(std::to_string(status));

      res.emplace_back(std::to_string(time_stamp_ns));
      return res;
    }
  };

  /**
   * @brief Class to store cartesian wrench in Kuka
   *
   */
  class KukaWrench
  {
  public:
    double wrench[6];
    uint64_t time_stamp_ns;

    KukaWrench() = default;
    KukaWrench(const iiwa_msgs::CartesianWrench &msg)
    {
      wrench[0] = msg.wrench.force.x;
      wrench[1] = msg.wrench.force.y;
      wrench[2] = msg.wrench.force.z;
      wrench[3] = msg.wrench.torque.x;
      wrench[4] = msg.wrench.torque.y;
      wrench[5] = msg.wrench.torque.z;

      time_stamp_ns = msg.header.stamp.toNSec();
    }

    std::vector<std::string> to_vector() const
    {
      std::vector<std::string> res;
      res.reserve(7);
      for (const auto &val : wrench)
      {
        res.emplace_back(std::to_string(val));
      }
      res.emplace_back(std::to_string(time_stamp_ns));
      return res;
    }
  };

  class KukaRecorder;
  class KukaWatchDog
  {
  private:
    std::mutex *mtx;
    ros::Duration threshold_time;
    ros::Duration sleep_time;
    ros::Time begin_time; // protected by mtx

  public:
    KukaWatchDog() = default;
    KukaWatchDog(KukaRecorder *pr, uint32_t timer = KUKA_WD_TIMEOUT_MIN);
    ~KukaWatchDog();

    void feed();

  private:
    void bark(KukaRecorder *);
    void watcher(KukaRecorder *);
  };

  /**
   * @brief Class to record cartesian position and wrench in Kuka
   *
   */
  class KukaRecorder
  {
    friend KukaWatchDog;
    const int WRENCH_HIS_QUEUE_SIZE = 32; // size of wrench_history queue
    const double WRENCH_LIMIT_Z = 10e0;   // threshold of the force on Z axis

  public:
    bool recorder_state = false; // protected by mtx

    std::mutex *mtx;        // mutex for recorder_state, 2 queue
    std::mutex *wrench_mtx; // mutex for wrench_history

    KukaWatchDog *dog;

  private:
    std::string robot_name;

    ros::Subscriber cart_pos_sub;
    ros::Subscriber wrench_sub;

    ros::Publisher wrench_pub;
    ros::Publisher wrench_limit_pub;

    std::vector<KukaCartesianPose> cart_pos_queue; // protected by mtx
    std::vector<KukaWrench> wrench_queue;          // protected by mtx

    std::ofstream cart_pos_stream;
    std::ofstream wrench_stream;

    geometry_msgs::Pose cart_pose;      // protected by mtx
    geometry_msgs::Twist cart_velocity; // protected by mtx
    int status;                         // protected by mtx
    ros::Time timestamp;                // timestamp

    // protected by wrench_mtx, max size = WRENCH_HIS_QUEUE_SIZE
    std::deque<geometry_msgs::Wrench> wrench_history;

    int wrench_callback_count = 0;
    int cart_pos_callback_count = 0;
    int velocity_cal_cnt = 0;

    /**
     * @brief Callback function for a new CartesianWrench message from Kuka
     *
     * @param msg
     */
    void wrench_callback(const iiwa_msgs::CartesianWrench &msg)
    {
      {
        // covert this wrench message to standard message, in order of visualization
        // selecting the correct frame_id is necessary !
        geometry_msgs::WrenchStamped wrench_msg;
        wrench_msg.wrench = msg.wrench;
        wrench_msg.header.frame_id = robot_name + "_link_7";

        if (abs(msg.wrench.force.z) >= WRENCH_LIMIT_Z)
        {
          wrench_limit_pub.publish(wrench_msg);
        }

        wrench_pub.publish(wrench_msg);
      }

      // as the kuka robot publishes topic at a period of 1 ms (line 340 in
      // ROSBaseApplication.java), slow recording time down to 5 ms
      if (++wrench_callback_count < 5)
        return;
      else
        wrench_callback_count = 0;

      {
        std::lock_guard<std::mutex> lock(*wrench_mtx);

        wrench_history.emplace_back(msg.wrench);

        while (wrench_history.size() > WRENCH_HIS_QUEUE_SIZE)
          wrench_history.pop_front();
      }

      {
        std::lock_guard<std::mutex> lock(*mtx);

        if (!recorder_state)
          return;

        wrench_queue.emplace_back(KukaWrench(msg));

        // select a prime number to avoid collision with the cart_pos_callback()
        // thread
        if (wrench_queue.size() >= 29)
          clean_wrench_queue();
      }
    }

    /**
     * @brief Callback function for a new CartesianPose message from Kuka
     *
     * @param msg
     */
    void cart_pos_callback(const iiwa_msgs::CartesianPose &msg)
    {
      // as the kuka robot publishes topic at a period of 1 ms (line 340 in
      // ROSBaseApplication.java), slow recording time down to 5 ms
      if (++cart_pos_callback_count < 5)
        return;
      else
        cart_pos_callback_count = 0;

      std::lock_guard<std::mutex> lock(*mtx);

      // calculate the velocity for every 50 ms
      if (++velocity_cal_cnt >= 10)
      {
        velocity_cal_cnt = 0;

        double delta_time = (msg.poseStamped.header.stamp - timestamp).toSec();
        const geometry_msgs::Pose &msg_pose = msg.poseStamped.pose;
        cart_velocity.linear.x = (msg_pose.position.x - cart_pose.position.x) / delta_time;
        cart_velocity.linear.y = (msg_pose.position.y - cart_pose.position.y) / delta_time;
        cart_velocity.linear.z = (msg_pose.position.z - cart_pose.position.z) / delta_time;

        double p0 = (msg_pose.orientation.w - cart_pose.orientation.w) / delta_time;
        double p1 = (msg_pose.orientation.x - cart_pose.orientation.x) / delta_time;
        double p2 = (msg_pose.orientation.y - cart_pose.orientation.y) / delta_time;
        double p3 = (msg_pose.orientation.z - cart_pose.orientation.z) / delta_time;

        double q0 = cart_pose.orientation.w;
        double q1 = cart_pose.orientation.x;
        double q2 = cart_pose.orientation.y;
        double q3 = cart_pose.orientation.z;

        cart_velocity.angular.x = (-q1 * p0 + q0 * p1 - q3 * p2 + q2 * p3) * 2;
        cart_velocity.angular.y = (-q2 * p0 + q3 * p1 + q0 * p2 - q1 * p3) * 2;
        cart_velocity.angular.z = (-q3 * p0 - q2 * p1 + q1 * p2 + q0 * p3) * 2;

        double velocity_error = q0 * p0 + q1 * p1 + q2 * p2 + q3 * p3;
        if (abs(velocity_error) > 1e-2)
          printf("Angular velocity computation error: %E has exceed limit, need to reduce sample time\n",
                 velocity_error);
      }

      cart_pose = msg.poseStamped.pose;
      status = msg.redundancy.status;
      timestamp = msg.poseStamped.header.stamp;

      if (!recorder_state)
        return;

      cart_pos_queue.emplace_back(KukaCartesianPose(msg));

      if (cart_pos_queue.size() >= 47)
        clean_cart_pos_queue();
    }

  public:
    KukaRecorder() = default;

    KukaRecorder(ros::NodeHandle &nh, std::string name)
        : cart_pos_stream(name + "_cart_path.csv"), wrench_stream(name + "_wrench.csv")
    {
      mtx = new std::mutex();
      wrench_mtx = new std::mutex();

      cart_pos_queue.reserve(30);
      wrench_queue.reserve(50);

      csv2::Writer<csv2::delimiter<','>> cart_pos_writer(cart_pos_stream);
      csv2::Writer<csv2::delimiter<','>> wrench_writer(wrench_stream);

      cart_pos_writer.write_row(std::vector<std::string>{"X", "Y", "Z", "w", "x", "y", "z", "status", "time(ns)"});
      wrench_writer.write_row(std::vector<std::string>{"Fx", "Fy", "Fz", "Tx", "Ty", "Tz", "time(ns)"});

      std::cout << name << std::endl;
      robot_name = name;

      std::string robot_ns = "/" + robot_name;

      wrench_sub = nh.subscribe(robot_ns + "/state/CartesianWrench", 20, &KukaRecorder::wrench_callback, this);

      cart_pos_sub = nh.subscribe(robot_ns + "/state/CartesianPose", 20, &KukaRecorder::cart_pos_callback, this);

      wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("/cam" + robot_ns + "/state/EndEffectorWrench", 10);

      wrench_limit_pub =
          nh.advertise<geometry_msgs::WrenchStamped>("/cam" + robot_ns + "/state/EndEffectorWrenchLimit", 2);

      dog = nullptr;
    }

    ~KukaRecorder()
    {
      if (cart_pos_stream.is_open())
        cart_pos_stream.close();
      if (wrench_stream.is_open())
        wrench_stream.close();
      if (dog)
        delete dog;
      delete mtx;
      delete wrench_mtx;
    }

    /**
     * @brief fill out the response data in EndEffectorPose
     *
     * @pre get the mutex
     *
     */
    void fill_response(iiwa_cam::EndEffectorPose::Response &res)
    {
      res.pose = cart_pose;
      res.status = status;
    }

    /**
     * @brief fill out the response data in EndEffectorState
     *
     * @pre get the mutex
     *
     */
    void fill_response(iiwa_cam::EndEffectorState::Response &res)
    {
      res.pose = cart_pose;
      res.velocity = cart_velocity;
      res.status = status;
      res.stamp = timestamp;

      res.wrenches.resize(WRENCH_HIS_QUEUE_SIZE);
      std::lock_guard<std::mutex> lock(*wrench_mtx);
      std::copy(wrench_history.begin(), wrench_history.end(), res.wrenches.begin());
    }

    const std::string &get_name() const { return robot_name; }

    /**
     * @brief convert data in cart_pos_queue to csv file.
     *
     * @pre get the mutex
     *
     */
    void clean_cart_pos_queue()
    {
      cart_pos_stream.open(robot_name + "_cart_path.csv", std::ios::out | std::ios::app);

      csv2::Writer<csv2::delimiter<','>> cart_pos_writer(cart_pos_stream);

      std::vector<std::vector<std::string>> output;
      output.reserve(50);

      for (const auto &val : cart_pos_queue)
      {
        output.emplace_back(val.to_vector());
      }

      cart_pos_writer.write_rows(output);

      cart_pos_queue.clear();
    }

    /**
     * @brief convert data in wrench_queue to csv file.
     *
     * @pre get the mutex
     *
     */
    void clean_wrench_queue()
    {
      wrench_stream.open(robot_name + "_wrench.csv", std::ios::out | std::ios::app);
      csv2::Writer<csv2::delimiter<','>> wrench_writer(wrench_stream);

      std::vector<std::vector<std::string>> output;
      output.reserve(30);

      for (const auto &val : wrench_queue)
      {
        output.emplace_back(val.to_vector());
      }

      wrench_writer.write_rows(output);

      wrench_queue.clear();
    }
  };

} // namespace cam

// implementation of watchdog
namespace cam
{

  KukaWatchDog::KukaWatchDog(KukaRecorder *recorder, uint32_t timer)
  {
    mtx = new std::mutex();
    threshold_time.sec = (timer < KUKA_WD_TIMEOUT_MIN) ? KUKA_WD_TIMEOUT_MIN : timer;
    sleep_time = threshold_time * 2;
    feed();
    std::thread(std::bind(&KukaWatchDog::watcher, this, recorder)).detach();
  }

  KukaWatchDog::~KukaWatchDog()
  {
    delete mtx;
    std::cout << "Watch Dog exits" << std::endl;
  }

  void KukaWatchDog::feed()
  {
    std::lock_guard<std::mutex> locker(*mtx);
    begin_time = ros::Time::now();
  }

  void KukaWatchDog::bark(KukaRecorder *recorder)
  {
    recorder->clean_cart_pos_queue();
    recorder->clean_wrench_queue();
    recorder->recorder_state = false;
    recorder->dog = nullptr;
    std::cout << "Watch Dog of " << recorder->get_name() << ": no feed, stop recording!" << std::endl;
    this->~KukaWatchDog();
  }

  void KukaWatchDog::watcher(KukaRecorder *recorder)
  {
    auto dog = this;
    while (ros::ok())
    {
      sleep_time.sleep();
      std::lock_guard<std::mutex> locker(*(recorder->mtx));

      // there is no dog or not the original dog
      if (!recorder->dog || recorder->dog != dog)
      {
        return;
      }

      mtx->lock();
      auto diff = ros::Time::now() - begin_time;
      mtx->unlock();

      if (diff > threshold_time)
      {
        bark(recorder);
        return;
      }
    }
  }

} // namespace cam

static std::unordered_map<std::string, cam::KukaRecorder *> pr_map;

/**
 * @brief Callback function for end effector path recording request
 *
 * @param req
 * @param res
 * @return true once the request is served
 */
bool pr_callback(iiwa_cam::PathRecorder::Request &req, iiwa_cam::PathRecorder::Response &res)
{
  auto pr_iter = pr_map.find(req.robot_name);
  if (pr_iter == pr_map.end())
  {
    res.error = "No robot named " + req.robot_name + " is being listening to";
    res.success = false;
    return true;
  }

  // path recorder of the robot
  auto &pr = pr_iter->second;

  std::lock_guard<std::mutex> lock(*(pr->mtx));

  // if use watchdog
  if (req.watchdog)
  {
    // if false -> true, set up a watchdog
    if (!pr->recorder_state && req.record)
    {
      pr->dog = new cam::KukaWatchDog(pr);
    }
    else if (req.record && pr->dog)
      pr->dog->feed();
  }

  // if true -> false, do cleaning
  if (pr->recorder_state && !req.record)
  {
    pr->clean_cart_pos_queue();
    pr->clean_wrench_queue();
    if (pr->dog)
    {
      delete pr->dog;
      pr->dog = nullptr;
    }
  }
  pr->recorder_state = req.record;

  res.success = true;
  return true;
}

/**
 * @brief Callback function for end effector current cartesian position inquiry
 *
 * @param req
 * @param res
 * @return true once the request is served
 */
bool ee_pos_callback(iiwa_cam::EndEffectorPose::Request &req, iiwa_cam::EndEffectorPose::Response &res)
{
  auto pr_iter = pr_map.find(req.robot_name);
  if (pr_iter == pr_map.end())
  {
    res.error = "No robot named " + req.robot_name + " is being listening to";
    res.success = false;
    return true;
  }
  auto &pr = pr_iter->second;

  std::lock_guard<std::mutex> lock(*(pr->mtx));

  pr->fill_response(res);

  res.success = true;

  return true;
}

/**
 * @brief Callback function for end effector current state inquiry
 *
 * @param req
 * @param res
 * @return true once the request is served
 */
bool ee_state_callback(iiwa_cam::EndEffectorState::Request &req, iiwa_cam::EndEffectorState::Response &res)
{
  auto pr_iter = pr_map.find(req.robot_name);
  if (pr_iter == pr_map.end())
  {
    res.error = "No robot named " + req.robot_name + " is being listening to";
    res.success = false;
    return true;
  }
  auto &pr = pr_iter->second;

  std::lock_guard<std::mutex> lock(*(pr->mtx));

  pr->fill_response(res);

  res.success = true;

  return true;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "end_effector_state_service");
  ros::NodeHandle nh;

  std::cout << "Kuka Path Recorder is listening from: " << std::endl;

  for (int i = 1; i < argc; i++)
  {
    std::cout << "(" << i << ") ";
    std::string name(argv[i]);

    pr_map[name] = new cam::KukaRecorder(nh, name);
  }

  ros::ServiceServer pr_service = nh.advertiseService("/cam/iiwa/PathRecorder", pr_callback);

  ros::ServiceServer ee_pos_service = nh.advertiseService("/cam/iiwa/EndEffectorPose", ee_pos_callback);

  ros::ServiceServer ee_wrench_service = nh.advertiseService("/cam/iiwa/EndEffectorState", ee_state_callback);

  ros::spin();

  std::cout << "\nShut down Kuka End Effector State Service" << std::endl;
  // do cleaning

  return 0;
}