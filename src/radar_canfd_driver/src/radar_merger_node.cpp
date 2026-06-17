#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <mutex>
#include <cmath>

// ============================================================
// Dual-radar point cloud merger.
// Transforms rear points into front frame, merges, publishes.
// ============================================================

static sensor_msgs::PointCloud2ConstPtr g_front;
static sensor_msgs::PointCloud2ConstPtr g_rear;
static std::mutex g_mutex;

static ros::Publisher g_pub;

// TF from ars620_front to ars620_rear
struct RigidTransform {
  float R[9];  // 3x3 rotation matrix (row-major)
  float t[3];  // translation
};

static RigidTransform g_T;
static bool g_flip_front_y = false;
static bool g_flip_rear_y  = true;

// Forward declaration
static void mergeAndPublish();

static void buildTransform(double x, double y, double z,
                           double yaw, double pitch, double roll,
                           RigidTransform* T) {
  double cy = std::cos(yaw), sy = std::sin(yaw);
  double cp = std::cos(pitch), sp = std::sin(pitch);
  double cr = std::cos(roll), sr = std::sin(roll);

  // R = Rz(yaw) * Ry(pitch) * Rx(roll)
  T->R[0] = cy*cp;  T->R[1] = cy*sp*sr - sy*cr;  T->R[2] = cy*sp*cr + sy*sr;
  T->R[3] = sy*cp;  T->R[4] = sy*sp*sr + cy*cr;  T->R[5] = sy*sp*cr - cy*sr;
  T->R[6] = -sp;    T->R[7] = cp*sr;              T->R[8] = cp*cr;

  T->t[0] = static_cast<float>(x);
  T->t[1] = static_cast<float>(y);
  T->t[2] = static_cast<float>(z);
}

static void frontCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_front = msg;
  }
  mergeAndPublish();
}

static void rearCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_rear = msg;
  }
  mergeAndPublish();
}

// Data-driven merge: called on every radar message (no timer, no duplicates)
static void mergeAndPublish() {
  static ros::Time last_pub;
  ros::Time now = ros::Time::now();
  if (!last_pub.isZero() && (now - last_pub).toSec() < 0.01) return;
  last_pub = now;
  sensor_msgs::PointCloud2ConstPtr front, rear;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    front = g_front;
    rear  = g_rear;
  }
  if (!front || !rear) return;

  // Build merged cloud header (inherit from front)
  sensor_msgs::PointCloud2 out;
  out.header = front->header;
  out.height = 1;
  out.is_bigendian = front->is_bigendian;
  out.is_dense = false;
  out.fields = front->fields;
  out.point_step = front->point_step;

  size_t nf = front->width;
  size_t nr = rear->width;
  out.width = nf + nr;
  out.row_step = out.point_step * out.width;
  out.data.resize(out.row_step);

  // Find field offsets
  int off_x = -1, off_y = -1, off_z = -1;
  for (const auto& f : out.fields) {
    if (f.name == "x") off_x = static_cast<int>(f.offset);
    if (f.name == "y") off_y = static_cast<int>(f.offset);
    if (f.name == "z") off_z = static_cast<int>(f.offset);
  }

  // Copy front points (with optional Y flip)
  if (nf > 0) {
    memcpy(out.data.data(), front->data.data(), front->data.size());
    if (g_flip_front_y && off_y >= 0) {
      for (size_t i = 0; i < nf; i++) {
        float* py = reinterpret_cast<float*>(out.data.data() + i * out.point_step + off_y);
        *py = -(*py);
      }
    }
  }

  // Copy rear points with transform: p_front = R * p_rear + t (with optional Y flip)
  for (size_t i = 0; i < nr; i++) {
    size_t src_off = i * rear->point_step;
    size_t dst_off = (nf + i) * out.point_step;
    memcpy(out.data.data() + dst_off, rear->data.data() + src_off, out.point_step);

    if (off_x < 0 || off_y < 0 || off_z < 0) continue;

    float* px = reinterpret_cast<float*>(out.data.data() + dst_off + off_x);
    float* py = reinterpret_cast<float*>(out.data.data() + dst_off + off_y);
    float* pz = reinterpret_cast<float*>(out.data.data() + dst_off + off_z);
    float xr = *px, yr = *py, zr = *pz;
    if (g_flip_rear_y) yr = -yr;
    *px = g_T.R[0]*xr + g_T.R[1]*yr + g_T.R[2]*zr + g_T.t[0];
    *py = g_T.R[3]*xr + g_T.R[4]*yr + g_T.R[5]*zr + g_T.t[1];
    *pz = g_T.R[6]*xr + g_T.R[7]*yr + g_T.R[8]*zr + g_T.t[2];
  }

  if (g_pub.getNumSubscribers() > 0)
    g_pub.publish(out);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "radar_merger_node");
  ros::NodeHandle nh("~");

  // Topics
  std::string front_topic, rear_topic, merged_topic, merged_frame;
  nh.param<std::string>("front_topic", front_topic, "/ars620_front/rdi_points");
  nh.param<std::string>("rear_topic", rear_topic, "/ars620_rear/rdi_points");
  nh.param<std::string>("merged_topic", merged_topic, "/ars620_merged/rdi_points");
  nh.param<std::string>("merged_frame", merged_frame, "ars620_front");

  // Rear position relative to front
  double rx, ry, rz, ryaw, rpitch, rroll;
  nh.param<double>("rear_x", rx, -0.10);
  nh.param<double>("rear_y", ry, 0.0);
  nh.param<double>("rear_z", rz, 0.0);
  nh.param<double>("rear_yaw", ryaw, M_PI);
  nh.param<double>("rear_pitch", rpitch, 0.0);
  nh.param<double>("rear_roll", rroll, 0.0);

  nh.param<bool>("flip_front_y", g_flip_front_y, false);
  nh.param<bool>("flip_rear_y",  g_flip_rear_y,  true);

  buildTransform(rx, ry, rz, ryaw, rpitch, rroll, &g_T);

  auto sub_front = nh.subscribe<sensor_msgs::PointCloud2>(front_topic, 10, frontCallback);
  auto sub_rear  = nh.subscribe<sensor_msgs::PointCloud2>(rear_topic, 10, rearCallback);
  g_pub = nh.advertise<sensor_msgs::PointCloud2>(merged_topic, 10);

  ROS_INFO("radar_merger: front=%s rear=%s → merged=%s (data-driven)",
           front_topic.c_str(), rear_topic.c_str(), merged_topic.c_str());
  ROS_INFO("radar_merger: rear→front T=(%.3f,%.3f,%.3f) yaw=%.2f° flip_doppler=%d flip_rear_y=%d flip_front_y=%d",
           rx, ry, rz, ryaw*180.0/M_PI, (g_T.R[0]<0.0)?1:0, g_flip_rear_y, g_flip_front_y);
  ROS_INFO("  rear→front: x=%.2f y=%.2f z=%.2f yaw=%.2f pitch=%.2f roll=%.2f",
           rx, ry, rz, ryaw, rpitch, rroll);

  ros::spin();
  return 0;
}
