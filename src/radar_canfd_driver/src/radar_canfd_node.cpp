#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sr75/controlcanfd.h>
#include <ars620_driver/ars620_decoder.h>
#include <ars620_driver/cycle_assembler.h>
#include <ars620_driver/pointcloud.h>
#include <dlfcn.h>
#include <pthread.h>
#include <csignal>
#include <cstring>
#include <vector>
#include <string>

// ============================================================
// ZCAN function pointers — types from sr75/controlcanfd.h
// ============================================================
typedef DEVICE_HANDLE  (*pOpenDevice)(UINT, UINT, UINT);
typedef UINT           (*pCloseDevice)(DEVICE_HANDLE);
typedef UINT           (*pGetDeviceInf)(DEVICE_HANDLE, ZCAN_DEVICE_INFO*);
typedef CHANNEL_HANDLE (*pInitCAN)(DEVICE_HANDLE, UINT, ZCAN_CHANNEL_INIT_CONFIG*);
typedef UINT           (*pStartCAN)(CHANNEL_HANDLE);
typedef UINT           (*pResetCAN)(CHANNEL_HANDLE);
typedef UINT           (*pClearBuffer)(CHANNEL_HANDLE);
typedef UINT           (*pGetReceiveNum)(CHANNEL_HANDLE, BYTE);
typedef UINT           (*pReceiveFD)(CHANNEL_HANDLE, ZCAN_ReceiveFD_Data*, UINT, int);
typedef UINT           (*pSetAbitBaud)(DEVICE_HANDLE, UINT, UINT);
typedef UINT           (*pSetDbitBaud)(DEVICE_HANDLE, UINT, UINT);
typedef UINT           (*pSetCANFDStandard)(DEVICE_HANDLE, UINT, UINT);

// NOTE: INVALID_DEVICE_HANDLE, INVALID_CHANNEL_HANDLE, GET_ID are provided by controlcanfd.h

// ============================================================
// Global state
// ============================================================
static volatile bool g_running = true;
static void* g_lib = nullptr;

static pOpenDevice      zcan_open        = nullptr;
static pCloseDevice     zcan_close       = nullptr;
static pInitCAN         zcan_init        = nullptr;
static pStartCAN        zcan_start       = nullptr;
static pResetCAN        zcan_reset       = nullptr;
static pClearBuffer     zcan_clear       = nullptr;
static pGetReceiveNum   zcan_recv_num    = nullptr;
static pReceiveFD       zcan_recv_fd     = nullptr;
static pSetAbitBaud     zcan_set_abit    = nullptr;
static pSetDbitBaud     zcan_set_dbit    = nullptr;
static pSetCANFDStandard zcan_set_canfd  = nullptr;

static DEVICE_HANDLE g_dev = INVALID_DEVICE_HANDLE;

// ============================================================
// Per-sensor structure
// ============================================================
struct Ars620Sensor {
  std::string name;
  std::string frame_id;
  CHANNEL_HANDLE ch;
  int ch_index;
  ars620_driver::RdiAssembler rdi_assembler;
  ars620_driver::OdAssembler  od_assembler;
  ros::Publisher pub_rdi_pc2;
  ros::Publisher pub_od_pc2;
  pthread_t thread;
  bool flip_y;
};

static std::vector<Ars620Sensor*> g_ars620_sensors;

static void flipPointCloudY(sensor_msgs::PointCloud2& cloud) {
  for (auto& f : cloud.fields) {
    if (f.name == "y" && f.datatype == sensor_msgs::PointField::FLOAT32) {
      for (size_t i = 0; i < cloud.width; i++) {
        float* py = reinterpret_cast<float*>(cloud.data.data() + i * cloud.point_step + f.offset);
        *py = -(*py);
      }
      return;
    }
  }
}

// ============================================================
static int loadLib(const std::string& path) {
  g_lib = dlopen(path.c_str(), RTLD_LAZY);
  if(!g_lib) { ROS_ERROR("dlopen %s: %s", path.c_str(), dlerror()); return -1; }
  zcan_open      = (pOpenDevice)     dlsym(g_lib,"ZCAN_OpenDevice");
  zcan_close     = (pCloseDevice)    dlsym(g_lib,"ZCAN_CloseDevice");
  zcan_init      = (pInitCAN)        dlsym(g_lib,"ZCAN_InitCAN");
  zcan_start     = (pStartCAN)       dlsym(g_lib,"ZCAN_StartCAN");
  zcan_reset     = (pResetCAN)       dlsym(g_lib,"ZCAN_ResetCAN");
  zcan_clear     = (pClearBuffer)    dlsym(g_lib,"ZCAN_ClearBuffer");
  zcan_recv_num  = (pGetReceiveNum)  dlsym(g_lib,"ZCAN_GetReceiveNum");
  zcan_recv_fd   = (pReceiveFD)      dlsym(g_lib,"ZCAN_ReceiveFD");
  zcan_set_abit  = (pSetAbitBaud)    dlsym(g_lib,"ZCAN_SetAbitBaud");
  zcan_set_dbit  = (pSetDbitBaud)    dlsym(g_lib,"ZCAN_SetDbitBaud");
  zcan_set_canfd = (pSetCANFDStandard)dlsym(g_lib,"ZCAN_SetCANFDStandard");
  if(!zcan_open||!zcan_close||!zcan_init||!zcan_start||!zcan_reset||
     !zcan_recv_fd||!zcan_set_abit||!zcan_set_dbit||!zcan_set_canfd)
    { ROS_ERROR("Missing ZCAN symbols"); return -2; }
  return 0;
}

static CHANNEL_HANDLE initChannel(int ch, int abit, int dbit, int canfd_std) {
  if(zcan_set_abit(g_dev, ch, abit)!=STATUS_OK)  { ROS_ERROR("CH%d abit fail",ch); return INVALID_CHANNEL_HANDLE; }
  if(zcan_set_dbit(g_dev, ch, dbit)!=STATUS_OK)  { ROS_ERROR("CH%d dbit fail",ch); return INVALID_CHANNEL_HANDLE; }
  if(zcan_set_canfd(g_dev, ch, canfd_std)!=STATUS_OK) { ROS_ERROR("CH%d CANFD fail",ch); return INVALID_CHANNEL_HANDLE; }
  ZCAN_CHANNEL_INIT_CONFIG cfg{};
  cfg.can_type=TYPE_CANFD; cfg.canfd.acc_mask=0xFFFFFFFF; cfg.canfd.filter=1;
  CHANNEL_HANDLE h = zcan_init(g_dev, ch, &cfg);
  if(h==INVALID_CHANNEL_HANDLE) { ROS_ERROR("CH%d init fail",ch); return h; }
  if(zcan_start(h)!=STATUS_OK)  { ROS_ERROR("CH%d start fail",ch); return INVALID_CHANNEL_HANDLE; }
  ROS_INFO("CH%d init OK (abit=%d dbit=%d)", ch, abit, dbit);
  return h;
}

// ============================================================
// ARS620 receive thread
// ============================================================
static void* ars620Thread(void* arg) {
  Ars620Sensor* sensor = static_cast<Ars620Sensor*>(arg);
  ZCAN_ReceiveFD_Data buf[128];
  int recv_cnt = 0;

  while(g_running) {
    int n = zcan_recv_fd(sensor->ch, buf, 128, 10);
    if(n>0 && ++recv_cnt%100==0)
      ROS_INFO_THROTTLE(2.0, "ARS620 %s CH%d: recv %d frames (total calls: %d)",
                        sensor->name.c_str(), sensor->ch_index, n, recv_cnt);

    for(int i=0;i<n;i++) {
      unsigned int raw_id = buf[i].frame.can_id;
      if(IS_EFF(raw_id) || IS_RTR(raw_id) || IS_ERR(raw_id)) continue;
      unsigned int id = GET_ID(raw_id);

      // Debug: log first 10 CAN IDs per sensor to diagnose decode issues
      static int debug_count = 0;
      if(debug_count < 10) {
        ROS_INFO("%s CH%d: CAN ID=0x%X len=%d", sensor->name.c_str(), sensor->ch_index, id, buf[i].frame.len);
        debug_count++;
      }

      ars620_driver::CanFrame f;
      f.id=id; f.len=buf[i].frame.len; f.timestamp_us=buf[i].timestamp;
      memcpy(f.data.data(), buf[i].frame.data, f.len);

      ars620_driver::ConfigState cs; ars620_driver::SystemStatus ss;
      ars620_driver::RdiHeader rh; ars620_driver::OdHeader oh;
      ars620_driver::RdiTargetFrame rt; std::vector<ars620_driver::OdTarget> ot;
      ars620_driver::RdiCycle rc; ars620_driver::OdCycle oc;
      std_msgs::Header hdr; hdr.stamp=ros::Time::now(); hdr.frame_id=sensor->frame_id;

      bool published = false;
      if(ars620_driver::decodeRdiHeader(f,&rh)) {
        if(sensor->rdi_assembler.processHeader(rh, hdr.stamp, &rc) && sensor->pub_rdi_pc2) {
          auto pc = ars620_driver::makeRdiPointCloud(hdr,rc.targets);
          if(sensor->flip_y) flipPointCloudY(pc);
          sensor->pub_rdi_pc2.publish(pc);
          published = true;
        }
      } else if(ars620_driver::decodeRdiTargetFrame(f,&rt)) {
        if(sensor->rdi_assembler.processTargets(rt, &rc) && sensor->pub_rdi_pc2) {
          auto pc = ars620_driver::makeRdiPointCloud(hdr,rc.targets);
          if(sensor->flip_y) flipPointCloudY(pc);
          sensor->pub_rdi_pc2.publish(pc);
          published = true;
        }
      } else if(ars620_driver::decodeOdHeader(f,&oh)) {
        if(sensor->od_assembler.processHeader(oh, hdr.stamp, &oc) && sensor->pub_od_pc2) {
          auto pc = ars620_driver::makeOdPointCloud(hdr,oc.targets);
          if(sensor->flip_y) flipPointCloudY(pc);
          sensor->pub_od_pc2.publish(pc);
          published = true;
        }
      } else if(ars620_driver::decodeOdTargets(f,&ot)) {
        if(sensor->od_assembler.processTargets(ot, &oc) && sensor->pub_od_pc2) {
          auto pc = ars620_driver::makeOdPointCloud(hdr,oc.targets);
          if(sensor->flip_y) flipPointCloudY(pc);
          sensor->pub_od_pc2.publish(pc);
          published = true;
        }
      }
      if(published) {
        ROS_INFO_THROTTLE(2.0, "%s CH%d: cycle published", sensor->name.c_str(), sensor->ch_index);
      }
    }

    // Timeout polling (for partial cycles that didn't complete in time)
    ars620_driver::RdiCycle rc; ars620_driver::OdCycle oc;
    std_msgs::Header hdr; hdr.stamp=ros::Time::now(); hdr.frame_id=sensor->frame_id;
    if(sensor->rdi_assembler.pollTimeout(hdr.stamp, &rc) && sensor->pub_rdi_pc2) {
      auto pc = ars620_driver::makeRdiPointCloud(hdr,rc.targets);
      if(sensor->flip_y) flipPointCloudY(pc);
      sensor->pub_rdi_pc2.publish(pc);
    }
    if(sensor->od_assembler.pollTimeout(hdr.stamp, &oc) && sensor->pub_od_pc2) {
      auto pc = ars620_driver::makeOdPointCloud(hdr,oc.targets);
      if(sensor->flip_y) flipPointCloudY(pc);
      sensor->pub_od_pc2.publish(pc);
    }

    usleep(1000);
  }
  return nullptr;
}

// ============================================================
int main(int argc, char** argv) {
  ros::init(argc, argv, "radar_canfd_node");
  ros::NodeHandle nh("~");

  // ---- Device params ----
  std::string lib_path; int dev_type, dev_idx, canfd_std;
  nh.param<std::string>("lib_path", lib_path, "libcontrolcanfd.so");
  nh.param<int>("dev_type", dev_type, USBCANFD_200U);
  nh.param<int>("dev_index", dev_idx, 0);
  nh.param<int>("canfd_standard", canfd_std, 0);

  // ---- CH0 config ----
  std::string ch0_mode;
  int ch0_abit, ch0_dbit;
  std::string ch0_ars620_name, ch0_ars620_frame;
  nh.param<std::string>("ch0_mode", ch0_mode, "ars620");
  nh.param<int>("ch0_abit_baud", ch0_abit, 500000);
  nh.param<int>("ch0_dbit_baud", ch0_dbit, 2000000);
  nh.param<std::string>("ch0_ars620_name", ch0_ars620_name, "ars620_front");
  nh.param<std::string>("ch0_ars620_frame", ch0_ars620_frame, "ars620_front");
  bool ch0_flip_y; nh.param<bool>("ch0_flip_y", ch0_flip_y, false);

  // ---- CH1 config ----
  std::string ch1_mode;
  int ch1_abit, ch1_dbit;
  std::string ch1_ars620_name, ch1_ars620_frame;
  nh.param<std::string>("ch1_mode", ch1_mode, "ars620");
  nh.param<int>("ch1_abit_baud", ch1_abit, 500000);
  nh.param<int>("ch1_dbit_baud", ch1_dbit, 2000000);
  nh.param<std::string>("ch1_ars620_name", ch1_ars620_name, "ars620_rear");
  nh.param<std::string>("ch1_ars620_frame", ch1_ars620_frame, "ars620_rear");
  bool ch1_flip_y; nh.param<bool>("ch1_flip_y", ch1_flip_y, true);

  // ---- Load library & open device ----
  if(loadLib(lib_path)!=0) return 1;
  g_dev = zcan_open(dev_type, dev_idx, 0);
  if(g_dev==INVALID_DEVICE_HANDLE) { ROS_ERROR("OpenDevice fail"); return 1; }
  ROS_INFO("Device opened (type=%d idx=%d)", dev_type, dev_idx);

  // ---- Setup CH0 ----
  if(ch0_mode == "ars620") {
    Ars620Sensor* s = new Ars620Sensor();
    s->name      = ch0_ars620_name;
    s->frame_id  = ch0_ars620_frame;
    s->ch_index  = 0;
    s->flip_y    = ch0_flip_y;
    s->rdi_assembler = ars620_driver::RdiAssembler(true, ros::Duration(0.1), 256);
    s->od_assembler  = ars620_driver::OdAssembler(true, ros::Duration(0.1));
    std::string ns = "/" + s->name;
    s->pub_rdi_pc2 = nh.advertise<sensor_msgs::PointCloud2>(ns + "/rdi_points", 10);
    s->pub_od_pc2  = nh.advertise<sensor_msgs::PointCloud2>(ns + "/od_points", 10);
    s->ch = initChannel(0, ch0_abit, ch0_dbit, canfd_std);
    if(s->ch != INVALID_CHANNEL_HANDLE) {
      g_ars620_sensors.push_back(s);
    } else { delete s; }
  } else {
    ROS_INFO("CH0 mode=%s → skipped", ch0_mode.c_str());
  }

  // ---- Setup CH1 ----
  if(ch1_mode == "ars620") {
    Ars620Sensor* s = new Ars620Sensor();
    s->name      = ch1_ars620_name;
    s->frame_id  = ch1_ars620_frame;
    s->ch_index  = 1;
    s->flip_y    = ch1_flip_y;
    s->rdi_assembler = ars620_driver::RdiAssembler(true, ros::Duration(0.1), 256);
    s->od_assembler  = ars620_driver::OdAssembler(true, ros::Duration(0.1));
    std::string ns = "/" + s->name;
    s->pub_rdi_pc2 = nh.advertise<sensor_msgs::PointCloud2>(ns + "/rdi_points", 10);
    s->pub_od_pc2  = nh.advertise<sensor_msgs::PointCloud2>(ns + "/od_points", 10);
    s->ch = initChannel(1, ch1_abit, ch1_dbit, canfd_std);
    if(s->ch != INVALID_CHANNEL_HANDLE) {
      g_ars620_sensors.push_back(s);
    } else { delete s; }
  } else {
    ROS_INFO("CH1 mode=%s → skipped", ch1_mode.c_str());
  }

  if(g_ars620_sensors.empty()) {
    ROS_ERROR("No channel opened.");
    return 1;
  }

  // ---- Launch threads ----
  for(auto* s : g_ars620_sensors) {
    pthread_create(&s->thread, nullptr, ars620Thread, s);
    ROS_INFO("ARS620 %s started on CH%d (frame=%s)", s->name.c_str(), s->ch_index, s->frame_id.c_str());
  }

  ROS_INFO("radar_canfd_node running. ARS620 sensors=%lu", g_ars620_sensors.size());
  ros::spin();

  // ---- Cleanup ----
  g_running = false;
  for(auto* s : g_ars620_sensors) { if(s->thread) pthread_join(s->thread, nullptr); }
  for(auto* s : g_ars620_sensors) { if(s->ch != INVALID_CHANNEL_HANDLE) zcan_reset(s->ch); delete s; }
  zcan_close(g_dev);
  dlclose(g_lib);
  return 0;
}
