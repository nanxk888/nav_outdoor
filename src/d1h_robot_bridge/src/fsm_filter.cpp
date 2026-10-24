// Read-only ROS adapter: no command publisher, mode change or parameter writes.
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <chrono>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

using Steady = std::chrono::steady_clock;
using List = controller_manager_msgs::srv::ListControllers;
static double seconds(Steady::time_point t) {
  return std::chrono::duration<double>(Steady::now()-t).count();
}
template<class Gid> static std::string gid_key(const Gid &gid) {
  std::ostringstream s;
  for (const auto b:gid) s<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(b);
  return s.str();
}

class Filter : public rclcpp::Node {
  struct Event {std::string state;int64_t stamp;Steady::time_point received;};
  std::string ns_, topic_;
  std::map<std::string,Event> events_;
  bool online_=false,pending_=false;
  Steady::time_point checked_=Steady::now()-std::chrono::hours(1);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Client<List>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr query_,tick_;
public:
  Filter():Node("nxk_fsm_filter") {
    ns_=declare_parameter<std::string>("robot_namespace","");
    if(ns_.empty() || ns_.find('/')!=std::string::npos)
      throw std::runtime_error("Set robot_namespace to namespace.sh ROBOT_NS, without slashes");
    topic_="/"+ns_+"/rl_controller/fsm";
    pub_=create_publisher<std_msgs::msg::String>("/nxk_receiver/fsm_status",rclcpp::QoS(1).reliable());
    sub_=create_subscription<std_msgs::msg::String>(topic_,rclcpp::QoS(32).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg,const rclcpp::MessageInfo &info) {
        const auto &meta=info.get_rmw_message_info();
        const auto &value=msg->data;
        if(meta.source_timestamp<=0 || value.empty() || value.size()>64) return;
        for(unsigned char c:value) if(!std::isalnum(c) && c!='_') return;
        auto key=gid_key(meta.publisher_gid.data);
        auto it=events_.find(key);
        if(it!=events_.end() && meta.source_timestamp<=it->second.stamp) return;
        if(it==events_.end() && events_.size()>=16) return; // bounded fail-closed cache
        events_.insert_or_assign(key,Event{value,meta.source_timestamp,Steady::now()});
      });
    client_=create_client<List>("/"+ns_+"/controller_manager/list_controllers");
    query_=create_wall_timer(std::chrono::milliseconds(500),[this] {
      if(pending_ || !client_->service_is_ready()) return;
      pending_=true;
      client_->async_send_request(std::make_shared<List::Request>(),[this](rclcpp::Client<List>::SharedFuture future) {
        pending_=false;online_=false;
        try {
          std::map<std::string,std::string> states;
          for(const auto &c:future.get()->controller) states[c.name]=c.state;
          online_=states["d1h_rl_controller"]=="active" && states["d1_rl_controller"]=="inactive"
            && states["teleop_command"]=="active";
          checked_=Steady::now();
        } catch(const std::exception &) {online_=false;}
      });
    });
    tick_=create_wall_timer(std::chrono::milliseconds(50),[this] {
      std::string writer;size_t matches=0;
      for(const auto &p:get_publishers_info_by_topic(topic_)) {
        if(p.node_name()=="d1h_rl_controller" && p.node_namespace()=="/"+ns_) {
          writer=gid_key(p.endpoint_gid());++matches;
        }
      }
      const bool online=online_ && seconds(checked_)<1.5 && matches==1;
      const auto it=events_.find(writer);
      const bool known=it!=events_.end();
      const bool valid=online && known && seconds(it->second.received)>.3;
      std::ostringstream s;
      s<<"{\"schema\":1,\"controllers_online\":"<<(online?"true":"false")
       <<",\"valid\":"<<(valid?"true":"false")<<",\"writer\":\""<<writer
       <<"\",\"state\":\""<<(known?it->second.state:"")<<"\",\"source_timestamp_ns\":"
       <<(known?it->second.stamp:0)<<",\"event_age_sec\":"<<(known?seconds(it->second.received):-1.)<<"}";
      std_msgs::msg::String msg;msg.data=s.str();pub_->publish(msg);
    });
  }
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<Filter>());}
  catch(const std::exception &e) {std::cerr<<"FSM filter failed: "<<e.what()<<std::endl;rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
