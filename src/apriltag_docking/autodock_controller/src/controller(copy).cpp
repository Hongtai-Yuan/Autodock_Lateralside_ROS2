#include "controller.h"

using namespace automatic_parking ;

void autodock_controller::docking_state_manage(){
    //RCLCPP_INFO(get_logger(),"%s | %s", docking_state.c_str(), last_docking_state.c_str());
    
    if (docking_state == "searching"){
        searching_state_fun();
    }
    if (docking_state == "centering"){
        centering_state_fun();
    }
    if (docking_state == "approach"){
        approach_state_fun();
    }
    if (docking_state == "final_approach"){
        final_approach_state_fun();
    }

}

void autodock_controller::set_docking_state(std::string new_docking_state){
    if (docking_state != new_docking_state){
        last_docking_state = docking_state;
        docking_state = new_docking_state;
        RCLCPP_INFO(get_logger(),"docking state: %s, last state: %s", docking_state.c_str(), last_docking_state.c_str());
    }
}

void autodock_controller::set_action_state(std::string new_action_state){
    if (action_state != new_action_state){
        last_action_state = action_state;
        action_state = new_action_state;
        RCLCPP_INFO(get_logger(),"action state: %s, last state: %s", action_state.c_str(), last_action_state.c_str());
    }
}

void autodock_controller::searching_state_fun(){
    if (!tag_y) tag_y = 0.5;
    neuron_forward(fabs(tag_y/1.5));
    set_docking_state("centering");
}

void autodock_controller::approach_state_fun(){
    //RCLCPP_INFO(get_logger(),"M_PI-fabs(tag_yaw): %3.3f, pose_set.theta_bounds: %3.3f", M_PI-fabs(tag_yaw), pose_set.theta_bounds);

    if (M_PI-fabs(tag_yaw)>pose_set.theta_bounds){
        //RCLCPP_INFO(get_logger(),"Approach Angle Reached");
        RCLCPP_INFO(get_logger(),"pose_set.theta_bounds:%3.3f,(M_PI-fabs(tag_yaw))*sign(-tag_yaw): %3.3f", pose_set.theta_bounds, (M_PI-fabs(tag_yaw))*sign(-tag_yaw));
        neuron_turn((M_PI-fabs(tag_yaw))*sign(-tag_yaw));
        return;
    }else{
        neuron_stop();
        //set_docking_state("centering");
        set_docking_state("final_approach");
    }
}

void autodock_controller::final_approach_state_fun(){
    double x_trans = tf_bot2dock.transform.translation.x;
    double y_trans = tf_bot2dock.transform.translation.y;
    double theta = atan2(-y_trans, x_trans);
    if (in_view && abs(theta)<=0.01){
        // && abs(tag_yaw)<=3.07 && abs(tag_yaw) > 2.92
        neuron_stop();
        set_docking_state("docked");
        //set_docking_state("docked");
    }
    else{
        // neuron_stop();
        if (!tag_y) tag_y = 0.5;
        neuron_forward(-fabs(tag_y/3.0));
        set_docking_state("final_approach");
    }
}

void autodock_controller::centering_state_fun(){
    double x_trans = tf_bot2dock.transform.translation.x;
    double y_trans = tf_bot2dock.transform.translation.y;
    double theta = atan2(-y_trans, x_trans);
    if (in_view && abs(theta)<=0.01){
        // && abs(tag_yaw)<=3.07 && abs(tag_yaw) > 2.92
        neuron_stop();
        set_docking_state("approach");
        //set_docking_state("docked");
    }
    else{
        // neuron_stop();
        neuron_forward(fabs(tag_y/3.0));
        set_docking_state("searching");
    }
}

void autodock_controller::neuron_forward(double distance){
    double cmd_vel_linear ;
    if (action_state == ""){
        set_action_state("forward");
        
        if (distance > 0){
            cmd_vel_linear = cmd_vel_linear_rate;
        }
        else{
            cmd_vel_linear = -cmd_vel_linear_rate;
        }

        if (fabs(pose_set.distance) < final_approach_distance){
            cmd_vel_linear = cmd_vel_linear/2;
        }
    }
    else{
        cmd_vel_linear = 0;
        return;
    }
    cmd_vel_msg.linear.x = cmd_vel_linear;
    robot_point_temp = robot_point;
    temp_distance = distance;
}

void autodock_controller::neuron_stop(){
    set_action_state("");
    cmd_vel_msg.linear.x = 0;
    cmd_vel_msg.angular.z = 0;
}

void autodock_controller::neuron_turn(double radians){
    double cmd_vel_angular;
    if (action_state == ""){
        set_action_state("turning");
        if (radians>0){
            cmd_vel_angular = -cmd_vel_angular_rate ;
        }
        else{
            cmd_vel_angular = cmd_vel_angular_rate;
        }
        if (fabs(radians) < 0.1){
            cmd_vel_angular = cmd_vel_angular/2;
        }
    }
    else{
        cmd_vel_angular = 0;
        return;
    }
    cmd_vel_msg.angular.z = cmd_vel_angular;
    robot_point_temp = robot_point;
    temp_theta = radians;
}

void autodock_controller::action_state_manage(){
    //RCLCPP_INFO(get_logger(), "%s | %s", action_state.c_str(), last_action_state.c_str());
    if (action_state == "jogging"){
        if (distance(robot_point_temp, robot_point) >= fabs(temp_distance)){
            neuron_stop();
            if (docking_state == "approach"){set_docking_state("centering");}
        }
    }
    if (action_state == "turning" ){
        if (fabs(robot_point_temp[2]-robot_point[2])>= fabs(temp_theta)){neuron_stop();}
        else if (fabs(pose_set.theta) < pose_set.theta_bounds and docking_state != "blind" and docking_state != "final_approach"){
            neuron_stop();}
    }
    if (tag_y){
        if ((fabs(tag_y)<0.02) and (desire_angle == tune_angle)){
            neuron_stop();
            final_counter += 1;
            if (final_counter > 3){
                desire_angle = 0;
            }
        }
    }
    else{
       desire_angle = tune_angle;
    }

    if (docking_state != "") vel_pub->publish(cmd_vel_msg);
}


void autodock_controller::fid2pos(){
    double x_trans = tf_bot2dock.transform.translation.x;
    double y_trans = tf_bot2dock.transform.translation.y;
    double theta = atan2(-y_trans, x_trans);
    double r = sqrt(pow(x_trans ,2) + pow(y_trans , 2));
    double theta_bounds=0.05;


    //RCLCPP_INFO(get_logger(),"Theta: %3.3f, distance: %3.3f, theta_bounds: %3.3f", theta, r, theta_bounds);
    pose_set = {theta-desire_angle*sign(tag_y), r, theta_bounds};
}

void autodock_controller::transform_filter(geometry_msgs::msg::TransformStamped &tf_){
    double time = tf_.header.stamp.sec + double(tf_dock2bot.header.stamp.nanosec)*(1e-9);
    if (time == last_time){
        in_view = false;
    }
    else{in_view = true;}
    last_time = time;
}

void autodock_controller::receive_tf(){
    try{
        tf_odom = buffer_->lookupTransform("odom","base_link",tf2::TimePointZero);
        odom_x = tf_odom.transform.translation.x;
        odom_y = tf_odom.transform.translation.y; 
        odom_yaw = tf2::getYaw(tf_odom.transform.rotation);
        robot_point = {odom_x , odom_y , odom_yaw};

        tf_dock2bot = buffer_->lookupTransform(tag_frame, "base_link", tf2::TimePointZero);
        tf_bot2dock = buffer_->lookupTransform("base_link", tag_frame, tf2::TimePointZero);
        transform_filter(tf_dock2bot);
        if (!in_view){
            //RCLCPP_WARN(get_logger(),"Tag Detection Lost");
            return;
        }
        tag_x = tf_dock2bot.transform.translation.x;
        tag_y = tf_dock2bot.transform.translation.y;
        tag_yaw = tf2::getYaw(tf_dock2bot.transform.rotation);
        //RCLCPP_INFO(get_logger(),"tag_x: %3.3f, tag_y: %3.3f, tag_yaw: %3.3f", tag_x, tag_y, tag_yaw); 
        fid2pos();
    }
    catch(tf2::TransformException &ex){
        RCLCPP_ERROR(get_logger(),"%s",ex.what());
        in_view = false;
    }
}

void autodock_controller::run(){
    receive_tf();
    docking_state_manage();
    action_state_manage();
    state_publish();
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto controller_node = std::make_shared<automatic_parking::autodock_controller>() ;
    rclcpp::Rate rate(30.0);
    controller_node->set_docking_state("");
    while (rclcpp::ok()){
        controller_node->run();
        rclcpp::spin_some(controller_node);
        rate.sleep();
    }
    return 0;
}