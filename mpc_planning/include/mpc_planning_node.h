#include <iLQRsolver.h>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>

using namespace chrono;


class Planner
{
private:
    ros::NodeHandle nh_; 
    ros::Subscriber sub_joy;
    ros::Subscriber sub_states;
    ros::Publisher pub_inputs;

    int N_state, N_input, Hp;
    iLQRsolver ilqr;
    VectorXd x;
    VectorXd u;
    vector<VectorXd> u_init;
    vector<VectorXd> x_array_opt;
    vector<VectorXd> u_array_opt;
    vector<VectorXd> k_array_opt;
    vector<MatrixXd> K_array_opt;
    microseconds cal_time, step_time;
    system_clock::time_point start_prev;

public:
    Planner(ros::NodeHandle* nodehandle);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
    void statesCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void solve_OP();
    void initialize();
    bool start_plan;
};

Planner::Planner(ros::NodeHandle* nodehandle) : nh_(*nodehandle)
{
    /* Topic sub & Pub */
    sub_joy = nh_.subscribe("joy", 1, &Planner::joyCallback, this);
    sub_states = nh_.subscribe("states", 1, &Planner::statesCallback, this);
    pub_inputs = nh_.advertise<std_msgs::Float32MultiArray>("control_inputs",100);

    this->initialize();
}

void Planner::initialize()
{
    /* initialization */
    start_plan = false;
    N_state = ilqr.state_dim;
    N_input = ilqr.input_dim;
    Hp = ilqr.N;
    x = VectorXd(ilqr.state_dim);
    u_init = vector<VectorXd>(Hp+1,VectorXd::Zero(N_input));
    x_array_opt = vector<VectorXd>(Hp+1,VectorXd::Zero(N_state));
    u_array_opt = vector<VectorXd>(Hp+1,VectorXd::Zero(N_input));
    k_array_opt = vector<VectorXd>(Hp,VectorXd::Zero(N_input));
    K_array_opt = vector<MatrixXd>(Hp,MatrixXd::Zero(N_input,N_state));
    x << 0, 0, 0, 0.*1000./3600.; // x, y , yaw, velocity
}

void Planner::solve_OP()
{
    system_clock::time_point start = system_clock::now();
    step_time =  duration_cast<microseconds>(start - start_prev);
    start_prev = start;
    // ROS_INFO("Step Time: %f", step_time.count()/1000.0);

    /* 
     * 
     * Solve Optimization Problem using iLQR solver
     * 
     **/
    u_init = u_array_opt;
    ilqr.ilqr_iterate(x, u_init, x_array_opt, u_array_opt, k_array_opt, K_array_opt);
    u = u_array_opt[0];
    

    system_clock::time_point end = system_clock::now();
    cal_time = duration_cast<microseconds>(end - start);
    ROS_INFO("Calculation Time: %f", cal_time.count()/1000.0);

    x = ilqr.dynamics_discrete(1, x, u);
    
    std::cout << "Input: \n" << u << std::endl;
    std::cout << "State: \n" << x << std::endl;
    
    
    /* Publish message for control inputs*/
    std_msgs::Float32MultiArray array;
    array.data.clear();
    //for loop, pushing data in the size of the array
    for (int i = 0; i < N_input; i++)
    {
        array.data.push_back(u(i));
    }

    pub_inputs.publish(array);

}

void Planner::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    if (msg->buttons[0] == 1){
        ROS_INFO("Start Motion Planning!! ");
        start_plan = true;
    } 

    if (msg->buttons[1] == 1){
        ROS_WARN("Planner is RESET!! ");
        initialize();
    }
}

void Planner::statesCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    double vx = msg->data[2], vy = msg->data[3];
    x(0) = msg->data[0];
    x(1) = msg->data[1];
    x(2) = msg->data[6];
    x(3) = sqrt(vx*vx+vy*vy);
}