#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <vector>
#include <numeric>

struct WallLine {
    double m; // Pendiente
    double b; // Distancia (ordenada en el origen)
    bool valid; // Para saber si ha detectado suficientes puntos
};

class CorridorNavigationNode : public rclcpp::Node
{
public:
    CorridorNavigationNode() : Node("corr_nav_node")
    {
        // Load parameters
        loadParameters();

        RCLCPP_INFO(this->get_logger(), "CorridorNavigationNode started");

        // Publisher for cmd_vel
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/PioneerP3DX/cmd_vel", 10);

        // Subscriber for robot pose
        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/PioneerP3DX/odom", 10,
            std::bind(&CorridorNavigationNode::odomCallback, this, std::placeholders::_1));

        // Subscriber for laser scan data
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/PioneerP3DX/laser_scan", 10,
            std::bind(&CorridorNavigationNode::laserCallback, this, std::placeholders::_1));

        // Timer to publish velocity commands at regular intervals
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(time_step_),
            std::bind(&CorridorNavigationNode::controlLoop, this));

        // Inicializar los publicadores para verlos en rqt
        pub_m_izq_ = this->create_publisher<std_msgs::msg::Float64>("/pared_izq/m", 10);
        pub_b_izq_ = this->create_publisher<std_msgs::msg::Float64>("/pared_izq/b", 10);
        pub_m_der_ = this->create_publisher<std_msgs::msg::Float64>("/pared_der/m", 10);
        pub_b_der_ = this->create_publisher<std_msgs::msg::Float64>("/pared_der/b", 10);

        RCLCPP_INFO(this->get_logger(), "Corridor Navigation Node Initialized");
        measured_data_=false;
    }

private:
    void loadParameters()
    {
         // Declare parameters of this node (name, initial_value)
        this->declare_parameter("time_step", 25);  // in milliseconds
        this->declare_parameter("max_linear_speed", 1.2);
        this->declare_parameter("max_angular_speed", 2.0);
        this->declare_parameter("wheel_base", 0.331);
        this->declare_parameter("wheel_radius", 0.097518);
        this->declare_parameter("corridor_width", 10.0);  // meters
        this->declare_parameter("look_ahead_distance", 1.0);  // meters 
        // Read parameters
        time_step_ = this->get_parameter("time_step").as_int();
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        corridor_width_ = this->get_parameter("corridor_width").as_double();
        look_ahead_distance_ = this->get_parameter("look_ahead_distance").as_double();

        RCLCPP_INFO(this->get_logger(), 
            "max_linear_speed: %.2f, max_angular_speed: %.2f, wheel_base: %.2f, wheel_radius: %.2f, corridor_width: %.2f, look_ahead_distance: %.2f", 
            max_linear_speed_, max_angular_speed_, wheel_base_, wheel_radius_, corridor_width_, look_ahead_distance_);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        // Extract yaw from quaternion
        double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
        double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
        current_theta_ = std::atan2(siny_cosp, cosy_cosp);
    }

    WallLine extractWall(const sensor_msgs::msg::LaserScan::SharedPtr& msg, double center_angle_rad, double window_size_deg = 30.0)
    {
        WallLine line;
        line.valid = false;
        double window = window_size_deg * M_PI / 180.0;
        double angle_min = msg->angle_min;
        double angle_inc = msg->angle_increment;
        int n_points = msg->ranges.size();

        // Variables para los Mínimos Cuadrados (Regresión Lineal)
        double sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
        int count = 0;

        for (int i = 0; i < n_points; i++) {
            double angle = angle_min + i * angle_inc;
            
            // Si el láser está dentro de la "ventana" que miramos (ej. +- 30º desde la izquierda)
            if (angle > center_angle_rad - window && angle < center_angle_rad + window) {
                double r = msg->ranges[i];
                if (std::isfinite(r) && r > 0.1) {
                    // 1. Pasar de Polares (r, ángulo) a Cartesianas (x, y) locales del robot
                    double x = r * std::cos(angle);
                    double y = r * std::sin(angle);
                    
                    // 2. Acumular sumatorios
                    sum_x += x;
                    sum_y += y;
                    sum_xx += (x * x);
                    sum_xy += (x * y);
                    count++;
                }
            }
        }

        // 3. Aplicar las fórmulas de la recta y=mx+b si tenemos suficientes puntos
        if (count > 10) { 
            double denominator = (count * sum_xx) - (sum_x * sum_x);
            if (std::abs(denominator) > 1e-6) { // Evitar dividir por 0
                line.m = ((count * sum_xy) - (sum_x * sum_y)) / denominator;
                line.b = (sum_y - line.m * sum_x) / count;
                line.valid = true;
            }
        }

        return line;
    }

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Extraemos la línea de la pared izquierda (mirando a +90º, con una ventana de 45º a cada lado)
        left_wall_ = extractWall(msg, M_PI/2, 45.0);
        
        // Extraemos la línea de la pared derecha (mirando a -90º, ventana de 45º)
        right_wall_ = extractWall(msg, -M_PI/2, 45.0);

        if (left_wall_.valid || right_wall_.valid) {
            measured_data_ = true;
        }
    }

    void controlLoop()
    {
        if (!measured_data_) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Esperando a tener paredes válidas...");
            return;
        }
        else {
            measured_data_ = false; 

            // 1 y 2. OBTENER DISTANCIAS Y ERROR DE ORIENTACIÓN (THETA)
            double theta = 0.0;
            double dL = 0.0;

            if (left_wall_.valid && right_wall_.valid) {
                // Caso ideal: Vemos las dos paredes
                double d_left = left_wall_.b;
                double d_right = std::abs(right_wall_.b); 
                dL = (d_left - d_right) / 2.0;
                
                double theta_left = -std::atan(left_wall_.m);
                double theta_right = -std::atan(right_wall_.m);
                theta = (theta_left + theta_right) / 2.0;

                RCLCPP_INFO(this->get_logger(), "\033[1;32mIzq [b: %.2f, m: %.2f] | Der [b: %.2f, m: %.2f]\033[0m", 
                                                left_wall_.b, left_wall_.m, right_wall_.b, right_wall_.m);

            } else if (left_wall_.valid) {
                // Solo vemos la pared izquierda (ej. tomando una curva)
                dL = left_wall_.b - (corridor_width_ / 2.0);
                theta = -std::atan(left_wall_.m);
                RCLCPP_INFO(this->get_logger(), "\033[1;32mIzq [b: %.2f, m: %.2f] | Der    [No válido]    \033[0m",
                                                left_wall_.b, left_wall_.m);

            } else if (right_wall_.valid) {
                // Solo vemos la pared derecha (ej. tomando la otra curva)
                dL = (corridor_width_ / 2.0) - std::abs(right_wall_.b);
                theta = -std::atan(right_wall_.m);
                RCLCPP_INFO(this->get_logger(), "\033[1;32mIzq    [No válido]     | Der [b: %.2f, m: %.2f]\033[0m",
                                                right_wall_.b, right_wall_.m);
            } else {
                // Por seguridad, si llega aquí sin paredes, no hace nada
                return; 
            }

            // 3. CONTROL DE PERSECUCIÓN PURA (Tu código matemático original, intacto)
            double d = look_ahead_distance_;
            double dy = dL * std::cos(theta) - d * std::sin(theta); 
            double k = 2.0 * dy / (d * d); 
            
            double linear_velocity = 0.7 * max_linear_speed_ * (1.0 - std::fabs(dL) / (corridor_width_/2.0));
            if (linear_velocity < 0.1) linear_velocity = 0.1; 
            
            double angular_velocity = k * linear_velocity;

            if (angular_velocity > max_angular_speed_) angular_velocity = max_angular_speed_;
            if (angular_velocity < -max_angular_speed_) angular_velocity = -max_angular_speed_;
            
            // 4. PUBLICAR COMANDOS DE VELOCIDAD
            geometry_msgs::msg::Twist cmd_vel_msg;
            cmd_vel_msg.linear.x = linear_velocity;
            cmd_vel_msg.angular.z = angular_velocity;
            cmd_vel_pub_->publish(cmd_vel_msg);

            // --- PUBLICAR VARIABLES PARA RQT ---
            std_msgs::msg::Float64 msg_m_izq, msg_b_izq, msg_m_der, msg_b_der;

            // Si la pared es válida mandamos su valor, si se pierde mandamos 0.0
            msg_m_izq.data = left_wall_.valid ? left_wall_.m : 0.0;
            msg_b_izq.data = left_wall_.valid ? left_wall_.b : 0.0;
            msg_m_der.data = right_wall_.valid ? right_wall_.m : 0.0;
            msg_b_der.data = right_wall_.valid ? right_wall_.b : 0.0;

            pub_m_izq_->publish(msg_m_izq);
            pub_b_izq_->publish(msg_b_izq);
            pub_m_der_->publish(msg_m_der);
            pub_b_der_->publish(msg_b_der);
        }
    }

    // Publishers and Subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    // (Al final de la clase, junto a cmd_vel_pub_...)
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_m_izq_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_b_izq_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_m_der_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_b_der_;

    // Parameters
    int time_step_;
    double max_linear_speed_;
    double max_angular_speed_;      
    double wheel_base_;
    double wheel_radius_;
    double corridor_width_;
    double look_ahead_distance_;

    // Actual robot position
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_theta_ = 0.0;  

    // Laser data
    WallLine left_wall_;
    WallLine right_wall_;
    bool measured_data_ = false;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    
};  

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CorridorNavigationNode>());
    rclcpp::shutdown();
    return 0;
}