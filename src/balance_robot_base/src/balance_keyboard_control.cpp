#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <map>
#include <cmath>
#include <sys/poll.h>

// Define key mappings
// Layout：
// Q   W   E
// A   S   D
// Z   X   C
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Z 0x7A
#define KEYCODE_X 0x78
#define KEYCODE_C 0x63

// Speed-adjustment keys
#define KEYCODE_I 0x69 // Increase linear speed
#define KEYCODE_COMMA 0x2C // decrease linear (,)
#define KEYCODE_J 0x6A // increase angular
#define KEYCODE_L 0x6C // decrease angular

// Constant definitions
constexpr double PI = 3.1415926535;
constexpr double DEG2RAD = PI / 180.0;
constexpr double LINEAR_MIN = 0.1;
constexpr double LINEAR_MAX = 0.5;
constexpr double LINEAR_STEP = 0.05;
constexpr double ANGULAR_MIN = 0.0;
constexpr double ANGULAR_MAX_DEG = 180.0;
constexpr double ANGULAR_STEP_DEG = 15.0;

class BalanceKeyboardControl : public rclcpp::Node
{
public:
    BalanceKeyboardControl() : Node("balance_keyboard_control")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        target_linear_vel_ = 0.3; // Default initial speed
        target_angular_vel_deg_ = 30.0; // Default initial angular speed (degrees))

        print_usage();
    }

    void run()
    {
        struct termios raw;
        tcgetattr(STDIN_FILENO, &raw);
        struct termios original = raw;

        // Set the terminal to raw mode
        raw.c_lflag &= ~(ICANON | ECHO);
        // Set the read timeout (critical for detecting key release))
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 1; // 0.1second timeout
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);

        char c;
        bool dirty = false; // Whether the display needs to be refreshed

        while (rclcpp::ok())
        {
            // Read a key (non-blocking/with timeout))
            int n = read(STDIN_FILENO, &c, 1);

            double x = 0;
            double th = 0;
            bool valid_key = false;

            if (n > 0)
            {
                // A key was detected
                switch(c)
                {
                    // --- 8direction controls ---
                    case KEYCODE_W: // Forward
                        x = 1.0; th = 0.0; valid_key = true; break;
                    case KEYCODE_X: // Backward
                        x = -1.0; th = 0.0; valid_key = true; break;
                    case KEYCODE_A: // Turn left in place)
                        x = 0.0; th = -1.0; valid_key = true; break;
                    case KEYCODE_D: // Turn right in place)
                        x = 0.0; th = 1.0; valid_key = true; break;
                    case KEYCODE_Q: // Forward + turn left
                        x = 1.0; th = -1.0; valid_key = true; break;
                    case KEYCODE_E: // Forward + turn right
                        x = 1.0; th = 1.0; valid_key = true; break;
                    case KEYCODE_Z: // Backward + turn left
                        x = -1.0; th = 1.0; valid_key = true; break; // Note: steering is conventionally reversed while moving backward
                    case KEYCODE_C: // Backward + turn right
                        x = -1.0; th = -1.0; valid_key = true; break;
                    case KEYCODE_S: // Force stop
                        x = 0.0; th = 0.0; valid_key = true; break;

                    // --- Speed adjustment ---
                    case KEYCODE_I: // Increase linear speed
                        target_linear_vel_ += LINEAR_STEP;
                        dirty = true; break;
                    case KEYCODE_COMMA: // Decrease linear speed (,)
                        target_linear_vel_ -= LINEAR_STEP;
                        dirty = true; break;
                    case KEYCODE_J: // Increase angular speed
                        target_angular_vel_deg_ += ANGULAR_STEP_DEG;
                        dirty = true; break;
                    case KEYCODE_L: // Decrease angular speed
                        target_angular_vel_deg_ -= ANGULAR_STEP_DEG;
                        dirty = true; break;
                    
                    // --- Exit ---
                    case 0x03: // Ctrl+C
                        rclcpp::shutdown();
                        break;
                }
            }

            // Range limits
            target_linear_vel_ = std::max(LINEAR_MIN, std::min(target_linear_vel_, LINEAR_MAX));
            target_angular_vel_deg_ = std::max(ANGULAR_MIN, std::min(target_angular_vel_deg_, ANGULAR_MAX_DEG));

            if (dirty)
            {
                print_status();
                dirty = false;
            }

            // Publish velocity
            geometry_msgs::msg::Twist twist;
            if (valid_key)
            {
                twist.linear.x = x * target_linear_vel_;
                twist.angular.z = th * (target_angular_vel_deg_ * DEG2RAD);
            }
            else
            {
                // If no key is pressed before timeout, send zero to stop
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
            }

            pub_->publish(twist);
        }

        // Restore terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &original);
    }

private:
    void print_usage()
    {
        std::cout << "------------------------------------" << std::endl;
        std::cout << "   Balance Robot Keyboard Control   " << std::endl;
        std::cout << "------------------------------------" << std::endl;
        std::cout << "Moving around:" << std::endl;
        std::cout << "   Q    W    E" << std::endl;
        std::cout << "   A    S    D" << std::endl;
        std::cout << "   Z    X    C" << std::endl;
        std::cout << "" << std::endl;
        std::cout << "W/X : Forward/Backward" << std::endl;
        std::cout << "A/D : Spin Left/Right" << std::endl;
        std::cout << "Q/E : Forward Left/Right" << std::endl;
        std::cout << "Z/C : Backward Left/Right" << std::endl;
        std::cout << "S   : Force Stop" << std::endl;
        std::cout << "" << std::endl;
        std::cout << "Speed Adjust:" << std::endl;
        std::cout << "i/, : +/- Linear Speed (Step 0.05)" << std::endl;
        std::cout << "j/l : +/- Angular Speed (Step 15 deg)" << std::endl;
        std::cout << "------------------------------------" << std::endl;
        std::cout << "CTRL-C to quit" << std::endl;
        print_status();
    }

    void print_status()
    {
        std::cout << "\rCurrent: Speed " << target_linear_vel_ << " m/s\tTurn " << target_angular_vel_deg_ << " deg/s   " << std::flush;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    double target_linear_vel_;
    double target_angular_vel_deg_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BalanceKeyboardControl>();
    node->run(); // Custom run loop
    rclcpp::shutdown();
    return 0;
}
