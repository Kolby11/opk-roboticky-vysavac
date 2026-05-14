#include "controls/Controls.hpp"

#include <chrono>
#include <sstream>
#include <sys/select.h>
#include <thread>
#include <unistd.h>

namespace
{
    constexpr auto KEY_REPEAT_GRACE = std::chrono::milliseconds(120);
}

Controls::Controls(double linear_speed, double angular_speed)
    : Node("controls"),
      command_publisher_(this->create_publisher<std_msgs::msg::String>("robot_command", 10)),
      twist_publisher_(this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10)),
      linear_speed_(linear_speed),
      angular_speed_(angular_speed)
{
    setupTerminal();
}

Controls::~Controls()
{
    restoreTerminal();
}

bool Controls::handleInput(int wait_ms)
{
    const int key = readInput(wait_ms);
    return handleKey(key);
}

bool Controls::handleKey(int key)
{
    if (key < 0)
        return true;

    const int ascii_key = key <= 255 ? (key & 0xff) : -1;
    const int low_byte = key & 0xff;

    const bool up_arrow = key == 0xff52 || key == 2490368 || (key > 255 && low_byte == 82);
    const bool down_arrow = key == 0xff54 || key == 2621440 || (key > 255 && low_byte == 84);
    const bool left_arrow = key == 0xff51 || key == 2424832 || (key > 255 && low_byte == 81);
    const bool right_arrow = key == 0xff53 || key == 2555904 || (key > 255 && low_byte == 83);

    switch (ascii_key)
    {
    case 27:
    case 'q':
    case 'Q':
        setActiveVelocity(0.0, 0.0);
        return false;
    default:
        break;
    }

    if (ascii_key == 'w' || ascii_key == 'W' || up_arrow)
    {
        setActiveVelocity(linear_speed_, 0.0);
    }
    else if (ascii_key == 's' || ascii_key == 'S' || down_arrow)
    {
        setActiveVelocity(-linear_speed_, 0.0);
    }
    else if (ascii_key == 'a' || ascii_key == 'A' || left_arrow)
    {
        setActiveVelocity(0.0, angular_speed_);
    }
    else if (ascii_key == 'd' || ascii_key == 'D' || right_arrow)
    {
        setActiveVelocity(0.0, -angular_speed_);
    }
    else if (ascii_key == ' ')
    {
        setActiveVelocity(0.0, 0.0);
    }

    return true;
}

void Controls::publishActiveCommand()
{
    if (std::chrono::steady_clock::now() <= active_until_)
    {
        publishVelocity(active_linear_, active_angular_);
        idle_stop_published_ = false;
        return;
    }

    if (!idle_stop_published_)
    {
        publishVelocity(0.0, 0.0);
        idle_stop_published_ = true;
    }
}

void Controls::setupTerminal()
{
    if (!isatty(STDIN_FILENO))
        return;

    if (tcgetattr(STDIN_FILENO, &original_terminal_) != 0)
        return;

    termios raw_terminal = original_terminal_;
    raw_terminal.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw_terminal.c_cc[VMIN] = 0;
    raw_terminal.c_cc[VTIME] = 0;

    terminal_configured_ = tcsetattr(STDIN_FILENO, TCSANOW, &raw_terminal) == 0;
}

void Controls::restoreTerminal()
{
    if (terminal_configured_)
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_terminal_);
        terminal_configured_ = false;
    }
}

int Controls::readInput(int wait_ms) const
{
    if (!isatty(STDIN_FILENO))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
        return -1;
    }

    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(STDIN_FILENO, &read_set);

    timeval timeout{};
    timeout.tv_sec = wait_ms / 1000;
    timeout.tv_usec = (wait_ms % 1000) * 1000;

    const int ready = select(STDIN_FILENO + 1, &read_set, nullptr, nullptr, &timeout);
    if (ready <= 0)
        return -1;

    unsigned char key = 0;
    return read(STDIN_FILENO, &key, 1) == 1 ? key : -1;
}

void Controls::publishVelocity(double linear, double angular)
{
    auto message = std_msgs::msg::String();
    std::ostringstream data;
    data << linear << ' ' << angular;
    message.data = data.str();
    command_publisher_->publish(message);

    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = linear;
    twist.angular.z = angular;
    twist_publisher_->publish(twist);
}

void Controls::setActiveVelocity(double linear, double angular)
{
    active_linear_ = linear;
    active_angular_ = angular;
    active_until_ = std::chrono::steady_clock::now() + KEY_REPEAT_GRACE;
    idle_stop_published_ = false;
    publishVelocity(active_linear_, active_angular_);
}
