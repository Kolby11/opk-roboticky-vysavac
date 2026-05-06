#include "controls/Controls.hpp"

#include <chrono>
#include <sstream>
#include <sys/select.h>
#include <thread>
#include <unistd.h>

Controls::Controls(double linear_speed, double angular_speed)
    : Node("controls"),
      publisher_(this->create_publisher<std_msgs::msg::String>("robot_command", 10)),
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
    if (key < 0)
        return true;

    switch (key)
    {
    case 27:
    case 'q':
    case 'Q':
        publishVelocity(0.0, 0.0);
        return false;
    case 'w':
    case 'W':
        publishVelocity(linear_speed_, 0.0);
        break;
    case 's':
    case 'S':
        publishVelocity(-linear_speed_, 0.0);
        break;
    case 'a':
    case 'A':
        publishVelocity(0.0, angular_speed_);
        break;
    case 'd':
    case 'D':
        publishVelocity(0.0, -angular_speed_);
        break;
    case ' ':
        publishVelocity(0.0, 0.0);
        break;
    default:
        break;
    }

    return true;
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
    publisher_->publish(message);
}
