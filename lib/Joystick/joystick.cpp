#include "joystick.h"

JoystickManager::JoystickManager() :
    joy_hub_(usb_joy_),
    hid_joy_(usb_joy_),
    joy_control_{
        JoystickController(usb_joy_),
        JoystickController(usb_joy_),
        JoystickController(usb_joy_),
        JoystickController(usb_joy_)
    }
{
}

void JoystickManager::begin()
{
    usb_joy_.begin();
}

void JoystickManager::task()
{
    usb_joy_.Task();
}

bool JoystickManager::isConnected()
{
    return (bool)joy_control_[0];
}

bool JoystickManager::isTimeout() const
{
    return (millis() - lastJoystickTime_ > JOYSTICK_TIMEOUT_MS);
}

bool JoystickManager::isManualInput() const
{
    return (axis.axis1_x != 0 || axis.axis1_y != 0 || axis.axis0_y != 0);
}

bool JoystickManager::isButtonYPressed()
{
    uint32_t rawButtons = joy_control_[0].getButtons();
    return (rawButtons & 8) != 0;
}

int JoystickManager::applyDeadzone(int value, int deadzone)
{
    if (abs(value) < deadzone)
    {
        return 0;
    }
    return value;
}

void JoystickManager::parseButtons(uint32_t buttons, int hatAxis)
{
    button.A = button.B = button.X = button.Y = 0;
    button.LB = button.RB = button.LT = button.RT = 0;
    button.start = button.select = button.home = 0;

    if (buttons & 2)
        button.A = 1;
    if (buttons & 4)
        button.B = 1;
    if (buttons & 1)
        button.X = 1;
    if (buttons & 8)
        button.Y = 1;
    if (buttons & 32)
        button.RB = 1;
    if (buttons & 16)
        button.LB = 1;
    if (buttons & 64)
        button.LT = 1;
    if (buttons & 128)
        button.RT = 1;
    if (buttons & 256)
        button.select = 1;
    if (buttons & 512)
        button.start = 1;
    if (buttons & 4096)
        button.home = 1;

    button.up = button.down = button.left = button.right = 0;
    switch (hatAxis)
    {
    case 0:
        button.up = 1;
        break;
    case 2:
        button.right = 1;
        break;
    case 4:
        button.down = 1;
        break;
    case 6:
        button.left = 1;
        break;
    }
}

void JoystickManager::handleSpeedToggle()
{
    if (button.up == 1 && !lastButtonUp_ && millis() - lastToggleTime_ > JOYSTICK_SPEED_DEBOUNCE_MS)
    {
        speedModeFast_ = !speedModeFast_;
        lastToggleTime_ = millis();
    }
    lastButtonUp_ = button.up;
}

bool JoystickManager::update()
{
    if (joy_control_[0].available() || joy_control_[0].axisMask() != 0)
    {
        lastJoystickTime_ = millis();
        joystickAxesSeen_ |= joy_control_[0].axisMask();

        uint32_t buttons = joy_control_[0].getButtons();
        parseButtons(buttons, joy_control_[0].getAxis(9));

        axis.axis1_x = applyDeadzone(joy_control_[0].getAxis(1) - 128);
        axis.axis1_y = applyDeadzone(joy_control_[0].getAxis(0) - 128);
        axis.axis0_x = applyDeadzone(joy_control_[0].getAxis(5) - 128);
        axis.axis0_y = applyDeadzone(joy_control_[0].getAxis(2) - 128);

        joy_control_[0].joystickDataClear();

        handleSpeedToggle();
        return true;
    }
    return false;
}

bool JoystickManager::qualifyReady()
{
    bool manualInput = isManualInput();
    if ((joystickAxesSeen_ & REQUIRED_JOYSTICK_AXES) != REQUIRED_JOYSTICK_AXES || manualInput)
    {
        return false;
    }
    joystickReady_ = true;
    return true;
}

void JoystickManager::reset()
{
    joystickReady_ = false;
    joystickAxesSeen_ = 0;
    axis = {};
    button = {};
}

void JoystickManager::handleDisconnect()
{
    lastJoystickTime_ = 0;
    joy_control_[0].joystickDataClear();
}
