#pragma once

#include <bitset>

#include <SDL.h>

#if !defined(__EMSCRIPTEN__) && !defined(__ANDROID__) && !(defined(__APPLE__) && TARGET_OS_IOS) && !defined(__amigaos4__)
#   define SDL_HAS_CAPTURE_AND_GLOBAL_MOUSE 1
#else
#   define SDL_HAS_CAPTURE_AND_GLOBAL_MOUSE 0
#endif

DECLARE_MESSAGE(KeyMapChanged);

enum EMouseButton
{
    MOUSE_INVALID = SDL_NUM_SCANCODES,
    MOUSE_1, // Left
    MOUSE_2, // Right
    MOUSE_3, // Middle
    MOUSE_4, // X1
    MOUSE_5, // X2
    MOUSE_MAX,
    MOUSE_COUNT = MOUSE_MAX - MOUSE_INVALID - 1
};

enum EControllerButton
{
    XR_CONTROLLER_BUTTON_INVALID = MOUSE_MAX,
    XR_CONTROLLER_BUTTON_A,
    XR_CONTROLLER_BUTTON_B,
    XR_CONTROLLER_BUTTON_X,
    XR_CONTROLLER_BUTTON_Y,
    XR_CONTROLLER_BUTTON_BACK,
    XR_CONTROLLER_BUTTON_GUIDE,
    XR_CONTROLLER_BUTTON_START,
    XR_CONTROLLER_BUTTON_LEFTSTICK,
    XR_CONTROLLER_BUTTON_RIGHTSTICK,
    XR_CONTROLLER_BUTTON_LEFTSHOULDER,
    XR_CONTROLLER_BUTTON_RIGHTSHOULDER,
    XR_CONTROLLER_BUTTON_DPAD_UP,
    XR_CONTROLLER_BUTTON_DPAD_DOWN,
    XR_CONTROLLER_BUTTON_DPAD_LEFT,
    XR_CONTROLLER_BUTTON_DPAD_RIGHT,
    XR_CONTROLLER_BUTTON_MISC1,    /* Xbox Series X share button, PS5 microphone button, Nintendo Switch Pro capture button */
    XR_CONTROLLER_BUTTON_PADDLE1,  /* Xbox Elite paddle P1 */
    XR_CONTROLLER_BUTTON_PADDLE2,  /* Xbox Elite paddle P3 */
    XR_CONTROLLER_BUTTON_PADDLE3,  /* Xbox Elite paddle P2 */
    XR_CONTROLLER_BUTTON_PADDLE4,  /* Xbox Elite paddle P4 */
    XR_CONTROLLER_BUTTON_TOUCHPAD, /* PS4/PS5 touchpad button */
    XR_CONTROLLER_BUTTON_MAX,
    XR_CONTROLLER_BUTTON_COUNT = XR_CONTROLLER_BUTTON_MAX - XR_CONTROLLER_BUTTON_INVALID - 1
};

// SDL has separate axes for X and Y, we don't. Except trigger. Trigger should be separated.
enum EControllerAxis
{
    XR_CONTROLLER_AXIS_INVALID = XR_CONTROLLER_BUTTON_MAX,
    XR_CONTROLLER_AXIS_LEFT,
    XR_CONTROLLER_AXIS_RIGHT,
    XR_CONTROLLER_AXIS_TRIGGER_LEFT,
    XR_CONTROLLER_AXIS_TRIGGER_RIGHT,
    XR_CONTROLLER_AXIS_MAX,
    XR_CONTROLLER_AXIS_COUNT = XR_CONTROLLER_AXIS_MAX - XR_CONTROLLER_AXIS_INVALID - 1
};

static_assert(SDL_CONTROLLER_AXIS_MAX == 6,
    "EControllerAxis needs to be updated to match changes in SDL_GameControllerAxis.");

struct ControllerAxisState
{
    union
    {
        Fvector2 xy;
        struct
        {
            float x, y;
        };
    };
    float magnitude;

    ControllerAxisState() = default;

    constexpr ControllerAxisState(const float v)
        : xy{ v, v }, magnitude{ v } {}

    constexpr ControllerAxisState(const float x, const float y, const float mag)
        : xy{ x, y }, magnitude{ mag } {}

    constexpr ControllerAxisState(const Fvector2& vec, const float mag)
        : xy{ vec }, magnitude{ mag } {}

    constexpr ControllerAxisState(Fvector2&& vec, const float mag)
        : xy{ std::move(vec) }, magnitude{ mag } {}

    ControllerAxisState(const float x, const float y)
        : ControllerAxisState(Fvector2{ x, y }) {}

    ControllerAxisState(const Fvector2& vec)
        : xy{ vec }, magnitude{ vec.magnitude() } {}
};

// Make sure it fits the C++ standard requirements on unions and we can use default constructor
static_assert(std::is_trivial_v<ControllerAxisState>);

struct ENGINE_API ControllerState
{
    union
    {
        ControllerAxisState axes[XR_CONTROLLER_AXIS_COUNT]{};
        struct
        {
            ControllerAxisState left;
            ControllerAxisState right;
            ControllerAxisState trigger_left;
            ControllerAxisState trigger_right;
        } axis;
    };
    static_assert(sizeof(axes) == sizeof(axis),
        "New axis added in EControllerAxis. "
        "Please, add new corresponding *named* axis.");

    std::bitset<XR_CONTROLLER_BUTTON_COUNT> buttons;

    Fvector gyroscope{};

    s32 id{ -1 }; // The current active controller ID

    constexpr ControllerState() = default;

    const ControllerAxisState& get_axis(const int key) const noexcept
    {
        if (key > XR_CONTROLLER_AXIS_INVALID && key < XR_CONTROLLER_AXIS_MAX)
        {
            const int idx = key - (XR_CONTROLLER_AXIS_INVALID + 1);
            return axes[idx];
        }
        static ControllerAxisState dummy{};
        return dummy;
    }

    bool attitude_changed() const;
};

class ENGINE_API IInputReceiver;

class ENGINE_API CInput
    : public pureFrame,
      public pureAppActivate,
      public pureAppDeactivate
{
public:
    enum FeedbackType
    {
        FeedbackController, // Entire gamepad
        FeedbackTriggers,
    };

    enum InputType : u8
    {
        KeyboardMouse,
        Controller,
    };

    enum
    {
        COUNT_MOUSE_BUTTONS = MOUSE_COUNT,
        COUNT_MOUSE_AXIS = 4,
        COUNT_KB_BUTTONS = SDL_NUM_SCANCODES,
    };

    struct InputStatistics
    {
        CStatTimer FrameTime;

        void FrameStart() { FrameTime.FrameStart(); }
        void FrameEnd() { FrameTime.FrameEnd(); }
    };

private:
    InputStatistics stats;

    std::bitset<COUNT_KB_BUTTONS> keyboardState;
    std::bitset<COUNT_MOUSE_BUTTONS> mouseState;
    int mouseAxisState[COUNT_MOUSE_AXIS];
    ControllerState controllerState;

    xr_vector<IInputReceiver*> cbStack;

    xr_vector<SDL_GameController*> controllers;

    void SetCurrentInputType(InputType type);

    void MouseUpdate();
    void KeyUpdate();
    void ControllerUpdate();

    void OpenController(int idx);

    MessageRegistry<pureKeyMapChanged> seqKeyMapChanged;

    int textInputCounter{};

    InputType currentInputType{ KeyboardMouse };

    bool exclusiveInput;
    bool inputGrabbed;

    SDL_Cursor* mouseCursors[SDL_NUM_SYSTEM_CURSORS]{};
    SDL_Cursor* lastCursor{};

public:
    const InputStatistics& GetStats() const { return stats; }
    void DumpStatistics(class IGameFont& font, class IPerformanceAlert* alert);

    void iCapture(IInputReceiver* pc);
    void iRelease(IInputReceiver* pc);

    bool iGetAsyncKeyState(const int key);
    const auto& iGetAsyncControllerState() const { return controllerState; }
    bool iAnyMouseButtonDown() const { return mouseState.any(); }
    bool iAnyKeyButtonDown() const { return keyboardState.any(); }
    bool iAnyControllerButtonDown() const { return controllerState.buttons.any(); }

    void iGetAsyncScrollPos(Ivector2& p) const;
    bool iGetAsyncMousePos(Ivector2& p, bool global = false) const;
    bool iSetMousePos(const Ivector2& p, bool global = false) const;

    void GrabInput(const bool grab);
    bool InputIsGrabbed() const;

    void ShowCursor(const bool show);
    void SetCursor(const SDL_SystemCursor cursor);

    void EnableTextInput();
    void DisableTextInput();
    bool IsTextInputEnabled() const;

    void RegisterKeyMapChangeWatcher(pureKeyMapChanged* watcher, int priority = REG_PRIORITY_NORMAL);
    void RemoveKeyMapChangeWatcher(pureKeyMapChanged* watcher);

    CInput(const bool exclusive = true);
    ~CInput();

    virtual void OnFrame();
    virtual void OnAppActivate();
    virtual void OnAppDeactivate();

    IInputReceiver* CurrentIR();

    bool IsControllerAvailable() const { return !controllers.empty(); }

    auto GetCurrentInputType() const { return currentInputType; }
    auto IsCurrentInputTypeController() const { return GetCurrentInputType() == InputType::Controller; }
    auto IsCurrentInputTypeKeyboardMouse() const { return GetCurrentInputType() == InputType::KeyboardMouse; }

public:
    void ExclusiveMode(const bool exclusive);
    bool IsExclusiveMode() const;
    bool GetKeyName(const int dik, pstr dest, int dest_sz);

    /**
    *  Start a gamepad vibration effect
    *  Each call to this function cancels any previous vibration effect, and calling it with 0 intensity stops any vibration.
    *
    *  @param type Feedback source
    *  @param s1 The intensity of the low frequency (left) motor or left trigger motor, from 0 to 1
    *  @param s2 The intensity of the high frequency (right) motor or right trigger motor, from 0 to 1
    *  @param duration The duration of the rumble effect, in seconds
    */
    void Feedback(FeedbackType type, float s1, float s2, float duration);
};

extern ENGINE_API CInput* pInput;
