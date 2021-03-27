#pragma once

#include <openvr.h>
#include "ivrinput_action.h"
#include "ivrinput_manifest.h"
#include "ivrinput_action_set.h"
#include "ivrinput_input_source.h"
#include <array>
#include <atomic>

namespace input
{
/*!
The IVRInput system is _very_ finnicky about strings matching and if they don't
match there are usually no errors logged reflecting that. constexpr auto strings
should be used to refer to all action manifest strings.
*/
namespace action_keys
{
    constexpr auto nextTrack = "/actions/music/in/NextTrack";
    constexpr auto previousTrack = "/actions/music/in/PreviousTrack";
    constexpr auto pausePlayTrack = "/actions/music/in/PausePlayTrack";
    constexpr auto stopTrack = "/actions/music/in/StopTrack";

    constexpr auto leftHandSpaceTurn = "/actions/motion/in/LeftHandSpaceTurn";
    constexpr auto rightHandSpaceTurn = "/actions/motion/in/RightHandSpaceTurn";
    constexpr auto leftHandSpaceDrag = "/actions/motion/in/LeftHandSpaceDrag";
    constexpr auto rightHandSpaceDrag = "/actions/motion/in/RightHandSpaceDrag";
    constexpr auto optionalOverrideLeftHandSpaceTurn
        = "/actions/motion/in/OptionalOverrideLeftHandSpaceTurn";
    constexpr auto optionalOverrideRightHandSpaceTurn
        = "/actions/motion/in/OptionalOverrideRightHandSpaceTurn";
    constexpr auto optionalOverrideLeftHandSpaceDrag
        = "/actions/motion/in/OptionalOverrideLeftHandSpaceDrag";
    constexpr auto optionalOverrideRightHandSpaceDrag
        = "/actions/motion/in/OptionalOverrideRightHandSpaceDrag";
    constexpr auto swapSpaceDragToLeftHandOverride
        = "/actions/motion/in/SwapSpaceDragToLeftHandOverride";
    constexpr auto swapSpaceDragToRightHandOverride
        = "/actions/motion/in/SwapSpaceDragToRightHandOverride";
    constexpr auto gravityToggle = "/actions/motion/in/GravityToggle";
    constexpr auto gravityReverse = "/actions/motion/in/GravityReverse";
    constexpr auto resetOffsets = "/actions/motion/in/ResetOffsets";
    constexpr auto heightToggle = "/actions/motion/in/HeightToggle";
    constexpr auto snapTurnLeft = "/actions/motion/in/SnapTurnLeft";
    constexpr auto snapTurnRight = "/actions/motion/in/SnapTurnRight";
    constexpr auto smoothTurnLeft = "/actions/motion/in/SmoothTurnLeft";
    constexpr auto smoothTurnRight = "/actions/motion/in/SmoothTurnRight";
    constexpr auto autoTurnToggle = "/actions/motion/in/AutoTurnToggle";
    constexpr auto addAutoAlignPointLeft
        = "/actions/motion/in/AddAutoAlignPointLeft";
    constexpr auto addAutoAlignPointRight
        = "/actions/motion/in/AddAutoAlignPointRight";

    constexpr auto xAxisLockToggle = "/actions/misc/in/XAxisLockToggle";
    constexpr auto yAxisLockToggle = "/actions/misc/in/YAxisLockToggle";
    constexpr auto zAxisLockToggle = "/actions/misc/in/ZAxisLockToggle";
    constexpr auto pushToTalk = "/actions/system/in/PushToTalk";

    constexpr auto keyboardOne = "/actions/misc/in/KeyboardOne";
    constexpr auto keyboardTwo = "/actions/misc/in/KeyboardTwo";
    constexpr auto keyboardThree = "/actions/misc/in/KeyboardThree";
    constexpr auto keyPressMisc = "/actions/misc/in/KeyPressMisc";
    constexpr auto keyPressSystem = "/actions/system/in/KeyPressSystem";

    constexpr auto exclusiveInputToggle
        = "/actions/system/in/ExclusiveInputToggle";

    constexpr auto hapticsLeft = "/actions/haptic/out/HapticsLeft";
    constexpr auto hapticsRight = "/actions/haptic/out/HapticsRight";
    constexpr auto proxSensor = "/actions/haptic/in/ProxSensor";
    constexpr auto addLeftHapticClick = "/actions/system/in/AddLeftHapticClick";
    constexpr auto addRightHapticClick
        = "/actions/system/in/AddRightHapticClick";

    constexpr auto chaperoneToggle = "/actions/misc/in/ChaperoneToggle";

} // namespace action_keys

/*!
Keys to get input source handles (things like hmd controllers etc.)
*/
namespace input_keys
{
    constexpr auto leftHand = "/user/hand/left";
    constexpr auto rightHand = "/user/hand/right";
} // namespace input_keys

/*!
Keys for different action sets
*/
namespace action_sets
{
    constexpr auto numberOfSets = 5;
    constexpr auto haptic = "/actions/haptic";
    constexpr auto music = "/actions/music";
    constexpr auto misc = "/actions/misc";
    constexpr auto motion = "/actions/motion";
    constexpr auto system = "/actions/system";
} // namespace action_sets

using ActiveActionSets
    = std::array<vr::VRActiveActionSet_t, action_sets::numberOfSets>;

/*!
Responsible for controller input.

UpdateStates should be called every frame.

Binary actions should only have one function,
and they should reflect the expected behaviour. I.e. if a button being pressed
and held should only return true the first time, that logic should be in the
function.

An action in the IVRInput API is entered in the actions manifest. This is a
.json file that is included with the final binary.
Currently an Action is created with a string and a type. The type determines
which function should be used for getting information from the API. The type in
the actions manifest must match the type in the source code.

Actions should be added as member objects to SteamIVRInput and initialized in
the constructor. They should then make a small number of accessor functions
available for outside use. The internal structs of the IVRInput API are not
entirely stable, and should not be leaked outside this class.

The name of the actions manifest must be set in ivrinput_manifest.h.
*/
class SteamIVRInput
{
public:
    SteamIVRInput();

    void UpdateStates();

    bool nextSong();
    bool previousSong();
    bool pausePlaySong();
    bool stopSong();
    bool leftHandSpaceTurn();
    bool rightHandSpaceTurn();
    bool leftHandSpaceDrag();
    bool rightHandSpaceDrag();
    bool optionalOverrideLeftHandSpaceTurn();
    bool optionalOverrideRightHandSpaceTurn();
    bool optionalOverrideLeftHandSpaceDrag();
    bool optionalOverrideRightHandSpaceDrag();
    bool swapSpaceDragToLeftHandOverride();
    bool swapSpaceDragToRightHandOverride();
    bool gravityToggle();
    bool gravityReverse();
    bool heightToggle();
    bool resetOffsets();
    bool snapTurnLeft();
    bool snapTurnRight();
    bool smoothTurnLeft();
    bool smoothTurnRight();
    bool autoTurnToggle();
    bool addAutoAlignPointLeft();
    bool addAutoAlignPointRight();
    bool xAxisLockToggle();
    bool yAxisLockToggle();
    bool zAxisLockToggle();
    bool chaperoneToggle();
    bool proxState();
    bool addLeftHapticClick();
    bool addRightHapticClick();

    bool pushToTalk();

    bool exclusiveInputToggle();

    bool keyboardOne();
    bool keyboardTwo();
    bool keyboardThree();
    bool keyPressMisc();
    bool keyPressSystem();

    void systemActionSetOnlyEnabled( bool value );
    void actionSetPriorityToggle( bool value );

    // false is all Sets, True IS System + haptics
    std::atomic<bool> m_exclusiveInputSetToggle = false;

    vr::VRActionHandle_t leftHapticActionHandle();
    vr::VRActionHandle_t rightHapticActionHandle();

    vr::VRInputValueHandle_t rightInputHandle();
    vr::VRInputValueHandle_t leftInputHandle();

    // Destructor. There are no terminating calls for the IVRInput API, so it
    // is left blank.
    ~SteamIVRInput() {}
    // These have been explicitly deleted to make sure there are no attempts at
    // copying the class in weird ways. It is not worth defining what should
    // happen on copy because it simply shouldn't be done.
    // They are declared public because compilers will give wrong error messages
    // if they are private.
    // Copy constructor
    SteamIVRInput( const SteamIVRInput& ) = delete;
    // Copy assignment
    SteamIVRInput& operator=( const SteamIVRInput& ) = delete;
    // Move constructor
    SteamIVRInput( SteamIVRInput&& ) = delete;
    // Move assignment
    SteamIVRInput& operator=( const SteamIVRInput&& ) = delete;

private:
    Manifest m_manifest;

    ActionSet m_haptics;
    ActionSet m_music;
    ActionSet m_motion;
    ActionSet m_misc;
    ActionSet m_system;

    // Music player bindings
    DigitalAction m_nextTrack;
    DigitalAction m_previousTrack;
    DigitalAction m_pausePlayTrack;
    DigitalAction m_stopTrack;

    // Space bindings
    DigitalAction m_leftHandSpaceTurn;
    DigitalAction m_rightHandSpaceTurn;
    DigitalAction m_leftHandSpaceDrag;
    DigitalAction m_rightHandSpaceDrag;
    DigitalAction m_optionalOverrideLeftHandSpaceTurn;
    DigitalAction m_optionalOverrideRightHandSpaceTurn;
    DigitalAction m_optionalOverrideLeftHandSpaceDrag;
    DigitalAction m_optionalOverrideRightHandSpaceDrag;
    DigitalAction m_swapSpaceDragToLeftHandOverride;
    DigitalAction m_swapSpaceDragToRightHandOverride;
    DigitalAction m_gravityToggle;
    DigitalAction m_gravityReverse;
    DigitalAction m_heightToggle;
    DigitalAction m_resetOffsets;
    DigitalAction m_snapTurnLeft;
    DigitalAction m_snapTurnRight;
    DigitalAction m_smoothTurnLeft;
    DigitalAction m_smoothTurnRight;
    DigitalAction m_autoTurnToggle;
    DigitalAction m_addAutoAlignPointLeft;
    DigitalAction m_addAutoAlignPointRight;
    DigitalAction m_xAxisLockToggle;
    DigitalAction m_yAxisLockToggle;
    DigitalAction m_zAxisLockToggle;

    // Keyboard Shortcuts
    DigitalAction m_keyboardOne;
    DigitalAction m_keyboardTwo;
    DigitalAction m_keyboardThree;
    DigitalAction m_keyPressMisc;
    DigitalAction m_keyPressSystem;

    // Misc Bindings
    DigitalAction m_chaperoneToggle;

    // Push To Talk
    DigitalAction m_pushToTalk;

    // Exclusive Input Toggle
    DigitalAction m_exclusiveInputToggle;

    // haptic bindings
    DigitalAction m_leftHaptic;
    DigitalAction m_rightHaptic;
    DigitalAction m_addLeftHapticClick;
    DigitalAction m_addRightHapticClick;

    // prox Sensor
    DigitalAction m_proxSensor;

    // input sources
    InputSource m_leftHand;
    InputSource m_rightHand;

    // Initialize the set of actions after everything else.
    ActiveActionSets m_sets;
    ActiveActionSets m_systemActionSets;
};

} // namespace input
