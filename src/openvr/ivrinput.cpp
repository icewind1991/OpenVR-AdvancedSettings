#include "ivrinput.h"
#include "ivrinput_action.h"
#include <openvr.h>
#include <iostream>
#include <array>
#include <easylogging++.h>

namespace input
{
/*!
Wrapper around the IVRInput GetDigitalActionData with error handling.

The struct is created and zero initalized in this function instead of being
passed as reference since the size is currently only 28 bytes, and currently
only two bools are used from the entire struct.
MSVC on /O2 will likely completely optimize the function call away but even if
it isn't the struct will still need to be created somewhere, and with move
semantics it almost doesn't matter if the struct is created in the calling
function and passed as reference, or created in this function.
*/
vr::InputDigitalActionData_t getDigitalActionData( DigitalAction& action )
{
    vr::InputDigitalActionData_t handleData = {};

    const auto error = vr::VRInput()->GetDigitalActionData(
        action.handle(),
        &handleData,
        sizeof( handleData ),
        vr::k_ulInvalidInputValueHandle );

    if ( error != vr::EVRInputError::VRInputError_None )
    {
        LOG( ERROR ) << "Error getting IVRInput Digital Action Data for handle "
                     << action.name() << ". SteamVR Error: " << error;
    }

    return handleData;
}

/*!
Wrapper around the IVRInput GetAnalogActionData with error handling.

The struct is created and zero initalized in this function instead of being
passed as reference since the size is currently only 28 bytes, and currently
only two bools are used from the entire struct.
MSVC on /O2 will likely completely optimize the function call away but even if
it isn't the struct will still need to be created somewhere, and with move
semantics it almost doesn't matter if the struct is created in the calling
function and passed as reference, or created in this function.
*/
vr::InputAnalogActionData_t getAnalogActionData( AnalogAction& action )
{
    vr::InputAnalogActionData_t handleData = {};

    const auto error
        = vr::VRInput()->GetAnalogActionData( action.handle(),
                                              &handleData,
                                              sizeof( handleData ),
                                              vr::k_ulInvalidInputValueHandle );

    if ( error != vr::EVRInputError::VRInputError_None )
    {
        LOG( ERROR ) << "Error getting IVRInput Digital Action Data for handle "
                     << action.name() << ". SteamVR Error: " << error;
    }

    return handleData;
}

/*!
Gets the action state of an Action.
This will only return true the first time the button is released. It is
necessary to release the button and the push it again in order for this function
to return true again.
*/
bool isDigitalActionActivatedOnce( DigitalAction& action )
{
    const auto handleData = getDigitalActionData( action );

    return handleData.bState && handleData.bChanged;
}

/*!
Gets the action state of an Action.
This will continually return true while the button is held down.
*/
bool isDigitalActionActivatedConstant( DigitalAction& action )
{
    const auto handleData = getDigitalActionData( action );

    return handleData.bState;
}

/*!
Sets up the input systems.
Actions should be added to the initializer list before the body of the
constructor. Actions should be certain that the action name strings are
correctly input, and that the correct type is provided.
Wrong action names may not result in direct error messages, but just features
mystically not working.
*/
SteamIVRInput::SteamIVRInput()
    : m_manifest(), m_haptics( action_sets::haptic ),
      m_music( action_sets::music ), m_motion( action_sets::motion ),
      m_misc( action_sets::misc ), m_system( action_sets::system ),
      m_nextTrack( action_keys::nextTrack ),
      m_previousTrack( action_keys::previousTrack ),
      m_pausePlayTrack( action_keys::pausePlayTrack ),
      m_stopTrack( action_keys::stopTrack ),
      m_leftHandSpaceTurn( action_keys::leftHandSpaceTurn ),
      m_rightHandSpaceTurn( action_keys::rightHandSpaceTurn ),
      m_leftHandSpaceDrag( action_keys::leftHandSpaceDrag ),
      m_rightHandSpaceDrag( action_keys::rightHandSpaceDrag ),
      m_optionalOverrideLeftHandSpaceTurn(
          action_keys::optionalOverrideLeftHandSpaceTurn ),
      m_optionalOverrideRightHandSpaceTurn(
          action_keys::optionalOverrideRightHandSpaceTurn ),
      m_optionalOverrideLeftHandSpaceDrag(
          action_keys::optionalOverrideLeftHandSpaceDrag ),
      m_optionalOverrideRightHandSpaceDrag(
          action_keys::optionalOverrideRightHandSpaceDrag ),
      m_swapSpaceDragToLeftHandOverride(
          action_keys::swapSpaceDragToLeftHandOverride ),
      m_swapSpaceDragToRightHandOverride(
          action_keys::swapSpaceDragToRightHandOverride ),
      m_gravityToggle( action_keys::gravityToggle ),
      m_gravityReverse( action_keys::gravityReverse ),
      m_heightToggle( action_keys::heightToggle ),
      m_resetOffsets( action_keys::resetOffsets ),
      m_snapTurnLeft( action_keys::snapTurnLeft ),
      m_snapTurnRight( action_keys::snapTurnRight ),
      m_smoothTurnLeft( action_keys::smoothTurnLeft ),
      m_smoothTurnRight( action_keys::smoothTurnRight ),
      m_autoTurnToggle( action_keys::autoTurnToggle ),
      m_addAutoAlignPointLeft( action_keys::addAutoAlignPointLeft ),
      m_addAutoAlignPointRight( action_keys::addAutoAlignPointRight ),
      m_xAxisLockToggle( action_keys::xAxisLockToggle ),
      m_yAxisLockToggle( action_keys::yAxisLockToggle ),
      m_zAxisLockToggle( action_keys::zAxisLockToggle ),
      m_keyboardOne( action_keys::keyboardOne ),
      m_keyboardTwo( action_keys::keyboardTwo ),
      m_keyboardThree( action_keys::keyboardThree ),
      m_keyPressMisc( action_keys::keyPressMisc ),
      m_keyPressSystem( action_keys::keyPressSystem ),
      m_chaperoneToggle( action_keys::chaperoneToggle ),
      m_pushToTalk( action_keys::pushToTalk ),
      m_exclusiveInputToggle( action_keys::exclusiveInputToggle ),
      m_leftHaptic( action_keys::hapticsLeft ),
      m_rightHaptic( action_keys::hapticsRight ),
      m_addLeftHapticClick( action_keys::addLeftHapticClick ),
      m_addRightHapticClick( action_keys::addRightHapticClick ),
      m_proxSensor( action_keys::proxSensor ),
      m_leftHand( input_keys::leftHand ), m_rightHand( input_keys::rightHand ),
      m_sets( { m_haptics.activeActionSet(),
                m_music.activeActionSet(),
                m_motion.activeActionSet(),
                m_misc.activeActionSet(),
                m_system.activeActionSet() } ),
      m_systemActionSets(
          { m_haptics.activeActionSet(), m_system.activeActionSet() } )
{
}
/*!
Returns true if the next media track should be played.

Will only return true the first time that the button is pressed. Holding the
button down will result in false until it has been released and pushed again.
*/
bool SteamIVRInput::nextSong()
{
    return isDigitalActionActivatedOnce( m_nextTrack );
}

/*!
Returns true if the previous media track should be played.

Will only return true the first time that the button is pressed. Holding the
button down will result in false until it has been released and pushed again.
*/
bool SteamIVRInput::previousSong()
{
    return isDigitalActionActivatedOnce( m_previousTrack );
}

/*!
Returns true if media playback should be paused/played.

Will only return true the first time that the button is pressed. Holding the
button down will result in false until it has been released and pushed again.
*/
bool SteamIVRInput::pausePlaySong()
{
    return isDigitalActionActivatedOnce( m_pausePlayTrack );
}

/*!
Returns true if media playback should be stopped.

Will only return true the first time that the button is pressed. Holding the
button down will result in false until it has been released and pushed again.
*/
bool SteamIVRInput::stopSong()
{
    return isDigitalActionActivatedOnce( m_stopTrack );
}

bool SteamIVRInput::leftHandSpaceTurn()
{
    return isDigitalActionActivatedConstant( m_leftHandSpaceTurn );
}

bool SteamIVRInput::rightHandSpaceTurn()
{
    return isDigitalActionActivatedConstant( m_rightHandSpaceTurn );
}

bool SteamIVRInput::leftHandSpaceDrag()
{
    return isDigitalActionActivatedConstant( m_leftHandSpaceDrag );
}

bool SteamIVRInput::rightHandSpaceDrag()
{
    return isDigitalActionActivatedConstant( m_rightHandSpaceDrag );
}
bool SteamIVRInput::optionalOverrideLeftHandSpaceTurn()
{
    return isDigitalActionActivatedConstant(
        m_optionalOverrideLeftHandSpaceTurn );
}

bool SteamIVRInput::optionalOverrideRightHandSpaceTurn()
{
    return isDigitalActionActivatedConstant(
        m_optionalOverrideRightHandSpaceTurn );
}

bool SteamIVRInput::optionalOverrideLeftHandSpaceDrag()
{
    return isDigitalActionActivatedConstant(
        m_optionalOverrideLeftHandSpaceDrag );
}

bool SteamIVRInput::optionalOverrideRightHandSpaceDrag()
{
    return isDigitalActionActivatedConstant(
        m_optionalOverrideRightHandSpaceDrag );
}

bool SteamIVRInput::swapSpaceDragToLeftHandOverride()
{
    return isDigitalActionActivatedConstant(
        m_swapSpaceDragToLeftHandOverride );
}

bool SteamIVRInput::swapSpaceDragToRightHandOverride()
{
    return isDigitalActionActivatedConstant(
        m_swapSpaceDragToRightHandOverride );
}

bool SteamIVRInput::gravityToggle()
{
    return isDigitalActionActivatedOnce( m_gravityToggle );
}

bool SteamIVRInput::gravityReverse()
{
    return isDigitalActionActivatedConstant( m_gravityReverse );
}

bool SteamIVRInput::heightToggle()
{
    return isDigitalActionActivatedOnce( m_heightToggle );
}

bool SteamIVRInput::resetOffsets()
{
    return isDigitalActionActivatedOnce( m_resetOffsets );
}

bool SteamIVRInput::snapTurnLeft()
{
    return isDigitalActionActivatedOnce( m_snapTurnLeft );
}

bool SteamIVRInput::snapTurnRight()
{
    return isDigitalActionActivatedOnce( m_snapTurnRight );
}

bool SteamIVRInput::smoothTurnLeft()
{
    return isDigitalActionActivatedConstant( m_smoothTurnLeft );
}

bool SteamIVRInput::smoothTurnRight()
{
    return isDigitalActionActivatedConstant( m_smoothTurnRight );
}
bool SteamIVRInput::autoTurnToggle()
{
    return isDigitalActionActivatedOnce( m_autoTurnToggle );
}
bool SteamIVRInput::addAutoAlignPointLeft()
{
    return isDigitalActionActivatedOnce( m_addAutoAlignPointLeft );
}

bool SteamIVRInput::addAutoAlignPointRight()
{
    return isDigitalActionActivatedOnce( m_addAutoAlignPointRight );
}

bool SteamIVRInput::xAxisLockToggle()
{
    return isDigitalActionActivatedOnce( m_xAxisLockToggle );
}

bool SteamIVRInput::yAxisLockToggle()
{
    return isDigitalActionActivatedOnce( m_yAxisLockToggle );
}

bool SteamIVRInput::zAxisLockToggle()
{
    return isDigitalActionActivatedOnce( m_zAxisLockToggle );
}

bool SteamIVRInput::pushToTalk()
{
    return isDigitalActionActivatedConstant( m_pushToTalk );
}

bool SteamIVRInput::chaperoneToggle()
{
    return isDigitalActionActivatedOnce( m_chaperoneToggle );
}

bool SteamIVRInput::proxState()
{
    return isDigitalActionActivatedConstant( m_proxSensor );
}

bool SteamIVRInput::addLeftHapticClick()
{
    return isDigitalActionActivatedConstant( m_addLeftHapticClick );
}

bool SteamIVRInput::addRightHapticClick()
{
    return isDigitalActionActivatedConstant( m_addRightHapticClick );
}

bool SteamIVRInput::keyboardOne()
{
    return isDigitalActionActivatedOnce( m_keyboardOne );
}

bool SteamIVRInput::keyboardTwo()
{
    return isDigitalActionActivatedOnce( m_keyboardTwo );
}
bool SteamIVRInput::keyboardThree()
{
    return isDigitalActionActivatedOnce( m_keyboardThree );
}
bool SteamIVRInput::keyPressMisc()
{
    return isDigitalActionActivatedConstant( m_keyPressMisc );
}
bool SteamIVRInput::keyPressSystem()
{
    return isDigitalActionActivatedConstant( m_keyPressSystem );
}
bool SteamIVRInput::exclusiveInputToggle()
{
    return isDigitalActionActivatedOnce( m_exclusiveInputToggle );
}
// Controls which action Sets are active, false = all, true = system + haptics
// Adjusts in update state call.
void SteamIVRInput::systemActionSetOnlyEnabled( bool value )
{
    m_exclusiveInputSetToggle = value;
}

// Sets Action Set Priority
void SteamIVRInput::actionSetPriorityToggle( bool value )
{
    if ( value )
    {
        for ( unsigned int i = 0; i < action_sets::numberOfSets; i++ )
        {
            m_sets.at( i ).nPriority = 0x01000001;
        }
    }
    else
    {
        for ( unsigned int i = 0; i < action_sets::numberOfSets; i++ )
        {
            m_sets.at( i ).nPriority = 0;
        }
    }
}

/*!
Returns the action handle of the Left Haptic Action
*/

vr::VRActionHandle_t SteamIVRInput::leftHapticActionHandle()
{
    return m_leftHaptic.handle();
}

/*!
Returns the action handle of the Right Haptic Action
*/
vr::VRActionHandle_t SteamIVRInput::rightHapticActionHandle()
{
    return m_rightHaptic.handle();
}

/*!
Returns the input handle of the Right hand
*/
vr::VRInputValueHandle_t SteamIVRInput::rightInputHandle()
{
    return m_rightHand.handle();
}

/*!
Returns the input handle of the Left hand
*/
vr::VRInputValueHandle_t SteamIVRInput::leftInputHandle()
{
    return m_leftHand.handle();
}

/*!
Updates the active action set(s).
Should be called every frame, or however often you want the input system to
update state.
*/
void SteamIVRInput::UpdateStates()
{
    vr::EVRInputError error;
    if ( !m_exclusiveInputSetToggle )
    {
        error = vr::VRInput()->UpdateActionState(
            m_sets.data(),
            sizeof( vr::VRActiveActionSet_t ),
            action_sets::numberOfSets );
    }
    else
    {
        error = vr::VRInput()->UpdateActionState(
            m_systemActionSets.data(), sizeof( vr::VRActiveActionSet_t ), 2 );
    }

    if ( error != vr::EVRInputError::VRInputError_None )
    {
        LOG( ERROR )
            << "Error during IVRInput action state update. OpenVR Error: "
            << error;
    }
}

} // namespace input
