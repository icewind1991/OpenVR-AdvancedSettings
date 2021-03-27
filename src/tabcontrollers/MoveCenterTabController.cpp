#include "MoveCenterTabController.h"
#include <QQuickWindow>
#include "../overlaycontroller.h"
#include "../quaternion/quaternion.h"
#include "../settings/settings.h"

void rotateCoordinates( double coordinates[3], double angle )
{
    if ( angle == 0 )
    {
        return;
    }
    double s = sin( angle );
    double c = cos( angle );
    double newX = coordinates[0] * c - coordinates[2] * s;
    double newZ = coordinates[0] * s + coordinates[2] * c;
    coordinates[0] = newX;
    coordinates[2] = newZ;
}

void rotateFloatCoordinates( float coordinates[3], float angle )
{
    if ( angle == 0 )
    {
        return;
    }
    // crazy casts for compiler warnings
    float s = static_cast<float>( sin( static_cast<double>( angle ) ) );
    float c = static_cast<float>( cos( static_cast<double>( angle ) ) );
    float newX = coordinates[0] * c - coordinates[2] * s;
    float newZ = coordinates[0] * s + coordinates[2] * c;
    coordinates[0] = newX;
    coordinates[2] = newZ;
}

// application namespace
namespace advsettings
{
void MoveCenterTabController::initStage1()
{
    reloadOffsetProfiles();
    m_lastDragUpdateTimePoint = std::chrono::steady_clock::now();
    m_lastGravityUpdateTimePoint = std::chrono::steady_clock::now();
}

void MoveCenterTabController::initStage2( OverlayController* var_parent )
{
    this->parent = var_parent;
    zeroOffsets();
}

void MoveCenterTabController::reloadOffsetProfiles()
{
    settings::loadAllObjects( m_offsetProfiles );
}

void MoveCenterTabController::saveOffsetProfiles()
{
    settings::saveAllObjects( m_offsetProfiles );
}

Q_INVOKABLE unsigned MoveCenterTabController::getOffsetProfileCount()
{
    return static_cast<unsigned int>( m_offsetProfiles.size() );
}

Q_INVOKABLE QString
    MoveCenterTabController::getOffsetProfileName( unsigned index )
{
    if ( index >= m_offsetProfiles.size() )
    {
        return QString();
    }
    else
    {
        return QString::fromStdString( m_offsetProfiles[index].profileName );
    }
}

void MoveCenterTabController::addOffsetProfile( QString name )
{
    OffsetProfile* profile = nullptr;
    for ( auto& p : m_offsetProfiles )
    {
        if ( p.profileName.compare( name.toStdString() ) == 0 )
        {
            profile = &p;
            break;
        }
    }
    if ( !profile )
    {
        auto i = m_offsetProfiles.size();
        m_offsetProfiles.emplace_back();
        profile = &m_offsetProfiles[i];
    }
    profile->profileName = name.toStdString();
    profile->offsetX = m_offsetX;
    profile->offsetY = m_offsetY;
    profile->offsetZ = m_offsetZ;
    profile->rotation = m_rotation;
    saveOffsetProfiles();
    emit offsetProfilesUpdated();
}

void MoveCenterTabController::applyOffsetProfile( unsigned index )
{
    if ( index < m_offsetProfiles.size() )
    {
        auto& profile = m_offsetProfiles[index];
        m_rotation = profile.rotation;
        m_offsetX = profile.offsetX;
        m_offsetY = profile.offsetY;
        m_offsetZ = profile.offsetZ;
        emit rotationChanged( m_rotation );
        emit offsetXChanged( m_offsetX );
        emit offsetYChanged( m_offsetY );
        emit offsetZChanged( m_offsetZ );
        LOG( INFO ) << "Applying Offset Profile:" << profile.profileName
                    << " X:" << m_offsetX << " Y:" << m_offsetY
                    << " Z:" << m_offsetZ << " Rotation:"
                    << ( static_cast<float>( m_rotation ) / 100 );
    }
}

void MoveCenterTabController::deleteOffsetProfile( unsigned index )
{
    if ( index < m_offsetProfiles.size() )
    {
        auto pos = m_offsetProfiles.begin() + index;
        m_offsetProfiles.erase( pos );
        saveOffsetProfiles();
        emit offsetProfilesUpdated();
    }
}

void MoveCenterTabController::outputLogPoses()
{
    vr::TrackedDevicePose_t devicePosesStanding[vr::k_unMaxTrackedDeviceCount];

    vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(
        vr::TrackingUniverseStanding,
        0.0f,
        devicePosesStanding,
        vr::k_unMaxTrackedDeviceCount );

    vr::HmdMatrix34_t hmdStanding
        = devicePosesStanding[0].mDeviceToAbsoluteTracking;

    LOG( INFO ) << "";
    LOG( INFO ) << " ____Begin Matrices Log Ouput__________";
    LOG( INFO ) << "|";
    LOG( INFO ) << "HMD POSE (standing universe)";
    outputLogHmdMatrix( hmdStanding );

    vr::TrackedDevicePose_t devicePosesSeated[vr::k_unMaxTrackedDeviceCount];

    vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(
        vr::TrackingUniverseSeated,
        0.0f,
        devicePosesSeated,
        vr::k_unMaxTrackedDeviceCount );

    vr::HmdMatrix34_t hmdSeated
        = devicePosesSeated[0].mDeviceToAbsoluteTracking;

    LOG( INFO ) << "HMD POSE (seated universe)";
    outputLogHmdMatrix( hmdSeated );

    auto leftHand = vr::VRSystem()->GetTrackedDeviceIndexForControllerRole(
        vr::TrackedControllerRole_LeftHand );
    auto rightHand = vr::VRSystem()->GetTrackedDeviceIndexForControllerRole(
        vr::TrackedControllerRole_RightHand );

    vr::HmdMatrix34_t leftHandMatrix
        = devicePosesStanding[leftHand].mDeviceToAbsoluteTracking;
    vr::HmdMatrix34_t rightHandMatrix
        = devicePosesStanding[rightHand].mDeviceToAbsoluteTracking;

    LOG( INFO ) << "LEFT Hand Controller Pose (standing universe)";
    outputLogHmdMatrix( leftHandMatrix );

    LOG( INFO ) << "RIGHT Hand Controller Pose (standing universe)";
    outputLogHmdMatrix( rightHandMatrix );

    vr::HmdMatrix34_t standingZero;
    vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(
        &standingZero );
    LOG( INFO ) << "GetWorkingStandingZeroPoseToRawTrackingPose";
    outputLogHmdMatrix( standingZero );

    vr::HmdMatrix34_t seatedZero;
    vr::VRChaperoneSetup()->GetWorkingSeatedZeroPoseToRawTrackingPose(
        &seatedZero );
    LOG( INFO ) << "GetWorkingSeatedZeroPoseToRawTrackingPose";
    outputLogHmdMatrix( seatedZero );

    vr::HmdMatrix34_t seatedZeroLive;
    vr::VRChaperoneSetup()->GetLiveSeatedZeroPoseToRawTrackingPose(
        &seatedZeroLive );
    LOG( INFO ) << "GetLiveSeatedZeroPoseToRawTrackingPose";
    outputLogHmdMatrix( seatedZeroLive );

    vr::HmdMatrix34_t seatedToStandingAbsolute
        = vr::VRSystem()->GetSeatedZeroPoseToStandingAbsoluteTrackingPose();
    LOG( INFO ) << "GetSeatedZeroPoseToStandingAbsoluteTrackingPose";
    outputLogHmdMatrix( seatedToStandingAbsolute );

    LOG( INFO ) << "m_universeCenterForReset";
    outputLogHmdMatrix( m_universeCenterForReset );

    LOG( INFO ) << "m_seatedCenterForReset";
    outputLogHmdMatrix( m_seatedCenterForReset );

    LOG( INFO ) << "|____End Matrices Log Ouput____________";
    LOG( INFO ) << "";
}

void MoveCenterTabController::outputLogHmdMatrix( vr::HmdMatrix34_t hmdMatrix )
{
    LOG( INFO ) << hmdMatrix.m[0][0] << "\t\t\t" << hmdMatrix.m[0][1]
                << "\t\t\t" << hmdMatrix.m[0][2] << "\t\t\t"
                << hmdMatrix.m[0][3];
    LOG( INFO ) << hmdMatrix.m[1][0] << "\t\t\t" << hmdMatrix.m[1][1]
                << "\t\t\t" << hmdMatrix.m[1][2] << "\t\t\t"
                << hmdMatrix.m[1][3];
    LOG( INFO ) << hmdMatrix.m[2][0] << "\t\t\t" << hmdMatrix.m[2][1]
                << "\t\t\t" << hmdMatrix.m[2][2] << "\t\t\t"
                << hmdMatrix.m[2][3];

    float atan2Yaw = std::atan2( hmdMatrix.m[0][2], hmdMatrix.m[2][2] );
    LOG( INFO ) << "atan2 Yaw Calculation:  " << atan2Yaw << "radians  "
                << ( ( static_cast<double>( atan2Yaw )
                       * k_radiansToCentidegrees )
                     / 100 )
                << "degrees";

    vr::HmdQuaternion_t hmdQuat = quaternion::fromHmdMatrix34( hmdMatrix );
    double hmdQuatYaw = quaternion::getYaw( hmdQuat );
    LOG( INFO ) << "Quaternion Yaw Calculation:  " << hmdQuatYaw << "radians  "
                << ( ( hmdQuatYaw * k_radiansToCentidegrees ) / 100 )
                << "degrees";
    LOG( INFO ) << "";
}

int MoveCenterTabController::trackingUniverse() const
{
    return static_cast<int>( m_trackingUniverse );
}

void MoveCenterTabController::setTrackingUniverse( int value, bool notify )
{
    if ( m_trackingUniverse != value )
    {
        vr::VRChaperoneSetup()->HideWorkingSetPreview();
        if ( !m_roomSetupModeDetected && value == vr::TrackingUniverseStanding )
        {
            reset();
        }

        m_trackingUniverse = value;
        if ( notify )
        {
            emit trackingUniverseChanged( m_trackingUniverse );
            LOG( INFO ) << "Tracking Universe: " << m_trackingUniverse;
        }
    }
}

float MoveCenterTabController::offsetX() const
{
    return m_offsetX;
}

void MoveCenterTabController::setOffsetX( float value, bool notify )
{
    if ( m_offsetX != value )
    {
        modOffsetX( value - m_offsetX, notify );
    }
}

float MoveCenterTabController::offsetY() const
{
    return m_offsetY;
}

void MoveCenterTabController::setOffsetY( float value, bool notify )
{
    if ( m_offsetY != value )
    {
        modOffsetY( value - m_offsetY, notify );
    }
}

float MoveCenterTabController::offsetZ() const
{
    return m_offsetZ;
}

void MoveCenterTabController::setOffsetZ( float value, bool notify )
{
    if ( m_offsetZ != value )
    {
        modOffsetZ( value - m_offsetZ, notify );
    }
}

int MoveCenterTabController::rotation() const
{
    return m_rotation;
}

int MoveCenterTabController::tempRotation() const
{
    return m_tempRotation;
}

void MoveCenterTabController::setRotation( int value, bool notify )
{
    if ( m_rotation != value )
    {
        if ( universeCenteredRotation() )
        {
            m_rotation = value;
            if ( notify )
            {
                emit rotationChanged( m_rotation );
            }
            return;
        }

        // Get hmd pose matrix.
        vr::TrackedDevicePose_t
            devicePosesForRot[vr::k_unMaxTrackedDeviceCount];

        // source must be current universe
        if ( m_trackingUniverse == vr::TrackingUniverseSeated )
        {
            vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(
                vr::TrackingUniverseSeated,
                0.0f,
                devicePosesForRot,
                vr::k_unMaxTrackedDeviceCount );
        }
        else
        {
            vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(
                vr::TrackingUniverseStanding,
                0.0f,
                devicePosesForRot,
                vr::k_unMaxTrackedDeviceCount );
        }

        vr::HmdMatrix34_t oldHmdPos
            = devicePosesForRot[0].mDeviceToAbsoluteTracking;
        vr::HmdVector3_t oldHmdPos3
            = { oldHmdPos.m[0][3], oldHmdPos.m[1][3], oldHmdPos.m[2][3] };
        setRotationAroundPivot( value, notify, oldHmdPos3 );
    }
}

void MoveCenterTabController::setRotationAroundPivot(
    int value,
    bool notify,
    const vr::HmdVector3_t& pivot )
{
    if ( m_rotation != value )
    {
        double angle = ( value - m_rotation ) * k_centidegreesToRadians;
        // Set up xyz coordinate values from pose matrix.
        double oldXyz[3] = { static_cast<double>( pivot.v[0] ),
                             static_cast<double>( pivot.v[1] ),
                             static_cast<double>( pivot.v[2] ) };
        double newXyz[3] = { static_cast<double>( pivot.v[0] ),
                             static_cast<double>( pivot.v[1] ),
                             static_cast<double>( pivot.v[2] ) };

        // Convert oldXyz into un-rotated coordinates.
        double oldAngle = -m_rotation * k_centidegreesToRadians;
        rotateCoordinates( oldXyz, oldAngle );

        // Set newXyz to have additional rotation from incoming angle change.
        rotateCoordinates( newXyz, oldAngle - angle );

        // find difference in x,z offset due to incoming angle change
        // (coordinates are in un-rotated axis).
        double hmdRotDiff[3]
            = { oldXyz[0] - newXyz[0], 0, oldXyz[2] - newXyz[2] };

        m_rotation = value;
        if ( notify )
        {
            emit rotationChanged( m_rotation );
        }

        // Update UI offsets.
        m_offsetX += static_cast<float>( hmdRotDiff[0] );
        m_offsetZ += static_cast<float>( hmdRotDiff[2] );
        if ( notify )
        {
            emit offsetXChanged( m_offsetX );
            emit offsetZChanged( m_offsetZ );
        }
    }
}

void MoveCenterTabController::setTempRotation( int value, bool notify )
{
    m_tempRotation = value;
    if ( notify )
    {
        emit tempRotationChanged( m_tempRotation );
    }
}

int MoveCenterTabController::snapTurnAngle() const
{
    return settings::getSetting(
        settings::IntSetting::PLAYSPACE_snapTurnAngle );
}

void MoveCenterTabController::setSnapTurnAngle( int value, bool notify )
{
    settings::setSetting( settings::IntSetting::PLAYSPACE_snapTurnAngle,
                          value );

    if ( notify )
    {
        emit snapTurnAngleChanged( value );
    }
}

int MoveCenterTabController::smoothTurnRate() const
{
    return settings::getSetting(
        settings::IntSetting::PLAYSPACE_smoothTurnRate );
}

int MoveCenterTabController::frictionPercent() const
{
    return settings::getSetting(
        settings::IntSetting::PLAYSPACE_frictionPercent );
}

void MoveCenterTabController::setSmoothTurnRate( int value, bool notify )
{
    settings::setSetting( settings::IntSetting::PLAYSPACE_smoothTurnRate,
                          value );

    if ( notify )
    {
        emit smoothTurnRateChanged( value );
    }
}

void MoveCenterTabController::setFrictionPercent( int value, bool notify )
{
    settings::setSetting( settings::IntSetting::PLAYSPACE_frictionPercent,
                          value );

    if ( notify )
    {
        emit frictionPercentChanged( value );
    }
}

bool MoveCenterTabController::adjustChaperone() const
{
    return settings::getSetting(
        settings::BoolSetting::PLAYSPACE_adjustChaperone2 );
}

void MoveCenterTabController::setAdjustChaperone( bool value, bool notify )
{
    settings::setSetting( settings::BoolSetting::PLAYSPACE_adjustChaperone2,
                          value );

    if ( notify )
    {
        emit adjustChaperoneChanged( value );
    }
}

bool MoveCenterTabController::moveShortcutRight() const
{
    return settings::getSetting(
        settings::BoolSetting::PLAYSPACE_moveShortcutRight );
}

void MoveCenterTabController::setMoveShortcutRight( bool value, bool notify )
{
    settings::setSetting( settings::BoolSetting::PLAYSPACE_moveShortcutRight,
                          value );
    if ( notify )
    {
        emit moveShortcutRightChanged( value );
    }
}

bool MoveCenterTabController::moveShortcutLeft() const
{
    return settings::getSetting(
        settings::BoolSetting::PLAYSPACE_moveShortcutLeft );
}

void MoveCenterTabController::setMoveShortcutLeft( bool value, bool notify )
{
    settings::setSetting( settings::BoolSetting::PLAYSPACE_moveShortcutLeft,
                          value );

    if ( notify )
    {
        emit moveShortcutLeftChanged( value );
    }
}

bool MoveCenterTabController::turnBindLeft() const
{
    return settings::getSetting(
        settings::BoolSetting::PLAYSPACE_turnBindLeft );
}

void MoveCenterTabController::setTurnBindLeft( bool value, bool notify )
{
    settings::setSetting( settings::BoolSetting::PLAYSPACE_turnBindLeft,
                          value );

    if ( notify )
    {
        emit turnBindLeftChanged( value );
    }
}

bool MoveCenterTabController::turnBindRight() const
{
    return settings::getSetting(
        settings::BoolSetting::PLAYSPACE_turnBindRight );
}

void MoveCenterTabController::setTurnBindRight( bool value, bool notify )
{
    settings::setSetting( settings::BoolSetting::PLAYSPACE_turnBindRight,
                          value );
    if ( notify )
    {
        emit turnBindRightChanged( value );
    }
}

bool MoveCenterTabController::dragBounds() const
{
    return settings::getSetting( settings::BoolSetting::PLAYSPACE_dragBounds );
}

void MoveCenterTabController::setDragBounds( bool value, bool notify )
{
    // detect deactivate
    if ( dragBounds() && !value )
    {
        // set force bounds back to default on deactivate
        vr::VRChaperone()->ForceBoundsVisible(
            parent->m_chaperoneTabController.forceBounds() );
    }

    settings::setSetting( settings::BoolSetting::PLAYSPACE_dragBounds, value );

    if ( notify )
    {
        emit dragBoundsChanged( value );
    }
}

bool MoveCenterTabController::turnBounds() const
{
    return settings::getSetting( settings::BoolSetting::PLAYSPACE_turnBounds );
}

void MoveCenterTabController::setTurnBounds( bool value, bool notify )
{
    // detect deactivate
    if ( turnBounds() && !value )
    {
        // set force bounds back to default on deactivate
        vr::VRChaperone()->ForceBoundsVisible(
            parent->m_chaperoneTabController.forceBounds() );
    }

    settings::setSetting( settings::BoolSetting::PLAYSPACE_turnBounds, value );

    if ( notify )
    {
        emit turnBoundsChanged( value );
    }
}

int MoveCenterTabController::dragComfortFactor() const
{
    return settings::getSetting(
        settings::IntSetting::PLAYSPACE_dragComfortFactor );
}

void MoveCenterTabController::setDragComfortFactor( int value, bool notify )
{
    settings::setSetting( settings::IntSetting::PLAYSPACE_dragComfortFactor,
                          value );

    if ( notify )
    {
        emit dragComfortFactorChanged( value );
    }
}

int MoveCenterTabController::turnComfortFactor() const
{
    return settings::getSetting(
        settings::IntSetting::PLAYSPACE_turnComfortFactor );
}

void MoveCenterTabController::setTurnComfortFactor( int value, bool notify )
{
    settings::setSetting( settings::IntSetting::PLAYSPACE_turnComfortFactor,
                          value );

    if ( notify )
    {
        emit turnComfortFactorChanged( value );
    }
}

bool MoveCenterTabController::heightToggle() const
{
    return m_heightToggle;
}

void MoveCenterTabController::setHeightToggle( bool value, bool notify )
{
    // detect new activate
    if ( !m_heightToggle && value )
    {
        // Don't move bump around on y axis if gravity is active
        if ( !m_gravityActive )
        {
            m_offsetY += heightToggleOffset();
            emit offsetYChanged( m_offsetY );
        }
        m_gravityFloor = heightToggleOffset();
    }
    // detect new deactivate
    else if ( m_heightToggle && !value )
    {
        // Don't move bump around on y axis if gravity is active
        if ( !m_gravityActive )
        {
            m_offsetY -= heightToggleOffset();
            emit offsetYChanged( m_offsetY );
        }
        m_gravityFloor = 0.0f;
    }

    m_heightToggle = value;
    if ( notify )
    {
        emit heightToggleChanged( m_heightToggle );
    }
}

float MoveCenterTabController::heightToggleOffset() const
{
    return static_cast<float>( settings::getSetting(
        settings::DoubleSetting::PLAYSPACE_heightToggleOffset ) );
}

void MoveCenterTabController::setHeightToggleOffset( float value, bool notify )
{
    settings::setSetting( settings::DoubleSetting::PLAYSPACE_heightToggleOffset,
                          static_cast<double>( value ) );

    if ( notify )
    {
        emit heightToggleOffsetChanged( value );
    }
}

float MoveCenterTabController::gravityStrength() const
{
    return static_cast<float>( settings::getSetting(
        settings::DoubleSetting::PLAYSPACE_gravityStrength ) );
}

void MoveCenterTabController::setGravityStrength( float value, bool notify )
{
    settings::setSetting( settings::DoubleSetting::PLAYSPACE_gravityStrength,
                          static_cast<double>( value ) );
    if ( notify )
    {
        emit gravityStrengthChanged( value );
    }
    m_lastGravityUpdateTimePoint = std::chrono::steady_clock::now();
}

float MoveCenterTabController::flingStrength() const
{
    return static_cast<float>( settings::getSetting(
        settings::DoubleSetting::PLAYSPACE_flingStrength ) );
}

void MoveCenterTabController::setFlingStrength( float value, bool notify )
{
    settings::setSetting( settings::DoubleSetting::PLAYSPACE_flingStrength,
                          static_cast<double>( value ) );

    if ( notify )
    {
        emit flingStrengthChanged( value );
    }
}

bool MoveCenterTabController::gravityActive() const
{
    return m_gravityActive;
}

void MoveCenterTabController::setGravityActive( bool value, bool notify )
{
    // skip if there's no change
    if ( m_gravityActive == value )
    {
        return;
    }

    // detect new activate
    if ( !m_gravityActive && value )
    {
        // zero out velocity if we aren't saving previous momentum
        if ( !momentumSave() )
        {
            m_velocity[0] = 0.0;
            m_velocity[1] = 0.0;
            m_velocity[2] = 0.0;
        }
        // make sure our time slice calculation doesn't use a slice from the
        // previous activation of gravity
        m_lastGravityUpdateTimePoint = std::chrono::steady_clock::now();
    }
    m_gravityActive = value;
    if ( notify )
    {
        emit gravityActiveChanged( m_gravityActive );
    }
}

bool MoveCenterTabController::momentumSave() const
{
    return settings::getSetting(
        settings::BoolSetting::PLAYSPACE_momentumSave );
}

void MoveCenterTabController::setMomentumSave( bool value, bool notify )
{
    settings::setSetting( settings::BoolSetting::PLAYSPACE_momentumSave,
                          value );

    if ( notify )
    {
        emit momentumSaveChanged( value );
    }
}

bool MoveCenterTabController::lockXToggle() const
{
    return settings::getSetting( settings::BoolSetting::PLAYSPACE_lockXToggle );
}

void MoveCenterTabController::setLockX( bool value, bool notify )
{
    settings::setSetting( settings::BoolSetting::PLAYSPACE_lockXToggle, value );

    if ( notify )
    {
        emit requireLockXChanged( value );
    }
}

bool MoveCenterTabController::lockYToggle() const
{
    return settings::getSetting( settings::BoolSetting::PLAYSPACE_lockYToggle );
}

void MoveCenterTabController::setLockY( bool value, bool notify )
{
    settings::setSetting( settings::BoolSetting::PLAYSPACE_lockYToggle, value );
    if ( notify )
    {
        emit requireLockYChanged( value );
    }
}

bool MoveCenterTabController::lockZToggle() const
{
    return settings::getSetting( settings::BoolSetting::PLAYSPACE_lockZToggle );
}

void MoveCenterTabController::setLockZ( bool value, bool notify )
{
    settings::setSetting( settings::BoolSetting::PLAYSPACE_lockZToggle, value );
    if ( notify )
    {
        emit requireLockZChanged( value );
    }
}

bool MoveCenterTabController::showLogMatricesButton() const
{
    return settings::getSetting(
        settings::BoolSetting::PLAYSPACE_showLogMatricesButton );
}

void MoveCenterTabController::setShowLogMatricesButton( bool value,
                                                        bool notify )
{
    settings::setSetting(
        settings::BoolSetting::PLAYSPACE_showLogMatricesButton, value );
    if ( notify )
    {
        emit showLogMatricesButtonChanged( value );
    }
}

bool MoveCenterTabController::allowExternalEdits() const
{
    return settings::getSetting(
        settings::BoolSetting::PLAYSPACE_allowExternalEdits );
}

void MoveCenterTabController::setAllowExternalEdits( bool value, bool notify )
{
    settings::setSetting( settings::BoolSetting::PLAYSPACE_allowExternalEdits,
                          value );

    if ( notify )
    {
        emit allowExternalEditsChanged( value );
    }
}

bool MoveCenterTabController::oldStyleMotion() const
{
    return settings::getSetting(
        settings::BoolSetting::PLAYSPACE_oldStyleMotion );
}

void MoveCenterTabController::setOldStyleMotion( bool value, bool notify )
{
    // detect incoming change to old style, and hide working set
    if ( value && !oldStyleMotion() )
    {
        vr::VRChaperoneSetup()->HideWorkingSetPreview();
    }

    settings::setSetting( settings::BoolSetting::PLAYSPACE_oldStyleMotion,
                          value );

    if ( notify )
    {
        emit oldStyleMotionChanged( value );
    }
}

bool MoveCenterTabController::universeCenteredRotation() const
{
    return settings::getSetting(
        settings::BoolSetting::PLAYSPACE_universeCenteredRotation );
}

void MoveCenterTabController::setUniverseCenteredRotation( bool value,
                                                           bool notify )
{
    settings::setSetting(
        settings::BoolSetting::PLAYSPACE_universeCenteredRotation, value );

    if ( notify )
    {
        emit universeCenteredRotationChanged( value );
    }
}

bool MoveCenterTabController::isInitComplete() const
{
    return m_initComplete;
}

bool MoveCenterTabController::enableSeatedMotion() const
{
    return settings::getSetting(
        settings::BoolSetting::PLAYSPACE_enableSeatedMotion );
}

void MoveCenterTabController::setEnableSeatedMotion( bool value, bool notify )
{
    settings::setSetting( settings::BoolSetting::PLAYSPACE_enableSeatedMotion,
                          value );

    if ( notify )
    {
        emit enableSeatedMotionChanged( value );
    }
}

bool MoveCenterTabController::simpleRecenter() const
{
    return settings::getSetting(
        settings::BoolSetting::PLAYSPACE_simpleRecenter );
}

void MoveCenterTabController::setSimpleRecenter( bool value, bool notify )
{
    settings::setSetting( settings::BoolSetting::PLAYSPACE_simpleRecenter,
                          value );

    if ( notify )
    {
        emit simpleRecenterChanged( value );
    }
}

void MoveCenterTabController::modOffsetX( float value, bool notify )
{
    if ( !lockXToggle() )
    {
        m_offsetX += value;
        if ( notify )
        {
            emit offsetXChanged( m_offsetX );
        }
    }
}

void MoveCenterTabController::modOffsetY( float value, bool notify )
{
    if ( !lockYToggle() )
    {
        m_offsetY += value;
        if ( notify )
        {
            emit offsetYChanged( m_offsetY );
        }
    }
}

void MoveCenterTabController::modOffsetZ( float value, bool notify )
{
    if ( !lockZToggle() )
    {
        m_offsetZ += value;
        if ( notify )
        {
            emit offsetZChanged( m_offsetZ );
        }
    }
}

void MoveCenterTabController::shutdown()
{
    reset();
    vr::VRChaperoneSetup()->HideWorkingSetPreview();
}

void MoveCenterTabController::incomingSeatedReset()
{
    if ( enableSeatedMotion() )
    {
        // if we didn't send the request from OVRAS, we need to send another
        // ResetSeatedZeroPose(). It seems that only after this is sent from
        // OVRAS does ReloadFromDisk return valid info on WMR.
        if ( !m_selfRequestedSeatedRecenter && !simpleRecenter() )
        {
            m_selfRequestedSeatedRecenter = true;
            vr::VRSystem()->ResetSeatedZeroPose();
        }
        else
        {
            updateSeatedResetData();
        }
    }
}

void MoveCenterTabController::reset()
{
    if ( !m_chaperoneBasisAcquired )
    {
        LOG( WARNING ) << "WARNING: Attempted reset offsets before chaperone "
                          "basis is acquired!";
        return;
    }
    if ( m_pendingSeatedRecenter )
    {
        vr::VRChaperoneSetup()->GetWorkingSeatedZeroPoseToRawTrackingPose(
            &m_seatedCenterForReset );
        m_pendingSeatedRecenter = false;
    }
    vr::VRChaperoneSetup()->HideWorkingSetPreview();
    m_heightToggle = false;
    emit heightToggleChanged( m_heightToggle );
    m_oldOffsetX = 0.0f;
    m_oldOffsetY = 0.0f;
    m_oldOffsetZ = 0.0f;
    m_oldRotation = 0;
    m_offsetX = 0.0f;
    m_offsetY = 0.0f;
    m_offsetZ = 0.0f;
    m_rotation = 0;
    m_lastMoveHand = vr::TrackedControllerRole_Invalid;
    m_lastRotateHand = vr::TrackedControllerRole_Invalid;
    applyChaperoneResetData();
    m_lastControllerPosition = { 0.0f, 0.0f, 0.0f };

    // For Center Marker
    // Needs to happen after apply chaperone
    if ( parent->m_chaperoneTabController.m_centerMarkerOverlayNeedsUpdate )
    {
        m_offsetmatrix = utils::k_forwardUpMatrix;
        if ( m_trackingUniverse == vr::TrackingUniverseSeated )
        {
            vr::HmdMatrix34_t temp;
            // This is not the floor it should be ~ chair height... this seems
            // to be intended behaviour from openvr
            vr::VRChaperoneSetup()->GetWorkingSeatedZeroPoseToRawTrackingPose(
                &temp );
            m_offsetmatrix.m[1][3] = temp.m[1][3];
            // TODO check orientation
        }
        else
        {
            m_offsetmatrix.m[1][3] = 0;
        }
        parent->m_chaperoneTabController.updateCenterMarkerOverlay(
            &m_offsetmatrix );
    }

    emit offsetXChanged( m_offsetX );
    emit offsetYChanged( m_offsetY );
    emit offsetZChanged( m_offsetZ );
    emit rotationChanged( m_rotation );
}

void MoveCenterTabController::zeroOffsets()
{
    // first we'll check if we're outside the max commit distance

    vr::HmdMatrix34_t currentCenter;
    vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(
        &currentCenter );
    double currentCenterYaw = static_cast<double>(
        std::atan2( currentCenter.m[0][2], currentCenter.m[2][2] ) );
    double currentCenterXyz[3]
        = { static_cast<double>( currentCenter.m[0][3] ),
            static_cast<double>( currentCenter.m[1][3] ),
            static_cast<double>( currentCenter.m[2][3] ) };
    // unrotate to get raw values of xyz
    rotateCoordinates( currentCenterXyz,
                       currentCenterYaw
                           - ( m_rotation * k_centidegreesToRadians ) );
    if ( abs( currentCenterXyz[0] ) > k_maxOpenvrCommitOffset )
    {
        LOG( INFO ) << "Attempted Zero Offsets out of commit bounds ( X: "
                    << currentCenterXyz[0] << " )";
        LOG( INFO ) << "GetWorkingStandingZeroPoseToRawTrackingPose";
        outputLogHmdMatrix( currentCenter );
        // if reset happens before init is complete we set the universe center
        // to the raw tracking zero point
        reset();
        if ( !m_chaperoneBasisAcquired )
        {
            LOG( WARNING )
                << "<[!]><[!]>EXECUTED RESET BEFORE BASIS ACQUIRED<[!]><[!]> "
                   "Setting universe center to raw tracking zero point.";
        }
        else
        {
            LOG( INFO ) << "-Resetting offsets-";
        }
    }
    else if ( abs( currentCenterXyz[1] ) > k_maxOpenvrCommitOffset )
    {
        LOG( INFO ) << "Attempted Zero Offsets out of commit bounds ( Y: "
                    << currentCenterXyz[1] << " )";
        LOG( INFO ) << "GetWorkingStandingZeroPoseToRawTrackingPose";
        outputLogHmdMatrix( currentCenter );
        // if reset happens before init is complete we set the universe center
        // to the raw tracking zero point
        reset();
        if ( !m_chaperoneBasisAcquired )
        {
            LOG( WARNING )
                << "<[!]><[!]>EXECUTED RESET BEFORE BASIS ACQUIRED<[!]><[!]> "
                   "Setting universe center to raw tracking zero point.";
        }
        else
        {
            LOG( INFO ) << "-Resetting offsets-";
        }
    }
    else if ( abs( currentCenterXyz[2] ) > k_maxOpenvrCommitOffset )
    {
        LOG( INFO ) << "Attempted Zero Offsets out of commit bounds ( Z: "
                    << currentCenterXyz[2] << " )";
        LOG( INFO ) << "GetWorkingStandingZeroPoseToRawTrackingPose";
        outputLogHmdMatrix( currentCenter );
        // if reset happens before init is complete we set the universe
        // center to the raw tracking zero point
        reset();
        if ( !m_chaperoneBasisAcquired )
        {
            LOG( WARNING )
                << "<[!]><[!]>EXECUTED RESET BEFORE BASIS ACQUIRED<[!]><[!]> "
                   "Setting universe center to raw tracking zero point.";
        }
        else
        {
            LOG( INFO ) << "-Resetting offsets-";
        }
    }

    // finished checking if out of bounds, proceed with normal zeroing offsets
    if ( vr::VRChaperone()->GetCalibrationState()
         == vr::ChaperoneCalibrationState_OK )
    {
        m_oldOffsetX = 0.0f;
        m_oldOffsetY = 0.0f;
        m_oldOffsetZ = 0.0f;
        m_oldRotation = 0;
        m_offsetX = 0.0f;
        m_offsetY = 0.0f;
        m_offsetZ = 0.0f;
        m_rotation = 0;
        emit offsetXChanged( m_offsetX );
        emit offsetYChanged( m_offsetY );
        emit offsetZChanged( m_offsetZ );
        emit rotationChanged( m_rotation );
        updateChaperoneResetData();
        m_pendingZeroOffsets = false;
        if ( !m_chaperoneBasisAcquired )
        {
            m_chaperoneBasisAcquired = true;
            if ( !m_initComplete )
            {
                setTrackingUniverse( vr::VRCompositor()->GetTrackingSpace() );
                if ( parent->isPreviousShutdownSafe() )
                {
                    // all init complete, safe to autosave chaperone profile
                    parent->m_chaperoneTabController.createNewAutosaveProfile();
                    m_initComplete = true;
                }
                else
                {
                    // shutdown was unsafe last session!
                    LOG( WARNING )
                        << "DETECTED UNSAFE SHUTDOWN FROM LAST SESSION";
                    m_initComplete = true;
                    if ( !parent->crashRecoveryDisabled() )
                    {
                        parent->m_chaperoneTabController
                            .applyAutosavedProfile();
                        LOG( INFO )
                            << "Applying last good chaperone profile autosave";
                    }
                }
                // Now mark previous shutdown as unsafe in case we crash some
                // time during this session. Previous shutdown will be marked as
                // being safe once more just before our app shuts down properly.
                parent->setPreviousShutdownSafe( false );
            }
        }
        if ( m_roomSetupModeDetected )
        {
            LOG( INFO ) << "room setup EXIT detected";
            m_roomSetupModeDetected = false;
        }

        LOG( INFO ) << "SUCCESS: Chaperone Data Updated and Offsets zeroed out";
    }
    else
    {
        if ( !m_pendingZeroOffsets )
        {
            LOG( INFO ) << "PENDING: Chaperone Data Update and Offsets zeroing";
        }
        m_pendingZeroOffsets = true;
    }
}

void MoveCenterTabController::sendSeatedRecenter()
{
    vr::VRSystem()->ResetSeatedZeroPose();
}

double MoveCenterTabController::getHmdYawTotal()
{
    return m_hmdYawTotal;
}

void MoveCenterTabController::resetHmdYawTotal()
{
    m_hmdYawTotal = 0.0;
    m_hmdYawOld = 0.0;
    m_hmdYawTurnCount = 0;
}

void MoveCenterTabController::clampVelocity( double* velocity )
{
    for ( int i = 0; i < 3; i++ )
    {
        // too fast! clamp to terminal velocity while preserving +/- sign
        if ( std::abs( velocity[i] ) >= k_terminalVelocity_mps )
        {
            velocity[i] = copysign( k_terminalVelocity_mps, velocity[i] );
        }
    }
}

void MoveCenterTabController::updateSeatedResetData()
{
    m_heightToggle = false;
    emit heightToggleChanged( m_heightToggle );
    m_oldOffsetX = 0.0f;
    m_oldOffsetY = 0.0f;
    m_oldOffsetZ = 0.0f;
    m_oldRotation = 0;
    m_offsetX = 0.0f;
    m_offsetY = 0.0f;
    m_offsetZ = 0.0f;
    m_rotation = 0;
    emit offsetXChanged( m_offsetX );
    emit offsetYChanged( m_offsetY );
    emit offsetZChanged( m_offsetZ );
    emit rotationChanged( m_rotation );
    vr::VRChaperoneSetup()->ReloadFromDisk( vr::EChaperoneConfigFile_Live );
    // done with this recenter, so set self request back to false for next time.
    m_selfRequestedSeatedRecenter = false;
    // set pending update here, will be processed on next instance of motion or
    // running the reset() function.
    m_pendingSeatedRecenter = true;
}

void MoveCenterTabController::updateChaperoneResetData()
{
    vr::VRChaperoneSetup()->RevertWorkingCopy();
    unsigned currentQuadCount = 0;
    vr::VRChaperoneSetup()->GetWorkingCollisionBoundsInfo( nullptr,
                                                           &currentQuadCount );
    m_collisionBoundsForReset = new vr::HmdQuad_t[currentQuadCount];
    m_collisionBoundsForOffset = new vr::HmdQuad_t[currentQuadCount];
    m_collisionBoundsCountForReset = currentQuadCount;
    vr::VRChaperoneSetup()->GetWorkingCollisionBoundsInfo(
        m_collisionBoundsForReset, &currentQuadCount );

    vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(
        &m_universeCenterForReset );
    vr::VRChaperoneSetup()->GetWorkingSeatedZeroPoseToRawTrackingPose(
        &m_seatedCenterForReset );

    updateCollisionBoundsForOffset();
    parent->m_chaperoneTabController.updateHeight( getBoundsBasisMaxY() );

    unsigned checkQuadCount = 0;
    vr::VRChaperoneSetup()->GetWorkingCollisionBoundsInfo( nullptr,
                                                           &checkQuadCount );
    if ( checkQuadCount > 0 )
    {
        parent->chaperoneUtils().loadChaperoneData( false );
    }
}

void MoveCenterTabController::updateCollisionBoundsForOffset()
{
    if ( m_collisionBoundsCountForReset > 0 )
    {
        float universeCenterForResetYaw
            = std::atan2( m_universeCenterForReset.m[0][2],
                          m_universeCenterForReset.m[2][2] );

        // we want to store m_collisionBoundsForOffset as spacially relative to
        // m_universeCenterForReset, so:

        // for every quad in the chaperone bounds...
        for ( unsigned quad = 0; quad < m_collisionBoundsCountForReset; quad++ )
        {
            // at every corner in that quad...
            for ( unsigned corner = 0; corner < 4; corner++ )
            {
                // copy from m_collisionBoundsForReset
                m_collisionBoundsForOffset[quad].vCorners[corner].v[0]
                    = m_collisionBoundsForReset[quad].vCorners[corner].v[0];
                m_collisionBoundsForOffset[quad].vCorners[corner].v[1]
                    = m_collisionBoundsForReset[quad].vCorners[corner].v[1];
                m_collisionBoundsForOffset[quad].vCorners[corner].v[2]
                    = m_collisionBoundsForReset[quad].vCorners[corner].v[2];

                // unrotate by universe center's yaw
                rotateFloatCoordinates(
                    m_collisionBoundsForOffset[quad].vCorners[corner].v,
                    -universeCenterForResetYaw );
            }
        }
    }
}

void MoveCenterTabController::applyChaperoneResetData()
{
    vr::VRChaperoneSetup()->HideWorkingSetPreview();
    vr::VRChaperoneSetup()->RevertWorkingCopy();
    if ( m_collisionBoundsCountForReset > 0 )
    {
        vr::VRChaperoneSetup()->SetWorkingCollisionBoundsInfo(
            m_collisionBoundsForReset, m_collisionBoundsCountForReset );
    }
    vr::VRChaperoneSetup()->SetWorkingStandingZeroPoseToRawTrackingPose(
        &m_universeCenterForReset );
    vr::VRChaperoneSetup()->SetWorkingSeatedZeroPoseToRawTrackingPose(
        &m_seatedCenterForReset );

    vr::VRChaperoneSetup()->CommitWorkingCopy( vr::EChaperoneConfigFile_Live );

    unsigned checkQuadCount = 0;
    vr::VRChaperoneSetup()->GetWorkingCollisionBoundsInfo( nullptr,
                                                           &checkQuadCount );
    if ( checkQuadCount > 0 )
    {
        parent->chaperoneUtils().loadChaperoneData( false );
    }
}

void MoveCenterTabController::setBoundsBasisHeight( float newHeight )
{
    if ( m_collisionBoundsCountForReset > 0 )
    {
        for ( unsigned b = 0; b < m_collisionBoundsCountForReset; b++ )
        {
            m_collisionBoundsForReset[b].vCorners[0].v[1] = 0.0;
            m_collisionBoundsForReset[b].vCorners[1].v[1] = newHeight;
            m_collisionBoundsForReset[b].vCorners[2].v[1] = newHeight;
            m_collisionBoundsForReset[b].vCorners[3].v[1] = 0.0;
        }

        updateCollisionBoundsForOffset();
        updateSpace( true );
    }
}

float MoveCenterTabController::getBoundsBasisMaxY()
{
    float result = FP_NAN;
    if ( m_collisionBoundsCountForReset > 0 )
    {
        for ( unsigned b = 0; b < m_collisionBoundsCountForReset; b++ )
        {
            int ci;
            if ( m_collisionBoundsForReset[b].vCorners[1].v[1]
                 >= m_collisionBoundsForReset[b].vCorners[2].v[1] )
            {
                ci = 1;
            }
            else
            {
                ci = 2;
            }
            if ( std::isnan( result )
                 || result < m_collisionBoundsForReset[b].vCorners[ci].v[1] )
            {
                result = m_collisionBoundsForReset[b].vCorners[ci].v[1];
            }
        }
    }
    return result;
}

// START of drag bindings:

void MoveCenterTabController::leftHandSpaceDrag( bool leftHandDragActive )
{
    if ( !moveShortcutLeft() )
    {
        return;
    }
    // cancel if other hand is active in override
    if ( !m_overrideRightHandDragPressed )
    {
        // detect new press
        if ( leftHandDragActive && !m_leftHandDragPressed )
        {
            m_activeDragHand = vr::TrackedControllerRole_LeftHand;
        }
        // detect new release
        else if ( !leftHandDragActive && m_leftHandDragPressed )
        {
            // check overrides
            // pass back to right hand?
            if ( m_overrideRightHandDragPressed )
            {
                m_activeDragHand = vr::TrackedControllerRole_RightHand;
            }
            // otherwise check right hand for activated swap drag
            if ( m_swapDragToRightHandActivated )
            {
                m_activeDragHand = vr::TrackedControllerRole_RightHand;
            }
            // override of left hand still active?
            else if ( m_overrideLeftHandDragPressed )
            {
                m_activeDragHand = vr::TrackedControllerRole_LeftHand;
            }
            // swap activation of left hand still active?
            else if ( m_swapDragToLeftHandActivated )
            {
                m_activeDragHand = vr::TrackedControllerRole_LeftHand;
            }
            // check non-override right hand action
            if ( m_rightHandDragPressed )
            {
                m_activeDragHand = vr::TrackedControllerRole_RightHand;
            }
            // nothing else is pressed, deactivate drag
            else
            {
                m_activeDragHand = vr::TrackedControllerRole_Invalid;
            }
        }
    }
    m_leftHandDragPressed = leftHandDragActive;
}

void MoveCenterTabController::rightHandSpaceDrag( bool rightHandDragActive )
{
    if ( !moveShortcutRight() )
    {
        return;
    }
    // cancel if other hand is active in override
    if ( !m_overrideLeftHandDragPressed )
    {
        // detect new press
        if ( rightHandDragActive && !m_rightHandDragPressed )
        {
            m_activeDragHand = vr::TrackedControllerRole_RightHand;
        }
        // detect new release
        else if ( !rightHandDragActive && m_rightHandDragPressed )
        {
            // check overrides
            // pass back to left hand?
            if ( m_overrideLeftHandDragPressed )
            {
                m_activeDragHand = vr::TrackedControllerRole_LeftHand;
            }
            // otherwise check left hand for activated swap drag
            if ( m_swapDragToLeftHandActivated )
            {
                m_activeDragHand = vr::TrackedControllerRole_LeftHand;
            }
            // override of right hand still active?
            else if ( m_overrideRightHandDragPressed )
            {
                m_activeDragHand = vr::TrackedControllerRole_RightHand;
            }
            // swap activation of right hand still active?
            else if ( m_swapDragToRightHandActivated )
            {
                m_activeDragHand = vr::TrackedControllerRole_RightHand;
            }
            // check non-override left hand action
            if ( m_leftHandDragPressed )
            {
                m_activeDragHand = vr::TrackedControllerRole_LeftHand;
            }
            // nothing else is pressed, deactivate drag
            else
            {
                m_activeDragHand = vr::TrackedControllerRole_Invalid;
            }
        }
    }
    m_rightHandDragPressed = rightHandDragActive;
}

void MoveCenterTabController::optionalOverrideLeftHandSpaceDrag(
    bool overrideLeftHandDragActive )
{
    if ( !moveShortcutLeft() )
    {
        return;
    }
    // detect new press
    if ( overrideLeftHandDragActive && !m_overrideLeftHandDragPressed )
    {
        m_activeDragHand = vr::TrackedControllerRole_LeftHand;
        // stop any active rotate because we're in override mode
        m_activeTurnHand = vr::TrackedControllerRole_Invalid;
    }
    // detect new release
    else if ( !overrideLeftHandDragActive && m_overrideLeftHandDragPressed )
    {
        // check if we should pass back to other hand
        // give priority to override action
        if ( m_overrideRightHandDragPressed )
        {
            m_activeDragHand = vr::TrackedControllerRole_RightHand;
        }
        // otherwise check right hand for activated swap drag
        if ( m_swapDragToRightHandActivated )
        {
            m_activeDragHand = vr::TrackedControllerRole_RightHand;
        }
        // otherwise check normal action
        else if ( m_rightHandDragPressed )
        {
            m_activeDragHand = vr::TrackedControllerRole_RightHand;
        }
        // swap activation of left hand still active?
        else if ( m_swapDragToLeftHandActivated )
        {
            m_activeDragHand = vr::TrackedControllerRole_LeftHand;
        }
        // check if we should fall back to non-override left hand
        else if ( m_leftHandDragPressed )
        {
            m_activeDragHand = vr::TrackedControllerRole_LeftHand;
        }
        // nothing else is pressed, deactivate drag
        else
        {
            m_activeDragHand = vr::TrackedControllerRole_Invalid;
        }
    }
    m_overrideLeftHandDragPressed = overrideLeftHandDragActive;
}

void MoveCenterTabController::optionalOverrideRightHandSpaceDrag(
    bool overrideRightHandDragActive )
{
    if ( !moveShortcutRight() )
    {
        return;
    }
    // detect new press
    if ( overrideRightHandDragActive && !m_overrideRightHandDragPressed )
    {
        m_activeDragHand = vr::TrackedControllerRole_RightHand;
        // stop any active rotate because we're in override mode
        m_activeTurnHand = vr::TrackedControllerRole_Invalid;
    }
    // detect new release
    else if ( !overrideRightHandDragActive && m_overrideRightHandDragPressed )
    {
        // check if we should pass back to other hand
        // give priority to override action
        if ( m_overrideLeftHandDragPressed )
        {
            m_activeDragHand = vr::TrackedControllerRole_LeftHand;
        }
        // otherwise check left hand for activated swap drag
        if ( m_swapDragToLeftHandActivated )
        {
            m_activeDragHand = vr::TrackedControllerRole_LeftHand;
        }
        // otherwise check normal action
        else if ( m_leftHandDragPressed )
        {
            m_activeDragHand = vr::TrackedControllerRole_LeftHand;
        }
        // swap activation of right hand still active?
        else if ( m_swapDragToRightHandActivated )
        {
            m_activeDragHand = vr::TrackedControllerRole_RightHand;
        }
        // check if we should fall back to non-override right hand
        else if ( m_rightHandDragPressed )
        {
            m_activeDragHand = vr::TrackedControllerRole_RightHand;
        }
        // nothing else is pressed, deactivate drag
        else
        {
            m_activeDragHand = vr::TrackedControllerRole_Invalid;
        }
    }
    m_overrideRightHandDragPressed = overrideRightHandDragActive;
}

void MoveCenterTabController::swapSpaceDragToLeftHandOverride(
    bool swapDragToLeftHand )
{
    if ( !moveShortcutLeft() )
    {
        return;
    }
    // detect new press
    if ( swapDragToLeftHand && !m_swapDragToLeftHandPressed )
    {
        // check for active right hand drag to swap from
        if ( m_activeDragHand == vr::TrackedControllerRole_RightHand )
        {
            m_activeDragHand = vr::TrackedControllerRole_LeftHand;
            // stop any active rotate because we're in override mode
            m_activeTurnHand = vr::TrackedControllerRole_Invalid;
            // set this hand's swap action to successfully activated for
            // tracking if we should pass the drag back if another hand is
            // released
            m_swapDragToLeftHandActivated = true;
        }
    }
    // detect new release
    else if ( !swapDragToLeftHand && m_swapDragToLeftHandPressed )
    {
        // check if the left hand swap had even activated
        if ( m_swapDragToLeftHandActivated
             && m_activeDragHand == vr::TrackedControllerRole_LeftHand )
        {
            // check if we should pass back to other hand
            // give priority to override action
            if ( m_overrideRightHandDragPressed )
            {
                m_activeDragHand = vr::TrackedControllerRole_RightHand;
            }
            // otherwise check if other hand's swap drag was active
            else if ( m_swapDragToRightHandActivated )
            {
                m_activeDragHand = vr::TrackedControllerRole_RightHand;
            }
            // otherwise check if we retain left hand via its override
            // action
            else if ( m_overrideLeftHandDragPressed )
            {
                m_activeDragHand = vr::TrackedControllerRole_LeftHand;
            }
            // otherwise check normal action on right hand
            else if ( m_rightHandDragPressed )
            {
                m_activeDragHand = vr::TrackedControllerRole_RightHand;
            }
            // otherwise check if we retain left hand via its normal action
            else if ( m_leftHandDragPressed )
            {
                m_activeDragHand = vr::TrackedControllerRole_LeftHand;
            }
            // nothing else is pressed, deactivate drag.
            else
            {
                m_activeDragHand = vr::TrackedControllerRole_Invalid;
            }
        }
    }
    m_swapDragToLeftHandPressed = swapDragToLeftHand;
    // we only set activated flag when the swap was successfull, but we always
    // deactivate when swap action isn't pressed.
    if ( !swapDragToLeftHand )
    {
        m_swapDragToLeftHandActivated = false;
    }
    // deactivate turning even when not detecting a new press as long as a swap
    // is successfully activated now
    if ( m_swapDragToLeftHandActivated )
    {
        m_activeTurnHand = vr::TrackedControllerRole_Invalid;
    }
}

void MoveCenterTabController::swapSpaceDragToRightHandOverride(
    bool swapDragToRightHand )
{
    if ( !moveShortcutRight() )
    {
        return;
    }
    // detect new press
    if ( swapDragToRightHand && !m_swapDragToRightHandPressed )
    {
        // check for active left hand drag to swap from
        if ( m_activeDragHand == vr::TrackedControllerRole_LeftHand )
        {
            m_activeDragHand = vr::TrackedControllerRole_RightHand;
            // stop any active rotate because we're in override mode
            m_activeTurnHand = vr::TrackedControllerRole_Invalid;
            // set this hand's swap action to successfully activated for
            // tracking if we should pass the drag back if another hand is
            // released
            m_swapDragToRightHandActivated = true;
        }
    }
    // detect new release
    else if ( !swapDragToRightHand && m_swapDragToRightHandPressed )
    {
        // check if the right hand swap had even activated
        if ( m_swapDragToRightHandActivated
             && m_activeDragHand == vr::TrackedControllerRole_RightHand )
        {
            // check if we should pass back to other hand
            // give priority to override action
            if ( m_overrideLeftHandDragPressed )
            {
                m_activeDragHand = vr::TrackedControllerRole_LeftHand;
            }
            // otherwise check if other hand's swap drag was active
            else if ( m_swapDragToLeftHandActivated )
            {
                m_activeDragHand = vr::TrackedControllerRole_LeftHand;
            }
            // otherwise check if we retain right hand via its override
            // action
            else if ( m_overrideRightHandDragPressed )
            {
                m_activeDragHand = vr::TrackedControllerRole_RightHand;
            }
            // otherwise check normal action on left hand
            else if ( m_leftHandDragPressed )
            {
                m_activeDragHand = vr::TrackedControllerRole_LeftHand;
            }
            // otherwise check if we retain right hand via its normal action
            else if ( m_rightHandDragPressed )
            {
                m_activeDragHand = vr::TrackedControllerRole_RightHand;
            }
            // nothing else is pressed, deactivate drag.
            else
            {
                m_activeDragHand = vr::TrackedControllerRole_Invalid;
            }
        }
    }

    m_swapDragToRightHandPressed = swapDragToRightHand;

    // we only set activated flag when the swap was successfull, but we always
    // deactivate when swap action isn't pressed.
    if ( !swapDragToRightHand )
    {
        m_swapDragToRightHandActivated = false;
    }
    // deactivate turning even when not detecting a new press as long as a swap
    // is successfully activated now
    if ( m_swapDragToRightHandActivated )
    {
        m_activeTurnHand = vr::TrackedControllerRole_Invalid;
    }
}

// END of drag bindings

// START of turn bindgins

void MoveCenterTabController::leftHandSpaceTurn( bool leftHandTurnActive )
{
    if ( !turnBindLeft() )
    {
        return;
    }
    // cancel if other hand is active in override
    if ( !m_overrideRightHandTurnPressed )
    {
        // detect new press
        if ( leftHandTurnActive && !m_leftHandTurnPressed )
        {
            m_activeTurnHand = vr::TrackedControllerRole_LeftHand;
        }
        // detect new release
        else if ( !leftHandTurnActive && m_leftHandTurnPressed )
        {
            // check overrides
            // pass back to right hand?
            if ( m_overrideRightHandTurnPressed )
            {
                m_activeTurnHand = vr::TrackedControllerRole_RightHand;
            }
            // override of left hand still active?
            else if ( m_overrideLeftHandTurnPressed )
            {
                m_activeTurnHand = vr::TrackedControllerRole_LeftHand;
            }
            // check non-override right hand action
            if ( m_rightHandTurnPressed )
            {
                m_activeTurnHand = vr::TrackedControllerRole_RightHand;
            }
            // nothing else is pressed, deactivate rotate
            else
            {
                m_activeTurnHand = vr::TrackedControllerRole_Invalid;
            }
        }
    }
    m_leftHandTurnPressed = leftHandTurnActive;
}

void MoveCenterTabController::rightHandSpaceTurn( bool rightHandTurnActive )
{
    if ( !turnBindRight() )
    {
        return;
    }
    // cancel if other hand is active in override
    if ( !m_overrideLeftHandTurnPressed )
    {
        // detect new press
        if ( rightHandTurnActive && !m_rightHandTurnPressed )
        {
            m_activeTurnHand = vr::TrackedControllerRole_RightHand;
        }
        // detect new release
        else if ( !rightHandTurnActive && m_rightHandTurnPressed )
        {
            // check overrides
            // pass back to left hand?
            if ( m_overrideLeftHandTurnPressed )
            {
                m_activeTurnHand = vr::TrackedControllerRole_LeftHand;
            }
            // override of right hand still active?
            else if ( m_overrideRightHandTurnPressed )
            {
                m_activeTurnHand = vr::TrackedControllerRole_RightHand;
            }
            // check non-override left hand action
            if ( m_leftHandTurnPressed )
            {
                m_activeTurnHand = vr::TrackedControllerRole_LeftHand;
            }
            // nothing else is pressed, deactivate rotate
            else
            {
                m_activeTurnHand = vr::TrackedControllerRole_Invalid;
            }
        }
    }
    m_rightHandTurnPressed = rightHandTurnActive;
}

void MoveCenterTabController::optionalOverrideLeftHandSpaceTurn(
    bool overrideLeftHandTurnActive )
{
    if ( !turnBindLeft() )
    {
        return;
    }
    // detect new press
    if ( overrideLeftHandTurnActive && !m_overrideLeftHandTurnPressed )
    {
        m_activeTurnHand = vr::TrackedControllerRole_LeftHand;
        // stop any active move because we're in override mode
        m_activeDragHand = vr::TrackedControllerRole_Invalid;
    }
    // detect new release
    else if ( !overrideLeftHandTurnActive && m_overrideLeftHandTurnPressed )
    {
        // check if we should pass back to other hand
        // give priority to override action
        if ( m_overrideRightHandTurnPressed )
        {
            m_activeTurnHand = vr::TrackedControllerRole_RightHand;
        }
        // otherwise check normal action
        else if ( m_rightHandTurnPressed )
        {
            m_activeTurnHand = vr::TrackedControllerRole_RightHand;
        }
        // check if we should fall back to non-override left hand
        else if ( m_leftHandTurnPressed )
        {
            m_activeTurnHand = vr::TrackedControllerRole_LeftHand;
        }
        // nothing else is pressed, deactivate rotate
        else
        {
            m_activeTurnHand = vr::TrackedControllerRole_Invalid;
        }
    }
    m_overrideLeftHandTurnPressed = overrideLeftHandTurnActive;
}

void MoveCenterTabController::optionalOverrideRightHandSpaceTurn(
    bool overrideRightHandTurnActive )
{
    if ( !turnBindRight() )
    {
        return;
    }
    // detect new press
    if ( overrideRightHandTurnActive && !m_overrideRightHandTurnPressed )
    {
        m_activeTurnHand = vr::TrackedControllerRole_RightHand;
        // stop any active move because we're in override mode
        m_activeDragHand = vr::TrackedControllerRole_Invalid;
    }
    // detect new release
    else if ( !overrideRightHandTurnActive && m_overrideRightHandTurnPressed )
    {
        // check if we should pass back to other hand
        // give priority to override action
        if ( m_overrideLeftHandTurnPressed )
        {
            m_activeTurnHand = vr::TrackedControllerRole_LeftHand;
        }
        // otherwise check normal action
        else if ( m_leftHandTurnPressed )
        {
            m_activeTurnHand = vr::TrackedControllerRole_LeftHand;
        }
        // check if we should fall back to non-override right hand
        else if ( m_rightHandTurnPressed )
        {
            m_activeTurnHand = vr::TrackedControllerRole_RightHand;
        }
        // nothing else is pressed, deactivate rotate
        else
        {
            m_activeTurnHand = vr::TrackedControllerRole_Invalid;
        }
    }
    m_overrideRightHandTurnPressed = overrideRightHandTurnActive;
}

// END of turn bindings.

// START of other bindings.

void MoveCenterTabController::gravityToggleAction(
    bool gravityToggleJustPressed )
{
    if ( !gravityToggleJustPressed )
    {
        return;
    }

    if ( !m_gravityActive )
    {
        setGravityActive( true );
    }
    else
    {
        setGravityActive( false );
    }
}

void MoveCenterTabController::gravityReverseAction( bool gravityReverseHeld )
{
    m_gravityReversed = gravityReverseHeld;
}

void MoveCenterTabController::heightToggleAction( bool heightToggleJustPressed )
{
    if ( !heightToggleJustPressed )
    {
        return;
    }

    if ( !m_heightToggle )
    {
        setHeightToggle( true );
    }
    else
    {
        setHeightToggle( false );
    }
}

void MoveCenterTabController::resetOffsets( bool resetOffsetsJustPressed )
{
    if ( resetOffsetsJustPressed )
    {
        reset();
    }
}

void MoveCenterTabController::snapTurnLeft( bool snapTurnLeftJustPressed )
{
    if ( !snapTurnLeftJustPressed )
    {
        return;
    }

    int newRotationAngleDeg = m_rotation - snapTurnAngle();
    // Keep angle within -18000 ~ 18000 centidegrees
    if ( newRotationAngleDeg > 18000 )
    {
        newRotationAngleDeg -= 36000;
    }
    else if ( newRotationAngleDeg < -18000 )
    {
        newRotationAngleDeg += 36000;
    }

    setRotation( newRotationAngleDeg );
}

void MoveCenterTabController::snapTurnRight( bool snapTurnRightJustPressed )
{
    if ( !snapTurnRightJustPressed )
    {
        return;
    }

    int newRotationAngleDeg = m_rotation + snapTurnAngle();
    // Keep angle within -18000 ~ 18000 centidegrees
    if ( newRotationAngleDeg > 18000 )
    {
        newRotationAngleDeg -= 36000;
    }
    else if ( newRotationAngleDeg < -18000 )
    {
        newRotationAngleDeg += 36000;
    }

    setRotation( newRotationAngleDeg );
}

void MoveCenterTabController::smoothTurnLeft( bool smoothTurnLeftActive )
{
    if ( !smoothTurnLeftActive )
    {
        return;
    }

    // Activates every tick. smoothTurnRate() effectively becomes a percentage
    // of a degree per tick. A setting of 100 would equal 90 degrees/sec or 15
    // RPM with a framerate of 90fps
    int newRotationAngleDeg = m_rotation - smoothTurnRate();
    // Keep angle within -18000 ~ 18000 centidegrees
    if ( newRotationAngleDeg > 18000 )
    {
        newRotationAngleDeg -= 36000;
    }
    else if ( newRotationAngleDeg < -18000 )
    {
        newRotationAngleDeg += 36000;
    }

    setRotation( newRotationAngleDeg );
}

void MoveCenterTabController::smoothTurnRight( bool smoothTurnRightActive )
{
    if ( !smoothTurnRightActive )
    {
        return;
    }

    // Activates every tick. smoothTurnRate() effectively becomes a percentage
    // of a degree per tick. A setting of 100 would equal 90 degrees/sec or 15
    // RPM with a framerate of 90fps
    int newRotationAngleDeg = m_rotation + smoothTurnRate();
    // Keep angle within -18000 ~ 18000 centidegrees
    if ( newRotationAngleDeg > 18000 )
    {
        newRotationAngleDeg -= 36000;
    }
    else if ( newRotationAngleDeg < -18000 )
    {
        newRotationAngleDeg += 36000;
    }

    setRotation( newRotationAngleDeg );
}

void MoveCenterTabController::xAxisLockToggle( bool xAxisLockToggleJustPressed )
{
    if ( !xAxisLockToggleJustPressed )
    {
        return;
    }

    if ( !lockXToggle() )
    {
        setLockX( true );
    }
    else
    {
        setLockX( false );
    }
}

void MoveCenterTabController::yAxisLockToggle( bool yAxisLockToggleJustPressed )
{
    if ( !yAxisLockToggleJustPressed )
    {
        return;
    }

    if ( !lockYToggle() )
    {
        setLockY( true );
    }
    else
    {
        setLockY( false );
    }
}

void MoveCenterTabController::zAxisLockToggle( bool zAxisLockToggleJustPressed )
{
    if ( !zAxisLockToggleJustPressed )
    {
        return;
    }

    if ( !lockZToggle() )
    {
        setLockZ( true );
    }
    else
    {
        setLockZ( false );
    }
}

// END of other bindings

void MoveCenterTabController::saveUncommittedChaperone()
{
    if ( !m_chaperoneCommitted )
    {
        vr::VRChaperoneSetup()->CommitWorkingCopy(
            vr::EChaperoneConfigFile_Live );
        vr::VRChaperoneSetup()->HideWorkingSetPreview();
        m_chaperoneCommitted = true;
        unsigned checkQuadCount = 0;
        vr::VRChaperoneSetup()->GetWorkingCollisionBoundsInfo(
            nullptr, &checkQuadCount );
        if ( checkQuadCount > 0 )
        {
            parent->chaperoneUtils().loadChaperoneData( false );
        }
    }
}

// NOTE this function will create bad output if User Rotates 180 Degrees in 1/7*
// frame-rate. (Worst Case 30 fps = ~770 deg/s)
void MoveCenterTabController::updateHmdRotationCounter(
    vr::TrackedDevicePose_t hmdPose,
    double angle )
{
    // If hmd tracking is bad, set m_lastHmdQuaternion invalid
    if ( !hmdPose.bPoseIsValid
         || hmdPose.eTrackingResult != vr::TrackingResult_Running_OK )
    {
        m_lastHmdQuaternion.w = k_quaternionInvalidValue;
        return;
    }

    // Get hmd pose matrix (in rotated coordinates)
    vr::HmdMatrix34_t hmdMatrix = hmdPose.mDeviceToAbsoluteTracking;

    // Set up (un)rotation matrix
    vr::HmdMatrix34_t hmdMatrixRotMat;
    vr::HmdMatrix34_t hmdMatrixAbsolute;
    utils::initRotationMatrix(
        hmdMatrixRotMat, 1, static_cast<float>( angle ) );

    // Get hmdMatrixAbsolute in un-rotated coordinates.
    utils::matMul33( hmdMatrixAbsolute, hmdMatrixRotMat, hmdMatrix );

    // Convert pose matrix to quaternion
    m_hmdQuaternion = quaternion::fromHmdMatrix34( hmdMatrixAbsolute );

    // Get rotation change of hmd
    // Checking for invalid quaternion using < because == isn't guaranteed
    // for doubles comparison.
    if ( m_lastHmdQuaternion.w < k_quaternionUnderIsInvalidValueThreshold )
    {
        m_lastHmdQuaternion = m_hmdQuaternion;
        return;
    }

    double hmdYawCurrent = quaternion::getYaw( m_hmdQuaternion );
    // double hmdPitchCurrent = quaternion::getPitch( m_hmdQuaternion );
    // double hmdRollCurrent = quaternion::getRoll( m_hmdQuaternion );

    // checks to see if hmd is in exact same position
    if ( m_hmdYawOld == hmdYawCurrent )
    {
        return;
    }
    // Checks if The points defined by Yaw old and current form an arc of no
    // more than 180 degrees, that MUST intersect With 180 degrees.

    // NOTE: This fails if there is > 180 degrees between the update Rate
    // Worst Case Scenario this should be ~770 deg/s turning speed. (30 fps)

    if ( std::abs( m_hmdYawOld - hmdYawCurrent ) > M_PI )
    {
        // Checks if the HMD is inverted, and skips if it is
        bool isInverted = ( hmdMatrixAbsolute.m[1][1] < 0 );
        if ( !isInverted )
        {
            if ( m_hmdYawOld >= 0 )
            {
                m_hmdYawTurnCount += 1;
            }
            else if ( m_hmdYawOld < 0 )
            {
                m_hmdYawTurnCount -= 1;
            }
        }
        else
        {
            LOG( WARNING ) << "HMD Was Inverted during a Turn Count, Turn "
                              "counter may be in-accurate";
        }
    }

    // Sets total rotation based on current orientation + Turn Count
    if ( m_hmdYawTurnCount == 0 )
    {
        m_hmdYawTotal = hmdYawCurrent;
    }
    else
    {
        m_hmdYawTotal = hmdYawCurrent + ( m_hmdYawTurnCount * 2 * M_PI );
    }
    m_hmdYawOld = hmdYawCurrent;
    m_lastHmdQuaternion = m_hmdQuaternion;
}

vr::HmdVector3_t MoveCenterTabController::relativeToAbsolute(
    const vr::HmdVector3_t& relative ) const
{
    double relativeControllerPosition[]
        = { static_cast<double>( relative.v[0] ),
            static_cast<double>( relative.v[1] ),
            static_cast<double>( relative.v[2] ) };
    rotateCoordinates( relativeControllerPosition,
                       -m_rotation * k_centidegreesToRadians );
    vr::HmdVector3_t absoluteControllerPosition = {
        static_cast<float>( relativeControllerPosition[0] ) + m_offsetX,
        static_cast<float>( relativeControllerPosition[1] ) + m_offsetY,
        static_cast<float>( relativeControllerPosition[2] ) + m_offsetZ,
    };
    return absoluteControllerPosition;
}

vr::HmdVector3_t MoveCenterTabController::absoluteToRelative(
    const vr::HmdVector3_t& absolute ) const
{
    double absoluteControllerPosition[]
        = { static_cast<double>( absolute.v[0] - m_offsetX ),
            static_cast<double>( absolute.v[1] - m_offsetY ),
            static_cast<double>( absolute.v[2] - m_offsetZ ) };
    rotateCoordinates( absoluteControllerPosition,
                       m_rotation * k_centidegreesToRadians );
    vr::HmdVector3_t relativeControllerPosition
        = { static_cast<float>( absoluteControllerPosition[0] ),
            static_cast<float>( absoluteControllerPosition[1] ),
            static_cast<float>( absoluteControllerPosition[2] ) };
    return relativeControllerPosition;
}

// Displace the entire universe by some difference. Both 'from' and 'to' are in
// absolute coordinates.
void MoveCenterTabController::displaceUniverse( const vr::HmdVector3_t& from,
                                                const vr::HmdVector3_t& to )
{
    double diff[3] = {
        static_cast<double>( to.v[0] - from.v[0] ),
        static_cast<double>( to.v[1] - from.v[1] ),
        static_cast<double>( to.v[2] - from.v[2] ),
    };

    // offset is un-rotated coordinates

    // prevent positional glitches from exceeding max openvr offset clamps.
    // We do this by detecting a drag larger than 100m in a single frame.
    if ( abs( diff[0] ) > 100.0 || abs( diff[1] ) > 100.0
         || abs( diff[2] ) > 100.0 )
    {
        reset();
    }

    // prevents updating if axis movement is locked
    if ( !lockXToggle() )
    {
        m_offsetX += static_cast<float>( diff[0] );
    }
    if ( !lockYToggle() )
    {
        m_offsetY += static_cast<float>( diff[1] );
    }
    if ( !lockZToggle() )
    {
        m_offsetZ += static_cast<float>( diff[2] );
    }

    double secondsSinceLastDragUpdate
        = std::chrono::duration<double>( std::chrono::steady_clock::now()
                                         - m_lastDragUpdateTimePoint )
              .count();

    // TODO: Add 'effects fling' boolean? or simply return diff[] as a
    // HmdVect3_t
    m_velocity[0] = ( diff[0] / secondsSinceLastDragUpdate )
                    * static_cast<double>( flingStrength() );
    m_velocity[1] = ( diff[1] / secondsSinceLastDragUpdate )
                    * static_cast<double>( flingStrength() );
    m_velocity[2] = ( diff[2] / secondsSinceLastDragUpdate )
                    * static_cast<double>( flingStrength() );
}

void MoveCenterTabController::updateHandDrag(
    vr::TrackedDevicePose_t* devicePoses )
{
    auto moveHandId = vr::VRSystem()->GetTrackedDeviceIndexForControllerRole(
        m_activeDragHand );

    if ( m_activeDragHand == vr::TrackedControllerRole_Invalid
         || moveHandId == vr::k_unTrackedDeviceIndexInvalid
         || moveHandId >= vr::k_unMaxTrackedDeviceCount )
    {
        // detect new release of drag bind
        if ( m_lastMoveHand != vr::TrackedControllerRole_Invalid )
        {
            emit offsetXChanged( m_offsetX );
            emit offsetYChanged( m_offsetY );
            emit offsetZChanged( m_offsetZ );

            // reset gravity update timepoint whenever a space drag was just
            // released
            m_lastGravityUpdateTimePoint = std::chrono::steady_clock::now();
        }
        m_lastMoveHand = m_activeDragHand;
        return;
    }

    vr::TrackedDevicePose_t* movePose;
    if ( m_seatedModeDetected )
    {
        vr::TrackedDevicePose_t
            seatedDevicePoses[vr::k_unMaxTrackedDeviceCount];
        vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(
            vr::TrackingUniverseSeated,
            0.0f,
            seatedDevicePoses,
            vr::k_unMaxTrackedDeviceCount );
        movePose = seatedDevicePoses + moveHandId;
    }
    else
    {
        movePose = devicePoses + moveHandId;
    }

    if ( !movePose->bPoseIsValid || !movePose->bDeviceIsConnected
         || movePose->eTrackingResult != vr::TrackingResult_Running_OK )
    {
        m_lastMoveHand = m_activeDragHand;
        return;
    }

    vr::HmdVector3_t relativeControllerPosition
        = { movePose->mDeviceToAbsoluteTracking.m[0][3],
            movePose->mDeviceToAbsoluteTracking.m[1][3],
            movePose->mDeviceToAbsoluteTracking.m[2][3] };
    auto absoluteControllerPosition
        = relativeToAbsolute( relativeControllerPosition );

    if ( m_lastMoveHand == m_activeDragHand )
    {
        // Displace from last controller position to current
        displaceUniverse( m_lastControllerPosition,
                          absoluteControllerPosition );
    }
    m_lastControllerPosition = absoluteControllerPosition;
    m_lastMoveHand = m_activeDragHand;
}

void MoveCenterTabController::updateHandTurn(
    vr::TrackedDevicePose_t* devicePoses,
    double angle )
{
    auto rotateHandId = vr::VRSystem()->GetTrackedDeviceIndexForControllerRole(
        m_activeTurnHand );

    if ( m_activeTurnHand == vr::TrackedControllerRole_Invalid
         || rotateHandId == vr::k_unTrackedDeviceIndexInvalid
         || rotateHandId >= vr::k_unMaxTrackedDeviceCount )
    {
        if ( m_lastRotateHand != vr::TrackedControllerRole_Invalid )
        {
            m_lastHandQuaternion.w = k_quaternionInvalidValue;
        }
        m_lastRotateHand = m_activeTurnHand;
        return;
    }
    vr::TrackedDevicePose_t* rotatePose = devicePoses + rotateHandId;
    if ( !rotatePose->bPoseIsValid || !rotatePose->bDeviceIsConnected
         || rotatePose->eTrackingResult != vr::TrackingResult_Running_OK )
    {
        m_lastRotateHand = m_activeTurnHand;
        return;
    }
    // Get hand's rotation.
    // handMatrix is in rotated coordinates.
    vr::HmdMatrix34_t handMatrix = rotatePose->mDeviceToAbsoluteTracking;

    // We need un-rotated coordinates for valid comparison between
    // handQuaternion and lastHandQuaternion. Set up (un)rotation
    // matrix.
    vr::HmdMatrix34_t handMatrixRotMat;
    vr::HmdMatrix34_t handMatrixAbsolute;
    utils::initRotationMatrix(
        handMatrixRotMat, 1, static_cast<float>( angle ) );

    // Get handMatrixAbsolute in un-rotated coordinates.
    utils::matMul33( handMatrixAbsolute, handMatrixRotMat, handMatrix );

    // Convert pose matrix to quaternion
    m_handQuaternion = quaternion::fromHmdMatrix34( handMatrixAbsolute );

    if ( m_lastRotateHand == m_activeTurnHand )
    {
        // Get rotation change of hand.
        // Checking for invalid quaternion using < because == isn't
        // guaranteed for doubles comparison.
        if ( m_lastHandQuaternion.w < k_quaternionUnderIsInvalidValueThreshold )
        {
            m_lastHandQuaternion = m_handQuaternion;
        }

        else
        {
            // Construct a quaternion representing difference
            // between old hand and new hand.
            vr::HmdQuaternion_t handDiffQuaternion = quaternion::multiply(
                m_handQuaternion,
                quaternion::conjugate( m_lastHandQuaternion ) );

            // Calculate yaw from quaternion.
            double handYawDiff = quaternion::getYaw( handDiffQuaternion );

            int newRotationAngleDeg = static_cast<int>(
                round( handYawDiff * k_radiansToCentidegrees ) + m_rotation );

            // Keep angle within -18000 ~ 18000 centidegrees
            if ( newRotationAngleDeg > 18000 )
            {
                newRotationAngleDeg -= 36000;
            }
            else if ( newRotationAngleDeg < -18000 )
            {
                newRotationAngleDeg += 36000;
            }

            setRotation( newRotationAngleDeg );
        }
    }
    m_lastHandQuaternion = m_handQuaternion;
    m_lastRotateHand = m_activeTurnHand;
}

void MoveCenterTabController::updateGravity()
{
    float currentGravity = gravityStrength();
    if ( m_gravityReversed )
    {
        currentGravity *= -1.0f;
    }
    double secondsSinceLastGravityUpdate
        = std::chrono::duration<double>( std::chrono::steady_clock::now()
                                         - m_lastGravityUpdateTimePoint )
              .count();

    // apply friction
    if ( frictionPercent() > 0 )
    {
        // if we have friction and all velocity components are less than
        // halt speed, we stop.
        if ( abs( m_velocity[0] ) < k_frictionHalt_mps
             && abs( m_velocity[1] ) < k_frictionHalt_mps
             && abs( m_velocity[2] ) < k_frictionHalt_mps )
        {
            m_velocity[0] = 0.0;
            m_velocity[1] = 0.0;
            m_velocity[2] = 0.0;
        }
        else
        {
            // frictionPercent() * 10 is scaler such that at 90fps we
            // reduce from terminal velocity (50m/s) to about 1mm/s in 1
            // second using 100% friction. This is an arbitrary scaler,
            // but doing a more accurate job would require a performance
            // hit of the math pow() function to an arbitrary exponent.
            double frictionVelocityMultiplier
                = ( 100.0
                    - ( static_cast<double>( frictionPercent() * 10 )
                        * secondsSinceLastGravityUpdate ) )
                  / 100.0;
            frictionVelocityMultiplier
                = std::clamp( frictionVelocityMultiplier, 0.0, 1.0 );

            m_velocity[0] *= frictionVelocityMultiplier;
            m_velocity[1] *= frictionVelocityMultiplier;
            m_velocity[2] *= frictionVelocityMultiplier;
        }
    }

    // prevent velocity underflow
    if ( std::isnan( m_velocity[0] ) )
    {
        m_velocity[0] = 0.0;
    }
    if ( std::isnan( m_velocity[1] ) )
    {
        m_velocity[1] = 0.0;
    }
    if ( std::isnan( m_velocity[2] ) )
    {
        m_velocity[2] = 0.0;
    }

    // clamp to terminal velocity
    clampVelocity( m_velocity );

    // make axis lock checkboxes lock velocity on that axis
    if ( lockXToggle() )
    {
        m_velocity[0] = 0.0;
    }
    if ( lockYToggle() )
    {
        m_velocity[1] = 0.0;
    }
    if ( lockZToggle() )
    {
        m_velocity[2] = 0.0;
    }

    // are we falling?
    // note: up is negative y
    // note: set to always fall if gravity reversed (currentGravity < 0)
    if ( ( m_offsetY < m_gravityFloor ) || ( currentGravity < 0 ) )
    {
        // check if we're about to land
        // note: we don't land if gravity is reversed (currentGravity < 0)
        if ( ( m_offsetY
                   + static_cast<float>( m_velocity[1]
                                         * secondsSinceLastGravityUpdate )
               >= m_gravityFloor )
             && currentGravity >= 0 )
        {
            // get ratio of how much from y velocity applied to overcome y
            // offset and get down to ground.
            double ratioVelocityScaledByTouchdown
                = 1
                  - ( ( static_cast<double>( m_offsetY - m_gravityFloor )
                        + ( m_velocity[1] * secondsSinceLastGravityUpdate ) )
                      / ( m_velocity[1] * secondsSinceLastGravityUpdate ) );
            // apply that ratio to X and Z velocties
            m_velocity[0] *= ratioVelocityScaledByTouchdown;
            m_velocity[2] *= ratioVelocityScaledByTouchdown;

            // update offsets
            m_offsetX += static_cast<float>( m_velocity[0]
                                             * secondsSinceLastGravityUpdate );
            m_offsetY = m_gravityFloor;
            m_offsetZ += static_cast<float>( m_velocity[2]
                                             * secondsSinceLastGravityUpdate );
            emit offsetXChanged( m_offsetX );
            emit offsetYChanged( m_offsetY );
            emit offsetZChanged( m_offsetZ );
        }

        // otherwise we're still falling
        else
        {
            // apply offset from velocity
            m_offsetX += static_cast<float>( m_velocity[0]
                                             * secondsSinceLastGravityUpdate );
            m_offsetY += static_cast<float>( m_velocity[1]
                                             * secondsSinceLastGravityUpdate );
            m_offsetZ += static_cast<float>( m_velocity[2]
                                             * secondsSinceLastGravityUpdate );
            emit offsetXChanged( m_offsetX );
            emit offsetYChanged( m_offsetY );
            emit offsetZChanged( m_offsetZ );

            // accelerate downward velocity for the next update
            // note: downward is positive y
            m_velocity[1] = m_velocity[1]
                            + ( static_cast<double>( currentGravity )
                                * secondsSinceLastGravityUpdate );
            return;
        }
    }
    // check if we're underground and force us to ground level.
    // note: downward is positive y
    if ( m_offsetY > m_gravityFloor )
    {
        m_offsetY = m_gravityFloor;
        emit offsetYChanged( m_offsetY );
    }
    // Touchdown! We've landed, velocity set to 0.
    m_velocity[0] = 0.0;
    m_velocity[1] = 0.0;
    m_velocity[2] = 0.0;
}

void MoveCenterTabController::updateSpace( bool forceUpdate )
{
    // Do nothing if all offsets and rotation are still the same...
    if ( m_offsetX == m_oldOffsetX && m_offsetY == m_oldOffsetY
         && m_offsetZ == m_oldOffsetZ && m_rotation == m_oldRotation
         && !forceUpdate )
    {
        return;
    }

    // reload from disk if we're at zero offsets and allow external edits
    if ( allowExternalEdits() && m_oldOffsetX == 0.0f && m_oldOffsetY == 0.0f
         && m_oldOffsetZ == 0.0f && m_oldRotation == 0 )
    {
        vr::VRChaperoneSetup()->ReloadFromDisk( vr::EChaperoneConfigFile_Live );
        vr::VRChaperoneSetup()->CommitWorkingCopy(
            vr::EChaperoneConfigFile_Live );
        updateChaperoneResetData();
    }

    // do a late on-demand setting of seated center basis when we need it for
    // motion. This gives the reload from disk a little more time to complete
    // before we apply the new seated basis.
    if ( m_pendingSeatedRecenter )
    {
        vr::VRChaperoneSetup()->GetWorkingSeatedZeroPoseToRawTrackingPose(
            &m_seatedCenterForReset );

        m_pendingSeatedRecenter = false;
    }

    vr::HmdMatrix34_t offsetUniverseCenter;

    // set offsetUniverseCenter to the current angle
    vr::HmdMatrix34_t rotationMatrix;
    utils::initRotationMatrix(
        rotationMatrix,
        1,
        static_cast<float>( m_rotation * k_centidegreesToRadians ) );
    utils::matMul33(
        offsetUniverseCenter, rotationMatrix, m_universeCenterForReset );
    // fill in matrix coordinates for basis universe zero point
    offsetUniverseCenter.m[0][3] = m_universeCenterForReset.m[0][3];
    offsetUniverseCenter.m[1][3] = m_universeCenterForReset.m[1][3];
    offsetUniverseCenter.m[2][3] = m_universeCenterForReset.m[2][3];

    // move offsetUniverseCenter to the current offsets
    offsetUniverseCenter.m[0][3]
        += m_universeCenterForReset.m[0][0] * m_offsetX;
    offsetUniverseCenter.m[1][3]
        += m_universeCenterForReset.m[1][0] * m_offsetX;
    offsetUniverseCenter.m[2][3]
        += m_universeCenterForReset.m[2][0] * m_offsetX;

    offsetUniverseCenter.m[0][3]
        += m_universeCenterForReset.m[0][1] * m_offsetY;
    offsetUniverseCenter.m[1][3]
        += m_universeCenterForReset.m[1][1] * m_offsetY;
    offsetUniverseCenter.m[2][3]
        += m_universeCenterForReset.m[2][1] * m_offsetY;

    offsetUniverseCenter.m[0][3]
        += m_universeCenterForReset.m[0][2] * m_offsetZ;
    offsetUniverseCenter.m[1][3]
        += m_universeCenterForReset.m[1][2] * m_offsetZ;
    offsetUniverseCenter.m[2][3]
        += m_universeCenterForReset.m[2][2] * m_offsetZ;

    // check if we just pushed offsetUniverseCenter out of bounds (40km)
    // (we reuse offsetUniverseCenterYaw to rotate the chaperone also)
    double offsetUniverseCenterYaw = static_cast<double>( std::atan2(
        offsetUniverseCenter.m[0][2], offsetUniverseCenter.m[2][2] ) );

    double offsetUniverseCenterXyz[3]
        = { static_cast<double>( offsetUniverseCenter.m[0][3] ),
            static_cast<double>( offsetUniverseCenter.m[1][3] ),
            static_cast<double>( offsetUniverseCenter.m[2][3] ) };
    // unrotate to get raw values of xyz

    rotateCoordinates( offsetUniverseCenterXyz,
                       offsetUniverseCenterYaw
                           - ( m_rotation * k_centidegreesToRadians ) );

    if ( abs( offsetUniverseCenterXyz[0] ) > k_maxOpenvrWorkingSetOffest
         || ( abs( offsetUniverseCenterXyz[0] )
                  > k_maxOvrasUniverseCenteredTurningOffset
              && universeCenteredRotation() ) )
    {
        LOG( INFO ) << "Raw universe center out of bounds ( X: "
                    << offsetUniverseCenterXyz[0] << " )";
        vr::HmdMatrix34_t standingZero;
        vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(
            &standingZero );
        LOG( INFO ) << "GetWorkingStandingZeroPoseToRawTrackingPose";
        outputLogHmdMatrix( standingZero );
        reset();
        parent->m_chaperoneTabController.applyAutosavedProfile();
        LOG( INFO ) << "-Resetting to autosaved chaperone profile-";
        return;
    }
    if ( abs( offsetUniverseCenterXyz[1] ) > k_maxOpenvrWorkingSetOffest
         || ( abs( offsetUniverseCenterXyz[1] )
                  > k_maxOvrasUniverseCenteredTurningOffset
              && universeCenteredRotation() ) )
    {
        LOG( INFO ) << "Raw universe center out of bounds ( Y: "
                    << offsetUniverseCenterXyz[1] << " )";
        vr::HmdMatrix34_t standingZero;
        vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(
            &standingZero );
        LOG( INFO ) << "GetWorkingStandingZeroPoseToRawTrackingPose";
        outputLogHmdMatrix( standingZero );
        reset();
        parent->m_chaperoneTabController.applyAutosavedProfile();
        LOG( INFO ) << "-Resetting to autosaved chaperone profile-";
        return;
    }
    if ( abs( offsetUniverseCenterXyz[2] ) > k_maxOpenvrWorkingSetOffest
         || ( abs( offsetUniverseCenterXyz[2] )
                  > k_maxOvrasUniverseCenteredTurningOffset
              && universeCenteredRotation() ) )
    {
        LOG( INFO ) << "Raw universe center out of bounds ( Z: "
                    << offsetUniverseCenterXyz[2] << " )";
        vr::HmdMatrix34_t standingZero;
        vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(
            &standingZero );
        LOG( INFO ) << "GetWorkingStandingZeroPoseToRawTrackingPose";
        outputLogHmdMatrix( standingZero );
        reset();
        parent->m_chaperoneTabController.applyAutosavedProfile();
        LOG( INFO ) << "-Resetting to autosaved chaperone profile-";
        return;
    }

    // keep the seated origin synced with offsets if in seated mode
    if ( m_trackingUniverse == vr::TrackingUniverseSeated )
    {
        vr::HmdMatrix34_t offsetSeatedCenter;

        // set offsetSeatedCenter to the current angle
        utils::matMul33(
            offsetSeatedCenter, rotationMatrix, m_seatedCenterForReset );
        // fill in matrix coordinates for basis universe zero point
        offsetSeatedCenter.m[0][3] = m_seatedCenterForReset.m[0][3];
        offsetSeatedCenter.m[1][3] = m_seatedCenterForReset.m[1][3];
        offsetSeatedCenter.m[2][3] = m_seatedCenterForReset.m[2][3];

        // move offsetSeatedCenter to the current offsets
        offsetSeatedCenter.m[0][3]
            += m_seatedCenterForReset.m[0][0] * m_offsetX;
        offsetSeatedCenter.m[1][3]
            += m_seatedCenterForReset.m[1][0] * m_offsetX;
        offsetSeatedCenter.m[2][3]
            += m_seatedCenterForReset.m[2][0] * m_offsetX;

        offsetSeatedCenter.m[0][3]
            += m_seatedCenterForReset.m[0][1] * m_offsetY;
        offsetSeatedCenter.m[1][3]
            += m_seatedCenterForReset.m[1][1] * m_offsetY;
        offsetSeatedCenter.m[2][3]
            += m_seatedCenterForReset.m[2][1] * m_offsetY;

        offsetSeatedCenter.m[0][3]
            += m_seatedCenterForReset.m[0][2] * m_offsetZ;
        offsetSeatedCenter.m[1][3]
            += m_seatedCenterForReset.m[1][2] * m_offsetZ;
        offsetSeatedCenter.m[2][3]
            += m_seatedCenterForReset.m[2][2] * m_offsetZ;

        vr::VRChaperoneSetup()->SetWorkingSeatedZeroPoseToRawTrackingPose(
            &offsetSeatedCenter );
    }

    // As of SVR 1.13.1 The Chaperone will follow Universe Center NOT raw Center
    // as such this should be off by defualt.

    if ( adjustChaperone() )
    {
        // make a copy of our bounds and
        // reorient relative to new universe center

        vr::HmdQuad_t* updatedBounds
            = new vr::HmdQuad_t[m_collisionBoundsCountForReset];

        // for every quad in the chaperone bounds...
        for ( unsigned quad = 0; quad < m_collisionBoundsCountForReset; quad++ )
        {
            // at every corner in that quad...
            for ( unsigned corner = 0; corner < 4; corner++ )
            {
                // copy the corner's xyz coordinates
                updatedBounds[quad].vCorners[corner].v[0]
                    = m_collisionBoundsForOffset[quad].vCorners[corner].v[0];

                updatedBounds[quad].vCorners[corner].v[1]
                    = m_collisionBoundsForOffset[quad].vCorners[corner].v[1];

                updatedBounds[quad].vCorners[corner].v[2]
                    = m_collisionBoundsForOffset[quad].vCorners[corner].v[2];

                // cancel universe center's xyz offsets to each corner's
                // position and shift over by original center position so that
                // we are mirroring the offset as reflected about the original
                // origin
                updatedBounds[quad].vCorners[corner].v[0]
                    -= offsetUniverseCenter.m[0][3]
                       - offsetUniverseCenter.m[0][3];
                //- m_universeCenterForReset.m[0][3];
                // but don't touch y=0 values to keep floor corners rooted down
                if ( updatedBounds[quad].vCorners[corner].v[1] != 0 )
                {
                    updatedBounds[quad].vCorners[corner].v[1]
                        -= offsetUniverseCenter.m[1][3]
                           - offsetUniverseCenter.m[1][3];
                    //- m_universeCenterForReset.m[1][3];
                }
                updatedBounds[quad].vCorners[corner].v[2]
                    -= offsetUniverseCenter.m[2][3]
                       - offsetUniverseCenter.m[2][3];
                //- m_universeCenterForReset.m[2][3];

                // rotate by universe center's yaw
                rotateFloatCoordinates(
                    updatedBounds[quad].vCorners[corner].v,
                    static_cast<float>( offsetUniverseCenterYaw ) );
            }
        }

        // update chaperone working set preview (this does not commit)
        vr::VRChaperoneSetup()->SetWorkingCollisionBoundsInfo(
            updatedBounds, m_collisionBoundsCountForReset );
        delete[] updatedBounds;
    }

    // Center Marker for playspace.
    if ( parent->m_chaperoneTabController.m_centerMarkerOverlayNeedsUpdate )
    {
        // Set Unrotated Coordinates
        float universePlayCenterTempCoords[3] = { 0.0f, 0.0f, 0.0f };
        universePlayCenterTempCoords[0]
            -= offsetUniverseCenter.m[0][3] - m_universeCenterForReset.m[0][3];
        universePlayCenterTempCoords[1]
            -= offsetUniverseCenter.m[1][3] - m_universeCenterForReset.m[1][3];
        universePlayCenterTempCoords[2]
            -= offsetUniverseCenter.m[2][3] - m_universeCenterForReset.m[2][3];
        // Rotate un-rotated to rotated
        rotateFloatCoordinates( universePlayCenterTempCoords,
                                static_cast<float>( offsetUniverseCenterYaw ) );
        // Set Up orientation properly away from raw center
        utils::matMul33(
            m_offsetmatrix, offsetUniverseCenter, utils::k_forwardUpMatrix );
        vr::HmdMatrix34_t rotMatrix;
        utils::initRotationMatrix(
            rotMatrix,
            1,
            static_cast<float>( -( ( m_rotation * k_centidegreesToRadians )
                                   + offsetUniverseCenterYaw ) ) );

        // Rotates orientation At playspace center
        vr::HmdMatrix34_t finalmatrix;

        utils::matMul33( finalmatrix, rotMatrix, m_offsetmatrix );

        finalmatrix.m[0][3] = universePlayCenterTempCoords[0];
        finalmatrix.m[1][3] = universePlayCenterTempCoords[1];
        finalmatrix.m[2][3] = universePlayCenterTempCoords[2];
        if ( m_trackingUniverse == vr::TrackingUniverseSeated )
        {
            vr::HmdMatrix34_t temp;
            vr::VRChaperoneSetup()->GetWorkingSeatedZeroPoseToRawTrackingPose(
                &temp );
            finalmatrix.m[1][3] += temp.m[1][3];
        }

        parent->m_chaperoneTabController.updateCenterMarkerOverlay(
            &finalmatrix );
    }

    vr::VRChaperoneSetup()->SetWorkingStandingZeroPoseToRawTrackingPose(
        &offsetUniverseCenter );

    if ( oldStyleMotion() )
    {
        // check if universe center is outside of OpenVR commit bounds (1km)
        if ( abs( offsetUniverseCenterXyz[0] ) > k_maxOpenvrCommitOffset )
        {
            LOG( INFO ) << "COMMIT FAILED: Raw universe center out of commit "
                           "bounds ( X: "
                        << offsetUniverseCenterXyz[0] << " )";
            vr::HmdMatrix34_t standingZero;
            vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(
                &standingZero );
            LOG( INFO ) << "GetWorkingStandingZeroPoseToRawTrackingPose";
            outputLogHmdMatrix( standingZero );
            reset();
            parent->m_chaperoneTabController.applyAutosavedProfile();
            LOG( INFO ) << "-Resetting to autosaved chaperone profile-";
            return;
        }
        if ( abs( offsetUniverseCenterXyz[1] ) > k_maxOpenvrCommitOffset )
        {
            LOG( INFO ) << "COMMIT FAILED: Raw universe center out of commit "
                           "bounds ( Y: "
                        << offsetUniverseCenterXyz[1] << " )";
            vr::HmdMatrix34_t standingZero;
            vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(
                &standingZero );
            LOG( INFO ) << "GetWorkingStandingZeroPoseToRawTrackingPose";
            outputLogHmdMatrix( standingZero );
            reset();
            parent->m_chaperoneTabController.applyAutosavedProfile();
            LOG( INFO ) << "-Resetting to autosaved chaperone profile-";
            return;
        }
        if ( abs( offsetUniverseCenterXyz[2] ) > k_maxOpenvrCommitOffset )
        {
            LOG( INFO ) << "COMMIT FAILED: Raw universe center out of commit "
                           "bounds ( Z: "
                        << offsetUniverseCenterXyz[2] << " )";
            vr::HmdMatrix34_t standingZero;
            vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(
                &standingZero );
            LOG( INFO ) << "GetWorkingStandingZeroPoseToRawTrackingPose";
            outputLogHmdMatrix( standingZero );
            reset();
            parent->m_chaperoneTabController.applyAutosavedProfile();
            LOG( INFO ) << "-Resetting to autosaved chaperone profile-";
            return;
        }
        vr::VRChaperoneSetup()->CommitWorkingCopy(
            vr::EChaperoneConfigFile_Live );
        m_chaperoneCommitted = true;
    }
    else
    {
        vr::VRChaperoneSetup()->ShowWorkingSetPreview();
        m_chaperoneCommitted = false;
    }

    // loadChaperoneData( false ), false so that we don't load live data, and
    // reference the working set instead.
    if ( m_collisionBoundsCountForReset > 0 )
    {
        parent->chaperoneUtils().loadChaperoneData( false );
    }

    m_oldOffsetX = m_offsetX;
    m_oldOffsetY = m_offsetY;
    m_oldOffsetZ = m_offsetZ;
    m_oldRotation = m_rotation;
}

void MoveCenterTabController::eventLoopTick(
    vr::ETrackingUniverseOrigin universe,
    vr::TrackedDevicePose_t* devicePoses )

{
    // detect if room setup is running
    if ( universe == vr::TrackingUniverseRawAndUncalibrated )
    {
        if ( !m_roomSetupModeDetected )
        {
            m_roomSetupModeDetected = true;

            // give up on any pending zero offsets so they aren't applied in the
            // middle of room setup.
            m_pendingZeroOffsets = false;

            LOG( INFO ) << "room setup ENTRY detected";
            vr::VRChaperoneSetup()->HideWorkingSetPreview();
            reset();
            vr::VRChaperoneSetup()->RevertWorkingCopy();
        }
        setTrackingUniverse( int( universe ) );
        return;
    }

    // detect entering seated mode
    if ( universe == vr::TrackingUniverseSeated && !m_seatedModeDetected )
    {
        m_seatedModeDetected = true;
        LOG( INFO ) << "seated mode ENTRY detected";
    }

    // detect exiting seated mode
    if ( universe == vr::TrackingUniverseStanding && m_seatedModeDetected )
    {
        m_seatedModeDetected = false;
        reset();
        LOG( INFO ) << "seated mode EXIT detected";
    }

    // If we're trying to redifine the origin point, but can't becaues of bad
    // chaperone calibration, we keep trying and hold off on motion features
    // until we get ChaperoneCalibrationState_OK

    // if we were in room setup, but got this far in the loop, universe is no
    // longer raw mode and we detect room setup exit and zeroOffsets() to apply
    // new setup.
    if ( m_pendingZeroOffsets || m_roomSetupModeDetected )
    {
        zeroOffsets();
        // m_roomSetupModeDetected is set to false in zeroOffsets() if it's
        // successful.
    }
    else
    {
        setTrackingUniverse( int( universe ) );

        // get current space rotation in radians
        double angle = m_rotation * k_centidegreesToRadians;

        // hmd rotations stats counting doesn't need to be smooth, so we skip
        // some frames for performance
        if ( m_hmdRotationStatsUpdateCounter >= k_hmdRotationCounterUpdateRate )
        {
            // device pose index 0 is always the hmd
            updateHmdRotationCounter( devicePoses[0], angle );
            m_hmdRotationStatsUpdateCounter = 0;
        }
        else
        {
            m_hmdRotationStatsUpdateCounter++;
        }

        // stop everything before processing motion if we're in seated mode and
        // don't have seated motion enabled
        if ( m_seatedModeDetected && !enableSeatedMotion() )
        {
            return;
        }

        // only update dynamic motion if the dash is closed
        if ( !parent->isDashboardVisible() )
        {
            // detect new dash closed
            if ( m_dashWasOpenPreviousFrame )
            {
                // reset velocity time points on dash closed so we don't factor
                // in the time motion was paused during the open dash
                m_lastDragUpdateTimePoint = std::chrono::steady_clock::now();
                m_lastGravityUpdateTimePoint = std::chrono::steady_clock::now();
            }

            // force chaperone bounds visible if turn or drag settings require
            if ( dragBounds()
                 && m_activeDragHand != vr::TrackedControllerRole_Invalid )
            {
                vr::VRChaperone()->ForceBoundsVisible( true );
            }
            else if ( turnBounds()
                      && m_activeTurnHand != vr::TrackedControllerRole_Invalid )
            {
                vr::VRChaperone()->ForceBoundsVisible( true );
            }
            // only set back to default every frame if setting is enabled
            else if ( turnBounds() || dragBounds() )
            {
                vr::VRChaperone()->ForceBoundsVisible(
                    parent->m_chaperoneTabController.forceBounds() );
            }

            // Smooth turn motion can cause sim-sickness so we check if the user
            // wants to skip frames to reduce vection. We use the factor squared
            // because of logarithmic human perception.
            if ( m_turnComfortFrameSkipCounter >= static_cast<unsigned>(
                     ( turnComfortFactor() * turnComfortFactor() ) ) )
            {
                updateHandTurn( devicePoses, angle );
                m_turnComfortFrameSkipCounter = 0;
            }
            else
            {
                m_turnComfortFrameSkipCounter++;
            }

            // Smooth drag motion can cause sim-sickness so we check if the user
            // wants to skip frames to reduce vection. We use the factor squared
            // because of logarithmic human perception.
            if ( m_dragComfortFrameSkipCounter >= static_cast<unsigned>(
                     ( dragComfortFactor() * dragComfortFactor() ) ) )
            {
                updateHandDrag( devicePoses );
                m_lastDragUpdateTimePoint = std::chrono::steady_clock::now();
                m_dragComfortFrameSkipCounter = 0;
            }
            else
            {
                m_dragComfortFrameSkipCounter++;
            }

            if ( m_gravityActive
                 && m_activeDragHand == vr::TrackedControllerRole_Invalid )
            {
                updateGravity();
                m_lastGravityUpdateTimePoint = std::chrono::steady_clock::now();
            }
            m_dashWasOpenPreviousFrame = false;
        }
        else
        {
            // dash is open this frame
            m_dashWasOpenPreviousFrame = true;
        }
        updateSpace();
    }
}
} // namespace advsettings
