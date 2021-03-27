
#pragma once

#include <QObject>
#include <openvr.h>
#include <chrono>
#include <qmath.h>
#include "../utils/Matrix.h"
#include "../utils/FrameRateUtils.h"
#include "../settings/settings_object.h"

class QQuickWindow;
// application namespace
namespace advsettings
{
// forward declaration
constexpr double k_centidegreesToRadians = M_PI / 18000.0;
constexpr double k_radiansToCentidegrees = 18000.0 / M_PI;
constexpr double k_quaternionInvalidValue = -1000.0;
constexpr double k_quaternionUnderIsInvalidValueThreshold = -900.0;
constexpr double k_terminalVelocity_mps = 50.0;
// k_frictionHalt_mps must be sufficiently small to allow a single tick of
// gravity through
constexpr double k_frictionHalt_mps = 0.0000001;
// give the max offset a buffer to avoid crossing when traveling at astronomical
// velocities
constexpr double k_maxOpenvrWorkingSetOffest = 39900.0;
constexpr double k_maxOpenvrCommitOffset = 990.0;
constexpr double k_maxOvrasUniverseCenteredTurningOffset = 25000.0;

class OverlayController;

struct OffsetProfile : settings::ISettingsObject
{
    std::string profileName;
    float offsetX = 0.0f;
    float offsetY = 0.0f;
    float offsetZ = 0.0f;
    int rotation = 0;

    virtual std::string settingsName() const override
    {
        return "MoveCenterTabController::OffsetProfile";
    }

    virtual settings::SettingsObjectData saveSettings() const override
    {
        settings::SettingsObjectData o;

        o.addValue( profileName );
        o.addValue( static_cast<double>( offsetX ) );
        o.addValue( static_cast<double>( offsetY ) );
        o.addValue( static_cast<double>( offsetZ ) );

        o.addValue( rotation );

        return o;
    }

    virtual void loadSettings( settings::SettingsObjectData& obj ) override
    {
        profileName = ( obj.getNextValueOrDefault( "" ) );
        offsetX = static_cast<float>( obj.getNextValueOrDefault( 0.0 ) );
        offsetY = static_cast<float>( obj.getNextValueOrDefault( 0.0 ) );
        offsetZ = static_cast<float>( obj.getNextValueOrDefault( 0.0 ) );

        rotation = obj.getNextValueOrDefault( 0 );
    }
};

class MoveCenterTabController : public QObject
{
    Q_OBJECT
    Q_PROPERTY( int trackingUniverse READ trackingUniverse WRITE
                    setTrackingUniverse NOTIFY trackingUniverseChanged )
    Q_PROPERTY(
        float offsetX READ offsetX WRITE setOffsetX NOTIFY offsetXChanged )
    Q_PROPERTY(
        float offsetY READ offsetY WRITE setOffsetY NOTIFY offsetYChanged )
    Q_PROPERTY(
        float offsetZ READ offsetZ WRITE setOffsetZ NOTIFY offsetZChanged )
    Q_PROPERTY(
        int rotation READ rotation WRITE setRotation NOTIFY rotationChanged )
    Q_PROPERTY( int tempRotation READ tempRotation WRITE setTempRotation NOTIFY
                    tempRotationChanged )
    Q_PROPERTY( int snapTurnAngle READ snapTurnAngle WRITE setSnapTurnAngle
                    NOTIFY snapTurnAngleChanged )
    Q_PROPERTY( int smoothTurnRate READ smoothTurnRate WRITE setSmoothTurnRate
                    NOTIFY smoothTurnRateChanged )
    Q_PROPERTY( int frictionPercent READ frictionPercent WRITE
                    setFrictionPercent NOTIFY frictionPercentChanged )
    Q_PROPERTY( bool adjustChaperone READ adjustChaperone WRITE
                    setAdjustChaperone NOTIFY adjustChaperoneChanged )
    Q_PROPERTY( bool moveShortcutRight READ moveShortcutRight WRITE
                    setMoveShortcutRight NOTIFY moveShortcutRightChanged )
    Q_PROPERTY( bool moveShortcutLeft READ moveShortcutLeft WRITE
                    setMoveShortcutLeft NOTIFY moveShortcutLeftChanged )
    Q_PROPERTY( bool turnBindLeft READ turnBindLeft WRITE setTurnBindLeft NOTIFY
                    turnBindLeftChanged )
    Q_PROPERTY( bool turnBindRight READ turnBindRight WRITE setTurnBindRight
                    NOTIFY turnBindRightChanged )
    Q_PROPERTY( bool dragBounds READ dragBounds WRITE setDragBounds NOTIFY
                    dragBoundsChanged )
    Q_PROPERTY( bool turnBounds READ turnBounds WRITE setTurnBounds NOTIFY
                    turnBoundsChanged )
    Q_PROPERTY( unsigned dragComfortFactor READ dragComfortFactor WRITE
                    setDragComfortFactor NOTIFY dragComfortFactorChanged )
    Q_PROPERTY( unsigned turnComfortFactor READ turnComfortFactor WRITE
                    setTurnComfortFactor NOTIFY turnComfortFactorChanged )
    Q_PROPERTY( bool heightToggle READ heightToggle WRITE setHeightToggle NOTIFY
                    heightToggleChanged )
    Q_PROPERTY( float heightToggleOffset READ heightToggleOffset WRITE
                    setHeightToggleOffset NOTIFY heightToggleOffsetChanged )
    Q_PROPERTY( float gravityStrength READ gravityStrength WRITE
                    setGravityStrength NOTIFY gravityStrengthChanged )
    Q_PROPERTY( float flingStrength READ flingStrength WRITE setFlingStrength
                    NOTIFY flingStrengthChanged )
    Q_PROPERTY( bool gravityActive READ gravityActive WRITE setGravityActive
                    NOTIFY gravityActiveChanged )
    Q_PROPERTY( bool momentumSave READ momentumSave WRITE setMomentumSave NOTIFY
                    momentumSaveChanged )
    Q_PROPERTY( bool lockXToggle READ lockXToggle WRITE setLockX NOTIFY
                    requireLockXChanged )
    Q_PROPERTY( bool lockYToggle READ lockYToggle WRITE setLockY NOTIFY
                    requireLockYChanged )
    Q_PROPERTY( bool lockZToggle READ lockZToggle WRITE setLockZ NOTIFY
                    requireLockZChanged )
    Q_PROPERTY(
        bool showLogMatricesButton READ showLogMatricesButton WRITE
            setShowLogMatricesButton NOTIFY showLogMatricesButtonChanged )
    Q_PROPERTY( bool allowExternalEdits READ allowExternalEdits WRITE
                    setAllowExternalEdits NOTIFY allowExternalEditsChanged )
    Q_PROPERTY( bool oldStyleMotion READ oldStyleMotion WRITE setOldStyleMotion
                    NOTIFY oldStyleMotionChanged )
    Q_PROPERTY(
        bool universeCenteredRotation READ universeCenteredRotation WRITE
            setUniverseCenteredRotation NOTIFY universeCenteredRotationChanged )
    Q_PROPERTY( bool enableSeatedMotion READ enableSeatedMotion WRITE
                    setEnableSeatedMotion NOTIFY enableSeatedMotionChanged )
    Q_PROPERTY( bool simpleRecenter READ simpleRecenter WRITE setSimpleRecenter
                    NOTIFY simpleRecenterChanged )

private:
    OverlayController* parent;

    int m_trackingUniverse = static_cast<int>( vr::TrackingUniverseStanding );
    bool m_chaperoneBasisAcquired = false;
    bool m_initComplete = false;
    float m_offsetX = 0.0f;
    float m_offsetY = 0.0f;
    float m_offsetZ = 0.0f;
    float m_oldOffsetX = 0.0f;
    float m_oldOffsetY = 0.0f;
    float m_oldOffsetZ = 0.0f;
    int m_rotation = 0;
    int m_oldRotation = 0;
    int m_tempRotation = 0;
    bool m_moveShortcutRightPressed = false;
    bool m_moveShortcutLeftPressed = false;
    vr::TrackedDeviceIndex_t m_activeMoveController;
    vr::HmdVector3_t m_lastControllerPosition;
    bool m_heightToggle = false;
    float m_gravityFloor = 0.0f;
    // Set lastHandQuaternion.w to -1000.0 when last hand is invalid.
    vr::HmdQuaternion_t m_lastHandQuaternion
        = { k_quaternionInvalidValue, 0.0, 0.0, 0.0 };
    vr::HmdQuaternion_t m_handQuaternion;
    // Set lastHmdQuaternion.w to -1000.0 when last hmd pose is invalid.
    vr::HmdQuaternion_t m_lastHmdQuaternion
        = { k_quaternionInvalidValue, 0.0, 0.0, 0.0 };
    vr::HmdQuaternion_t m_hmdQuaternion;
    double m_hmdYawTotal = 0.0;
    double m_hmdYawOld = 0.0;
    int m_hmdYawTurnCount = 0;
    vr::ETrackedControllerRole m_activeDragHand
        = vr::TrackedControllerRole_Invalid;
    vr::ETrackedControllerRole m_lastMoveHand
        = vr::TrackedControllerRole_Invalid;
    vr::ETrackedControllerRole m_activeTurnHand
        = vr::TrackedControllerRole_Invalid;
    vr::ETrackedControllerRole m_lastRotateHand
        = vr::TrackedControllerRole_Invalid;
    bool m_leftHandDragPressed = false;
    bool m_rightHandDragPressed = false;
    bool m_overrideLeftHandDragPressed = false;
    bool m_overrideRightHandDragPressed = false;
    bool m_leftHandTurnPressed = false;
    bool m_rightHandTurnPressed = false;
    bool m_overrideLeftHandTurnPressed = false;
    bool m_overrideRightHandTurnPressed = false;
    bool m_swapDragToLeftHandPressed = false;
    bool m_swapDragToRightHandPressed = false;
    bool m_swapDragToLeftHandActivated = false;
    bool m_swapDragToRightHandActivated = false;
    bool m_gravityActive = false;
    bool m_gravityReversed = false;
    bool m_chaperoneCommitted = true;
    bool m_pendingZeroOffsets = true;
    bool m_pendingSeatedRecenter = false;
    bool m_selfRequestedSeatedRecenter = false;
    bool m_dashWasOpenPreviousFrame = false;
    bool m_roomSetupModeDetected = false;
    bool m_seatedModeDetected = false;
    unsigned settingsUpdateCounter = 0;
    int m_hmdRotationStatsUpdateCounter = 0;
    unsigned m_dragComfortFrameSkipCounter = 0;
    unsigned m_turnComfortFrameSkipCounter = 0;

    // Matrix used For Center Marker
    vr::HmdMatrix34_t m_offsetmatrix = utils::k_forwardUpMatrix;

    double m_velocity[3] = { 0.0, 0.0, 0.0 };
    std::chrono::steady_clock::time_point m_lastGravityUpdateTimePoint;
    std::chrono::steady_clock::time_point m_lastDragUpdateTimePoint;
    vr::HmdQuad_t* m_collisionBoundsForReset;
    uint32_t m_collisionBoundsCountForReset = 0;
    vr::HmdMatrix34_t m_universeCenterForReset
        = { { { 1.0f, 0.0f, 0.0f, 0.0f },
              { 0.0f, 1.0f, 0.0f, 0.0f },
              { 0.0f, 0.0f, 1.0f, 0.0f } } };
    vr::HmdMatrix34_t m_seatedCenterForReset
        = { { { 1.0f, 0.0f, 0.0f, 0.0f },
              { 0.0f, 1.0f, 0.0f, 0.0f },
              { 0.0f, 0.0f, 1.0f, 0.0f } } };
    vr::HmdQuad_t* m_collisionBoundsForOffset;
    void updateCollisionBoundsForOffset();

    void updateHmdRotationCounter( vr::TrackedDevicePose_t hmdPose,
                                   double angle );
    void updateHandDrag( vr::TrackedDevicePose_t* devicePoses );
    void updateHandTurn( vr::TrackedDevicePose_t* devicePoses, double angle );
    void updateGravity();
    void updateSpace( bool forceUpdate = false );
    void clampVelocity( double* velocity );
    void applyChaperoneResetData();
    void saveUncommittedChaperone();
    void outputLogHmdMatrix( vr::HmdMatrix34_t hmdMatrix );

    std::vector<OffsetProfile> m_offsetProfiles;

public:
    void initStage1();
    void initStage2( OverlayController* parent );

    void eventLoopTick( vr::ETrackingUniverseOrigin universe,
                        vr::TrackedDevicePose_t* devicePoses );

    float offsetX() const;
    float offsetY() const;
    float offsetZ() const;
    int rotation() const;
    int tempRotation() const;
    int snapTurnAngle() const;
    int smoothTurnRate() const;
    int frictionPercent() const;
    bool adjustChaperone() const;
    bool moveShortcutRight() const;
    bool moveShortcutLeft() const;
    bool turnBindRight() const;
    bool turnBindLeft() const;
    bool dragBounds() const;
    bool turnBounds() const;
    int dragComfortFactor() const;
    int turnComfortFactor() const;
    bool heightToggle() const;
    float heightToggleOffset() const;
    float gravityStrength() const;
    float flingStrength() const;
    bool gravityActive() const;
    bool momentumSave() const;
    bool lockXToggle() const;
    bool lockYToggle() const;
    bool lockZToggle() const;
    bool showLogMatricesButton() const;
    bool allowExternalEdits() const;
    bool oldStyleMotion() const;
    bool universeCenteredRotation() const;
    bool enableSeatedMotion() const;
    bool simpleRecenter() const;
    bool isInitComplete() const;
    double getHmdYawTotal();
    void resetHmdYawTotal();
    void incomingSeatedReset();
    void setBoundsBasisHeight( float newHeight );
    float getBoundsBasisMaxY();
    void setRotationAroundPivot( int value,
                                 bool notify,
                                 const vr::HmdVector3_t& pivot );
    void displaceUniverse( const vr::HmdVector3_t& from,
                           const vr::HmdVector3_t& to );

    vr::HmdVector3_t
        relativeToAbsolute( const vr::HmdVector3_t& coordinates_in ) const;
    vr::HmdVector3_t
        absoluteToRelative( const vr::HmdVector3_t& absolute ) const;

    void updateChaperoneResetData();

    void reloadOffsetProfiles();
    void saveOffsetProfiles();
    Q_INVOKABLE unsigned getOffsetProfileCount();
    Q_INVOKABLE QString getOffsetProfileName( unsigned index );

    // actions:
    void leftHandSpaceDrag( bool leftHandDragActive );
    void rightHandSpaceDrag( bool rightHandDragActive );
    void optionalOverrideLeftHandSpaceDrag( bool overrideLeftHandDragActive );
    void optionalOverrideRightHandSpaceDrag( bool overrideRightHandDragActive );
    void leftHandSpaceTurn( bool leftHandTurnActive );
    void rightHandSpaceTurn( bool rightHandTurnActive );
    void optionalOverrideLeftHandSpaceTurn( bool overrideLeftHandTurnActive );
    void optionalOverrideRightHandSpaceTurn( bool overrideRightHandTurnActive );
    void swapSpaceDragToLeftHandOverride( bool swapDragToLeftHandActive );
    void swapSpaceDragToRightHandOverride( bool swapDragToRightHandActive );
    void gravityToggleAction( bool gravityToggleJustPressed );
    void gravityReverseAction( bool gravityReverseHeld );
    void heightToggleAction( bool heightToggleJustPressed );
    void resetOffsets( bool resetOffsetsJustPressed );
    void snapTurnLeft( bool snapTurnLeftJustPressed );
    void snapTurnRight( bool snapTurnRightJustPressed );
    void smoothTurnLeft( bool smoothTurnLeftActive );
    void smoothTurnRight( bool smoothTurnRightActive );
    void xAxisLockToggle( bool xAxisLockToggleJustPressed );
    void yAxisLockToggle( bool yAxisLockToggleJustPressed );
    void zAxisLockToggle( bool zAxisLockToggleJustPressed );

public slots:
    int trackingUniverse() const;
    void setTrackingUniverse( int value, bool notify = true );

    void setOffsetX( float value, bool notify = true );
    void setOffsetY( float value, bool notify = true );
    void setOffsetZ( float value, bool notify = true );

    void setRotation( int value, bool notify = true );
    void setTempRotation( int value, bool notify = true );
    void setSnapTurnAngle( int value, bool notify = true );
    void setSmoothTurnRate( int value, bool notify = true );
    void setFrictionPercent( int value, bool notify = true );

    void setAdjustChaperone( bool value, bool notify = true );
    void setMoveShortcutRight( bool value, bool notify = true );
    void setMoveShortcutLeft( bool value, bool notify = true );
    void setTurnBindRight( bool value, bool notify = true );
    void setTurnBindLeft( bool value, bool notify = true );
    void setDragComfortFactor( int value, bool notify = true );
    void setTurnComfortFactor( int value, bool notify = true );
    void setDragBounds( bool value, bool notify = true );
    void setTurnBounds( bool value, bool notify = true );
    void setHeightToggle( bool value, bool notify = true );
    void setHeightToggleOffset( float value, bool notify = true );
    void setGravityStrength( float value, bool notify = true );
    void setFlingStrength( float value, bool notify = true );
    void setGravityActive( bool value, bool notify = true );
    void setMomentumSave( bool value, bool notify = true );

    void modOffsetX( float value, bool notify = true );
    void modOffsetY( float value, bool notify = true );
    void modOffsetZ( float value, bool notify = true );

    void setLockX( bool value, bool notify = true );
    void setLockY( bool value, bool notify = true );
    void setLockZ( bool value, bool notify = true );
    void setShowLogMatricesButton( bool value, bool notify = true );
    void setAllowExternalEdits( bool value, bool notify = true );
    void setOldStyleMotion( bool value, bool notify = true );
    void setUniverseCenteredRotation( bool value, bool notify = true );
    void setEnableSeatedMotion( bool value, bool notify = true );
    void setSimpleRecenter( bool value, bool notify = true );

    void shutdown();
    void reset();
    void updateSeatedResetData();
    void outputLogPoses();
    void zeroOffsets();
    void sendSeatedRecenter();

    void addOffsetProfile( QString name );
    void applyOffsetProfile( unsigned index );
    void deleteOffsetProfile( unsigned index );

signals:
    void trackingUniverseChanged( int value );
    void offsetXChanged( float value );
    void offsetYChanged( float value );
    void offsetZChanged( float value );
    void rotationChanged( int value );
    void tempRotationChanged( int value );
    void snapTurnAngleChanged( int value );
    void smoothTurnRateChanged( int value );
    void frictionPercentChanged( int value );
    void adjustChaperoneChanged( bool value );
    void moveShortcutRightChanged( bool value );
    void moveShortcutLeftChanged( bool value );
    void turnBindRightChanged( bool value );
    void turnBindLeftChanged( bool value );
    void dragBoundsChanged( bool value );
    void turnBoundsChanged( bool value );
    void dragComfortFactorChanged( int value );
    void turnComfortFactorChanged( int value );
    void heightToggleChanged( bool value );
    void heightToggleOffsetChanged( float value );
    void gravityStrengthChanged( float value );
    void flingStrengthChanged( float value );
    void gravityActiveChanged( bool value );
    void momentumSaveChanged( bool value );
    void requireLockXChanged( bool value );
    void requireLockYChanged( bool value );
    void requireLockZChanged( bool value );
    void showLogMatricesButtonChanged( bool value );
    void allowExternalEditsChanged( bool value );
    void oldStyleMotionChanged( bool value );
    void universeCenteredRotationChanged( bool value );
    void enableSeatedMotionChanged( bool value );
    void simpleRecenterChanged( bool value );

    void offsetProfilesUpdated();
};

} // namespace advsettings
