#include "overlaycontroller.h"
#include <QOpenGLFramebufferObjectFormat>
#include <QOpenGLPaintDevice>
#include <QPainter>
#include <QQuickView>
#include <QApplication>
#include <QQmlEngine>
#include <QQmlContext>
#include <QtWidgets/QWidget>
#include <QMouseEvent>
#include <QtWidgets/QGraphicsSceneMouseEvent>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGraphicsEllipseItem>
#include <QOpenGLExtraFunctions>
#include <QCursor>
#include <QProcess>
#include <QMessageBox>
#include <iostream>
#include <cmath>
#include <openvr.h>
#include <easylogging++.h>
#include "utils/Matrix.h"
#include "keyboard_input/input_sender.h"
#include "settings/settings.h"

// application namespace
namespace advsettings
{
int verifyCustomTickRate( const int tickRate )
{
    if ( tickRate < 1 )
    {
        return 1;
    }
    else if ( tickRate > k_maxCustomTickRate )
    {
        return k_maxCustomTickRate;
    }

    return tickRate;
}

OverlayController::OverlayController( bool desktopMode,
                                      bool noSound,
                                      QQmlEngine& qmlEngine )
    : QObject(), m_desktopMode( desktopMode ), m_noSound( noSound ),
      m_verifiedCustomTickRateMs( verifyCustomTickRate( settings::getSetting(
          settings::IntSetting::APPLICATION_customTickRateMs ) ) ),
      m_actions(), m_alarm()
{
    // Arbitrarily chosen Max Length of Directory path, should be sufficient for
    // Any set-up
    const uint32_t maxLength = 16192;
    uint32_t requiredLength;

    char tempRuntimePath[maxLength];
    bool pathIsGood
        = vr::VR_GetRuntimePath( tempRuntimePath, maxLength, &requiredLength );

    // Throw Error If over 16k characters in path string
    if ( !pathIsGood )
    {
        LOG( ERROR ) << "Error Finding VR Runtime Path, Attempting Recovery: ";
        uint32_t maxLengthRe = requiredLength;
        LOG( INFO ) << "Open VR reporting Required path length of: "
                    << maxLengthRe;
    }

    m_runtimePathUrl = QUrl::fromLocalFile( tempRuntimePath );
    LOG( INFO ) << "VR Runtime Path: " << m_runtimePathUrl.toLocalFile();

    constexpr auto clickSoundURL = "res/sounds/click.wav";
    const auto activationSoundFile
        = paths::binaryDirectoryFindFile( clickSoundURL );

    if ( activationSoundFile.has_value() )
    {
        m_activationSoundEffect.setSource( QUrl::fromLocalFile(
            QString::fromStdString( ( *activationSoundFile ) ) ) );
        m_activationSoundEffect.setVolume( 0.7 );
    }
    else
    {
        LOG( ERROR ) << "Could not find activation sound file "
                     << clickSoundURL;
    }
    constexpr auto focusChangedSoundURL = "res/sounds/focus.wav";
    const auto focusChangedSoundFile
        = paths::binaryDirectoryFindFile( focusChangedSoundURL );

    if ( focusChangedSoundFile.has_value() )
    {
        m_focusChangedSoundEffect.setSource( QUrl::fromLocalFile(
            QString::fromStdString( ( *focusChangedSoundFile ) ) ) );
        m_focusChangedSoundEffect.setVolume( 0.7 );
    }
    else
    {
        LOG( ERROR ) << "Could not find focus Changed sound file "
                     << focusChangedSoundURL;
    }

    constexpr auto alarmFileName = "res/sounds/alarm01.wav";
    const auto alarm01SoundFile
        = paths::binaryDirectoryFindFile( alarmFileName );

    if ( alarm01SoundFile.has_value() )
    {
        m_alarm01SoundEffect.setSource( QUrl::fromLocalFile(
            QString::fromStdString( ( *alarm01SoundFile ) ) ) );
        m_alarm01SoundEffect.setVolume( 1.0 );
    }
    else
    {
        LOG( ERROR ) << "Could not find alarm01 sound file " << alarmFileName;
    }

    QSurfaceFormat format;
    // Qt's QOpenGLPaintDevice is not compatible with OpenGL versions >= 3.0
    // NVIDIA does not care, but unfortunately AMD does
    // Are subtle changes to the semantics of OpenGL functions actually covered
    // by the compatibility profile, and this is an AMD bug?
    format.setVersion( 2, 1 );
    // format.setProfile( QSurfaceFormat::CompatibilityProfile );
    format.setDepthBufferSize( 16 );
    format.setStencilBufferSize( 8 );
    format.setSamples( 16 );

    m_openGLContext.setFormat( format );
    if ( !m_openGLContext.create() )
    {
        throw std::runtime_error( "Could not create OpenGL context" );
    }

    // create an offscreen surface to attach the context and FBO to
    m_offscreenSurface.setFormat( m_openGLContext.format() );
    m_offscreenSurface.create();
    m_openGLContext.makeCurrent( &m_offscreenSurface );

    if ( !vr::VROverlay() )
    {
        QMessageBox::critical(
            nullptr, "OpenVR Advanced Settings Overlay", "Is OpenVR running?" );
        throw std::runtime_error( std::string( "No Overlay interface" ) );
    }

    // Init controllers
    m_steamVRTabController.initStage1();
    m_chaperoneTabController.initStage1();
    m_moveCenterTabController.initStage1();
    m_audioTabController.initStage1();
    m_settingsTabController.initStage1();
    m_videoTabController.initStage1();
    m_rotationTabController.initStage1();

    // init action handles

    m_chaperoneTabController.setLeftHapticActionHandle(
        m_actions.leftHapticActionHandle() );
    m_chaperoneTabController.setRightHapticActionHandle(
        m_actions.rightHapticActionHandle() );
    m_chaperoneTabController.setLeftInputHandle( m_actions.leftInputHandle() );
    m_chaperoneTabController.setRightInputHandle(
        m_actions.rightInputHandle() );

    // Set qml context
    qmlEngine.rootContext()->setContextProperty( "applicationVersion",
                                                 getVersionString() );
    qmlEngine.rootContext()->setContextProperty( "vrRuntimePath",
                                                 getVRRuntimePathUrl() );

    // Pretty disgusting trick to allow qmlRegisterSingletonType to continue
    // working with the lambdas that were already there. The callback function
    // in qmlRegisterSingletonType won't work with any lambdas that capture the
    // environment. The alternative to making a static pointer to this was
    // rewriting all QML to not be singletons, which should probably be done
    // whenever possible.
    static OverlayController* const objectAddress = this;
    constexpr auto qmlSingletonImportName = "ovras.advsettings";
    qmlRegisterSingletonType<OverlayController>(
        qmlSingletonImportName,
        1,
        0,
        "OverlayController",
        []( QQmlEngine*, QJSEngine* ) {
            QObject* obj = objectAddress;
            QQmlEngine::setObjectOwnership( obj, QQmlEngine::CppOwnership );
            return obj;
        } );
    // It is unknown if it it intended for the generic in
    // qmlRegisterSingletonType to be <SteamVRTabController> in all the
    // remaining function calls, or if it's just a copy paste accident that
    // happens to work.
    qmlRegisterSingletonType<SteamVRTabController>(
        qmlSingletonImportName,
        1,
        0,
        "SteamVRTabController",
        []( QQmlEngine*, QJSEngine* ) {
            QObject* obj = &( objectAddress->m_steamVRTabController );
            QQmlEngine::setObjectOwnership( obj, QQmlEngine::CppOwnership );
            return obj;
        } );
    qmlRegisterSingletonType<SteamVRTabController>(
        qmlSingletonImportName,
        1,
        0,
        "ChaperoneTabController",
        []( QQmlEngine*, QJSEngine* ) {
            QObject* obj = &( objectAddress->m_chaperoneTabController );
            QQmlEngine::setObjectOwnership( obj, QQmlEngine::CppOwnership );
            return obj;
        } );
    qmlRegisterSingletonType<SteamVRTabController>(
        qmlSingletonImportName,
        1,
        0,
        "MoveCenterTabController",
        []( QQmlEngine*, QJSEngine* ) {
            QObject* obj = &( objectAddress->m_moveCenterTabController );
            QQmlEngine::setObjectOwnership( obj, QQmlEngine::CppOwnership );
            return obj;
        } );
    qmlRegisterSingletonType<SteamVRTabController>(
        qmlSingletonImportName,
        1,
        0,
        "FixFloorTabController",
        []( QQmlEngine*, QJSEngine* ) {
            QObject* obj = &( objectAddress->m_fixFloorTabController );
            QQmlEngine::setObjectOwnership( obj, QQmlEngine::CppOwnership );
            return obj;
        } );
    qmlRegisterSingletonType<SteamVRTabController>(
        qmlSingletonImportName,
        1,
        0,
        "AudioTabController",
        []( QQmlEngine*, QJSEngine* ) {
            QObject* obj = &( objectAddress->m_audioTabController );
            QQmlEngine::setObjectOwnership( obj, QQmlEngine::CppOwnership );
            return obj;
        } );
    qmlRegisterSingletonType<SteamVRTabController>(
        qmlSingletonImportName,
        1,
        0,
        "StatisticsTabController",
        []( QQmlEngine*, QJSEngine* ) {
            QObject* obj = &( objectAddress->m_statisticsTabController );
            QQmlEngine::setObjectOwnership( obj, QQmlEngine::CppOwnership );
            return obj;
        } );
    qmlRegisterSingletonType<SteamVRTabController>(
        qmlSingletonImportName,
        1,
        0,
        "SettingsTabController",
        []( QQmlEngine*, QJSEngine* ) {
            QObject* obj = &( objectAddress->m_settingsTabController );
            QQmlEngine::setObjectOwnership( obj, QQmlEngine::CppOwnership );
            return obj;
        } );
    qmlRegisterSingletonType<SteamVRTabController>(
        qmlSingletonImportName,
        1,
        0,
        "UtilitiesTabController",
        []( QQmlEngine*, QJSEngine* ) {
            QObject* obj = &( objectAddress->m_utilitiesTabController );
            QQmlEngine::setObjectOwnership( obj, QQmlEngine::CppOwnership );
            return obj;
        } );
    qmlRegisterSingletonType<SteamVRTabController>(
        qmlSingletonImportName,
        1,
        0,
        "VideoTabController",
        []( QQmlEngine*, QJSEngine* ) {
            QObject* obj = &( objectAddress->m_videoTabController );
            QQmlEngine::setObjectOwnership( obj, QQmlEngine::CppOwnership );
            return obj;
        } );
    qmlRegisterSingletonType<SteamVRTabController>(
        qmlSingletonImportName,
        1,
        0,
        "RotationTabController",
        []( QQmlEngine*, QJSEngine* ) {
            QObject* obj = &( objectAddress->m_rotationTabController );
            QQmlEngine::setObjectOwnership( obj, QQmlEngine::CppOwnership );
            return obj;
        } );
    qmlRegisterSingletonType<alarm_clock::VrAlarm>(
        qmlSingletonImportName, 1, 0, "VrAlarm", []( QQmlEngine*, QJSEngine* ) {
            QObject* obj = &( objectAddress->m_alarm );
            QQmlEngine::setObjectOwnership( obj, QQmlEngine::CppOwnership );
            return obj;
        } );

    // Grab local version number
    QStringList verNumericalString
        = QString( application_strings::applicationVersionString ).split( "-" );
    QStringList verMajorMinorPatchString = verNumericalString[0].split( "." );
    m_localVersionMajor = verMajorMinorPatchString[0].toInt();
    m_localVersionMinor = verMajorMinorPatchString[1].toInt();
    m_localVersionPatch = verMajorMinorPatchString[2].toInt();

    // Init network manager
    connect( netManager,
             SIGNAL( finished( QNetworkReply* ) ),
             this,
             SLOT( OnNetworkReply( QNetworkReply* ) ) );

    if ( !disableVersionCheck() )
    {
        QNetworkRequest netRequest;
        netRequest.setUrl( QUrl( application_strings::versionCheckUrl ) );
        netManager->get( netRequest );
    }
    else
    {
        LOG( INFO ) << "Version Check: Feature disabled. Not checking version.";
    }

    LOG( INFO ) << "OPENSSL VERSION: "
                << QSslSocket::sslLibraryBuildVersionString();
}

OverlayController::~OverlayController()
{
    Shutdown();
}

void OverlayController::exitApp()
{
    // save to settings that shutdown was safe
    setPreviousShutdownSafe( true );

    settings::saveAllSettings();

    m_moveCenterTabController.shutdown();
    // Un-mute mic before Exiting VR, as it is set at system level Not
    // Vr level.
    // m_audioTabController.setMicMuted( false, false );
    m_audioTabController.shutdown();
    m_chaperoneTabController.shutdown();

    Shutdown();
    QApplication::exit();

    LOG( INFO ) << "All systems exited.";
    exit( EXIT_SUCCESS );
    // Does not fallthrough
}

void OverlayController::Shutdown()
{
    disconnect( &m_pumpEventsTimer,
                SIGNAL( timeout() ),
                this,
                SLOT( OnTimeoutPumpEvents() ) );
    m_pumpEventsTimer.stop();

    if ( m_pRenderTimer )
    {
        disconnect( &m_renderControl,
                    SIGNAL( renderRequested() ),
                    this,
                    SLOT( OnRenderRequest() ) );
        disconnect( &m_renderControl,
                    SIGNAL( sceneChanged() ),
                    this,
                    SLOT( OnRenderRequest() ) );
        disconnect( m_pRenderTimer.get(),
                    SIGNAL( timeout() ),
                    this,
                    SLOT( renderOverlay() ) );
        m_pRenderTimer->stop();
        m_pRenderTimer.reset();
    }
    m_pFbo.reset();
}

void OverlayController::SetWidget( QQuickItem* quickItem,
                                   const std::string& name,
                                   const std::string& key )
{
    if ( !m_desktopMode )
    {
        vr::VROverlayError overlayError
            = vr::VROverlay()->CreateDashboardOverlay(
                key.c_str(),
                name.c_str(),
                &m_ulOverlayHandle,
                &m_ulOverlayThumbnailHandle );
        if ( overlayError != vr::VROverlayError_None )
        {
            if ( overlayError == vr::VROverlayError_KeyInUse )
            {
                QMessageBox::critical( nullptr,
                                       "OpenVR Advanced Settings Overlay",
                                       "Another instance is already running." );
            }
            throw std::runtime_error( std::string(
                "Failed to create Overlay: "
                + std::string( vr::VROverlay()->GetOverlayErrorNameFromEnum(
                    overlayError ) ) ) );
        }
        vr::VROverlay()->SetOverlayWidthInMeters( m_ulOverlayHandle, 2.5f );
        vr::VROverlay()->SetOverlayInputMethod(
            m_ulOverlayHandle, vr::VROverlayInputMethod_Mouse );
        vr::VROverlay()->SetOverlayFlag(
            m_ulOverlayHandle,
            vr::VROverlayFlags_SendVRSmoothScrollEvents,
            true );

        constexpr auto thumbiconFilename = "res/img/icons/thumbicon.png";
        const auto thumbIconPath
            = paths::binaryDirectoryFindFile( thumbiconFilename );
        if ( thumbIconPath.has_value() )
        {
            vr::VROverlay()->SetOverlayFromFile( m_ulOverlayThumbnailHandle,
                                                 thumbIconPath->c_str() );
        }
        else
        {
            LOG( ERROR ) << "Could not find thumbnail icon \""
                         << thumbiconFilename << "\"";
        }

        // Too many render calls in too short time overwhelm Qt and an
        // assertion gets thrown. Therefore we use an timer to delay render
        // calls
        m_pRenderTimer.reset( new QTimer() );
        m_pRenderTimer->setSingleShot( true );
        m_pRenderTimer->setInterval( 5 );
        connect( m_pRenderTimer.get(),
                 SIGNAL( timeout() ),
                 this,
                 SLOT( renderOverlay() ) );

        QOpenGLFramebufferObjectFormat fboFormat;
        fboFormat.setAttachment(
            QOpenGLFramebufferObject::CombinedDepthStencil );
        fboFormat.setTextureTarget( GL_TEXTURE_2D );
        m_pFbo.reset( new QOpenGLFramebufferObject(
            static_cast<int>( quickItem->width() ),
            static_cast<int>( quickItem->height() ),
            fboFormat ) );

        m_window.setRenderTarget( m_pFbo.get() );
        quickItem->setParentItem( m_window.contentItem() );
        m_window.setGeometry( 0,
                              0,
                              static_cast<int>( quickItem->width() ),
                              static_cast<int>( quickItem->height() ) );
        m_renderControl.initialize( &m_openGLContext );

        vr::HmdVector2_t vecWindowSize
            = { static_cast<float>( quickItem->width() ),
                static_cast<float>( quickItem->height() ) };
        vr::VROverlay()->SetOverlayMouseScale( m_ulOverlayHandle,
                                               &vecWindowSize );

        connect( &m_renderControl,
                 SIGNAL( renderRequested() ),
                 this,
                 SLOT( OnRenderRequest() ) );
        connect( &m_renderControl,
                 SIGNAL( sceneChanged() ),
                 this,
                 SLOT( OnRenderRequest() ) );
    }

    connect( &m_pumpEventsTimer,
             SIGNAL( timeout() ),
             this,
             SLOT( OnTimeoutPumpEvents() ) );

    // Every 1ms we check if the current frame has advanced (for vsync)
    m_pumpEventsTimer.setInterval( 1 );

    m_pumpEventsTimer.start();

    m_steamVRTabController.initStage2( this );
    m_chaperoneTabController.initStage2( this );
    m_fixFloorTabController.initStage2( this );
    m_audioTabController.initStage2();
    m_statisticsTabController.initStage2( this );
    m_settingsTabController.initStage2( this );
    m_utilitiesTabController.initStage2( this );
    m_moveCenterTabController.initStage2( this );
    m_rotationTabController.initStage2( this );
    m_videoTabController.initStage2();
}

void OverlayController::OnRenderRequest()
{
    if ( m_pRenderTimer && !m_pRenderTimer->isActive() )
    {
        m_pRenderTimer->start();
    }
}

void OverlayController::renderOverlay()
{
    if ( !m_desktopMode )
    {
        // skip rendering if the overlay isn't visible
        if ( !vr::VROverlay()
             || ( !vr::VROverlay()->IsOverlayVisible( m_ulOverlayHandle )
                  && !vr::VROverlay()->IsOverlayVisible(
                      m_ulOverlayThumbnailHandle ) ) )
            return;
        m_renderControl.polishItems();
        m_renderControl.sync();
        m_renderControl.render();

        GLuint unTexture = m_pFbo->texture();
        if ( unTexture != 0 )
        {
#if defined _WIN64 || defined _LP64
            // To avoid any compiler warning because of cast to a larger
            // pointer type (warning C4312 on VC)
            vr::Texture_t texture = { reinterpret_cast<void*>(
                                          static_cast<uint64_t>( unTexture ) ),
                                      vr::TextureType_OpenGL,
                                      vr::ColorSpace_Auto };
#else
            vr::Texture_t texture = { reinterpret_cast<void*>( unTexture ),
                                      vr::TextureType_OpenGL,
                                      vr::ColorSpace_Auto };
#endif
            vr::VROverlay()->SetOverlayTexture( m_ulOverlayHandle, &texture );
        }
        m_openGLContext.functions()->glFlush(); // We need to flush otherwise
                                                // the texture may be empty.*/
    }
}

bool OverlayController::pollNextEvent( vr::VROverlayHandle_t ulOverlayHandle,
                                       vr::VREvent_t* pEvent )
{
    if ( isDesktopMode() )
    {
        return vr::VRSystem()->PollNextEvent( pEvent, sizeof( vr::VREvent_t ) );
    }
    else
    {
        return vr::VROverlay()->PollNextOverlayEvent(
            ulOverlayHandle, pEvent, sizeof( vr::VREvent_t ) );
    }
}

QPoint OverlayController::getMousePositionForEvent( vr::VREvent_Mouse_t mouse )
{
    float y = mouse.y;
#ifdef __linux__
    float h = static_cast<float>( m_window.height() );
    y = h - y;
#endif
    return QPoint( static_cast<int>( mouse.x ), static_cast<int>( y ) );
}

void OverlayController::processMediaKeyBindings()
{
    if ( m_actions.nextSong() )
    {
        m_utilitiesTabController.sendMediaNextSong();
    }
    if ( m_actions.previousSong() )
    {
        m_utilitiesTabController.sendMediaPreviousSong();
    }
    if ( m_actions.pausePlaySong() )
    {
        m_utilitiesTabController.sendMediaPausePlay();
    }
    if ( m_actions.stopSong() )
    {
        m_utilitiesTabController.sendMediaStopSong();
    }
}

void OverlayController::processMotionBindings()
{
    // Execution order for moveCenterTabController actions is important.
    // Don't reorder these. Override actions must always come after normal
    // because active priority is set based on which action is "newest"
    // normal actions:
    m_moveCenterTabController.leftHandSpaceDrag(
        m_actions.leftHandSpaceDrag() );
    m_moveCenterTabController.rightHandSpaceDrag(
        m_actions.rightHandSpaceDrag() );
    m_moveCenterTabController.leftHandSpaceTurn(
        m_actions.leftHandSpaceTurn() );
    m_moveCenterTabController.rightHandSpaceTurn(
        m_actions.rightHandSpaceTurn() );
    m_moveCenterTabController.gravityToggleAction( m_actions.gravityToggle() );
    m_moveCenterTabController.gravityReverseAction(
        m_actions.gravityReverse() );
    m_moveCenterTabController.heightToggleAction( m_actions.heightToggle() );
    m_moveCenterTabController.resetOffsets( m_actions.resetOffsets() );
    m_moveCenterTabController.snapTurnLeft( m_actions.snapTurnLeft() );
    m_moveCenterTabController.snapTurnRight( m_actions.snapTurnRight() );
    m_moveCenterTabController.smoothTurnLeft( m_actions.smoothTurnLeft() );
    m_moveCenterTabController.smoothTurnRight( m_actions.smoothTurnRight() );
    m_moveCenterTabController.xAxisLockToggle( m_actions.xAxisLockToggle() );
    m_moveCenterTabController.yAxisLockToggle( m_actions.yAxisLockToggle() );
    m_moveCenterTabController.zAxisLockToggle( m_actions.zAxisLockToggle() );

    // override actions:
    m_moveCenterTabController.optionalOverrideLeftHandSpaceDrag(
        m_actions.optionalOverrideLeftHandSpaceDrag() );
    m_moveCenterTabController.optionalOverrideRightHandSpaceDrag(
        m_actions.optionalOverrideRightHandSpaceDrag() );
    m_moveCenterTabController.optionalOverrideLeftHandSpaceTurn(
        m_actions.optionalOverrideLeftHandSpaceTurn() );
    m_moveCenterTabController.optionalOverrideRightHandSpaceTurn(
        m_actions.optionalOverrideRightHandSpaceTurn() );
    m_moveCenterTabController.swapSpaceDragToLeftHandOverride(
        m_actions.swapSpaceDragToLeftHandOverride() );
    m_moveCenterTabController.swapSpaceDragToRightHandOverride(
        m_actions.swapSpaceDragToRightHandOverride() );
}

void OverlayController::processChaperoneBindings()
{
    if ( m_actions.chaperoneToggle() )
    {
        m_chaperoneTabController.setDisableChaperone(
            !( m_chaperoneTabController.disableChaperone() ), true );
    }
    m_chaperoneTabController.setProxState( m_actions.proxState() );
    m_chaperoneTabController.addLeftHapticClick(
        m_actions.addLeftHapticClick() );
    m_chaperoneTabController.addRightHapticClick(
        m_actions.addRightHapticClick() );
}

void OverlayController::processPushToTalkBindings()
{
    const auto pushToTalkCannotChange = !m_audioTabController.pttChangeValid();
    const auto pushToTalkEnabled = m_audioTabController.pttEnabled();

    const auto proxSensorActivated = m_actions.proxState();
    const auto useProxSensor = m_audioTabController.micProximitySensorCanMute();

    if ( useProxSensor )
    {
        if ( !proxSensorActivated )
        {
            m_audioTabController.setMicMuted( true );
            return;
        }
        // strictly speaking this is not the most elegant solution, but
        // should work well enough.
        else if ( !pushToTalkEnabled )
        {
            m_audioTabController.setMicMuted( false );
        }
    }

    if ( pushToTalkCannotChange || !pushToTalkEnabled )
    {
        return;
    }

    const auto pushToTalkButtonActivated = m_actions.pushToTalk();
    const auto pushToTalkCurrentlyActive = m_audioTabController.pttActive();

    if ( pushToTalkButtonActivated && !pushToTalkCurrentlyActive )
    {
        m_audioTabController.startPtt();
    }
    else if ( !pushToTalkButtonActivated && pushToTalkCurrentlyActive )
    {
        m_audioTabController.stopPtt();
    }
}

void OverlayController::processKeyboardBindings()
{
    if ( m_actions.keyboardOne() )
    {
        const auto commands = settings::getSetting(
            settings::StringSetting::KEYBOARDSHORTCUT_keyboardOne );

        sendStringAsInput( commands );
    }

    if ( m_actions.keyboardTwo() )
    {
        const auto commands = settings::getSetting(
            settings::StringSetting::KEYBOARDSHORTCUT_keyboardTwo );

        sendStringAsInput( commands );
    }

    if ( m_actions.keyboardThree() )
    {
        const auto commands = settings::getSetting(
            settings::StringSetting::KEYBOARDSHORTCUT_keyboardThree );

        sendStringAsInput( commands );
    }
    // Press Key One
    if ( m_actions.keyPressMisc() && !m_keyPressOneState )
    {
        const auto commands = settings::getSetting(
            settings::StringSetting::KEYBOARDSHORTCUT_keyPressMisc );
        sendFirstCharAsInput( commands, KeyStatus::Down );
        m_keyPressOneState = true;
    }
    if ( m_keyPressOneState && !m_actions.keyPressMisc() )
    {
        const auto commands = settings::getSetting(
            settings::StringSetting::KEYBOARDSHORTCUT_keyPressMisc );
        sendFirstCharAsInput( commands, KeyStatus::Up );
        m_keyPressOneState = false;
    }

    // Press Key Two
    if ( m_actions.keyPressSystem() && !m_keyPressTwoState )
    {
        const auto commands = settings::getSetting(
            settings::StringSetting::KEYBOARDSHORTCUT_keyPressSystem );
        sendFirstCharAsInput( commands, KeyStatus::Down );
        m_keyPressTwoState = true;
    }
    if ( m_keyPressTwoState && !m_actions.keyPressSystem() )
    {
        const auto commands = settings::getSetting(
            settings::StringSetting::KEYBOARDSHORTCUT_keyPressSystem );
        sendFirstCharAsInput( commands, KeyStatus::Up );
        m_keyPressTwoState = false;
    }
}

void OverlayController::processExclusiveInputBinding()
{
    if ( m_actions.exclusiveInputToggle() && exclusiveInputEnabled() )
    {
        m_exclusiveState = !m_exclusiveState;
        m_actions.systemActionSetOnlyEnabled( !m_exclusiveState );
        m_actions.actionSetPriorityToggle( m_exclusiveState );
    }
}

void OverlayController::processRotationBindings()
{
    if ( m_actions.autoTurnToggle() )
    {
        m_rotationTabController.setAutoTurnEnabled(
            !( m_rotationTabController.autoTurnEnabled() ) );
    }
    if ( m_actions.addAutoAlignPointLeft() )
    {
        m_rotationTabController.addAutoAlignPoint( false );
    }
    if ( m_actions.addAutoAlignPointRight() )
    {
        m_rotationTabController.addAutoAlignPoint( true );
    }
}
/*!
Checks if an action has been activated and dispatches the related action if
it has been.
*/
void OverlayController::processInputBindings()
{
    processExclusiveInputBinding();

    processMediaKeyBindings();

    processMotionBindings();

    processPushToTalkBindings();

    processChaperoneBindings();

    processKeyboardBindings();

    processRotationBindings();
}

bool OverlayController::exclusiveInputEnabled() const
{
    return settings::getSetting(
        settings::BoolSetting::APPLICATION_enableExclusiveInput );
}

void OverlayController::setExclusiveInputEnabled( bool value, bool notify )
{
    settings::setSetting(
        settings::BoolSetting::APPLICATION_enableExclusiveInput, value );
    // Note: These Calls technically modify data that could be accessed
    // elsewhere and cause threading issues. Since this should only be
    // accessible in dashboard, and input is disabled while dashboard is up,
    // there should be no reason to lock it. If it becomes an issue this needs
    // to be locked with a mutex or similiar against the
    // processexclusivebindings
    if ( value )
    {
        // Re-Enable Required SteamVR key every toggle just in case.
        ovr_settings_wrapper::setBool(
            vr::k_pch_SteamVR_Section,
            vr::k_pch_SteamVR_AllowGlobalActionSetPriority,
            value );

        // To setup Exclusive Input OVRAS actions should be off besides System
        // (+haptics) which are always on.
        m_actions.systemActionSetOnlyEnabled( true );
    }
    else
    {
        // Enable All Action Sets
        m_actions.systemActionSetOnlyEnabled( false );
        // Remove All priority
        m_actions.actionSetPriorityToggle( false );
    }
    if ( notify )
    {
        emit exclusiveInputEnabledChanged( value );
    }
}

bool OverlayController::crashRecoveryDisabled() const
{
    return settings::getSetting(
        settings::BoolSetting::APPLICATION_crashRecoveryDisabled );
}

void OverlayController::setCrashRecoveryDisabled( bool value, bool notify )
{
    settings::setSetting(
        settings::BoolSetting::APPLICATION_crashRecoveryDisabled, value );
    if ( notify )
    {
        emit crashRecoveryDisabledChanged( value );
    }
}

bool OverlayController::vsyncDisabled() const
{
    return settings::getSetting(
        settings::BoolSetting::APPLICATION_vsyncDisabled );
}

void OverlayController::setVsyncDisabled( bool value, bool notify )
{
    settings::setSetting( settings::BoolSetting::APPLICATION_vsyncDisabled,
                          value );
    if ( notify )
    {
        emit vsyncDisabledChanged( value );
    }
}

bool OverlayController::enableDebug() const
{
    return settings::getSetting(
        settings::BoolSetting::APPLICATION_enableDebug );
}

void OverlayController::setEnableDebug( bool value, bool notify )
{
    settings::setSetting( settings::BoolSetting::APPLICATION_enableDebug,
                          value );
    if ( notify )
    {
        emit enableDebugChanged( value );
    }
}

bool OverlayController::disableVersionCheck() const
{
    return settings::getSetting(
        settings::BoolSetting::APPLICATION_disableVersionCheck );
}

void OverlayController::setDisableVersionCheck( bool value, bool notify )
{
    if ( !value )
    {
        QNetworkRequest netRequest;
        netRequest.setUrl( QUrl( application_strings::versionCheckUrl ) );
        netManager->get( netRequest );
    }
    settings::setSetting(
        settings::BoolSetting::APPLICATION_disableVersionCheck, value );
    if ( notify )
    {
        emit disableVersionCheckChanged( value );
    }
}

bool OverlayController::newVersionDetected() const
{
    return m_newVersionDetected;
}

void OverlayController::setNewVersionDetected( bool value, bool notify )
{
    if ( m_newVersionDetected == value )
    {
        return;
    }
    m_newVersionDetected = value;
    if ( notify )
    {
        emit newVersionDetectedChanged( m_newVersionDetected );
    }
}

QString OverlayController::versionCheckText() const
{
    return m_versionCheckText;
}

void OverlayController::setVersionCheckText( QString value, bool notify )
{
    m_versionCheckText = value;
    LOG( INFO ) << "m_versionCheckText = " << m_versionCheckText;
    if ( notify )
    {
        emit versionCheckTextChanged( m_versionCheckText );
    }
}

int OverlayController::debugState() const
{
    return settings::getSetting( settings::IntSetting::APPLICATION_debugState );
}

void OverlayController::setDebugState( int value, bool notify )
{
    settings::setSetting( settings::IntSetting::APPLICATION_debugState, value );

    if ( notify )
    {
        emit debugStateChanged( value );
    }
}

void OverlayController::setPreviousShutdownSafe( bool value )
{
    settings::setSetting(
        settings::BoolSetting::APPLICATION_previousShutdownSafe, value );
}

int OverlayController::customTickRateMs() const
{
    return m_verifiedCustomTickRateMs;
}

void OverlayController::setCustomTickRateMs( int value, bool notify )
{
    const auto verifiedTickRate = verifyCustomTickRate( value );

    settings::setSetting( settings::IntSetting::APPLICATION_customTickRateMs,
                          verifiedTickRate );
    m_verifiedCustomTickRateMs = verifiedTickRate;

    if ( notify )
    {
        emit customTickRateMsChanged( verifiedTickRate );
    }
}

// vsync implementation:
// this function triggers every 1ms
// this function should remain lightweight and only check if it's time to
// run mainEventLoop() or not.
void OverlayController::OnTimeoutPumpEvents()
{
    if ( vsyncDisabled() )
    {
        // check if it's time for a custom tick rate tick
        if ( m_customTickRateCounter > customTickRateMs() )
        {
            mainEventLoop();
            m_customTickRateCounter = 0;
            updateRate.incrementCounter();
        }
        else
        {
            m_customTickRateCounter++;
        }
    }

    // vsync is enabled
    else
    {
        // get the current frame number from the VRSystem frame counter
        vr::VRSystem()->GetTimeSinceLastVsync( nullptr, &m_currentFrame );

        // Check if we are in the next frame yet
        if ( m_currentFrame > m_lastFrame )
        {
            // If the frame has advanced since last check, it's time for our
            // main event loop. (this function should trigger about every
            // 11ms assuming 90fps compositor)
            mainEventLoop();
            updateRate.incrementCounter();

            // wait for the next frame after executing our main event loop
            // once.
            m_lastFrame = m_currentFrame;
            m_vsyncTooLateCounter = 0;
        }
        else if ( m_vsyncTooLateCounter >= k_nonVsyncTickRate )
        {
            mainEventLoop();
            // m_lastFrame = m_currentFrame + 1 skips the next vsync frame
            // in case it was just about to trigger, to prevent double
            // updates faster than 11ms.
            m_lastFrame = m_currentFrame + 1;
            m_vsyncTooLateCounter = 0;
        }
        else
        {
            m_vsyncTooLateCounter++;
        }
    }
}

void OverlayController::mainEventLoop()
{
    if ( !vr::VRSystem() )
        return;

    m_actions.UpdateStates();

    processInputBindings();

    vr::VREvent_t vrEvent;
    bool chaperoneDataAlreadyUpdated = false;
    while ( pollNextEvent( m_ulOverlayHandle, &vrEvent ) )
    {
        switch ( vrEvent.eventType )
        {
        case vr::VREvent_MouseMove:
        {
            QPoint ptNewMouse = getMousePositionForEvent( vrEvent.data.mouse );
            if ( ptNewMouse != m_ptLastMouse )
            {
                QMouseEvent mouseEvent( QEvent::MouseMove,
                                        ptNewMouse,
                                        m_window.mapToGlobal( ptNewMouse ),
                                        Qt::NoButton,
                                        m_lastMouseButtons,
                                        nullptr );
                m_ptLastMouse = ptNewMouse;
                QCoreApplication::sendEvent( &m_window, &mouseEvent );
                OnRenderRequest();
            }
        }
        break;

        case vr::VREvent_MouseButtonDown:
        {
            QPoint ptNewMouse = getMousePositionForEvent( vrEvent.data.mouse );
            Qt::MouseButton button
                = vrEvent.data.mouse.button == vr::VRMouseButton_Right
                      ? Qt::RightButton
                      : Qt::LeftButton;
            m_lastMouseButtons |= button;
            QMouseEvent mouseEvent( QEvent::MouseButtonPress,
                                    ptNewMouse,
                                    m_window.mapToGlobal( ptNewMouse ),
                                    button,
                                    m_lastMouseButtons,
                                    nullptr );
            QCoreApplication::sendEvent( &m_window, &mouseEvent );
        }
        break;

        case vr::VREvent_MouseButtonUp:
        {
            QPoint ptNewMouse = getMousePositionForEvent( vrEvent.data.mouse );
            Qt::MouseButton button
                = vrEvent.data.mouse.button == vr::VRMouseButton_Right
                      ? Qt::RightButton
                      : Qt::LeftButton;
            m_lastMouseButtons &= ~button;
            QMouseEvent mouseEvent( QEvent::MouseButtonRelease,
                                    ptNewMouse,
                                    m_window.mapToGlobal( ptNewMouse ),
                                    button,
                                    m_lastMouseButtons,
                                    nullptr );
            QCoreApplication::sendEvent( &m_window, &mouseEvent );
        }
        break;

        case vr::VREvent_ScrollSmooth:
        {
            // Wheel speed is defined as 1/8 of a degree
            QWheelEvent wheelEvent(
                m_ptLastMouse,
                m_window.mapToGlobal( m_ptLastMouse ),
                QPoint(),
                QPoint( static_cast<int>( vrEvent.data.scroll.xdelta
                                          * ( 360.0f * 8.0f ) ),
                        static_cast<int>( vrEvent.data.scroll.ydelta
                                          * ( 360.0f * 8.0f ) ) ),
                0,
                Qt::Vertical,
                m_lastMouseButtons,
                nullptr );
            QCoreApplication::sendEvent( &m_window, &wheelEvent );
        }
        break;

        case vr::VREvent_OverlayShown:
        {
            m_window.update();
        }
        break;

        case vr::VREvent_Quit:
        {
            LOG( INFO ) << "Received quit request.";
            vr::VRSystem()->AcknowledgeQuit_Exiting(); // Let us buy some
                                                       // time just in case

            exitApp();
            // Won't fallthrough, but also exitApp() wont, but QT won't
            // acknowledge
            exit( EXIT_SUCCESS );
        }

        case vr::VREvent_DashboardActivated:
        {
            LOG( DEBUG ) << "Dashboard activated";
            m_dashboardVisible = true;
        }
        break;

        case vr::VREvent_DashboardDeactivated:
        {
            LOG( DEBUG ) << "Dashboard deactivated";
            m_dashboardVisible = false;
            settings::saveChangedSettings();
        }
        break;

        case vr::VREvent_KeyboardDone:
        {
            char keyboardBuffer[1024];
            vr::VROverlay()->GetKeyboardText( keyboardBuffer, 1024 );
            emit keyBoardInputSignal( QString( keyboardBuffer ),
                                      static_cast<unsigned long>(
                                          vrEvent.data.keyboard.uUserValue ) );
        }
        break;

        case vr::VREvent_SeatedZeroPoseReset:
        {
            m_moveCenterTabController.incomingSeatedReset();
            LOG( INFO ) << "Game Triggered Seated Zero-Position Reset";
        }
        break;

        // Multiple ChaperoneUniverseHasChanged are often emitted at the
        // same time (some with a little bit of delay) There is no sure way
        // to recognize redundant events, we can only exclude redundant
        // events during the same call of OnTimeoutPumpEvents() INFO Removed
        // logging on play space mover for possible crashing issues.
        case vr::VREvent_ChaperoneUniverseHasChanged:
        {
            uint64_t previousUniverseId
                = vrEvent.data.chaperone.m_nPreviousUniverse;
            uint64_t currentUniverseId
                = vrEvent.data.chaperone.m_nCurrentUniverse;
            LOG( INFO ) << "(VREvent) ChaperoneUniverseHasChanged... Previous:"
                        << previousUniverseId
                        << " Current:" << currentUniverseId;

            if ( !chaperoneDataAlreadyUpdated )
            {
                // LOG(INFO) << "Re-loading chaperone data ...";
                m_chaperoneUtils.loadChaperoneData();
                // LOG(INFO) << "Found " << m_chaperoneUtils.quadsCount() <<
                // " chaperone quads."; if
                // (m_chaperoneUtils.isChaperoneWellFormed()) { LOG(INFO) <<
                // "Chaperone data seems to be well-formed.";
                //} else {
                // LOG(INFO) << "Chaperone data is NOT well-formed.";
                //}
                chaperoneDataAlreadyUpdated = true;
            }
        }
        break;

        case vr::VREvent_ChaperoneDataHasChanged:
        {
            if ( !chaperoneDataAlreadyUpdated )
            {
                // LOG(INFO) << "Re-loading chaperone data ...";
                m_chaperoneUtils.loadChaperoneData();
                // LOG(INFO) << "Found " << m_chaperoneUtils.quadsCount() <<
                // " chaperone quads."; if
                // (m_chaperoneUtils.isChaperoneWellFormed()) { LOG(INFO) <<
                // "Chaperone data seems to be well-formed.";
                //} else {
                // LOG(INFO) << "Chaperone data is NOT well-formed.";
                //}
                chaperoneDataAlreadyUpdated = true;
            }
        }
        break;
        }
    }

    vr::TrackedDevicePose_t devicePoses[vr::k_unMaxTrackedDeviceCount];
    vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(
        vr::TrackingUniverseStanding,
        0.0f,
        devicePoses,
        vr::k_unMaxTrackedDeviceCount );

    // HMD/Controller Velocities
    auto leftId = vr::VRSystem()->GetTrackedDeviceIndexForControllerRole(
        vr::TrackedControllerRole_LeftHand );
    float leftSpeed = 0.0f;
    if ( leftId != vr::k_unTrackedDeviceIndexInvalid
         && devicePoses[leftId].bPoseIsValid
         && devicePoses[leftId].eTrackingResult
                == vr::TrackingResult_Running_OK )
    {
        auto& vel = devicePoses[leftId].vVelocity.v;
        leftSpeed
            = std::sqrt( vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2] );
    }
    auto rightId = vr::VRSystem()->GetTrackedDeviceIndexForControllerRole(
        vr::TrackedControllerRole_RightHand );
    auto rightSpeed = 0.0f;
    if ( rightId != vr::k_unTrackedDeviceIndexInvalid
         && devicePoses[rightId].bPoseIsValid
         && devicePoses[rightId].eTrackingResult
                == vr::TrackingResult_Running_OK )
    {
        auto& vel = devicePoses[rightId].vVelocity.v;
        rightSpeed
            = std::sqrt( vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2] );
    }
    auto universe = vr::VRCompositor()->GetTrackingSpace();

    m_moveCenterTabController.eventLoopTick( universe, devicePoses );
    m_utilitiesTabController.eventLoopTick();
    m_statisticsTabController.eventLoopTick(
        devicePoses, leftSpeed, rightSpeed );
    m_chaperoneTabController.eventLoopTick( universe, devicePoses );
    m_audioTabController.eventLoopTick();
    m_rotationTabController.eventLoopTick( devicePoses );

    m_alarm.eventLoopTick();

    if ( vr::VROverlay()->IsDashboardVisible() )
    {
        m_settingsTabController.dashboardLoopTick();
        m_steamVRTabController.dashboardLoopTick();
        m_fixFloorTabController.dashboardLoopTick( devicePoses );
        m_videoTabController.dashboardLoopTick();
        m_chaperoneTabController.dashboardLoopTick();
    }

    if ( m_ulOverlayThumbnailHandle != vr::k_ulOverlayHandleInvalid )
    {
        while ( vr::VROverlay()->PollNextOverlayEvent(
            m_ulOverlayThumbnailHandle, &vrEvent, sizeof( vrEvent ) ) )
        {
            switch ( vrEvent.eventType )
            {
            case vr::VREvent_OverlayShown:
            {
                m_window.update();
            }
            break;
            }
        }
    }
}

void OverlayController::AddOffsetToUniverseCenter(
    vr::ETrackingUniverseOrigin universe,
    unsigned axisId,
    float offset,
    bool adjustBounds,
    bool commit )
{
    float offsetArray[3] = { 0, 0, 0 };
    offsetArray[axisId] = offset;
    AddOffsetToUniverseCenter( universe, offsetArray, adjustBounds, commit );
}

void OverlayController::AddOffsetToUniverseCenter(
    vr::ETrackingUniverseOrigin universe,
    float offset[3],
    bool adjustBounds,
    bool commit )
{
    if ( offset[0] != 0.0f || offset[1] != 0.0f || offset[2] != 0.0f )
    {
        if ( commit )
        {
            vr::VRChaperoneSetup()->HideWorkingSetPreview();
            vr::VRChaperoneSetup()->RevertWorkingCopy();
        }
        vr::HmdMatrix34_t curPos;
        if ( universe == vr::TrackingUniverseStanding )
        {
            vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(
                &curPos );
        }
        else
        {
            vr::VRChaperoneSetup()->GetWorkingSeatedZeroPoseToRawTrackingPose(
                &curPos );
        }
        for ( int i = 0; i < 3; i++ )
        {
            curPos.m[0][3] += curPos.m[0][i] * offset[i];
            curPos.m[1][3] += curPos.m[1][i] * offset[i];
            curPos.m[2][3] += curPos.m[2][i] * offset[i];
        }
        if ( universe == vr::TrackingUniverseStanding )
        {
            vr::VRChaperoneSetup()->SetWorkingStandingZeroPoseToRawTrackingPose(
                &curPos );
        }
        else
        {
            vr::VRChaperoneSetup()->SetWorkingSeatedZeroPoseToRawTrackingPose(
                &curPos );
        }
        if ( adjustBounds && universe == vr::TrackingUniverseStanding )
        {
            float collisionOffset[] = { -offset[0], -offset[1], -offset[2] };
            AddOffsetToCollisionBounds( collisionOffset, false );
        }
        if ( commit )
        {
            vr::VRChaperoneSetup()->CommitWorkingCopy(
                vr::EChaperoneConfigFile_Live );
        }
    }
}

void OverlayController::RotateUniverseCenter(
    vr::ETrackingUniverseOrigin universe,
    float yAngle,
    bool adjustBounds,
    bool commit )
{
    if ( yAngle != 0.0f )
    {
        if ( commit )
        {
            vr::VRChaperoneSetup()->HideWorkingSetPreview();
            vr::VRChaperoneSetup()->RevertWorkingCopy();
        }
        vr::HmdMatrix34_t curPos;
        if ( universe == vr::TrackingUniverseStanding )
        {
            vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(
                &curPos );
        }
        else
        {
            vr::VRChaperoneSetup()->GetWorkingSeatedZeroPoseToRawTrackingPose(
                &curPos );
        }

        vr::HmdMatrix34_t rotMat;
        vr::HmdMatrix34_t newPos;
        utils::initRotationMatrix( rotMat, 1, yAngle );
        utils::matMul33( newPos, rotMat, curPos );
        newPos.m[0][3] = curPos.m[0][3];
        newPos.m[1][3] = curPos.m[1][3];
        newPos.m[2][3] = curPos.m[2][3];
        if ( universe == vr::TrackingUniverseStanding )
        {
            vr::VRChaperoneSetup()->SetWorkingStandingZeroPoseToRawTrackingPose(
                &newPos );
        }
        else
        {
            vr::VRChaperoneSetup()->SetWorkingSeatedZeroPoseToRawTrackingPose(
                &newPos );
        }
        if ( adjustBounds && universe == vr::TrackingUniverseStanding )
        {
            RotateCollisionBounds( -yAngle, false );
        }
        if ( commit )
        {
            vr::VRChaperoneSetup()->CommitWorkingCopy(
                vr::EChaperoneConfigFile_Live );
        }
    }
}

void OverlayController::AddOffsetToCollisionBounds( unsigned axisId,
                                                    float offset,
                                                    bool commit )
{
    float offsetArray[3] = { 0, 0, 0 };
    offsetArray[axisId] = offset;
    AddOffsetToCollisionBounds( offsetArray, commit );
}

void OverlayController::AddOffsetToCollisionBounds( float offset[3],
                                                    bool commit )
{
    // Apparently Valve sanity-checks the y-coordinates of the collision
    // bounds (and only the y-coordinates) I can move the bounds on the
    // xz-plane, I can make the "ceiling" of the chaperone cage
    // lower/higher, but when I dare to set one single lower corner to
    // something non-zero, every corner gets its y-coordinates reset to the
    // defaults.
    if ( commit )
    {
        vr::VRChaperoneSetup()->HideWorkingSetPreview();
        vr::VRChaperoneSetup()->RevertWorkingCopy();
    }
    unsigned collisionBoundsCount = 0;
    vr::VRChaperoneSetup()->GetWorkingCollisionBoundsInfo(
        nullptr, &collisionBoundsCount );
    if ( collisionBoundsCount > 0 )
    {
        vr::HmdQuad_t* collisionBounds
            = new vr::HmdQuad_t[collisionBoundsCount];
        vr::VRChaperoneSetup()->GetWorkingCollisionBoundsInfo(
            collisionBounds, &collisionBoundsCount );
        for ( unsigned b = 0; b < collisionBoundsCount; b++ )
        {
            for ( unsigned c = 0; c < 4; c++ )
            {
                collisionBounds[b].vCorners[c].v[0] += offset[0];

                // keep the lower corners on the ground so it doesn't reset
                // all y cooridinates. this causes the caperone to "grow" up
                // instead of not moving up at all. note that Valve still
                // forces a minimum height so we can't go into the ground
                if ( collisionBounds[b].vCorners[c].v[1] != 0 )
                {
                    collisionBounds[b].vCorners[c].v[1] += offset[1];
                }
                collisionBounds[b].vCorners[c].v[2] += offset[2];
            }
        }
        vr::VRChaperoneSetup()->SetWorkingCollisionBoundsInfo(
            collisionBounds, collisionBoundsCount );
        delete[] collisionBounds;
    }
    if ( commit && collisionBoundsCount > 0 )
    {
        vr::VRChaperoneSetup()->CommitWorkingCopy(
            vr::EChaperoneConfigFile_Live );
    }
}

void OverlayController::RotateCollisionBounds( float angle, bool commit )
{
    if ( commit )
    {
        vr::VRChaperoneSetup()->HideWorkingSetPreview();
        vr::VRChaperoneSetup()->RevertWorkingCopy();
    }
    unsigned collisionBoundsCount = 0;
    vr::VRChaperoneSetup()->GetWorkingCollisionBoundsInfo(
        nullptr, &collisionBoundsCount );
    if ( collisionBoundsCount > 0 )
    {
        vr::HmdQuad_t* collisionBounds
            = new vr::HmdQuad_t[collisionBoundsCount];
        vr::VRChaperoneSetup()->GetWorkingCollisionBoundsInfo(
            collisionBounds, &collisionBoundsCount );

        vr::HmdMatrix34_t rotMat;
        utils::initRotationMatrix( rotMat, 1, angle );
        for ( unsigned b = 0; b < collisionBoundsCount; b++ )
        {
            for ( unsigned c = 0; c < 4; c++ )
            {
                auto& corner = collisionBounds[b].vCorners[c];
                vr::HmdVector3_t newVal;
                utils::matMul33( newVal, rotMat, corner );
                corner = newVal;
            }
        }
        vr::VRChaperoneSetup()->SetWorkingCollisionBoundsInfo(
            collisionBounds, collisionBoundsCount );
        delete[] collisionBounds;
    }
    if ( commit && collisionBoundsCount > 0 )
    {
        vr::VRChaperoneSetup()->CommitWorkingCopy(
            vr::EChaperoneConfigFile_Live );
    }
}

bool OverlayController::isPreviousShutdownSafe()
{
    return settings::getSetting(
        settings::BoolSetting::APPLICATION_previousShutdownSafe );
}

QString OverlayController::getVersionString()
{
    return QString( application_strings::applicationVersionString );
}

QUrl OverlayController::getVRRuntimePathUrl()
{
    return m_runtimePathUrl;
}

bool OverlayController::soundDisabled()
{
    return m_noSound;
}

const vr::VROverlayHandle_t& OverlayController::overlayHandle()
{
    return m_ulOverlayHandle;
}

const vr::VROverlayHandle_t& OverlayController::overlayThumbnailHandle()
{
    return m_ulOverlayThumbnailHandle;
}

void OverlayController::showKeyboard( QString existingText,
                                      unsigned long userValue )
{
    vr::VROverlay()->ShowKeyboardForOverlay(
        m_ulOverlayHandle,
        vr::k_EGamepadTextInputModeNormal,
        vr::k_EGamepadTextInputLineModeSingleLine,
        0,
        "Advanced Settings Overlay",
        1024,
        existingText.toStdString().c_str(),
        userValue );
    setKeyboardPos();
}

void OverlayController::setKeyboardPos()
{
    vr::HmdVector2_t emptyvec;
    emptyvec.v[0] = 0;
    emptyvec.v[1] = 0;
    vr::HmdRect2_t empty;
    empty.vTopLeft = emptyvec;
    empty.vBottomRight = emptyvec;
    vr::VROverlay()->SetKeyboardPositionForOverlay( m_ulOverlayHandle, empty );
}

void OverlayController::playActivationSound()
{
    if ( !m_noSound )
    {
        m_activationSoundEffect.play();
    }
}

void OverlayController::playFocusChangedSound()
{
    if ( !m_noSound )
    {
        m_focusChangedSoundEffect.play();
    }
}

void OverlayController::playAlarm01Sound( bool loop )
{
    if ( !m_noSound && !m_alarm01SoundEffect.isPlaying() )
    {
        if ( loop )
        {
            m_alarm01SoundEffect.setLoopCount( QSoundEffect::Infinite );
        }
        else
        {
            m_alarm01SoundEffect.setLoopCount( 1 );
        }
        m_alarm01SoundEffect.play();
    }
}

void OverlayController::setAlarm01SoundVolume( float vol )
{
    m_alarm01SoundEffect.setVolume( static_cast<double>( vol ) );
}

void OverlayController::cancelAlarm01Sound()
{
    m_alarm01SoundEffect.stop();
}

void OverlayController::OnNetworkReply( QNetworkReply* reply )
{
    if ( reply->error() == QNetworkReply::NoError )
    {
        QByteArray replyByteData = reply->readAll();
        LOG( INFO ) << "Version Check: Recieved Data: " << replyByteData;
        QJsonParseError parseError;
        m_remoteVersionJsonDocument
            = QJsonDocument::fromJson( replyByteData, &parseError );
        if ( parseError.error == QJsonParseError::NoError )
        {
            m_remoteVersionJsonObject = m_remoteVersionJsonDocument.object();
            m_remoteVersionMajor
                = m_remoteVersionJsonObject.value( "major" ).toInt();
            m_remoteVersionMinor
                = m_remoteVersionJsonObject.value( "minor" ).toInt();
            m_remoteVersionPatch
                = m_remoteVersionJsonObject.value( "patch" ).toInt();
            m_updateMessage
                = m_remoteVersionJsonObject.value( "updateMessage" ).toString();
            m_optionalMessage
                = m_remoteVersionJsonObject.value( "optionalMessage" )
                      .toString();

            // this is a little convoluted to ensure if our local version is
            // somehow higher than remote, it doesn't detect an update from
            // just higher remote "minor" or "patch" values.
            if ( ( m_remoteVersionMajor > m_localVersionMajor )
                 || ( m_remoteVersionMajor == m_localVersionMajor
                      && m_remoteVersionMinor > m_localVersionMinor )
                 || ( m_remoteVersionMajor == m_localVersionMajor
                      && m_remoteVersionMinor == m_localVersionMinor
                      && m_remoteVersionPatch > m_localVersionPatch ) )
            {
                setNewVersionDetected( true );
                if ( m_updateMessage.length() > 0 )
                {
                    setVersionCheckText( m_updateMessage );
                }
                else
                {
                    setVersionCheckText(
                        "Newer version ("
                        + QString::number( m_remoteVersionMajor ) + "."
                        + QString::number( m_remoteVersionMinor ) + "."
                        + QString::number( m_remoteVersionPatch )
                        + ") available." );
                }
                LOG( INFO )
                    << "Version Check: Newer version (" << m_remoteVersionMajor
                    << "." << m_remoteVersionMinor << "."
                    << m_remoteVersionPatch << ") available.";
            }
            else
            {
                if ( m_optionalMessage.length() > 0 )
                {
                    setVersionCheckText( m_optionalMessage );
                }
                setNewVersionDetected( false );
                LOG( INFO ) << "Version Check: Installed version is latest "
                               "release.";
            }
        }
        else
        {
            LOG( ERROR )
                << "Version Check: Error parsing json. QJsonParseError = "
                << parseError.error;
        }
    }
    else
    {
        LOG( ERROR ) << "Version Check: Error connecting to network. "
                        "QNetworkReply::NetworkError = "
                     << reply->error();
    }
    reply->deleteLater();
}

} // namespace advsettings
