import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.3
import ovras.advsettings 1.0
import "../../common"

GroupBox {
    id: redirectedGroupBox
    Layout.fillWidth: true

    label: MyText {
        leftPadding: 10
        text: "Redirected Walking"
        bottomPadding: -10
    }
    background: Rectangle {
        color: "transparent"
        border.color: "#ffffff"
        radius: 8
    }

    ColumnLayout {
        anchors.fill: parent

        Rectangle {
            color: "#ffffff"
            height: 1
            Layout.fillWidth: true
            Layout.bottomMargin: 5
        }

        RowLayout {
            Layout.fillWidth: true

            MyToggleButton {
                id: autoTurn
                text: "Toggle On/Off"
                onCheckedChanged: {
                    RotationTabController.setAutoTurnEnabled(this.checked, true);                }
            }

            Item {
                Layout.preferredWidth: 150
            }

            MyText {
                text: "Activation Dist:"
                horizontalAlignment: Text.AlignRight
                Layout.rightMargin: 10
            }

            MySlider {
                id: activationSlider
                from: 0
                to: 1.0
                stepSize: 0.1
                value: .4
                Layout.fillWidth: true
                onPositionChanged: {
                    var val = this.value
                    activationValueText.text = val.toFixed(2)
                }
                onValueChanged: {

                    RotationTabController.setAutoTurnActivationDistance(value, true)
                }
            }

            MyTextField {
                id: activationValueText
                text: "0.4"
                keyBoardUID: 1001
                Layout.preferredWidth: 100
                Layout.leftMargin: 10
                horizontalAlignment: Text.AlignHCenter
                function onInputEvent(input) {
                    var val = parseFloat(input)
                    if (!isNaN(val)) {
                        if (val < 0) {
                            val = 0
                        } else if (val > 22) {
                            val = 22
                        }
                            activationSlider.value = v;
                    }
                    text =  RotationTabController.autoTurnActivationDistance;
                }
            }
        }
        RowLayout {
            Layout.fillWidth: true

            MyToggleButton {
                id: cornerAngle
                text: "Use Corner Angle"
                onCheckedChanged: {
                    RotationTabController.setAutoTurnUseCornerAngle(this.checked, true);                }
            }

            Item {
                Layout.preferredWidth: 150
            }

            MyText {
                text: "DeActivation Dist:"
                horizontalAlignment: Text.AlignRight
                Layout.rightMargin: 10
            }

            MySlider {
                id: deactivationSlider
                from: 0
                to: 1.0
                stepSize: 0.1
                value: .15
                Layout.fillWidth: true
                onPositionChanged: {
                    var val = this.value
                    deactivationValueText.text = val.toFixed(2)
                }
                onValueChanged: {

                    RotationTabController.setAutoTurnDeactivationDistance(value, true)
                }
            }

            MyTextField {
                id: deactivationValueText
                text: "0.15"
                keyBoardUID: 1002
                Layout.preferredWidth: 100
                Layout.leftMargin: 10
                horizontalAlignment: Text.AlignHCenter
                function onInputEvent(input) {
                    var val = parseFloat(input)
                    if (!isNaN(val)) {
                        if (val < 0) {
                            val = 0
                        } else if (val > 22) {
                            val = 22
                        }
                            deactivationSlider.value = v;
                    }
                    text =  RotationTabController.autoTurnDeactivationDistance;
                }
            }
        }

        RowLayout {
            MyToggleButton {
                id: autoTurnModeToggle
                text: "Use Smooth Turn"
                onCheckedChanged: {
                    if(this.checked){

                        RotationTabController.setAutoTurnMode(1, true);
                    }else{
                        RotationTabController.setAutoTurnMode(0, true);
                    }

                }
            }

            Item {
                Layout.preferredWidth: 150
            }
            Layout.fillWidth: true
            MyText {
                text: "Turn Speed (deg/sec):"
                horizontalAlignment: Text.AlignRight
                Layout.rightMargin: 10
            }

            MySlider {
                id: speedSlider
                from: 0
                to: 360
                stepSize: 1
                value: 90
                Layout.fillWidth: true
                onPositionChanged: {
                    var val = this.value
                    speedValueText.text = val.toFixed()
                }
                onValueChanged: {

                    RotationTabController.setAutoTurnSpeed(value, true)
                }
            }

            MyTextField {
                id: speedValueText
                text: "90"
                keyBoardUID: 1003
                Layout.preferredWidth: 100
                Layout.leftMargin: 10
                horizontalAlignment: Text.AlignHCenter
                function onInputEvent(input) {
                    var val = parseFloat(input)
                    if (!isNaN(val)) {
                        if (val < 1) {
                            val = 1
                        //caps user set Speed to 1000 deg/Second
                        //used as overflow prevention
                        } else if (val > 1000) {
                            val = 1000
                        }
                            speedSlider.value = v;
                    }
                    //converts the centidegrees to degrees
                    text =  (((RotationTabController.autoTurnSpeed)/100).toFixed());
                }
            }


        }
        RowLayout{
            MyToggleButton {
                id: redirectedModeToggle
                text: "Toggle Re-directed Walking: "
                onCheckedChanged: {
                    RotationTabController.setVestibularMotionEnabled(this.checked, true);

                }
            }
            MyText{
                text: "Redirected Walking Radius: "
                horizontalAlignment: Text.AlignRight
                Layout.rightMargin: 10

            }
            MyTextField {
                id: redirectedWalkingRadiusText
                text: "11.0"
                keyBoardUID: 1004
                Layout.preferredWidth: 100
                Layout.leftMargin: 10
                horizontalAlignment: Text.AlignHCenter
                function onInputEvent(input) {
                    var val = parseFloat(input)
                    if (!isNaN(val)) {
                        if (val < 0.5) {
                            val = 0.5
                        } else if (val > 50) {
                            val = 50
                        }
                            RotationTabController.setVestibularMotionRadius(val, true);
                    }
                    text =  ((RotationTabController.vestibularMotionRadius).toFixed(2));
                }
            }
        }


    }

    Component.onCompleted: {
        activationSlider.value = RotationTabController.autoTurnActivationDistance
        autoTurn.checked = RotationTabController.autoTurnEnabled
        deactivationSlider.value = RotationTabController.autoTurnDeactivationDistance
        cornerAngle.checked = RotationTabController.autoTurnUseCornerAngle
        speedSlider.value = (RotationTabController.autoTurnSpeed*100).toFixed()
        if(RotationTabController.autoTurnMode === 1){

            autoTurnModeToggle.checked = true;

        }else{

             autoTurnModeToggle.checked = false;
        }
        redirectedWalkingRadiusText.text = ((RotationTabController.vestibularMotionRadius).toFixed(2))
        redirectedModeToggle.checked = RotationTabController.vestibularMotionEnabled

    }

    Connections {
        target: RotationTabController

        onAutoTurnEnabledChanged: {
            autoTurn.checked = RotationTabController.autoTurnEnabled
        }
        onAutoTurnActivationDistanceChanged: {
            activationSlider.value = RotationTabController.autoTurnActivationDistance
        }
        onAutoTurnDeactivationDistanceChanged:{
            deactivationSlider.value = RotationTabController.autoTurnDeactivationDistance
        }
        onAutoTurnUseCornerAngleChanged:{
            cornerAngle.checked = RotationTabController.autoTurnUseCornerAngle
        }
        onAutoTurnSpeedChanged:{
            var val = RotationTabController.autoTurnSpeed
            speedSlider.value = ((val/100).toFixed())
        }
        onAutoTurnModeChanged:{
            if(RotationTabController.autoTurnMode === 1){

                autoTurnModeToggle.checked = true;

            }else{

                 autoTurnModeToggle.checked = false;
            }
        }
        onVestibularMotionEnabledChanged:{
         redirectedModeToggle.checked = RotationTabController.vestibularMotionEnabled
        }
        onVestibularMotionRadiusChanged:{
            redirectedWalkingRadiusText.text =  ((RotationTabController.vestibularMotionRadius).toFixed(2))
        }
    }
}