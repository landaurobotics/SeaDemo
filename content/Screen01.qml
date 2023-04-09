

/*
This is a UI file (.ui.qml) that is intended to be edited in Qt Design Studio only.
It is supposed to be strictly declarative and only uses a subset of QML. If you edit
this file manually, you might introduce QML code that is not supported by Qt Design Studio.
Check out https://doc.qt.io/qtcreator/creator-quick-ui-forms.html for details on .ui.qml files.
*/
import QtQuick 6.2
import QtQuick.Controls 6.2
import QtQuick3D 6.2
import SeaProject
//import QtQuick.Studio.Components 1.0
import io.qt.Backend

Rectangle {
    width: parent.width
    height: parent.height

    color: Constants.backgroundColor

    Backend {
        id: back
        impStiff: stiffness_slider.value
        impDamp: damping_slider.value

        onStatusChanged: {
            if(status === 0 ) {
                stat.text = "Stopped"
            }
            else if(status === 1) {
                stat.text = "Running"
            }
            else if(status === 2) {
                stat.text = "Error"
            }
        }
    }

    View3D {
        id: view3D
        anchors.fill: parent

        environment: sceneEnvironment

        SceneEnvironment {
            id: sceneEnvironment
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.High
        }

        Node {
            id: scene
            DirectionalLight {
                id: directionalLight
                x: 172.551
                y: 212.524
                brightness: 1
                eulerRotation.z: -13.82829
                eulerRotation.y: 36.42999
                eulerRotation.x: -38.97548
                z: 201.60832

            }

            PerspectiveCamera {
                id: sceneCamera
                x: 122.1
                y: 59.94
                eulerRotation.z: -0.95995
                eulerRotation.y: 63.98463
                eulerRotation.x: -15.97192
                z: 168.42131
            }

            Model {
                source: "qrc:/content/sdj_20.mesh"
                scale.z: 1000
                scale.y: 1000
                scale.x: 1000
                materials: defaultMaterial

                PropertyAnimation on eulerRotation.y {
                    from: 0
                    to: -360
                    duration: 6000
                    loops: Animation.Infinite
                }

            }
        }
    }

    Item {
        id: __materialLibrary__
        DefaultMaterial {
            id: defaultMaterial
            objectName: "Default Material"
            //            diffuseColor: "#4aee45"
            diffuseColor: "#ffcccccc"
        }
    }

    Image {
        id: image
        anchors.right: parent.right
        anchors.rightMargin: 20
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 20
        x: 1714
        y: 876
        width: 188
        height: 188
        source: "qrc:/content/rocos-logo.png"
        fillMode: Image.PreserveAspectFit
        smooth: true

    }

    Text {
        id: tiltle
        width: 1324
        height: 112
        text: qsTr("S.E.A Compliant Motion Demo")
        anchors.top: parent.top
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        font.capitalization: Font.SmallCaps
        font.italic: false
        font.family: "Verdana"
        font.bold: true
        anchors.horizontalCenterOffset: 0
        font.pointSize: 50
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.topMargin: 20
    }

    Item {
        id: stiffness
        x: 92
        y: 792

        Slider {
            id: stiffness_slider
            x: 212
            y: 8
            width: 672
            height: 48
            to: 30
            value: 15
        }

        Text {
            x: 0
            y: 0
            width: 220
            height: 64
            text: qsTr("Stiffness")
            horizontalAlignment: Text.AlignLeft
            verticalAlignment: Text.AlignVCenter
            font.capitalization: Font.SmallCaps
            font.pointSize: 30
            font.bold: true
            font.family: "Verdana"
            font.italic: false
        }
    }

    Item {
        id: damping
        x: 92
        y: 872
        Slider {
            id: damping_slider
            x: 238
            y: 8
            width: 646
            height: 48
            to: 3
            value: 0.5
        }

        Text {
            x: 0
            y: 0
            width: 220
            height: 64
            text: qsTr("Damping")
            horizontalAlignment: Text.AlignLeft
            verticalAlignment: Text.AlignVCenter
            font.capitalization: Font.SmallCaps
            font.pointSize: 30
            font.bold: true
            font.family: "Verdana"
            font.italic: false
        }
    }

    Item {
        x: 124
        y: 284

        RoundButton {
            id: imp_btn
            x: 0
            y: 0
            width: 380
            height: 376
            text: "Impdent \n Control"
            autoExclusive: true
            checked: true
            display: AbstractButton.TextUnderIcon
            font.bold: true
            font.pointSize: 30
            font.family: "Verdana"
            checkable: true
            radius: 30

            onClicked: back.ctrlMode = 11
        }

        RoundButton {
            id: zero_btn
            x: 460
            y: 0
            width: 380
            height: 376
            text: "Zero \n Force"
            autoExclusive: true
            display: AbstractButton.TextUnderIcon
            font.bold: true
            font.pointSize: 30
            font.family: "Verdana"
            checkable: true
            radius: 30

            onClicked: back.ctrlMode = 22
        }
    }

    Text {
        id: stat
        width: 228
        height: 58
        text: qsTr("Stopped")
        anchors.top: parent.top
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        font.bold: false
        font.pointSize: 20
        font.capitalization: Font.AllUppercase
        font.family: "Verdana"
        font.italic: true
        anchors.horizontalCenterOffset: 516
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.topMargin: 218
    }
}

/*##^##
Designer {
    D{i:0;formeditorZoom:0.5}D{i:4}D{i:5}D{i:6}D{i:3}D{i:13}D{i:14}D{i:20}
}
##^##*/

