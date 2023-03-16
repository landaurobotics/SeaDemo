import QtQuick
import QtQuick3D
Node {
    id: scene
    rotation: Qt.quaternion(0.707107, -0.707107, 0, 0)
    scale: Qt.vector3d(1000, 1000, 1000)
    Model {
        id: sDJ_20_36
        source: "sdj_20.mesh"

        DefaultMaterial {
            id: ___material
            diffuseColor: "#ffcccccc"
//            diffuseColor: "#ff000000"

        }
        materials: [
            ___material
        ]
    }
}
