import QtQuick
import QtQuick3D
import TerrainLayout 1.0

Item {
    id: root
    width: 1600
    height: 900

    Rectangle {
        anchors.fill: parent
        gradient: Gradient {
            GradientStop { position: 0.0; color: "#02060d" }
            GradientStop { position: 0.18; color: "#07131d" }
            GradientStop { position: 0.28; color: "#164357" }
            GradientStop { position: 0.40; color: "#0c1b25" }
            GradientStop { position: 1.0; color: "#101c26" }
        }
    }

    View3D {
        anchors.fill: parent
        environment: SceneEnvironment {
            backgroundMode: SceneEnvironment.Transparent
            clearColor: "transparent"
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.VeryHigh
            fog: Fog {
                enabled: true
                color: "#081723"
                density: 0.18
                depthEnabled: true
                depthNear: 32000
                depthFar: 62000
                depthCurve: 1.05
                heightEnabled: false
            }
        }

        Node {
            id: cameraPivot
            position: Qt.vector3d(0, 980, 0)
            eulerRotation: Qt.vector3d(-43, -42, 0)

            PerspectiveCamera {
                position: Qt.vector3d(0, 0, 23200)
                clipNear: 10
                clipFar: 140000
                fieldOfView: 49
            }
        }

        DirectionalLight {
            eulerRotation: Qt.vector3d(-35, -52, 0)
            brightness: 2.45
            color: "#fff0d6"
            castsShadow: false
        }

        DirectionalLight {
            eulerRotation: Qt.vector3d(-68, 132, 0)
            brightness: 0.56
            color: "#4faec6"
            castsShadow: false
        }

        Model {
            geometry: LayoutTerrainGeometry {}
            castsShadows: false
            receivesShadows: false
            materials: PrincipledMaterial {
                baseColor: "#ffffff"
                vertexColorsEnabled: true
                cullMode: Material.NoCulling
                roughness: 0.96
                specularAmount: 0.02
                emissiveFactor: Qt.vector3d(0.014, 0.020, 0.026)
            }
        }

        Model {
            geometry: LayoutLineGeometry {
                kind: "hazardGrid"
                widthValue: 9
            }
            castsShadows: false
            receivesShadows: false
            materials: PrincipledMaterial {
                baseColor: Qt.rgba(1.0, 0.25, 0.03, 0.92)
                alphaMode: PrincipledMaterial.Blend
                opacity: 0.92
                cullMode: Material.NoCulling
                emissiveFactor: Qt.vector3d(1.25, 0.18, 0.02)
                roughness: 0.70
            }
        }

        Model {
            geometry: LayoutLineGeometry {
                kind: "hazardContour"
                widthValue: 12
            }
            castsShadows: false
            receivesShadows: false
            materials: PrincipledMaterial {
                baseColor: Qt.rgba(1.0, 0.07, 0.02, 0.86)
                alphaMode: PrincipledMaterial.Blend
                opacity: 0.88
                cullMode: Material.NoCulling
                emissiveFactor: Qt.vector3d(1.45, 0.10, 0.02)
                roughness: 0.70
            }
        }

        Model {
            geometry: LayoutLineGeometry {
                kind: "buffer"
                widthValue: 22
            }
            castsShadows: false
            receivesShadows: false
            materials: PrincipledMaterial {
                baseColor: Qt.rgba(0.55, 1.0, 1.0, 0.68)
                alphaMode: PrincipledMaterial.Blend
                opacity: 0.68
                cullMode: Material.NoCulling
                emissiveFactor: Qt.vector3d(0.12, 0.70, 0.78)
                roughness: 0.80
            }
        }

        Model {
            geometry: LayoutLineGeometry {
                kind: "original"
                widthValue: 62
            }
            castsShadows: false
            receivesShadows: false
            materials: PrincipledMaterial {
                baseColor: Qt.rgba(1.0, 0.08, 0.03, 0.88)
                alphaMode: PrincipledMaterial.Blend
                opacity: 0.88
                cullMode: Material.NoCulling
                emissiveFactor: Qt.vector3d(1.55, 0.11, 0.02)
                roughness: 0.70
            }
        }

        Model {
            geometry: LayoutLineGeometry {
                kind: "routeGlow"
                widthValue: 280
            }
            castsShadows: false
            receivesShadows: false
            materials: PrincipledMaterial {
                baseColor: Qt.rgba(0.02, 0.95, 1.0, 0.25)
                alphaMode: PrincipledMaterial.Blend
                opacity: 0.36
                cullMode: Material.NoCulling
                emissiveFactor: Qt.vector3d(0.08, 1.05, 1.28)
                roughness: 0.82
            }
        }

        Model {
            geometry: LayoutLineGeometry {
                kind: "routeCore"
                widthValue: 80
            }
            castsShadows: false
            receivesShadows: false
            materials: PrincipledMaterial {
                baseColor: Qt.rgba(0.28, 1.0, 1.0, 0.94)
                alphaMode: PrincipledMaterial.Blend
                opacity: 0.94
                cullMode: Material.NoCulling
                emissiveFactor: Qt.vector3d(0.24, 1.65, 1.85)
                roughness: 0.68
            }
        }

        Model {
            geometry: LayoutLineGeometry {
                kind: "waypoint"
                widthValue: 34
            }
            castsShadows: false
            receivesShadows: false
            materials: PrincipledMaterial {
                baseColor: Qt.rgba(0.58, 1.0, 1.0, 0.82)
                alphaMode: PrincipledMaterial.Blend
                opacity: 0.82
                cullMode: Material.NoCulling
                emissiveFactor: Qt.vector3d(0.18, 1.05, 1.20)
                roughness: 0.70
            }
        }
    }

    Rectangle {
        z: 20
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        height: parent.height * 0.22
        gradient: Gradient {
            GradientStop { position: 0.0; color: "#00000000" }
            GradientStop { position: 0.62; color: "#12384c30" }
            GradientStop { position: 0.82; color: "#1f789160" }
            GradientStop { position: 1.0; color: "#00000000" }
        }
    }
}
