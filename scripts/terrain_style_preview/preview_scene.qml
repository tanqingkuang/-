import QtQuick
import QtQuick3D
import TerrainPreview 1.0

Item {
    id: root
    width: 1600
    height: 900

    property string previewStyle: initialPreviewStyle
    property bool styleA: previewStyle === "a"

    Rectangle {
        anchors.fill: parent
        gradient: Gradient {
            GradientStop { position: 0.0; color: "#05080d" }
            GradientStop { position: 0.58; color: "#0b121b" }
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
                color: root.styleA ? "#101c26" : "#07111a"
                density: root.styleA ? 0.42 : 0.58
                depthEnabled: true
                depthNear: root.styleA ? 15000 : 9200
                depthFar: root.styleA ? 28000 : 24500
                depthCurve: root.styleA ? 1.25 : 1.32
                heightEnabled: !root.styleA
                leastIntenseY: 3800
                mostIntenseY: -400
                heightCurve: 0.55
            }
        }

        Node {
            id: cameraPivot
            position: Qt.vector3d(120, 780, 140)
            eulerRotation: Qt.vector3d(-34, -38, 0)

            PerspectiveCamera {
                id: camera
                position: Qt.vector3d(0, 0, 11200)
                clipNear: 10
                clipFar: 80000
                fieldOfView: 50
            }
        }

        DirectionalLight {
            eulerRotation: root.styleA ? Qt.vector3d(-35, -52, 0) : Qt.vector3d(-58, -36, 0)
            brightness: root.styleA ? 1.62 : 0.66
            castsShadow: false
        }

        DirectionalLight {
            eulerRotation: Qt.vector3d(-68, 132, 0)
            brightness: root.styleA ? 0.18 : 0.24
            castsShadow: false
        }

        PointLight {
            position: Qt.vector3d(-5600, 4600, 5000)
            brightness: root.styleA ? 0.55 : 0.45
            color: root.styleA ? "#b8d9ff" : "#2fd6c7"
        }

        Model {
            geometry: TerrainPreviewGeometry {
                styleName: root.previewStyle
            }
            castsShadows: false
            receivesShadows: false
            materials: PrincipledMaterial {
                baseColor: "#ffffff"
                vertexColorsEnabled: true
                cullMode: Material.NoCulling
                roughness: root.styleA ? 0.96 : 0.98
                specularAmount: root.styleA ? 0.02 : 0.0
            }
        }

        Model {
            visible: root.styleA
            geometry: HazardPatchGeometry {
                styleName: root.previewStyle
            }
            castsShadows: false
            receivesShadows: false
            materials: PrincipledMaterial {
                baseColor: Qt.rgba(1.0, 0.04, 0.02, 0.42)
                alphaMode: PrincipledMaterial.Blend
                opacity: 0.58
                vertexColorsEnabled: true
                cullMode: Material.NoCulling
                roughness: 0.75
                emissiveFactor: Qt.vector3d(0.98, 0.06, 0.01)
            }
        }

        Model {
            visible: !root.styleA
            geometry: LinePreviewGeometry {
                kind: "contour"
                styleName: root.previewStyle
                widthValue: 34
            }
            castsShadows: false
            receivesShadows: false
            materials: PrincipledMaterial {
                baseColor: Qt.rgba(0.18, 1.0, 0.80, 0.62)
                alphaMode: PrincipledMaterial.Blend
                opacity: 0.74
                cullMode: Material.NoCulling
                emissiveFactor: Qt.vector3d(0.22, 1.35, 1.10)
                roughness: 0.88
            }
        }

        Model {
            visible: !root.styleA
            geometry: LinePreviewGeometry {
                kind: "grid"
                styleName: root.previewStyle
                widthValue: 18
            }
            castsShadows: false
            receivesShadows: false
            materials: PrincipledMaterial {
                baseColor: Qt.rgba(0.12, 0.84, 0.74, 0.30)
                alphaMode: PrincipledMaterial.Blend
                opacity: 0.40
                cullMode: Material.NoCulling
                emissiveFactor: Qt.vector3d(0.08, 0.68, 0.54)
                roughness: 0.96
            }
        }

        Model {
            geometry: LinePreviewGeometry {
                kind: "hazardGrid"
                styleName: root.previewStyle
                widthValue: root.styleA ? 34 : 38
            }
            castsShadows: false
            receivesShadows: false
            materials: PrincipledMaterial {
                baseColor: Qt.rgba(1.0, 0.23, 0.03, 0.92)
                alphaMode: PrincipledMaterial.Blend
                opacity: 0.95
                cullMode: Material.NoCulling
                emissiveFactor: Qt.vector3d(1.80, 0.28, 0.02)
                roughness: 0.66
            }
        }

        Model {
            geometry: LinePreviewGeometry {
                kind: "hazard"
                styleName: root.previewStyle
                widthValue: root.styleA ? 46 : 42
            }
            castsShadows: false
            receivesShadows: false
            materials: PrincipledMaterial {
                baseColor: Qt.rgba(1.0, 0.05, 0.02, 0.86)
                alphaMode: PrincipledMaterial.Blend
                opacity: root.styleA ? 0.92 : 1.0
                cullMode: Material.NoCulling
                emissiveFactor: Qt.vector3d(2.10, 0.16, 0.02)
                roughness: 0.70
            }
        }

        Model {
            geometry: LinePreviewGeometry {
                kind: "buffer"
                styleName: root.previewStyle
                widthValue: 34
            }
            castsShadows: false
            receivesShadows: false
            materials: PrincipledMaterial {
                baseColor: Qt.rgba(0.55, 1.0, 1.0, 0.72)
                alphaMode: PrincipledMaterial.Blend
                opacity: 0.70
                cullMode: Material.NoCulling
                emissiveFactor: Qt.vector3d(0.18, 0.95, 1.05)
                roughness: 0.78
            }
        }

        Model {
            geometry: LinePreviewGeometry {
                kind: "blockedRoute"
                styleName: root.previewStyle
                widthValue: 78
            }
            castsShadows: false
            receivesShadows: false
            materials: PrincipledMaterial {
                baseColor: Qt.rgba(1.0, 0.08, 0.03, 0.86)
                alphaMode: PrincipledMaterial.Blend
                opacity: 0.96
                cullMode: Material.NoCulling
                emissiveFactor: Qt.vector3d(2.10, 0.16, 0.03)
                roughness: 0.70
            }
        }

        Model {
            geometry: LinePreviewGeometry {
                kind: "blockedCross"
                styleName: root.previewStyle
                widthValue: 68
            }
            castsShadows: false
            receivesShadows: false
            materials: PrincipledMaterial {
                baseColor: Qt.rgba(1.0, 0.05, 0.02, 0.95)
                alphaMode: PrincipledMaterial.Blend
                opacity: 0.95
                cullMode: Material.NoCulling
                emissiveFactor: Qt.vector3d(2.40, 0.12, 0.02)
                roughness: 0.70
            }
        }

        Model {
            geometry: LinePreviewGeometry {
                kind: "routeGlow"
                styleName: root.previewStyle
                widthValue: 300
            }
            castsShadows: false
            receivesShadows: false
            materials: PrincipledMaterial {
                baseColor: Qt.rgba(0.02, 0.95, 1.0, 0.25)
                alphaMode: PrincipledMaterial.Blend
                opacity: root.styleA ? 0.42 : 0.48
                cullMode: Material.NoCulling
                emissiveFactor: Qt.vector3d(0.08, 1.15, 1.38)
                roughness: 0.82
            }
        }

        Model {
            geometry: LinePreviewGeometry {
                kind: "routeCore"
                styleName: root.previewStyle
                widthValue: 88
            }
            castsShadows: false
            receivesShadows: false
            materials: PrincipledMaterial {
                baseColor: Qt.rgba(0.28, 1.0, 1.0, 0.93)
                alphaMode: PrincipledMaterial.Blend
                opacity: 0.94
                cullMode: Material.NoCulling
                emissiveFactor: Qt.vector3d(0.24, 1.75, 1.95)
                roughness: 0.68
            }
        }

        Model {
            geometry: LinePreviewGeometry {
                kind: "waypoint"
                styleName: root.previewStyle
                widthValue: 44
            }
            castsShadows: false
            receivesShadows: false
            materials: PrincipledMaterial {
                baseColor: Qt.rgba(0.58, 1.0, 1.0, 0.86)
                alphaMode: PrincipledMaterial.Blend
                opacity: 0.86
                cullMode: Material.NoCulling
                emissiveFactor: Qt.vector3d(0.20, 1.35, 1.50)
                roughness: 0.70
            }
        }

        Model {
            geometry: LinePreviewGeometry {
                kind: "drone"
                styleName: root.previewStyle
                widthValue: 42
            }
            castsShadows: false
            receivesShadows: false
            materials: PrincipledMaterial {
                baseColor: Qt.rgba(0.92, 0.96, 1.0, 0.92)
                alphaMode: PrincipledMaterial.Blend
                opacity: 0.92
                cullMode: Material.NoCulling
                emissiveFactor: Qt.vector3d(0.74, 0.82, 0.90)
                roughness: 0.60
            }
        }
    }
}
