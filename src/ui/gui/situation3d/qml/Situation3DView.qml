import QtQuick
import QtQuick3D
import Simu3D 1.0

Item {
    id: root
    width: 1120
    height: 760

    property real yaw: -38
    property real pitch: -34
    property real distance: 1800
    property real focusX: 0
    property real focusY: 600
    property real focusZ: 0
    property string cameraMode: "自由"
    property string sceneTime: "0.0s"
    property string sceneSummary: "等待快照"
    property real lastMouseX: 0
    property real lastMouseY: 0

    ListModel { id: aircraftModel }
    ListModel { id: trailModel }
    ListModel { id: routeModel }
    ListModel { id: obstacleModel }

    function updateScene(payload) {
        if (!payload || payload.length === 0) {
            return
        }
        const data = JSON.parse(payload)
        aircraftModel.clear()
        for (const item of data.aircraft || []) {
            aircraftModel.append({
                nodeId: item.nodeId,
                role: item.role,
                health: item.health,
                color: item.color,
                sx: item.x,
                sy: item.y,
                sz: item.z,
                yawDeg: item.yawDeg,
                speed: item.speed
            })
        }
        trailModel.clear()
        for (const item of data.trailPoints || []) {
            trailModel.append({
                color: item.color,
                sx: item.x,
                sy: item.y,
                sz: item.z,
                size: item.size,
                opacity: item.opacity
            })
        }
        routeModel.clear()
        for (const item of data.routePoints || []) {
            routeModel.append({
                color: item.color,
                sx: item.x,
                sy: item.y,
                sz: item.z,
                size: item.size
            })
        }
        obstacleModel.clear()
        for (const item of data.obstacles || []) {
            obstacleModel.append({
                kind: item.kind,
                obstacleId: item.id,
                sx: item.x,
                sy: item.y,
                sz: item.z,
                widthValue: item.width,
                depthValue: item.depth,
                heightValue: item.height
            })
        }
        const surface = data.terrain && data.terrain.surface ? data.terrain.surface : null
        if (surface) {
            terrainSurfaceModel.visible = true
            terrainSurfaceModel.position = Qt.vector3d(surface.x, surface.y, surface.z)
            terrainGeometry.widthValue = surface.width
            terrainGeometry.depthValue = surface.depth
            terrainGeometry.amplitudeValue = surface.height
        } else {
            terrainSurfaceModel.visible = false
        }
        if (data.camera) {
            focusX = data.camera.focusX
            focusY = data.camera.focusY
            focusZ = data.camera.focusZ
            distance = data.camera.distance
            yaw = data.camera.yaw
            pitch = data.camera.pitch
        }
        sceneTime = Number(data.time || 0).toFixed(1) + "s"
        const counts = data.counts || {}
        sceneSummary = "飞机 " + (counts.aircraft || 0) + " / 障碍 " + (counts.obstacles || 0)
    }

    function resetCamera() {
        yaw = -38
        pitch = -34
        cameraMode = "自由"
        if (typeof sceneBridge !== "undefined") {
            updateScene(sceneBridge.sceneData())
        }
    }

    function setTopView() {
        yaw = 0
        pitch = -89
        cameraMode = "俯视"
    }

    function setSideView() {
        yaw = -90
        pitch = -8
        cameraMode = "侧视"
    }

    function setFollowView() {
        if (aircraftModel.count > 0) {
            const lead = aircraftModel.get(0)
            focusX = lead.sx
            focusY = lead.sy
            focusZ = lead.sz
            yaw = lead.yawDeg - 35
            pitch = -22
            distance = Math.max(520, distance * 0.55)
        }
        cameraMode = "跟随"
    }

    Component.onCompleted: {
        if (typeof sceneBridge !== "undefined") {
            updateScene(sceneBridge.sceneData())
        }
    }

    Connections {
        target: sceneBridge
        function onSceneDataChanged(payload) {
            root.updateScene(payload)
        }
    }

    View3D {
        id: view3d
        anchors.fill: parent
        environment: SceneEnvironment {
            backgroundMode: SceneEnvironment.Color
            clearColor: "#101923"
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.High
        }

        Node {
            id: cameraPivot
            position: Qt.vector3d(root.focusX, root.focusY, root.focusZ)
            eulerRotation: Qt.vector3d(root.pitch, root.yaw, 0)

            PerspectiveCamera {
                id: camera
                position: Qt.vector3d(0, 0, root.distance)
                clipNear: 1
                clipFar: 100000
            }
        }

        DirectionalLight {
            eulerRotation: Qt.vector3d(-38, -52, 0)
            brightness: 4.2
            castsShadow: false
        }

        DirectionalLight {
            eulerRotation: Qt.vector3d(-68, 138, 0)
            brightness: 0.28
            castsShadow: false
        }

        PointLight {
            position: Qt.vector3d(root.focusX - 6200, root.focusY + 3600, root.focusZ + 4800)
            brightness: 5.6
        }

        Model {
            id: terrainSurfaceModel
            geometry: TerrainGeometry {
                id: terrainGeometry
            }
            position: Qt.vector3d(0, 0, 0)
            receivesShadows: false
            castsShadows: false
            materials: PrincipledMaterial {
                baseColor: Qt.rgba(0.30, 0.45, 0.32, 1.0)
                cullMode: Material.NoCulling
                emissiveFactor: Qt.vector3d(0.010, 0.018, 0.012)
                roughness: 0.96
                specularAmount: 0.02
            }
        }

        Repeater3D {
            model: routeModel
            delegate: Model {
                source: "#Sphere"
                position: Qt.vector3d(model.sx, model.sy, model.sz)
                scale: Qt.vector3d(model.size / 100.0, model.size / 100.0, model.size / 100.0)
                materials: PrincipledMaterial {
                    baseColor: model.color
                    emissiveFactor: Qt.vector3d(0.10, 0.42, 0.52)
                }
            }
        }

        Repeater3D {
            model: trailModel
            delegate: Model {
                source: "#Sphere"
                position: Qt.vector3d(model.sx, model.sy, model.sz)
                scale: Qt.vector3d(model.size / 100.0, model.size / 100.0, model.size / 100.0)
                materials: PrincipledMaterial {
                    baseColor: model.color
                    alphaMode: PrincipledMaterial.Blend
                    opacity: model.opacity
                    emissiveFactor: Qt.vector3d(0.12, 0.10, 0.18)
                }
            }
        }

        Repeater3D {
            model: obstacleModel
            delegate: Model {
                source: model.kind === "circle" ? "#Cylinder" : "#Cube"
                position: Qt.vector3d(model.sx, model.sy, model.sz)
                scale: Qt.vector3d(model.widthValue / 100.0, model.heightValue / 100.0, model.depthValue / 100.0)
                receivesShadows: true
                materials: PrincipledMaterial {
                    baseColor: Qt.rgba(0.97, 0.45, 0.45, 0.20)
                    alphaMode: PrincipledMaterial.Blend
                    opacity: 0.58
                    roughness: 0.78
                }
            }
        }

        Repeater3D {
            model: aircraftModel
            delegate: Node {
                position: Qt.vector3d(model.sx, model.sy, model.sz)
                eulerRotation: Qt.vector3d(0, model.yawDeg, 0)

                Model {
                    source: "#Cylinder"
                    eulerRotation: Qt.vector3d(0, 0, 90)
                    scale: Qt.vector3d(0.10, 0.68, 0.10)
                    castsShadows: true
                    materials: PrincipledMaterial {
                        baseColor: model.color
                        emissiveFactor: Qt.vector3d(0.05, 0.06, 0.09)
                        roughness: 0.32
                        metalness: 0.18
                    }
                }

                Model {
                    source: "#Cone"
                    position: Qt.vector3d(72, 0, 0)
                    eulerRotation: Qt.vector3d(0, 0, -90)
                    scale: Qt.vector3d(0.10, 0.24, 0.10)
                    castsShadows: true
                    materials: PrincipledMaterial {
                        baseColor: "#e7edf4"
                        roughness: 0.30
                        metalness: 0.10
                    }
                }

                Model {
                    source: "#Cube"
                    position: Qt.vector3d(-8, -2, 0)
                    scale: Qt.vector3d(0.30, 0.020, 1.36)
                    castsShadows: true
                    materials: PrincipledMaterial {
                        baseColor: model.color
                        emissiveFactor: Qt.vector3d(0.03, 0.04, 0.08)
                        roughness: 0.36
                    }
                }

                Model {
                    source: "#Cube"
                    position: Qt.vector3d(-66, 7, 0)
                    scale: Qt.vector3d(0.18, 0.020, 0.58)
                    castsShadows: true
                    materials: PrincipledMaterial {
                        baseColor: "#334155"
                        roughness: 0.52
                    }
                }

                Model {
                    source: "#Cube"
                    position: Qt.vector3d(-72, 26, 0)
                    scale: Qt.vector3d(0.12, 0.34, 0.036)
                    castsShadows: true
                    materials: PrincipledMaterial {
                        baseColor: "#334155"
                        roughness: 0.52
                    }
                }

                Model {
                    source: "#Sphere"
                    position: Qt.vector3d(22, 14, 0)
                    scale: Qt.vector3d(0.20, 0.072, 0.12)
                    materials: PrincipledMaterial {
                        baseColor: Qt.rgba(0.75, 0.90, 1.0, 0.68)
                        alphaMode: PrincipledMaterial.Blend
                        roughness: 0.18
                    }
                }
            }
        }
    }

    MouseArea {
        anchors.fill: parent
        acceptedButtons: Qt.LeftButton | Qt.RightButton | Qt.MiddleButton
        onPressed: function(mouse) {
            root.lastMouseX = mouse.x
            root.lastMouseY = mouse.y
        }
        onPositionChanged: function(mouse) {
            const dx = mouse.x - root.lastMouseX
            const dy = mouse.y - root.lastMouseY
            if (mouse.buttons & Qt.LeftButton) {
                root.yaw += dx * 0.25
                root.pitch = Math.max(-88, Math.min(-6, root.pitch + dy * 0.18))
                root.cameraMode = "自由"
            } else if ((mouse.buttons & Qt.RightButton) || (mouse.buttons & Qt.MiddleButton)) {
                root.focusX -= dx * root.distance / 1800.0
                root.focusZ += dy * root.distance / 1800.0
                root.cameraMode = "自由"
            }
            root.lastMouseX = mouse.x
            root.lastMouseY = mouse.y
        }
        onWheel: function(wheel) {
            const factor = wheel.angleDelta.y > 0 ? 0.88 : 1.14
            root.distance = Math.max(220, Math.min(50000, root.distance * factor))
            root.cameraMode = "自由"
            wheel.accepted = true
        }
    }

    Rectangle {
        id: overlay
        z: 10
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.margins: 12
        width: 240
        height: 136
        radius: 8
        color: "#dd151d26"
        border.color: "#2a3644"

        Column {
            anchors.fill: parent
            anchors.margins: 10
            spacing: 8

            Text {
                text: "3D态势  " + root.sceneTime
                color: "#e7edf4"
                font.pixelSize: 15
                font.bold: true
            }

            Text {
                text: root.sceneSummary + " / 视角 " + root.cameraMode
                color: "#94a3b8"
                font.pixelSize: 12
            }

            Row {
                spacing: 8
                ControlButton { label: "重置"; onClicked: root.resetCamera() }
                ControlButton { label: "俯视"; onClicked: root.setTopView() }
                ControlButton { label: "侧视"; onClicked: root.setSideView() }
                ControlButton { label: "跟随"; onClicked: root.setFollowView() }
            }
        }
    }

    component ControlButton: Rectangle {
        id: button
        property string label: ""
        signal clicked()
        width: 48
        height: 30
        radius: 6
        color: mouseArea.containsMouse ? "#14b8a6" : "#0f1720"
        border.color: mouseArea.containsMouse ? "#14b8a6" : "#2a3644"

        Text {
            anchors.centerIn: parent
            text: button.label
            color: mouseArea.containsMouse ? "#071318" : "#e7edf4"
            font.pixelSize: 12
            font.bold: true
        }

        MouseArea {
            id: mouseArea
            anchors.fill: parent
            hoverEnabled: true
            onClicked: button.clicked()
        }
    }
}
