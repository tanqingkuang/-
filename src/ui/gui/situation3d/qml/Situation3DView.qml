import QtQuick
import QtQuick.Controls
import QtQuick3D
import QtQuick3D.AssetUtils
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
    property bool followEnabled: false
    property string sceneTime: "0.0s"
    property string sceneSummary: "等待快照"
    property int sceneApplyCount: 0
    property string aircraftModelValue: "tb2"
    property string aircraftModelSource: "assets/BayraktarTB2.glb"
    property real aircraftYawOffsetDeg: 90
    property real aircraftBaseScale: 1.0035
    property real aircraftUnitWingspan: 11.957
    property real aircraftRealWingspanM: 12.0
    property string modelOptionsSignature: ""
    // 1800m 是默认自由视角量级；航线近距离保留 25% 线宽，避免放大后变成粗色带。
    property real nearViewWidthScale: Math.max(0.25, Math.min(1.0, distance / 1800.0))
    // 飞机视觉缩放单独保持远景可辨识；尾迹近景按翼展 1/5 显示，远景不继续加粗。
    property real aircraftVisualScale: aircraftBaseScale * Math.max(1.0, distance * 0.0207 / aircraftRealWingspanM)
    property real routeDashWidthScale: nearViewWidthScale
    property real trailWidthScale: Math.min(0.17, aircraftVisualScale * aircraftUnitWingspan / 5.0 / 44.0)
    property real terrainSpan: 20000
    property real terrainEffectiveSpan: 20000
    property real lastMouseX: 0
    property real lastMouseY: 0
    property bool cameraInitialized: false
    // 飞机与尾迹末段共享唯一展示时钟；真实队列点不参与 60Hz 补间，也不会增加容量。
    property real presentationProgress: 1.0
    // 正常 100ms 数据帧与 90ms 展示补间不会积压；容量 2 只保护偶发提前到帧。
    property var pendingSceneUpdates: []
    readonly property int presentationQueueCapacity: 2
    // 静态内容签名：与 payload 的 staticKey 对比，决定是否重建航线与风险区模型。
    property string staticContentKey: ""
    // 地形顶点色保持静态；真实障碍的闭合告警边界与贴地填充共用这一个呼吸值。
    property real alertBoundaryPulse: 0.48
    // 填充层把同一呼吸值线性映射到 0.10~0.35，保证与边界同相位闪烁但不遮挡地形细节。
    readonly property real riskFillPulse: 0.10 + (alertBoundaryPulse - 0.48) * (0.25 / 0.44)
    // 跟随目标 nodeId:按长机角色解析,更新快照时据此逐帧刷新相机焦点。
    property string followNodeId: ""
    // 跟随焦点与飞机共用唯一展示时钟，避免两套动画时长不同导致飞机相对镜头周期性抖动。
    onPresentationProgressChanged: {
        if (followEnabled) {
            applyFollowFocus()
        }
    }

    ListModel { id: aircraftModel }
    ListModel { id: modelOptions }
    ListModel { id: trailModel }
    ListModel { id: routeDashModel }
    ListModel { id: routeModel }
    ListModel { id: blockedRouteDashModel }
    ListModel { id: blockedRouteModel }
    ListModel { id: riskLineModel }
    ListModel { id: riskBufferModel }
    ListModel { id: riskFillModel }

    // 1 秒周期呼吸：500ms 变亮 + 500ms 变暗，危险提示节奏明显但不刺眼。
    SequentialAnimation on alertBoundaryPulse {
        loops: Animation.Infinite
        running: true
        NumberAnimation {
            from: 0.48
            to: 0.92
            duration: 500
            easing.type: Easing.InOutSine
        }
        NumberAnimation {
            from: 0.92
            to: 0.48
            duration: 500
            easing.type: Easing.InOutSine
        }
    }

    NumberAnimation {
        id: presentationMotion
        target: root
        property: "presentationProgress"
        from: 0.0
        to: 1.0
        duration: 90
        easing.type: Easing.Linear
        onFinished: root.consumePendingSceneUpdate()
    }

    function clampPitch(value) {
        return Math.max(-88, Math.min(-6, value))
    }

    function applyCameraDrag(dx, dy, pointerY) {
        const yawSign = pointerY < height / 2.0 ? 1.0 : -1.0
        yaw += dx * 0.25 * yawSign
        pitch = clampPitch(pitch - dy * 0.18)
        cameraMode = "自由"
    }

    function applyGroundPan(dx, dy) {
        followEnabled = false
        const scale = distance / 1800.0
        const yawRadians = yaw * Math.PI / 180.0
        const cosYaw = Math.cos(yawRadians)
        const sinYaw = Math.sin(yawRadians)
        focusX += (-dx * cosYaw - dy * sinYaw) * scale
        focusZ += (dx * sinYaw - dy * cosYaw) * scale
    }

    function applyPayloadCamera(camera) {
        if (!camera) {
            return false
        }
        focusX = camera.focusX
        focusY = camera.focusY
        focusZ = camera.focusZ
        distance = camera.distance
        yaw = camera.yaw
        pitch = camera.pitch
        cameraInitialized = true
        return true
    }

    function syncAircraftStyle(style) {
        if (!style) {
            return
        }
        if (style.value !== undefined) {
            aircraftModelValue = String(style.value)
        }
        if (style.modelSource !== undefined) {
            aircraftModelSource = String(style.modelSource)
        }
        if (style.yawOffsetDeg !== undefined) {
            aircraftYawOffsetDeg = Number(style.yawOffsetDeg)
        }
        if (style.baseScale !== undefined) {
            aircraftBaseScale = Number(style.baseScale)
        }
        if (style.unitWingspan !== undefined) {
            aircraftUnitWingspan = Number(style.unitWingspan)
        }
        if (style.realWingspanM !== undefined) {
            aircraftRealWingspanM = Number(style.realWingspanM)
        }
        syncModelComboIndex()
    }

    function modelOptionIndex(value) {
        for (let index = 0; index < modelOptions.count; index += 1) {
            if (modelOptions.get(index).value === value) {
                return index
            }
        }
        return -1
    }

    function syncModelComboIndex() {
        if (typeof aircraftCombo === "undefined") {
            return
        }
        const index = modelOptionIndex(aircraftModelValue)
        aircraftCombo.currentIndex = index >= 0 ? index : 0
    }

    function syncModelOptions(items) {
        const signatures = []
        for (const item of items || []) {
            signatures.push(String(item.value) + "|" + String(item.label))
        }
        const signature = signatures.join(";")
        if (signature === modelOptionsSignature) {
            syncModelComboIndex()
            return
        }
        modelOptionsSignature = signature
        modelOptions.clear()
        for (const item of items || []) {
            modelOptions.append({
                value: String(item.value),
                label: String(item.label)
            })
        }
        syncModelComboIndex()
    }

    function applyFallbackCamera() {
        focusX = 0
        focusY = 600
        focusZ = 0
        distance = 1800
        yaw = -38
        pitch = -34
        cameraInitialized = false
    }

    function findAircraftIndex(nodeId) {
        for (let index = 0; index < aircraftModel.count; index += 1) {
            if (aircraftModel.get(index).nodeId === nodeId) {
                return index
            }
        }
        return -1
    }

    function presentationPosition(fromX, fromY, fromZ, targetX, targetY, targetZ) {
        const ratio = Math.max(0.0, Math.min(1.0, presentationProgress))
        return Qt.vector3d(
            fromX + (targetX - fromX) * ratio,
            fromY + (targetY - fromY) * ratio,
            fromZ + (targetZ - fromZ) * ratio
        )
    }

    function currentAircraftPositions() {
        const positions = {}
        for (let index = 0; index < aircraftModel.count; index += 1) {
            const item = aircraftModel.get(index)
            const position = presentationPosition(
                item.fromX, item.fromY, item.fromZ,
                item.sx, item.sy, item.sz
            )
            positions[item.nodeId] = { x: position.x, y: position.y, z: position.z }
        }
        return positions
    }

    function currentTrailTipPositions() {
        const positions = {}
        for (let index = 0; index < trailModel.count; index += 1) {
            const item = trailModel.get(index)
            const position = presentationPosition(
                item.fromX, item.fromY, item.fromZ,
                item.sx, item.sy, item.sz
            )
            positions[item.nodeId] = { x: position.x, y: position.y, z: position.z }
        }
        return positions
    }

    function aircraftTargetPositions(items) {
        const positions = {}
        for (const item of items || []) {
            positions[item.nodeId] = { x: item.x, y: item.y, z: item.z }
        }
        return positions
    }

    function syncAircraftModel(items, visualPositions) {
        const seen = {}
        for (const item of items || []) {
            const index = findAircraftIndex(item.nodeId)
            const start = visualPositions[item.nodeId] || { x: item.x, y: item.y, z: item.z }
            const entry = {
                nodeId: item.nodeId,
                role: item.role,
                health: item.health,
                color: item.color,
                fromX: start.x,
                fromY: start.y,
                fromZ: start.z,
                sx: item.x,
                sy: item.y,
                sz: item.z,
                yawDeg: item.yawDeg,
                speed: item.speed
            }
            if (index >= 0) {
                aircraftModel.set(index, entry)
            } else {
                aircraftModel.append(entry)
            }
            seen[item.nodeId] = true
        }
        for (let index = aircraftModel.count - 1; index >= 0; index -= 1) {
            if (!seen[aircraftModel.get(index).nodeId]) {
                aircraftModel.remove(index)
            }
        }
    }

    function findTrailIndex(nodeId) {
        for (let index = 0; index < trailModel.count; index += 1) {
            if (trailModel.get(index).nodeId === nodeId) {
                return index
            }
        }
        return -1
    }

    function syncTrailModel(items, targetPositions, visualPositions) {
        const seen = {}
        for (const item of items || []) {
            const target = targetPositions[item.nodeId]
            if (!target) {
                continue
            }
            const index = findTrailIndex(item.nodeId)
            const start = visualPositions[item.nodeId] || target
            const entry = {
                nodeId: item.nodeId,
                color: item.color,
                widthValue: item.width,
                pathValue: item.pathValue,
                tipPreviousX: item.tipPreviousX,
                tipPreviousY: item.tipPreviousY,
                tipPreviousZ: item.tipPreviousZ,
                tipStartX: item.tipStartX,
                tipStartY: item.tipStartY,
                tipStartZ: item.tipStartZ,
                fromX: start.x,
                fromY: start.y,
                fromZ: start.z,
                sx: target.x,
                sy: target.y,
                sz: target.z
            }
            if (index >= 0) {
                trailModel.set(index, entry)
            } else {
                trailModel.append(entry)
            }
            seen[item.nodeId] = true
        }
        for (let index = trailModel.count - 1; index >= 0; index -= 1) {
            if (!seen[trailModel.get(index).nodeId]) {
                trailModel.remove(index)
            }
        }
    }

    function enqueueSceneUpdate(payload, forceCamera) {
        if (!presentationMotion.running && pendingSceneUpdates.length === 0) {
            return updateScene(payload, forceCamera)
        }
        pendingSceneUpdates.push({ payload: payload, forceCamera: forceCamera === true })
        if (pendingSceneUpdates.length <= presentationQueueCapacity) {
            return false
        }
        // 极端积压时先完成当前共同位置，再按顺序消费最老消息；不得丢 delta 或单独跳飞机。
        presentationMotion.stop()
        presentationProgress = 1.0
        const next = pendingSceneUpdates.shift()
        return updateScene(next.payload, next.forceCamera)
    }

    function consumePendingSceneUpdate() {
        if (pendingSceneUpdates.length === 0) {
            return
        }
        const next = pendingSceneUpdates.shift()
        updateScene(next.payload, next.forceCamera)
    }

    function rebuildStaticModels(data) {
        routeDashModel.clear()
        for (const item of data.routeDashes || []) {
            routeDashModel.append({
                color: item.color,
                widthValue: item.width,
                pathValue: item.pathValue
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
        blockedRouteDashModel.clear()
        for (const item of data.blockedRouteDashes || []) {
            blockedRouteDashModel.append({
                color: item.color,
                widthValue: item.width,
                pathValue: item.pathValue
            })
        }
        blockedRouteModel.clear()
        for (const item of data.blockedRoutePoints || []) {
            blockedRouteModel.append({
                color: item.color,
                sx: item.x,
                sy: item.y,
                sz: item.z,
                size: item.size
            })
        }
        riskLineModel.clear()
        for (const item of data.riskZoneLines || []) {
            riskLineModel.append({
                color: item.color,
                widthValue: item.width,
                pathValue: item.pathValue,
                pulseValue: item.pulse === true
            })
        }
        riskBufferModel.clear()
        for (const item of data.riskZoneBuffers || []) {
            riskBufferModel.append({
                color: item.color,
                widthValue: item.width,
                pathValue: item.pathValue
            })
        }
        riskFillModel.clear()
        for (const item of data.riskZoneFills || []) {
            riskFillModel.append({
                color: item.color,
                meshValue: item.meshValue
            })
        }
    }

    function updateScene(payload, forceCamera) {
        if (!payload || payload.length === 0) {
            return false
        }
        const data = JSON.parse(payload)
        sceneApplyCount += 1
        const visualPositions = currentAircraftPositions()
        const targetPositions = aircraftTargetPositions(data.aircraft || [])
        presentationMotion.stop()
        presentationProgress = 0.0
        syncAircraftStyle(data.aircraftStyle)
        syncModelOptions(data.modelOptions || [])
        syncAircraftModel(data.aircraft || [], visualPositions)
        syncTrailModel(data.trailRibbons || [], targetPositions, visualPositions)
        if ((data.aircraft || []).length > 0) {
            presentationMotion.restart()
        } else {
            // 初始空桥接数据不能占用展示队列，否则首个真实快照会无端等待 90ms。
            presentationProgress = 1.0
            // 空帧没有动画 onFinished 信号；异步续取可排空后继消息，并避免连续空帧同步递归。
            Qt.callLater(function() { root.consumePendingSceneUpdate() })
        }
        if (followEnabled) {
            // 跟随是持续行为:每帧刷新焦点,长机移动后画面中心不掉队。
            applyFollowFocus()
        }
        // 航线/风险区是静态内容：每帧 clear+append 会销毁重建上百个几何模型,
        // 造成周期性掉帧(飞机"一跳一跳")。签名不变时整体跳过静态模型重建。
        const staticChanged = (data.staticKey || "") !== staticContentKey
        if (staticChanged) {
            staticContentKey = data.staticKey || ""
            rebuildStaticModels(data)
        }
        const surface = data.terrain && data.terrain.surface ? data.terrain.surface : null
        if (surface) {
            terrainSurfaceModel.visible = true
            terrainSurfaceModel.position = Qt.vector3d(surface.x, surface.y, surface.z)
            root.terrainSpan = Math.max(surface.width || 0, surface.depth || 0, 20000)
            root.terrainEffectiveSpan = Math.max(surface.effectiveSpan || 0, 20000)
            if (surface.mode === "layout") {
                terrainGeometry.resolutionValue = surface.resolution || 641
                terrainGeometry.layoutFile = surface.layoutFile || ""
                // revision 含 mtime 和高度场就绪标志:原地改文件或后台生成完成都会触发重建。
                terrainGeometry.layoutRevision = String(surface.revision || "")
            } else {
                terrainGeometry.layoutFile = ""
                terrainGeometry.widthValue = surface.width
                terrainGeometry.depthValue = surface.depth
                terrainGeometry.amplitudeValue = surface.height
            }
            if (staticChanged) {
                // 障碍只改变地形顶点色，不再创建遮挡飞机和尾迹的红色柱体或方盒。
                terrainGeometry.riskAreasValue = JSON.stringify(data.terrainRiskAreas || [])
            }
        } else {
            terrainSurfaceModel.visible = false
            terrainGeometry.layoutFile = ""
            terrainGeometry.riskAreasValue = "[]"
        }
        let cameraApplied = false
        if (data.camera && (!cameraInitialized || forceCamera === true)) {
            cameraApplied = applyPayloadCamera(data.camera)
        }
        sceneTime = Number(data.time || 0).toFixed(1) + "s"
        const counts = data.counts || {}
        sceneSummary = "飞机 " + (counts.aircraft || 0) + " / 风险区 " + (counts.riskZones || 0)
        return cameraApplied
    }

    function resetCamera() {
        cameraMode = "自由"
        followEnabled = false
        followNodeId = ""
        if (typeof sceneBridge !== "undefined") {
            const payload = sceneBridge.sceneData()
            if (payload && payload.length > 0) {
                const data = JSON.parse(payload)
                // 重置相机不得重放整帧场景；否则会绕过展示 FIFO，令尾迹 delta 游标时间倒退。
                if (data.camera && applyPayloadCamera(data.camera)) {
                    return
                }
            }
        }
        applyFallbackCamera()
    }

    function setTopView() {
        yaw = 0
        pitch = -76
        cameraMode = "俯视"
    }

    function setSideView() {
        yaw = -90
        pitch = -8
        cameraMode = "侧视"
    }

    function leaderAircraftIndex() {
        for (let index = 0; index < aircraftModel.count; index += 1) {
            const role = String(aircraftModel.get(index).role || "").toLowerCase()
            if (role.indexOf("leader") >= 0) {
                return index
            }
        }
        return aircraftModel.count > 0 ? 0 : -1
    }

    function applyFollowFocus() {
        const index = followNodeId ? findAircraftIndex(followNodeId) : -1
        const resolved = index >= 0 ? index : leaderAircraftIndex()
        if (resolved < 0) {
            return
        }
        const lead = aircraftModel.get(resolved)
        followNodeId = lead.nodeId
        const position = presentationPosition(
            lead.fromX, lead.fromY, lead.fromZ,
            lead.sx, lead.sy, lead.sz
        )
        focusX = position.x
        focusY = position.y
        focusZ = position.z
    }

    function setFollowView() {
        if (followEnabled) {
            followEnabled = false
            followNodeId = ""
            return
        }
        // 跟随目标按长机角色选取(演示配置里第 0 项是僚机),并记录 nodeId 供逐帧跟踪。
        followNodeId = ""
        followEnabled = true
        applyFollowFocus()
    }

    Component.onCompleted: {
        if (typeof sceneBridge !== "undefined") {
            updateScene(sceneBridge.sceneData())
        }
    }

    Connections {
        target: sceneBridge
        function onSceneDataChanged(payload) {
            root.enqueueSceneUpdate(payload)
        }
    }

    Rectangle {
        anchors.fill: parent
        gradient: Gradient {
            GradientStop { position: 0.0; color: "#16263c" }
            GradientStop { position: 0.16; color: "#22405c" }
            GradientStop { position: 0.22; color: "#3a6a8c" }
            GradientStop { position: 0.34; color: "#2a4a66" }
            GradientStop { position: 1.0; color: "#2e4458" }
        }
    }

    View3D {
        id: view3d
        anchors.fill: parent
        environment: SceneEnvironment {
            backgroundMode: SceneEnvironment.Transparent
            clearColor: "transparent"
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.VeryHigh
            // 屏幕空间 AO 补足峡谷、山脚和相邻岩脊之间的接触阴影，尺度按千米级地形设置。
            aoEnabled: true
            aoStrength: 34
            aoDistance: 1050
            aoSoftness: 40
            aoDither: true
            aoSampleRate: 4
            aoBias: 24
            fog: Fog {
                enabled: true
                // 清晨薄雾:雾色取天际亮蓝灰(比地面亮),密度减半、消隐距离推远,远景保持轮廓可辨。
                color: "#3d5876"
                density: 0.16
                depthEnabled: true
                depthNear: Math.max(3200, root.distance * 1.45)
                depthFar: Math.max(11000, root.distance * 3.10)
                depthCurve: 1.18
                heightEnabled: false
            }
        }

        Node {
            id: cameraPivot
            position: Qt.vector3d(root.focusX, root.focusY, root.focusZ)
            eulerRotation: Qt.vector3d(root.pitch, root.yaw, 0)

            PerspectiveCamera {
                id: camera
                position: Qt.vector3d(0, 0, root.distance)
                clipNear: 10
                clipFar: 100000
                fieldOfView: 50
            }
        }

        DirectionalLight {
            eulerRotation: Qt.vector3d(-35, -52, 0)
            // 清晨低角度暖阳:主方向光。反照率已重标为白天量级,亮度按"岩面高光不过曝"配平。
            brightness: 0.72
            castsShadow: true
            shadowFactor: 64
            shadowMapQuality: Light.ShadowMapQualityUltra
            shadowMapFar: 76000
            shadowBias: 6.0
            softShadowQuality: Light.PCF16
            pcfFactor: 2.0
            use32BitShadowmap: true
            color: "#ffe9c8"
        }

        DirectionalLight {
            eulerRotation: Qt.vector3d(-68, 132, 0)
            // 冷色补光负责背光坡可读性：暗部应是蓝灰可读，而不是纯黑糊成一片。
            brightness: 0.42
            castsShadow: false
            color: "#6db6d8"
        }

        DirectionalLight {
            eulerRotation: Qt.vector3d(-90, 0, 0)
            // 天空环境光:半球光的方向光近似,从正上方给全场景铺清晨天光,抬高暗部曝光。
            brightness: 0.34
            castsShadow: false
            color: "#9cc2e0"
        }

        PointLight {
            position: Qt.vector3d(root.focusX - 5600, root.focusY + 3820, root.focusZ + 5000)
            brightness: 0.46
            color: "#6fc6d8"
        }

        Model {
            id: terrainSurfaceModel
            geometry: TerrainGeometry {
                id: terrainGeometry
            }
            position: Qt.vector3d(0, 0, 0)
            receivesShadows: true
            castsShadows: true
            materials: PrincipledMaterial {
                // 顶点色承担海拔渐变，基色保持白色避免二次染色。
                baseColor: "#ffffff"
                baseColorMap: Texture {
                    source: "assets/terrain_detail_albedo.png"
                    tilingModeHorizontal: Texture.Repeat
                    tilingModeVertical: Texture.Repeat
                    scaleU: 36
                    scaleV: 36
                    generateMipmaps: true
                    mipFilter: Texture.Linear
                }
                cullMode: Material.NoCulling
                vertexColorsEnabled: true
                // 近全粗糙、零镜面量消除浅绿塑料光泽；体积层次交给顶点色与法线光照。
                roughness: 0.99
                specularAmount: 0.0
                // 仅保留极弱冷色底线，防止背光沟壑死黑，同时不抬亮整片地表。
                emissiveFactor: Qt.vector3d(0.002, 0.003, 0.005)
                // 近景细节法线:顶点间距 68m,顶点色插值在极限放大时糊成水彩;
                // 平铺细节法线在像素级补出岩面粗糙度,曝光不受影响(P1.5)。
                normalMap: Texture {
                    source: "assets/terrain_detail_normal.png"
                    tilingModeHorizontal: Texture.Repeat
                    tilingModeVertical: Texture.Repeat
                    scaleU: 148
                    scaleV: 148
                    generateMipmaps: true
                    mipFilter: Texture.Linear
                }
                normalStrength: 0.92
            }
        }

        Repeater3D {
            model: riskFillModel
            delegate: Model {
                geometry: RiskFillGeometry {
                    meshValue: model.meshValue
                }
                // 贴地薄层不参与阴影，也永远不能遮挡飞机、尾迹和航线的可读性。
                castsShadows: false
                receivesShadows: false
                materials: PrincipledMaterial {
                    baseColor: model.color
                    alphaMode: PrincipledMaterial.Blend
                    // 与告警边界共用同一呼吸源：填充映射到 0.10~0.35 的低透明度区间。
                    opacity: root.riskFillPulse
                    cullMode: Material.NoCulling
                    roughness: 1.0
                    specularAmount: 0.0
                    // 少量自发光让填充在背光坡上仍呈红色警示，而不是被阴影压成暗棕。
                    emissiveFactor: Qt.vector3d(0.42, 0.06, 0.02)
                }
            }
        }

        Repeater3D {
            model: riskLineModel
            delegate: Model {
                geometry: TrailRibbonGeometry {
                    pathValue: model.pathValue
                    // 与主航线同一距离缩放,任何距离下风险网格都细于航线,层级不反转。
                    widthValue: model.widthValue * root.routeDashWidthScale
                    alphaMode: "solid"
                }
                castsShadows: false
                receivesShadows: false
                materials: PrincipledMaterial {
                    baseColor: model.color
                    alphaMode: PrincipledMaterial.Blend
                    // 兼容网格保持静态；真实告警边界才跟随低频呼吸值。
                    opacity: model.pulseValue ? root.alertBoundaryPulse : 0.95
                    cullMode: Material.NoCulling
                    vertexColorsEnabled: true
                    roughness: 0.66
                    // 发光降档:风险网是提示层,不允许压过主任务航线成为画面主角。
                    emissiveFactor: Qt.vector3d(1.05, 0.16, 0.01)
                }
            }
        }

        Repeater3D {
            model: riskBufferModel
            delegate: Model {
                geometry: TrailRibbonGeometry {
                    pathValue: model.pathValue
                    widthValue: model.widthValue * root.routeDashWidthScale
                    alphaMode: "solid"
                }
                castsShadows: false
                receivesShadows: false
                materials: PrincipledMaterial {
                    baseColor: model.color
                    alphaMode: PrincipledMaterial.Blend
                    opacity: 0.70
                    cullMode: Material.NoCulling
                    vertexColorsEnabled: true
                    roughness: 0.78
                    emissiveFactor: Qt.vector3d(0.12, 0.62, 0.68)
                }
            }
        }

        Repeater3D {
            model: routeDashModel
            delegate: Model {
                geometry: TrailRibbonGeometry {
                    pathValue: model.pathValue
                    widthValue: model.widthValue * root.routeDashWidthScale
                    alphaMode: "solid"
                }
                castsShadows: false
                materials: PrincipledMaterial {
                    baseColor: model.color
                    alphaMode: PrincipledMaterial.Blend
                    opacity: 0.88
                    cullMode: Material.NoCulling
                    vertexColorsEnabled: true
                    roughness: 0.88
                    // 主任务航线辉光加强一档:风格 A 的态势层级里航线优先于风险提示。
                    emissiveFactor: Qt.vector3d(0.18, 0.55, 0.62)
                }
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
            model: blockedRouteDashModel
            delegate: Model {
                geometry: TrailRibbonGeometry {
                    pathValue: model.pathValue
                    widthValue: model.widthValue * root.routeDashWidthScale
                    alphaMode: "solid"
                }
                castsShadows: false
                materials: PrincipledMaterial {
                    baseColor: model.color
                    alphaMode: PrincipledMaterial.Blend
                    opacity: 0.88
                    cullMode: Material.NoCulling
                    vertexColorsEnabled: true
                    roughness: 0.88
                    emissiveFactor: Qt.vector3d(0.62, 0.14, 0.10)
                }
            }
        }

        Repeater3D {
            model: blockedRouteModel
            delegate: Model {
                source: "#Sphere"
                position: Qt.vector3d(model.sx, model.sy, model.sz)
                scale: Qt.vector3d(model.size / 100.0, model.size / 100.0, model.size / 100.0)
                materials: PrincipledMaterial {
                    baseColor: model.color
                    emissiveFactor: Qt.vector3d(0.62, 0.14, 0.10)
                }
            }
        }

        Repeater3D {
            model: trailModel
            delegate: Model {
                geometry: TrailRibbonGeometry {
                    pathValue: model.pathValue
                    widthValue: model.widthValue * root.trailWidthScale
                }
                castsShadows: false
                materials: PrincipledMaterial {
                    id: trailMaterial
                    baseColor: model.color
                    alphaMode: PrincipledMaterial.Blend
                    // 淡出完全交给几何体顶点 alpha(0.08~0.72)控制,这里不再叠加全局系数,
                    // 避免和顶点 alpha 相乘后整体过淡。
                    opacity: 1.0
                    cullMode: Material.NoCulling
                    vertexColorsEnabled: true
                    roughness: 0.9
                    // 发光色跟随角色颜色本身,而不是固定色,否则整体偏淡时会盖过长机/僚机的颜色区分。
                    emissiveFactor: Qt.vector3d(
                        Qt.color(model.color).r * 0.35,
                        Qt.color(model.color).g * 0.35,
                        Qt.color(model.color).b * 0.35
                    )
                }

                Model {
                    geometry: TrailTipGeometry {
                        previousPosition: Qt.vector3d(
                            model.tipPreviousX, model.tipPreviousY, model.tipPreviousZ
                        )
                        startPosition: Qt.vector3d(
                            model.tipStartX, model.tipStartY, model.tipStartZ
                        )
                        // 只有固定六顶点小网格按共同展示进度变化；历史大网格在两帧之间保持不动。
                        endPosition: root.presentationPosition(
                            model.fromX, model.fromY, model.fromZ,
                            model.sx, model.sy, model.sz
                        )
                        widthValue: model.widthValue * root.trailWidthScale
                    }
                    castsShadows: false
                    materials: [trailMaterial]
                }
            }
        }

        Node {
            id: aircraftGroup

            DirectionalLight {
                // 全编队共享一盏轮廓侧逆光:scope 限定机群子树,把机体从山体背景里分离。
                // 正式 D3D 后端方向光上限 4 盏,禁止按机实例化(五机会到 8 盏被裁剪)。
                scope: aircraftGroup
                eulerRotation: Qt.vector3d(-18, 148, 0)
                brightness: 2.4
                castsShadow: false
                color: "#dceeff"
            }

            Repeater3D {
                model: aircraftModel
                delegate: Node {
                    position: root.presentationPosition(
                        model.fromX, model.fromY, model.fromZ,
                        model.sx, model.sy, model.sz
                    )
                    eulerRotation: Qt.vector3d(0, model.yawDeg, 0)

                    RuntimeLoader {
                        source: Qt.resolvedUrl(root.aircraftModelSource)
                        // 资产机头朝向由 Python 策略给出偏航校正，统一转到本场景机头朝 +X 的约定。
                        eulerRotation: Qt.vector3d(0, root.aircraftYawOffsetDeg, 0)
                        // 近观按真实翼展 1:1 显示；相机拉远后改为恒定视角大小(翼展约占视距 2%)，避免退化成小点。
                        scale: Qt.vector3d(root.aircraftVisualScale, root.aircraftVisualScale, root.aircraftVisualScale)
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
                root.applyCameraDrag(dx, dy, mouse.y)
            } else if ((mouse.buttons & Qt.RightButton) || (mouse.buttons & Qt.MiddleButton)) {
                root.applyGroundPan(dx, dy)
            }
            root.lastMouseX = mouse.x
            root.lastMouseY = mouse.y
        }
        onWheel: function(wheel) {
            const factor = wheel.angleDelta.y > 0 ? 0.88 : 1.14
            root.distance = Math.max(220, Math.min(50000, root.distance * factor))
            wheel.accepted = true
        }
    }

    Rectangle {
        id: overlay
        z: 10
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.margins: 12
        width: 260
        height: 178
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
                text: root.sceneSummary + " / 视角 " + root.cameraMode + (root.followEnabled ? " · 跟随中" : "")
                color: "#94a3b8"
                font.pixelSize: 12
            }

            Row {
                spacing: 8
                Text {
                    anchors.verticalCenter: parent.verticalCenter
                    width: 34
                    text: "机型"
                    color: "#94a3b8"
                    font.pixelSize: 12
                }
                ComboBox {
                    id: aircraftCombo
                    width: 186
                    height: 30
                    model: modelOptions
                    textRole: "label"
                    background: Rectangle {
                        color: "#0f1720"
                        radius: 6
                        border.color: aircraftCombo.hovered ? "#14b8a6" : "#2a3644"
                    }
                    contentItem: Text {
                        leftPadding: 10
                        rightPadding: 24
                        text: aircraftCombo.displayText
                        color: "#e7edf4"
                        font.pixelSize: 12
                        verticalAlignment: Text.AlignVCenter
                        elide: Text.ElideRight
                    }
                    indicator: Text {
                        x: aircraftCombo.width - width - 10
                        y: (aircraftCombo.height - height) / 2
                        text: "v"
                        color: "#94a3b8"
                        font.pixelSize: 11
                    }
                    delegate: ItemDelegate {
                        width: aircraftCombo.width
                        height: 30
                        contentItem: Text {
                            text: label
                            color: "#e7edf4"
                            font.pixelSize: 12
                            verticalAlignment: Text.AlignVCenter
                            elide: Text.ElideRight
                        }
                        background: Rectangle {
                            color: highlighted ? "#14b8a6" : "#151d26"
                        }
                    }
                    popup.background: Rectangle {
                        color: "#151d26"
                        border.color: "#2a3644"
                    }
                    onActivated: function(index) {
                        if (index < 0 || index >= modelOptions.count || typeof sceneBridge === "undefined") {
                            return
                        }
                        sceneBridge.selectModel(modelOptions.get(index).value)
                    }
                }
            }

            Row {
                spacing: 8
                ControlButton { label: "重置"; onClicked: root.resetCamera() }
                ControlButton { label: "俯视"; onClicked: root.setTopView() }
                ControlButton { label: "侧视"; onClicked: root.setSideView() }
                ControlButton { label: "跟随"; active: root.followEnabled; onClicked: root.setFollowView() }
            }
        }
    }

    component ControlButton: Rectangle {
        id: button
        property string label: ""
        property bool active: false
        signal clicked()
        width: 48
        height: 30
        radius: 6
        color: button.active || mouseArea.containsMouse ? "#14b8a6" : "#0f1720"
        border.color: button.active || mouseArea.containsMouse ? "#14b8a6" : "#2a3644"

        Text {
            anchors.centerIn: parent
            text: button.label
            color: button.active || mouseArea.containsMouse ? "#071318" : "#e7edf4"
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
