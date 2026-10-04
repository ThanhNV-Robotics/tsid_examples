#include "Robot_Simulator.h"

// ============================================================================
// 1. UIctr Implementation
// ============================================================================

static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    auto* ui = static_cast<UIctr*>(glfwGetWindowUserPointer(window));
    if (ui) ui->Scroll(xoffset, yoffset);
}

static void mouseMoveCallback(GLFWwindow* window, double xpos, double ypos) {
    auto* ui = static_cast<UIctr*>(glfwGetWindowUserPointer(window));
    if (ui) ui->Mouse_move(xpos, ypos);
}

static void mouseButtonCallback(GLFWwindow* window, int button, int act, int mods) {
    auto* ui = static_cast<UIctr*>(glfwGetWindowUserPointer(window));
    if (ui) ui->Mouse_button(button, act, mods);
}

static void keyboardCallback(GLFWwindow* window, int key, int scancode, int act, int mods) {
    auto* ui = static_cast<UIctr*>(glfwGetWindowUserPointer(window));
    if (ui) ui->Keyboard(key, scancode, act, mods);
}

UIctr::UIctr(mjModel* modelIn, mjData* dataIn)
    : mj_model(modelIn), mj_data(dataIn) {
    cam = mjvCamera();
    opt = mjvOption();
    scn = mjvScene();
    con = mjrContext();
    pert = mjvPerturb();
}

void UIctr::iniGLFW() {
    if (!glfwInit()) {
        mju_error("Could not initialize GLFW");
    }
}

void UIctr::createWindow(const char* windowTitle, bool saveVideo) {
    window = glfwCreateWindow(width, height, windowTitle, NULL, NULL);
    if (!window) {
        mju_error("Could not create GLFW window");
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjv_defaultFreeCamera(mj_model, &cam);

    if (isTrack) {
        cam.type = mjCAMERA_TRACKING;
        cam.trackbodyid = 1;
    } else {
        cam.type = mjCAMERA_FREE;
    }

    mjv_defaultOption(&opt);
    opt.frame = mjFRAME_WORLD;
    mjv_defaultPerturb(&pert);
    pert.select = 0;
    pert.flexselect = -1;
    pert.skinselect = -1;

    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(mj_model, &scn, 2000);
    mjr_makeContext(mj_model, &con, mjFONTSCALE_150);

    mjv_moveCamera(mj_model, mjMOUSE_ROTATE_H, 0.0, 0.0, &cam);

    glfwSetWindowUserPointer(window, this);
    glfwSetKeyCallback(window, keyboardCallback);
    glfwSetCursorPosCallback(window, mouseMoveCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetScrollCallback(window, scrollCallback);

    save_video = saveVideo;
    if (save_video) {
        image_rgb_ = (unsigned char*)malloc(3 * width * height * sizeof(unsigned char));
        image_depth_ = (float*)malloc(sizeof(float) * width * height);

        file = fopen("../record/rgbRec.out", "wb");
        if (!file) {
            mju_error("Could not open rgbfile for writing");
        }
    }
}

void UIctr::initSensorFigure(const char* title, const char* lineNames[], const float lineColors[][3], int numLines) {
    numLines = std::min(numLines, mjMAXLINE);
    mjv_defaultFigure(&figSensor);
    figSensor.flg_legend = 1;
    figSensor.flg_extend = 1;
    std::snprintf(figSensor.title, sizeof(figSensor.title), "%s", title);
    std::snprintf(figSensor.xlabel, sizeof(figSensor.xlabel), "time (s)");
    figSensor.range[0][0] = 0;
    figSensor.range[0][1] = 1;
    figSensor.range[1][0] = 0;
    figSensor.range[1][1] = 0;

    for (int n = 0; n < numLines; n++) {
        std::snprintf(figSensor.linename[n], sizeof(figSensor.linename[n]), "%s", lineNames[n]);
        figSensor.linergb[n][0] = lineColors[n][0];
        figSensor.linergb[n][1] = lineColors[n][1];
        figSensor.linergb[n][2] = lineColors[n][2];
        figSensor.linepnt[n] = 0;
    }
    sensorFigureIni = true;
}

void UIctr::updateSensorFigure(double time, const double* values, int numLines) {
    if (!sensorFigureIni) return;
    numLines = std::min(numLines, mjMAXLINE);

    for (int n = 0; n < numLines; n++) {
        int pnt = mjMIN(sensorFigMaxPnt, figSensor.linepnt[n] + 1);
        for (int i = pnt - 1; i > 0; i--) {
            figSensor.linedata[n][2 * i] = figSensor.linedata[n][2 * i - 2];
            figSensor.linedata[n][2 * i + 1] = figSensor.linedata[n][2 * i - 1];
        }
        figSensor.linepnt[n] = pnt;
        figSensor.linedata[n][0] = time;
        figSensor.linedata[n][1] = values[n];
    }

    int oldestIdx = figSensor.linepnt[0] - 1;
    float xMin = figSensor.linedata[0][2 * oldestIdx];
    float xMax = figSensor.linedata[0][0];
    figSensor.range[0][0] = xMin;
    figSensor.range[0][1] = (xMax > xMin) ? xMax : xMin + 1;
}

void UIctr::updateScene() {
    if (!isContinuous) runSim = false;

    buttonRead.key_w = false;
    buttonRead.key_a = false;
    buttonRead.key_s = false;
    buttonRead.key_d = false;
    buttonRead.key_space = false;
    buttonRead.key_h = false;
    buttonRead.key_j = false;

    mjrRect viewport = {0, 0, 0, 0};
    glfwMakeContextCurrent(window);
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    mjv_updateScene(mj_model, mj_data, &opt, &pert, &cam, mjCAT_ALL, &scn);

    for (const auto& arr : custom_arrows_) {
        if (scn.ngeom < scn.maxgeom) {
            mjvGeom* geom = &scn.geoms[scn.ngeom];
            mjv_initGeom(geom, arr.type, nullptr, nullptr, nullptr, arr.rgba);
            mjv_connector(geom, arr.type, arr.width, arr.from, arr.to);
            scn.ngeom++;
        }
    }
    custom_arrows_.clear();

    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    mjr_render(viewport, &scn, &con);

    char buffer[100];
    std::sprintf(buffer, "Time: %.3f", mj_data->time);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport, buffer, NULL, &con);

    if (sensorFigureIni) {
        mjrRect figViewport = {viewport.width - viewport.width / 2, 0, viewport.width / 2, viewport.height / 2};
        mjr_figure(figViewport, &figSensor, &con);
    }

    glfwSwapBuffers(window);
    glfwPollEvents();

    if (save_video && file) {
        mjr_readPixels(image_rgb_, image_depth_, viewport, &con);
        fwrite(image_rgb_, sizeof(unsigned char), 3 * width * height, file);
    }
}

void UIctr::Keyboard(int key, int scancode, int act, int mods) {
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
        mj_resetData(mj_model, mj_data);
        mj_forward(mj_model, mj_data);
    }
    if (act == GLFW_RELEASE && key == GLFW_KEY_1) {
        runSim = !runSim;
        isContinuous = true;
    }
    if (act == GLFW_RELEASE && key == GLFW_KEY_2) {
        runSim = true;
        isContinuous = false;
    }
    if (act == GLFW_RELEASE && key == GLFW_KEY_W) buttonRead.key_w = true;
    if (act == GLFW_RELEASE && key == GLFW_KEY_A) buttonRead.key_a = true;
    if (act == GLFW_RELEASE && key == GLFW_KEY_S) buttonRead.key_s = true;
    if (act == GLFW_RELEASE && key == GLFW_KEY_D) buttonRead.key_d = true;
    if (act == GLFW_RELEASE && key == GLFW_KEY_H) buttonRead.key_h = true;
    if (act == GLFW_RELEASE && key == GLFW_KEY_J) buttonRead.key_j = true;
    if (act == GLFW_RELEASE && key == GLFW_KEY_SPACE) buttonRead.key_space = true;
    if (act == GLFW_RELEASE && key == GLFW_KEY_F) toggleWorldFrame();
}

void UIctr::Mouse_button(int button, int act, int mods) {
    button_left   = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    glfwGetCursorPos(window, &lastx, &lasty);

    if (act == GLFW_RELEASE) {
        pert.active = 0;
        return;
    }

    if (button != GLFW_MOUSE_BUTTON_LEFT && button != GLFW_MOUSE_BUTTON_RIGHT) {
        return;
    }

    bool mod_ctrl = (mods & GLFW_MOD_CONTROL) != 0;

    double now = glfwGetTime();
    bool doubleClick = (button == lastClickButton_) && (now - lastClickTime_ < kDoubleClickSeconds);
    lastClickTime_ = now;
    lastClickButton_ = button;

    if (doubleClick) {
        int fbWidth, fbHeight;
        glfwGetFramebufferSize(window, &fbWidth, &fbHeight);
        if (fbWidth <= 0 || fbHeight <= 0) return;

        mjtNum selpnt[3];
        int selgeom, selflex, selskin;
        int selbody = mjv_select(mj_model, mj_data, &opt,
                                 static_cast<mjtNum>(fbWidth) / fbHeight,
                                 lastx / fbWidth, 1.0 - lasty / fbHeight,
                                 &scn, selpnt, &selgeom, &selflex, &selskin);

        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            pert.active = 0;
            if (selbody >= 0) {
                pert.select = selbody;
                pert.flexselect = selflex;
                pert.skinselect = selskin;

                mjtNum tmp[3];
                mju_sub3(tmp, selpnt, mj_data->xpos + 3 * selbody);
                mju_mulMatTVec(pert.localpos, mj_data->xmat + 9 * selbody, tmp, 3, 3);
            } else {
                pert.select = 0;
                pert.flexselect = -1;
                pert.skinselect = -1;
            }
        } else {
            if (selbody >= 0) {
                mju_copy3(cam.lookat, selpnt);
            }
            if (mod_ctrl && selbody > 0) {
                cam.type = mjCAMERA_TRACKING;
                cam.trackbodyid = selbody;
                cam.fixedcamid = -1;
            }
        }
        return;
    }

    if (mod_ctrl && pert.select > 0) {
        mjv_initPerturb(mj_model, mj_data, &scn, &pert);
        pert.active = (button == GLFW_MOUSE_BUTTON_RIGHT) ? mjPERT_TRANSLATE : mjPERT_ROTATE;
    }
}

void UIctr::Mouse_move(double xpos, double ypos) {
    if (!button_left && !button_middle && !button_right) return;

    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    int w, h;
    glfwGetWindowSize(window, &w, &h);

    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    if (pert.active) {
        mjv_movePerturb(mj_model, mj_data, action, dx / h, dy / h, &scn, &pert);
    } else {
        mjv_moveCamera(mj_model, action, dx / h, dy / h, &cam);
    }
}

void UIctr::Scroll(double xoffset, double yoffset) {
    mjv_moveCamera(mj_model, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &cam);
}

void UIctr::applyPerturbation() {
    if (!pert.select) return;

    if (runSim) {
        mju_zero(mj_data->xfrc_applied, 6 * mj_model->nbody);
        mjv_applyPerturbPose(mj_model, mj_data, &pert, 0);
        mjv_applyPerturbForce(mj_model, mj_data, &pert);
    } else {
        mjv_applyPerturbPose(mj_model, mj_data, &pert, 1);
    }
}

void UIctr::Close() {
    if (mj_data) mj_deleteData(mj_data);
    if (mj_model) mj_deleteModel(mj_model);
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    if (save_video && file) {
        fclose(file);
        free(image_rgb_);
        free(image_depth_);
    }
    glfwTerminate();
}

void UIctr::enableTracking() { isTrack = true; }
void UIctr::disableTracking() { isTrack = false; }

UIctr::ButtonState UIctr::getButtonState() {
    ButtonState tmp = buttonRead;
    buttonRead.key_w = false;
    buttonRead.key_a = false;
    buttonRead.key_s = false;
    buttonRead.key_d = false;
    buttonRead.key_h = false;
    buttonRead.key_j = false;
    buttonRead.key_space = false;
    return tmp;
}

void UIctr::addArrow(const double pos[3], const double vec[3], double scale, const float rgba[4], double widthIn) {
    VisualArrow arr;
    arr.from[0] = pos[0]; arr.from[1] = pos[1]; arr.from[2] = pos[2];
    arr.to[0] = pos[0] + scale * vec[0];
    arr.to[1] = pos[1] + scale * vec[1];
    arr.to[2] = pos[2] + scale * vec[2];

    double dx = arr.to[0] - arr.from[0];
    double dy = arr.to[1] - arr.from[1];
    double dz = arr.to[2] - arr.from[2];
    if (std::sqrt(dx * dx + dy * dy + dz * dz) < 1e-3) return;

    if (rgba) {
        for (int i = 0; i < 4; ++i) arr.rgba[i] = rgba[i];
    } else {
        arr.rgba[0] = 1.0f; arr.rgba[1] = 0.2f; arr.rgba[2] = 0.2f; arr.rgba[3] = 0.8f;
    }
    arr.width = widthIn;
    custom_arrows_.push_back(arr);
}

void UIctr::addArrow(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, double scale, const float rgba[4], double widthIn) {
    addArrow(pos.data(), vec.data(), scale, rgba, widthIn);
}

void UIctr::addLine(const Eigen::Vector3d& from, const Eigen::Vector3d& to, const float rgba[4], double widthPixels) {
    VisualArrow arr;
    arr.from[0] = from(0); arr.from[1] = from(1); arr.from[2] = from(2);
    arr.to[0]   = to(0);   arr.to[1]   = to(1);   arr.to[2]   = to(2);
    if ((to - from).norm() < 1e-4) return;

    if (rgba) {
        for (int i = 0; i < 4; ++i) arr.rgba[i] = rgba[i];
    } else {
        arr.rgba[0] = 0.2f; arr.rgba[1] = 0.8f; arr.rgba[2] = 0.2f; arr.rgba[3] = 0.6f;
    }
    arr.width = widthPixels;
    arr.type = mjGEOM_LINE;
    custom_arrows_.push_back(arr);
}



// ============================================================================
// 3. RealtimePlot Implementation
// ============================================================================

namespace {
constexpr float kPalette[][3] = {
    {0.20f, 0.60f, 1.00f},
    {1.00f, 0.40f, 0.30f},
    {0.30f, 0.85f, 0.45f},
    {1.00f, 0.80f, 0.20f},
    {0.75f, 0.40f, 0.95f},
    {1.00f, 0.55f, 0.15f},
    {0.20f, 0.85f, 0.85f},
    {0.95f, 0.35f, 0.65f},
};
constexpr int kPaletteSize = sizeof(kPalette) / sizeof(kPalette[0]);
}

RealtimePlot::RealtimePlot(mjModel* model, int width, int height, const char* title, double timeWindowSeconds)
    : timeWindowSeconds_(timeWindowSeconds), title_(title) {
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    window_ = glfwCreateWindow(width, height, title, NULL, NULL);
    glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
    if (!window_) {
        std::fprintf(stderr, "RealtimePlot: failed to create window \"%s\"\n", title);
        return;
    }

    glfwShowWindow(window_);
    glfwFocusWindow(window_);
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(0);

    mjr_defaultContext(&con_);
    mjr_makeContext(model, &con_, mjFONTSCALE_150);

    mjv_defaultFigure(&fig_);
    fig_.flg_legend = 1;
    fig_.flg_extend = 1;
    refreshTitle();
    std::snprintf(fig_.xlabel, sizeof(fig_.xlabel), "time (s)");
    fig_.range[0][0] = 0;
    fig_.range[0][1] = 1;
    fig_.range[1][0] = 0;
    fig_.range[1][1] = 0;
}

RealtimePlot::~RealtimePlot() {
    if (window_) {
        glfwMakeContextCurrent(window_);
        mjr_freeContext(&con_);
        glfwDestroyWindow(window_);
    }
}

void RealtimePlot::refreshTitle() {
    if (yLabel_.empty()) {
        std::snprintf(fig_.title, sizeof(fig_.title), "%s", title_.c_str());
    } else {
        std::snprintf(fig_.title, sizeof(fig_.title), "%s  (%s)", title_.c_str(), yLabel_.c_str());
    }
}

void RealtimePlot::setXLabel(const std::string& label) {
    if (!window_) return;
    std::snprintf(fig_.xlabel, sizeof(fig_.xlabel), "%s", label.c_str());
}

void RealtimePlot::setYLabel(const std::string& label) {
    if (!window_) return;
    yLabel_ = label;
    refreshTitle();
}

void RealtimePlot::setXLimit(double xMin, double xMax) {
    if (!window_) return;
    xRangeManual_ = true;
    fig_.range[0][0] = static_cast<float>(xMin);
    fig_.range[0][1] = static_cast<float>(xMax);
}

void RealtimePlot::setXLimitAuto() {
    xRangeManual_ = false;
}

void RealtimePlot::setYLimit(double yMin, double yMax, bool autoExtend) {
    if (!window_) return;
    fig_.range[1][0] = static_cast<float>(yMin);
    fig_.range[1][1] = static_cast<float>(yMax);
    fig_.flg_extend = autoExtend ? 1 : 0;
}

void RealtimePlot::setYLimitAuto() {
    if (!window_) return;
    fig_.range[1][0] = 0;
    fig_.range[1][1] = 0;
    fig_.flg_extend = 1;
}

void RealtimePlot::setLineWidth(float width) {
    if (!window_) return;
    fig_.linewidth = width;
}

void RealtimePlot::setPlotStyle(PlotStyle style) {
    if (!window_) return;
    fig_.flg_barplot = (style == PlotStyle::Segments) ? 1 : 0;
}

int RealtimePlot::getOrCreateLine(const std::string& name) {
    auto it = nameToLineIndex_.find(name);
    if (it != nameToLineIndex_.end()) return it->second;

    if (numLines_ >= mjMAXLINE) {
        std::fprintf(stderr, "RealtimePlot: mjMAXLINE (%d) reached, ignoring \"%s\"\n", mjMAXLINE, name.c_str());
        return -1;
    }

    int idx = numLines_++;
    std::snprintf(fig_.linename[idx], sizeof(fig_.linename[idx]), "%s", name.c_str());
    int paletteIdx = idx % kPaletteSize;
    fig_.linergb[idx][0] = kPalette[paletteIdx][0];
    fig_.linergb[idx][1] = kPalette[paletteIdx][1];
    fig_.linergb[idx][2] = kPalette[paletteIdx][2];
    fig_.linepnt[idx] = 0;

    nameToLineIndex_[name] = idx;
    return idx;
}

void RealtimePlot::addPoint(const std::string& signalName, double time, double value) {
    if (!window_) return;

    int idx = getOrCreateLine(signalName);
    if (idx < 0) return;

    if (fig_.linepnt[idx] > 0 && time <= fig_.linedata[idx][0]) return;

    int pnt = mjMIN(mjMAXLINEPNT, fig_.linepnt[idx] + 1);
    for (int i = pnt - 1; i > 0; i--) {
        fig_.linedata[idx][2 * i] = fig_.linedata[idx][2 * i - 2];
        fig_.linedata[idx][2 * i + 1] = fig_.linedata[idx][2 * i - 1];
    }
    fig_.linepnt[idx] = pnt;
    fig_.linedata[idx][0] = time;
    fig_.linedata[idx][1] = value;

    if (xRangeManual_) return;

    double latest = time;
    for (int n = 0; n < numLines_; n++) {
        if (fig_.linepnt[n] > 0) {
            latest = std::max(latest, static_cast<double>(fig_.linedata[n][0]));
        }
    }

    if (timeWindowSeconds_ > 0.0) {
        double pageStart = std::floor(latest / timeWindowSeconds_) * timeWindowSeconds_;
        fig_.range[0][0] = static_cast<float>(pageStart);
        fig_.range[0][1] = static_cast<float>(pageStart + timeWindowSeconds_);

        for (int n = 0; n < numLines_; n++) {
            int kept = 0;
            while (kept < fig_.linepnt[n] && fig_.linedata[n][2 * kept] >= pageStart) kept++;
            fig_.linepnt[n] = kept;
        }
    } else {
        float xMin = 1e30f, xMax = -1e30f;
        for (int n = 0; n < numLines_; n++) {
            if (fig_.linepnt[n] == 0) continue;
            xMax = std::max(xMax, fig_.linedata[n][0]);
            xMin = std::min(xMin, fig_.linedata[n][2 * (fig_.linepnt[n] - 1)]);
        }
        if (xMax > xMin) {
            fig_.range[0][0] = xMin;
            fig_.range[0][1] = xMax;
        }
    }
}

bool RealtimePlot::shouldClose() const {
    return window_ && glfwWindowShouldClose(window_);
}

void RealtimePlot::render() {
    if (!window_ || glfwWindowShouldClose(window_) || numLines_ == 0) return;

    glfwMakeContextCurrent(window_);

    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
    if (viewport.width <= 0 || viewport.height <= 0) return;

    glClearColor(0.10f, 0.10f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    mjr_figure(viewport, &fig_, &con_);
    glfwSwapBuffers(window_);
}


// ============================================================================
// 4. MJ_Interface Implementation
// ============================================================================

#ifdef HAS_YAML_CPP
MJ_Interface::MJ_Interface(mjModel* mj_modelIn, mjData* mj_dataIn, const char* yamlPath)
    : mj_model(mj_modelIn), mj_data(mj_dataIn) {
    try {
        YAML::Node root_read = YAML::LoadFile(yamlPath);
        for (const auto& kv : root_read) {
            JointName.push_back(kv.first.as<std::string>());
        }
    } catch (const std::exception& e) {
        std::cerr << "MJ_Interface: Warning reading YAML file: " << e.what() << "\n";
    }
    initJointMapping();
}
#endif

MJ_Interface::MJ_Interface(mjModel* mj_modelIn, mjData* mj_dataIn, const std::vector<std::string>& jointNames)
    : mj_model(mj_modelIn), mj_data(mj_dataIn), JointName(jointNames) {
    initJointMapping();
}

MJ_Interface::MJ_Interface(mjModel* mj_modelIn, mjData* mj_dataIn)
    : mj_model(mj_modelIn), mj_data(mj_dataIn) {
    if (mj_model) {
        for (int a = 0; a < mj_model->nu; ++a) {
            if (mj_model->actuator_trntype[a] == mjTRN_JOINT) {
                int jId = mj_model->actuator_trnid[2 * a];
                const char* name = mj_id2name(mj_model, mjOBJ_JOINT, jId);
                if (name) {
                    JointName.push_back(name);
                }
            }
        }
    }
    initJointMapping();
}

void MJ_Interface::initJointMapping() {
    jointNum = static_cast<int>(JointName.size());
    jntId_qpos.assign(jointNum, 0);
    jntId_qvel.assign(jointNum, 0);
    jntId_dctl.assign(jointNum, 0);

    motor_pos.assign(jointNum, 0.0);
    motor_pos_Old.assign(jointNum, 0.0);
    motor_vel.assign(jointNum, 0.0);
    motor_accel.assign(jointNum, 0.0);
    motor_torque.assign(jointNum, 0.0);

    if (!mj_model) return;

    for (int i = 0; i < jointNum; ++i) {
        int tmpId = mj_name2id(mj_model, mjOBJ_JOINT, JointName[i].c_str());
        if (tmpId == -1) {
            std::cerr << "MJ_Interface: Joint " << JointName[i] << " not found in model\n";
            continue;
        }

        jntId_qpos[i] = mj_model->jnt_qposadr[tmpId];
        jntId_qvel[i] = mj_model->jnt_dofadr[tmpId];

        int actuatorId = -1;
        for (int a = 0; a < mj_model->nu; ++a) {
            if (mj_model->actuator_trntype[a] == mjTRN_JOINT &&
                mj_model->actuator_trnid[2 * a] == tmpId) {
                actuatorId = a;
                break;
            }
        }
        jntId_dctl[i] = actuatorId;
    }

    baseBodyId = mj_name2id(mj_model, mjOBJ_BODY, baseName.c_str());
    orientataionSensorId = mj_name2id(mj_model, mjOBJ_SENSOR, orientationSensorName.c_str());
    velSensorId = mj_name2id(mj_model, mjOBJ_SENSOR, velSensorName.c_str());
    gyroSensorId = mj_name2id(mj_model, mjOBJ_SENSOR, gyroSensorName.c_str());
    accSensorId = mj_name2id(mj_model, mjOBJ_SENSOR, accSensorName.c_str());
    touchSensorId_L = mj_name2id(mj_model, mjOBJ_SENSOR, touchSensorName_L.c_str());
    touchSensorId_R = mj_name2id(mj_model, mjOBJ_SENSOR, touchSensorName_R.c_str());
}

void MJ_Interface::updateSensorValues() {
    if (!mj_model || !mj_data) return;

    for (int i = 0; i < jointNum; ++i) {
        motor_pos_Old[i] = motor_pos[i];
        motor_pos[i] = mj_data->qpos[jntId_qpos[i]];
        motor_vel[i] = mj_data->qvel[jntId_qvel[i]];
        motor_accel[i] = mj_data->qacc[jntId_qvel[i]];
        motor_torque[i] = mj_data->qfrc_actuator[jntId_qvel[i]];
    }

    if (orientataionSensorId >= 0) {
        for (int i = 0; i < 4; ++i) {
            baseQuat[i] = mj_data->sensordata[mj_model->sensor_adr[orientataionSensorId] + i];
        }
        double tmp = baseQuat[0];
        baseQuat[0] = baseQuat[1];
        baseQuat[1] = baseQuat[2];
        baseQuat[2] = baseQuat[3];
        baseQuat[3] = tmp;

        rpy[0] = std::atan2(2 * (baseQuat[3] * baseQuat[0] + baseQuat[1] * baseQuat[2]),
                            1 - 2 * (baseQuat[0] * baseQuat[0] + baseQuat[1] * baseQuat[1]));
        rpy[1] = std::asin(std::clamp(2 * (baseQuat[3] * baseQuat[1] - baseQuat[0] * baseQuat[2]), -1.0, 1.0));
        rpy[2] = std::atan2(2 * (baseQuat[3] * baseQuat[2] + baseQuat[0] * baseQuat[1]),
                            1 - 2 * (baseQuat[1] * baseQuat[1] + baseQuat[2] * baseQuat[2]));

        if ((rpy[2] - yaw_simgle) > 3.1415926 * 0.5) yaw_N -= 1;
        else if ((rpy[2] - yaw_simgle) < -3.1415926 * 0.5) yaw_N += 1;

        yaw_simgle = rpy[2];
        rpy[2] = yaw_simgle + yaw_N * 2.0 * 3.1415926;
    }

    if (baseBodyId >= 0) {
        for (int i = 0; i < 3; ++i) {
            double posOld = basePos[i];
            basePos[i] = mj_data->xpos[3 * baseBodyId + i];
            if (accSensorId >= 0) baseAcc[i] = mj_data->sensordata[mj_model->sensor_adr[accSensorId] + i];
            if (gyroSensorId >= 0) baseAngVel[i] = mj_data->sensordata[mj_model->sensor_adr[gyroSensorId] + i];
            if (mj_model->opt.timestep > 0) baseLinVel[i] = (basePos[i] - posOld) / mj_model->opt.timestep;
        }
    }

    if (touchSensorId_L >= 0) touch_lf = mj_data->sensordata[mj_model->sensor_adr[touchSensorId_L]];
    if (touchSensorId_R >= 0) touch_rf = mj_data->sensordata[mj_model->sensor_adr[touchSensorId_R]];
}

void MJ_Interface::setMotorsTorque(const std::vector<double>& tauIn) {
    if (!mj_data) return;
    int limit = std::min(jointNum, static_cast<int>(tauIn.size()));
    for (int i = 0; i < limit; ++i) {
        if (jntId_dctl[i] >= 0 && jntId_dctl[i] < mj_model->nu) {
            mj_data->ctrl[jntId_dctl[i]] = tauIn[i];
        }
    }
}

void MJ_Interface::printInfo() {
    std::printf("Number of joints: %d\n", jointNum);
    for (int i = 0; i < jointNum; ++i) {
        std::printf("  [%2d] %-25s (qpos: %d, qvel: %d, ctrl: %d)\n",
                    i, JointName[i].c_str(), jntId_qpos[i], jntId_qvel[i], jntId_dctl[i]);
    }
}

void MJ_Interface::printJointPos() {
    for (int i = 0; i < jointNum; ++i) {
        std::printf("Joint %-25s position: %.3f\n", JointName[i].c_str(), motor_pos[i]);
    }
}