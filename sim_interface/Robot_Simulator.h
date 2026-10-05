#pragma once

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include "data_type.h"


// Optional YAML support for joint configurations
#if __has_include(<yaml-cpp/yaml.h>)
#include <yaml-cpp/yaml.h>
#define HAS_YAML_CPP 1
#endif

// Forward declarations for optional DataBus integrations
class DataBus;
struct RobotSensor;

// ============================================================================
// 1. UIctr - Single-Threaded Raw GLFW Interactive Simulation Viewer
// ============================================================================
// Call updateScene() once per iteration from your own physics loop on the
// same thread (no second thread required).
//
// Features:
//   - Mouse perturbation (double-click body to select; Ctrl+left-drag to twist,
//     Ctrl+right-drag to push)
//   - Camera tracking / free camera / look-at point selection
//   - Keyboard controls ('1': toggle pause/run, '2': single step, Space, W/A/S/D/H/J)
//   - World frame toggle ('F')
//   - Visual 3D arrows and line segments (e.g. contact forces, friction cones)
//   - Built-in rolling sensor graph overlay (initSensorFigure / updateSensorFigure)
//   - Video recording to RGB stream
// ============================================================================

class UIctr {
public:
    GLFWwindow* window{nullptr};

    // Keyboard button state
    struct ButtonState {
        bool key_w{false};
        bool key_s{false};
        bool key_a{false};
        bool key_d{false};
        bool key_h{false};
        bool key_j{false};
        bool key_space{false};
    } buttonRead;

    // Mouse interaction
    bool button_left{false};
    bool button_middle{false};
    bool button_right{false};

    bool runSim{true};
    bool isContinuous{true};
    double lastx{0};
    double lasty{0};
    mjModel* mj_model{nullptr};
    mjData* mj_data{nullptr};

    UIctr(mjModel* modelIn = nullptr, mjData* dataIn = nullptr);
    virtual ~UIctr() = default;

    void iniGLFW();
    void createWindow(const char* windowTitle, bool saveVideo = false);
    void updateScene();

    // Real-time sensor plot overlay
    void initSensorFigure(const char* title, const char* lineNames[], const float lineColors[][3], int numLines);
    void updateSensorFigure(double time, const double* values, int numLines);

    // Callbacks
    void Keyboard(int key, int scancode, int act, int mods);
    void Mouse_button(int button, int act, int mods);
    void Mouse_move(double xpos, double ypos);
    void Scroll(double xoffset, double yoffset);

    ButtonState getButtonState();

    // Apply current mouse perturbation force/torque
    void applyPerturbation();

    virtual void Close();

    void enableTracking();
    void disableTracking();

    void enableWorldFrame() { opt.frame = mjFRAME_WORLD; }
    void disableWorldFrame() { opt.frame = mjFRAME_NONE; }
    void toggleWorldFrame() { opt.frame = (opt.frame == mjFRAME_WORLD) ? mjFRAME_NONE : mjFRAME_WORLD; }

    // 3D Visual Arrow / Line (for contact forces, CoM velocity, friction cones)
    struct VisualArrow {
        mjtNum from[3];
        mjtNum to[3];
        float rgba[4];
        mjtNum width;
        int type{mjGEOM_ARROW}; // mjGEOM_ARROW or mjGEOM_LINE
    };
    void addArrow(const double pos[3], const double vec[3], double scale = 0.002, const float rgba[4] = nullptr, double width = 0.012);
    void addArrow(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, double scale = 0.002, const float rgba[4] = nullptr, double width = 0.012);
    void addLine(const Eigen::Vector3d& from, const Eigen::Vector3d& to, const float rgba[4] = nullptr, double widthPixels = 1.5);
    void clearArrows() { custom_arrows_.clear(); custom_spheres_.clear(); }

    // 3D sphere marker (e.g. CoM, target points); like arrows, it is drawn for
    // one frame only, so add it again before every updateScene()
    struct VisualSphere {
        mjtNum pos[3];
        mjtNum radius;
        float rgba[4];
    };
    void addSphere(const Eigen::Vector3d& pos, double radius = 0.02, const float rgba[4] = nullptr);

    bool shouldClose() const { return window ? glfwWindowShouldClose(window) : true; }

protected:
    std::vector<VisualArrow> custom_arrows_;
    std::vector<VisualSphere> custom_spheres_;
    unsigned char* image_rgb_{nullptr};
    float* image_depth_{nullptr};
    FILE* file{nullptr};

    int width{1200};
    int height{800};
    bool save_video{false};
    bool isTrack{false};

    double lastClickTime_{0};
    int lastClickButton_{-1};
    static constexpr double kDoubleClickSeconds = 0.3;

    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;
    mjvPerturb pert;

    mjvFigure figSensor;
    bool sensorFigureIni{false};
    static constexpr int sensorFigMaxPnt = 150;
};


// ============================================================================
// 2. Robot_Simulator - Unified Simulator Class
// ============================================================================
// Inherits all UIctr visualization/interaction capabilities and adds convenient
// simulation control, model loading, state querying, and actuation helpers.
// ============================================================================

class Robot_Simulator : public UIctr {
public:
    // Default constructor
    Robot_Simulator();

    // Construct directly by loading MuJoCo scene XML
    explicit Robot_Simulator(const std::string& xml_path);

    // Backward-compatible constructor accepting pre-existing mjModel / mjData
    Robot_Simulator(mjModel* modelIn, mjData* dataIn);

    virtual ~Robot_Simulator();

    // Load/reload model from XML file
    bool loadModel(const std::string& xml_path);
    bool isLoaded() const { return mj_model != nullptr && mj_data != nullptr; }

    // Direct accessors for mjModel and mjData
    mjModel* model() const { return mj_model; }
    mjData* data() const { return mj_data; }
    mjModel* m() const { return mj_model; }
    mjData* d() const { return mj_data; }
    mjModel* getModel() const { return mj_model; }
    mjData* getData() const { return mj_data; }

    // Model dimensions & timing
    int nq() const { return mj_model ? mj_model->nq : 0; }
    int nv() const { return mj_model ? mj_model->nv : 0; }
    int nu() const { return mj_model ? mj_model->nu : 0; }
    int na() const { return mj_model ? mj_model->nu : 0; }
    double time() const { return mj_data ? mj_data->time : 0.0; }
    double getTime() const { return time(); }

    // Print model summary (joints, actuators, dimensions)
    void printModelInfo() const;

    // Apply initial joint configuration (handles floating-base index offset automatically)
    void setInitConfiguration(const Eigen::VectorXd& q0);

    // Read actuated joint states (qpos and qvel)
    void getActuatedState(Eigen::VectorXd& q_out, Eigen::VectorXd& v_out, int na = -1) const;
    Eigen::VectorXd getActuatedJointPos(int na = -1) const;
    Eigen::VectorXd getActuatedJointVel(int na = -1) const;

    // Whole-robot centre of mass in the world frame, from MuJoCo (ground truth)
    Eigen::Vector3d getCoM() const;

    // Draw a CoM marker for the next frame: sphere at com, a vertical line to
    // the floor (z = 0) and a dot at the ground projection
    void addCoMMarker(const Eigen::Vector3d& com, const float rgba[4] = nullptr);

    // Read actuated joint states, base IMU and foot touch sensors (see common/data_type.h)
    RobotSensor getRobotSensorValues() const;

    // Apply control torques to actuators
    void setControl(const Eigen::VectorXd& tau);
    void setControl(const double* tau, int size);
    void setActuatorForces(const Eigen::VectorXd& tau) { setControl(tau); }

    // Step physics without updating the viewport (for fast inner control loops)
    void stepPhysics();
    void stepPhysics(const Eigen::VectorXd& tau);

    // Step physics and render viewport (substeps iterations)
    void step(int substeps = 1);
    void step(const Eigen::VectorXd& tau, int substeps = 1);

    // Initialize GLFW and window in one call
    void init(const char* windowTitle = "Robot Simulator", bool saveVideo = false) {
        iniGLFW();
        createWindow(windowTitle, saveVideo);
    }

    // Reset simulation data
    void reset();

    // Close window and release resources
    void Close() override;

private:
    bool owns_model_{false};
};



// ============================================================================
// 4. RealtimePlot - Standalone Real-Time Line Plotting Window
// ============================================================================
// Separate GLFW window that renders live signal curves using MuJoCo's mjvFigure.
// Automatically auto-colors signals and manages paged or rolling time axes.
// ============================================================================

class RealtimePlot {
public:
    RealtimePlot(mjModel* model, int width, int height, const char* title,
                 double timeWindowSeconds = 10.0);
    ~RealtimePlot();

    RealtimePlot(const RealtimePlot&) = delete;
    RealtimePlot& operator=(const RealtimePlot&) = delete;

    void addPoint(const std::string& signalName, double time, double value);

    void setTimeWindow(double timeWindowSeconds) { timeWindowSeconds_ = timeWindowSeconds; }
    double timeWindow() const { return timeWindowSeconds_; }

    void setXLabel(const std::string& label);
    void setYLabel(const std::string& label);

    void setXLimit(double xMin, double xMax);
    void setXLimitAuto();

    void setYLimit(double yMin, double yMax, bool autoExtend = true);
    void setYLimitAuto();

    void setLineWidth(float width);

    enum class PlotStyle { Line, Segments };
    void setPlotStyle(PlotStyle style);

    void render();
    bool shouldClose() const;
    bool isValid() const { return window_ != nullptr; }

private:
    GLFWwindow* window_{nullptr};
    mjrContext con_;
    mjvFigure fig_;
    double timeWindowSeconds_{10.0};
    bool xRangeManual_{false};
    std::string title_, yLabel_;
    std::unordered_map<std::string, int> nameToLineIndex_;
    int numLines_{0};

    int getOrCreateLine(const std::string& name);
    void refreshTitle();
};


// ============================================================================
// 5. MJ_Interface - Robot Joint Sensor & Actuation Interface
// ============================================================================
// Maps robot joint names to mjModel/mjData indices, reads sensor states
// (qpos, qvel, qacc, torque, IMU, foot sensors), and commands torques.
// ============================================================================

class MJ_Interface {
public:
    int jointNum{0};
    std::vector<double> motor_pos;
    std::vector<double> motor_pos_Old;
    std::vector<double> motor_vel;
    std::vector<double> motor_accel;
    std::vector<double> motor_torque;

    double rpy[3]{0};
    double yaw_simgle{0};
    int yaw_N{0};
    double baseQuat[4]{0}; // [x,y,z,w]
    double f3d[3][2]{0};
    double basePos[3]{0};
    double baseAcc[3]{0};
    double baseAngVel[3]{0};
    double baseLinVel[3]{0};

    std::string baseName{"base_link"};
    std::string orientationSensorName{"baselink-quat"};
    std::string velSensorName{"baselink-velocity"};
    std::string gyroSensorName{"baselink-gyro"};
    std::string accSensorName{"baselink-baseAcc"};
    std::string touchSensorName_L{"lf-touch"};
    std::string touchSensorName_R{"rf-touch"};

    double touch_lf{0}, touch_rf{0};
    std::vector<std::string> JointName;

    // Constructors
#ifdef HAS_YAML_CPP
    MJ_Interface(mjModel* mj_modelIn, mjData* mj_dataIn, const char* yamlPath);
#endif
    MJ_Interface(mjModel* mj_modelIn, mjData* mj_dataIn, const std::vector<std::string>& jointNames);
    MJ_Interface(mjModel* mj_modelIn, mjData* mj_dataIn);

    void updateSensorValues();
    void setMotorsTorque(const std::vector<double>& tauIn);

    std::vector<double> getJointPos() const { return motor_pos; }
    std::vector<double> getJointVel() const { return motor_vel; }
    std::vector<double> getJointAccel() const { return motor_accel; }
    std::vector<double> getJointTorque() const { return motor_torque; }

    void printInfo();
    void printJointPos();

private:
    void initJointMapping();

    mjModel* mj_model{nullptr};
    mjData* mj_data{nullptr};
    std::vector<int> jntId_qpos;
    std::vector<int> jntId_qvel;
    std::vector<int> jntId_dctl;

    int orientataionSensorId{-1};
    int velSensorId{-1};
    int gyroSensorId{-1};
    int accSensorId{-1};
    int baseBodyId{-1};
    int touchSensorId_L{-1};
    int touchSensorId_R{-1};
};