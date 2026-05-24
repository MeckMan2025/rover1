// Angstrong/ASJ camera SDK wrapper for the Nuwa60C (HP60C model in vendor terms).
//
// Exposes a single `Camera` class that opens the first USB camera found,
// configures the RGB stream per the vendor JSON config, and serves the
// latest BGR8 frame as a numpy array.
//
// Lifecycle mirrors src/CameraSrv.cpp from the vendor reference code, but
// with no ROS deps and a single-camera focus.

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <chrono>
#include <cstring>
#include <dirent.h>
#include <list>
#include <mutex>
#include <condition_variable>
#include <stdexcept>
#include <string>
#include <vector>

#include "as_camera_sdk_api.h"
#include "as_camera_sdk_def.h"

namespace py = pybind11;

namespace {

std::string config_prefix_for_model(AS_SDK_CAM_MODEL_E m) {
    // Mirrors CameraSrv::getConfigFile prefixes.
    switch (m) {
        case AS_SDK_CAM_MODEL_KONDYOR:
        case AS_SDK_CAM_MODEL_KONDYOR_NET:    return "kondyor_";
        case AS_SDK_CAM_MODEL_NUWA_XB40:
        case AS_SDK_CAM_MODEL_NUWA_X100:
        case AS_SDK_CAM_MODEL_NUWA_HP60:
        case AS_SDK_CAM_MODEL_NUWA_HP60V:     return "nuwa_";
        case AS_SDK_CAM_MODEL_KUNLUN_A:
        case AS_SDK_CAM_MODEL_KUNLUN_C:       return "kunlun_";
        case AS_SDK_CAM_MODEL_HP60C:          return "hp60c_";
        case AS_SDK_CAM_MODEL_HP60CN:         return "hp60cn_";
        case AS_SDK_CAM_MODEL_VEGA:           return "vega_";
        case AS_SDK_CAM_MODEL_CHANGJIANG_B:   return "changjiangB_";
        case AS_SDK_CAM_MODEL_TANGGULA:       return "tanggula_";
        case AS_SDK_CAM_MODEL_TANGGULA_A:     return "tanggulaA_";
        case AS_SDK_CAM_MODEL_TAISHAN:        return "taishan_";
        case AS_SDK_CAM_MODEL_TANGGULA_B:     return "tanggulaB_";
        default:                              return "";
    }
}

// Recursive directory scan, same intent as CameraSrv::scanDir.
void scan_dir(const std::string& dir, std::vector<std::string>& out) {
    DIR* d = opendir(dir.c_str());
    if (!d) return;
    struct dirent* ent;
    while ((ent = readdir(d)) != nullptr) {
        if (ent->d_name[0] == '.') continue;
        std::string full = dir + "/" + ent->d_name;
        if (ent->d_type == DT_REG) {
            out.push_back(full);
        } else if (ent->d_type == DT_DIR) {
            scan_dir(full, out);
        }
    }
    closedir(d);
}

std::string find_config_file(const std::string& config_dir, AS_SDK_CAM_MODEL_E model) {
    std::string prefix = config_prefix_for_model(model);
    if (prefix.empty()) return "";
    std::vector<std::string> files;
    scan_dir(config_dir, files);
    for (const auto& f : files) {
        auto slash = f.find_last_of('/');
        std::string base = (slash == std::string::npos) ? f : f.substr(slash + 1);
        if (base.find(prefix) == 0) return f;
    }
    return "";
}

} // namespace

class Camera {
public:
    Camera(const std::string& config_dir, double timeout_s);
    ~Camera();
    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;

    py::object read();          // returns numpy ndarray (h, w, 3) BGR uint8, or None
    int width()  const { return width_; }
    int height() const { return height_; }
    int frame_id() const {
        std::lock_guard<std::mutex> lk(frame_mutex_);
        return frame_id_;
    }
    void close();

private:
    static void on_attached_tramp(AS_CAM_ATTR_S* attr, void* priv) {
        static_cast<Camera*>(priv)->on_attached(attr);
    }
    static void on_detached_tramp(AS_CAM_ATTR_S*, void*) {}
    static void on_frame_tramp(AS_CAM_PTR, const AS_SDK_Data_s* d, void* priv) {
        static_cast<Camera*>(priv)->on_frame(d);
    }

    void on_attached(AS_CAM_ATTR_S* attr);
    void on_frame(const AS_SDK_Data_s* d);

    std::string config_dir_;
    AS_CAM_PTR  handle_       = nullptr;
    bool sdk_initialized_     = false;
    bool listener_started_    = false;
    bool stream_started_      = false;
    bool closed_              = false;

    mutable std::mutex frame_mutex_;
    std::vector<uint8_t> frame_buffer_;
    int width_   = 0;
    int height_  = 0;
    int frame_id_ = 0;

    std::mutex              attach_mutex_;
    std::condition_variable attach_cv_;
    bool                    camera_ready_ = false;
    std::string             attach_error_;
};

Camera::Camera(const std::string& config_dir, double timeout_s) : config_dir_(config_dir) {
    int ret = AS_SDK_Init();
    if (ret != 0) {
        throw std::runtime_error("AS_SDK_Init failed: " + std::to_string(ret));
    }
    sdk_initialized_ = true;

    AS_LISTENER_CALLBACK_S cb{};
    cb.onAttached   = &Camera::on_attached_tramp;
    cb.onDetached   = &Camera::on_detached_tramp;
    cb.privateData  = this;

    {
        py::gil_scoped_release release;
        ret = AS_SDK_StartListener(cb, AS_LISTENNER_TYPE_USB, true);
        if (ret != 0) {
            throw std::runtime_error("AS_SDK_StartListener failed: " + std::to_string(ret));
        }
        listener_started_ = true;

        std::unique_lock<std::mutex> lk(attach_mutex_);
        auto until = std::chrono::steady_clock::now()
                   + std::chrono::milliseconds(static_cast<int>(timeout_s * 1000.0));
        if (!attach_cv_.wait_until(lk, until, [this] { return camera_ready_ || !attach_error_.empty(); })) {
            throw std::runtime_error("Timed out waiting for camera attach after "
                                     + std::to_string(timeout_s) + "s");
        }
        if (!attach_error_.empty()) {
            throw std::runtime_error("Camera attach failed: " + attach_error_);
        }
    }
}

Camera::~Camera() { close(); }

void Camera::on_attached(AS_CAM_ATTR_S* attr) {
    int ret = 0;
    AS_CAM_PTR dev = nullptr;

    auto fail = [this](const std::string& msg) {
        std::lock_guard<std::mutex> lk(attach_mutex_);
        if (attach_error_.empty()) attach_error_ = msg;
        attach_cv_.notify_all();
    };

    {
        std::lock_guard<std::mutex> lk(attach_mutex_);
        if (camera_ready_) return;   // already have a camera; ignore extras
    }

    ret = AS_SDK_CreateCamHandle(dev, attr);
    if (ret != 0) { fail("AS_SDK_CreateCamHandle " + std::to_string(ret)); return; }

    AS_SDK_CAM_MODEL_E model = AS_SDK_CAM_MODEL_UNKNOWN;
    ret = AS_SDK_GetCameraModel(dev, model);
    if (ret != 0) { fail("AS_SDK_GetCameraModel " + std::to_string(ret)); return; }

    std::string config_file = find_config_file(config_dir_, model);
    if (config_file.empty()) {
        fail("no config file in " + config_dir_ + " for model " + std::to_string(model));
        return;
    }

    ret = AS_SDK_OpenCamera(dev, config_file.c_str());
    if (ret != 0) { fail("AS_SDK_OpenCamera " + std::to_string(ret)); return; }

    AS_CAM_Stream_Cb_s scb{};
    scb.callback    = &Camera::on_frame_tramp;
    scb.privateData = this;
    ret = AS_SDK_RegisterStreamCallback(dev, &scb);
    if (ret != 0) { fail("AS_SDK_RegisterStreamCallback " + std::to_string(ret)); return; }

    ret = AS_SDK_StartStream(dev, 0);
    if (ret != 0) { fail("AS_SDK_StartStream " + std::to_string(ret)); return; }

    {
        std::lock_guard<std::mutex> lk(attach_mutex_);
        handle_         = dev;
        stream_started_ = true;
        camera_ready_   = true;
        attach_cv_.notify_all();
    }
}

void Camera::on_frame(const AS_SDK_Data_s* d) {
    if (d == nullptr) return;
    const AS_Frame_s& rgb = d->rgbImg;
    if (rgb.size == 0 || rgb.data == nullptr || rgb.width == 0 || rgb.height == 0) return;

    const size_t expected = static_cast<size_t>(rgb.width) * rgb.height * 3;
    const size_t copy_size = (rgb.size < expected) ? rgb.size : expected;

    std::lock_guard<std::mutex> lk(frame_mutex_);
    width_  = static_cast<int>(rgb.width);
    height_ = static_cast<int>(rgb.height);
    if (frame_buffer_.size() != expected) frame_buffer_.resize(expected, 0);
    std::memcpy(frame_buffer_.data(), rgb.data, copy_size);
    frame_id_++;
}

py::object Camera::read() {
    std::lock_guard<std::mutex> lk(frame_mutex_);
    if (frame_id_ == 0 || width_ == 0 || height_ == 0) return py::none();

    py::array_t<uint8_t> arr({height_, width_, 3});
    std::memcpy(arr.mutable_data(), frame_buffer_.data(), frame_buffer_.size());
    return arr;
}

void Camera::close() {
    if (closed_) return;
    closed_ = true;
    py::gil_scoped_release release;
    if (stream_started_ && handle_) {
        AS_SDK_StopStream(handle_, 0);
        stream_started_ = false;
    }
    if (handle_) {
        AS_SDK_CloseCamera(handle_);
        AS_SDK_DestoryCamHandle(handle_);
        handle_ = nullptr;
    }
    if (listener_started_) {
        AS_SDK_StopListener();
        listener_started_ = false;
    }
    if (sdk_initialized_) {
        AS_SDK_Deinit();
        sdk_initialized_ = false;
    }
}

PYBIND11_MODULE(_ascam_ext, m) {
    m.doc() = "Angstrong/ASJ vendor SDK Python binding (Nuwa60C / HP60C)";
    py::class_<Camera>(m, "Camera")
        .def(py::init<const std::string&, double>(),
             py::arg("config_dir"), py::arg("timeout") = 5.0,
             "Open the first USB camera found and start the RGB stream.")
        .def("read",     &Camera::read,
             "Return the latest BGR frame as a numpy uint8 ndarray (H, W, 3), or None if none yet.")
        .def("width",    &Camera::width)
        .def("height",   &Camera::height)
        .def("frame_id", &Camera::frame_id,
             "Monotonic counter incremented on every callback frame; used for FPS tracking.")
        .def("close",    &Camera::close);
}
