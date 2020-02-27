#include "openvslam/config.h"
#include "openvslam/camera/perspective.h"
#include "openvslam/camera/fisheye.h"
#include "openvslam/camera/equirectangular.h"

#include <iostream>
#include <memory>

#include <spdlog/spdlog.h>

namespace openvslam {

config::config(const std::string& config_file_path)
    : config(YAML::LoadFile(config_file_path), config_file_path) {}

config::config(const YAML::Node& yaml_node, const std::string& config_file_path)
    : config_file_path_(config_file_path), yaml_node_(yaml_node) {
    spdlog::debug("CONSTRUCT: config");

    spdlog::info("config file loaded: {}", config_file_path_);

    //========================//
    // Load Camera Parameters //
    //========================//

    spdlog::debug("load camera model type");
    const auto camera_model_type = camera::base::load_model_type(yaml_node_);

    spdlog::debug("load camera model parameters");
    try {
        switch (camera_model_type) {
            case camera::model_type_t::Perspective: {
                camera_ = new camera::perspective(yaml_node_);
                break;
            }
            case camera::model_type_t::Fisheye: {
                camera_ = new camera::fisheye(yaml_node_);
                break;
            }
            case camera::model_type_t::Equirectangular: {
                camera_ = new camera::equirectangular(yaml_node_);
                break;
            }
        }
    }
    catch (const std::exception& e) {
        spdlog::debug("failed in loading camera model parameters: {}", e.what());
        delete camera_;
        camera_ = nullptr;
        throw;
    }

    //=====================//
    // Load ORB Parameters //
    //=====================//

    spdlog::debug("load ORB parameters");
    try {
        orb_params_ = feature::orb_params(yaml_node_);
    }
    catch (const std::exception& e) {
        spdlog::debug("failed in loading ORB parameters: {}", e.what());
        delete camera_;
        camera_ = nullptr;
        throw;
    }

    //==========================//
    // Load Tracking Parameters //
    //==========================//

    spdlog::debug("load tracking parameters");

    spdlog::debug("load depth threshold");
    if (camera_->setup_type_ == camera::setup_type_t::Stereo || camera_->setup_type_ == camera::setup_type_t::RGBD) {
        // ベースライン長の一定倍より遠いdepthは無視する
        const auto depth_thr_factor = yaml_node_["depth_threshold"].as<double>(40.0);

        switch (camera_->model_type_) {
            case camera::model_type_t::Perspective: {
                auto camera = static_cast<camera::perspective*>(camera_);
                true_depth_thr_ = camera->true_baseline_ * depth_thr_factor;
                break;
            }
            case camera::model_type_t::Fisheye: {
                auto camera = static_cast<camera::fisheye*>(camera_);
                true_depth_thr_ = camera->true_baseline_ * depth_thr_factor;
                break;
            }
            case camera::model_type_t::Equirectangular: {
                throw std::runtime_error("Not implemented: Stereo or RGBD of equirectangular camera model");
            }
        }
    }

    spdlog::debug("load depthmap factor");
    if (camera_->setup_type_ == camera::setup_type_t::RGBD) {
        depthmap_factor_ = yaml_node_["depthmap_factor"].as<double>(1.0);
    }
}

/*
    Expose VSLAM config parameters directly, without needing config file
*/

config::config(const std::string& name,
               const std::string& cam_type, 
               const unsigned int cols, 
               const unsigned int rows, 
               const double fps,
               const unsigned int max_num_keypts, 
               const float scale_factor, 
               const unsigned int num_levels,
               const unsigned int ini_fast_thr, 
               const unsigned int min_fast_thr,
               const std::vector<std::vector<float>>& mask,
               const double fx, const double fy, const double cx, const double cy,
               const double k1, const double k2, const double k3,
               const double p1, const double p2) {

    spdlog::debug("CONSTRUCT: config");

    //========================//
    // Load Camera Parameters //
    //========================//

    spdlog::debug("load camera model type");                      
    const auto camera_model_type = camera::base::load_model_type(cam_type);

// perspective::perspective(
//                  const std::string& name, const setup_type_t& setup_type, const color_order_t& color_order,
//                  const unsigned int cols, const unsigned int rows, const double fps,
//                  const double fx, const double fy, const double cx, const double cy,
//                  const double k1, const double k2, const double p1, const double p2, const double k3,
//                  const double focal_x_baseline)


    spdlog::debug("load camera model parameters");
    try {
        switch (camera_model_type) {
            case camera::model_type_t::Perspective: {
                camera_ = new camera::perspective(name,
                                                  camera::setup_type_t::Monocular,
                                                  camera::color_order_t::RGB,
                                                  cols, rows, fps,
                                                  fx, fy, cx, cy,
                                                  k1, k2, p1, p2, k3,
                                                  0.0
                                                 );
                break;
            }
            case camera::model_type_t::Fisheye: {
                camera_ = new camera::fisheye(yaml_node_);
                break;
            }
            case camera::model_type_t::Equirectangular: {
                camera_ = new camera::equirectangular(name, camera::color_order_t::RGB, cols, rows, fps);
                break;
            }
        }
    }
    catch (const std::exception& e) {
        spdlog::debug("failed in loading camera model parameters: {}", e.what());
        delete camera_; camera_ = nullptr;
        throw;
    }

    //=====================//
    // Load ORB Parameters //
    //=====================//

    spdlog::debug("load ORB parameters");
    try {
        orb_params_ = feature::orb_params(max_num_keypts, scale_factor, num_levels, 
                                          ini_fast_thr, min_fast_thr, mask);
    }
    catch (const std::exception& e) {
        spdlog::debug("failed in loading ORB parameters: {}", e.what());
        delete camera_; camera_ = nullptr;
        throw;
    }
}
config::~config() {
    delete camera_;
    camera_ = nullptr;

    spdlog::debug("DESTRUCT: config");
}

std::ostream& operator<<(std::ostream& os, const config& cfg) {
    std::cout << "Camera Configuration:" << std::endl;
    cfg.camera_->show_parameters();

    std::cout << "ORB Configuration:" << std::endl;
    cfg.orb_params_.show_parameters();

    if (cfg.camera_->setup_type_ == camera::setup_type_t::Stereo || cfg.camera_->setup_type_ == camera::setup_type_t::RGBD) {
        std::cout << "Stereo Configuration:" << std::endl;
        std::cout << "- true baseline: " << cfg.camera_->true_baseline_ << std::endl;
        std::cout << "- true depth threshold: " << cfg.true_depth_thr_ << std::endl;
        std::cout << "- depth threshold factor: " << cfg.true_depth_thr_ / cfg.camera_->true_baseline_ << std::endl;
    }
    if (cfg.camera_->setup_type_ == camera::setup_type_t::RGBD) {
        std::cout << "Depth Image Configuration:" << std::endl;
        std::cout << "- depthmap factor: " << cfg.depthmap_factor_ << std::endl;
    }

    return os;
}

} // namespace openvslam
