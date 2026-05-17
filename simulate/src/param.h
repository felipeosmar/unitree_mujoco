#pragma once

#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

namespace param
{

inline struct SimulationConfig
{
    std::string robot;
    std::filesystem::path robot_scene;

    int domain_id;
    std::string interface;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;

    int print_scene_information;

    int enable_elastic_band;
    int band_attached_link = 0;
    double elastic_band_initial_length = 0.0; // m; non-zero = rope acts as a safety harness rather than yanking the torso to the anchor

    // Tilting platform — only active when the scene declares "platform_pitch" /
    // "platform_roll" joints. platform_mode: "off" (no control), "remote" (drive
    // from rt/platform_cmd), "boat" (server-side sine-wave generator).
    std::string platform_mode = "off";
    double platform_kp = 800.0;
    double platform_kd = 80.0;
    double platform_default_max_rate_deg_s = 30.0;
    double boat_pitch_amp_deg = 5.0;
    double boat_pitch_period_s = 4.0;
    double boat_roll_amp_deg = 7.0;
    double boat_roll_period_s = 3.0;
    double boat_phase_offset_rad = 1.5707963267948966; // pi/2
    double boat_rampup_s = 30.0; // amplitudes scale 0→1 linearly over this window

    void load_from_yaml(const std::string &filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            robot = cfg["robot"].as<std::string>();
            robot_scene = cfg["robot_scene"].as<std::string>();
            domain_id = cfg["domain_id"].as<int>();
            interface = cfg["interface"].as<std::string>();
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            print_scene_information = cfg["print_scene_information"].as<int>();
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();
            if (cfg["elastic_band_initial_length"])
                elastic_band_initial_length = cfg["elastic_band_initial_length"].as<double>();
            if (cfg["platform_mode"]) platform_mode = cfg["platform_mode"].as<std::string>();
            if (cfg["platform_kp"]) platform_kp = cfg["platform_kp"].as<double>();
            if (cfg["platform_kd"]) platform_kd = cfg["platform_kd"].as<double>();
            if (cfg["platform_default_max_rate_deg_s"])
                platform_default_max_rate_deg_s = cfg["platform_default_max_rate_deg_s"].as<double>();
            if (auto b = cfg["platform_boat"]) {
                if (b["pitch_amp_deg"]) boat_pitch_amp_deg = b["pitch_amp_deg"].as<double>();
                if (b["pitch_period_s"]) boat_pitch_period_s = b["pitch_period_s"].as<double>();
                if (b["roll_amp_deg"]) boat_roll_amp_deg = b["roll_amp_deg"].as<double>();
                if (b["roll_period_s"]) boat_roll_period_s = b["roll_period_s"].as<double>();
                if (b["phase_offset_rad"]) boat_phase_offset_rad = b["phase_offset_rad"].as<double>();
                if (b["rampup_s"]) boat_rampup_s = b["rampup_s"].as<double>();
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }
    }
} config;

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv)
{
    po::options_description desc("Unitree Mujoco");
    desc.add_options()
        ("help,h", "Show help message")
        ("domain_id,i", po::value<int>(&config.domain_id), "DDS domain ID; -i 0")
        ("network,n", po::value<std::string>(&config.interface), "DDS network interface; -n eth0")
        ("robot,r", po::value<std::string>(&config.robot), "Robot type; -r go2")
        ("scene,s", po::value<std::filesystem::path>(&config.robot_scene), "Robot scene file; -s scene_terrain.xml")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    return vm;
}

}