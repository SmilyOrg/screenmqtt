#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <conio.h>

#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <bitset>
#include <algorithm>
#include <string>
#include <functional>
#include <sstream>
#include <thread>
#include <condition_variable>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winnt.h>
#include <tchar.h>
#include "PhysicalMonitorEnumerationAPI.h"
#include "LowLevelMonitorConfigurationAPI.h"
#include "HighLevelMonitorConfigurationAPI.h"

#include "yaml-cpp/yaml.h"

#include "msgqueue.h"

const std::string name = "screenmqtt 0.4.0";

MessageQueueOptions global_options;

MessageQueue queue;

std::string global_topic_prefix;
std::mutex mutex;
std::condition_variable cv;
bool done = false;
std::string topic_all_power_state;
std::string topic_all_power_command;
HWND window;


void receive_monitors_all(const std::string &payload) {
    if (payload == "ON") {
        // -1 doesn't seem to work, so send mouse move event instead
        INPUT input = { INPUT_MOUSE };
        input.mi.dwFlags = MOUSEEVENTF_MOVE;
        SendInput(1, &input, sizeof(INPUT));
    }
    else if (payload == "OFF") {
        SendMessage(window, WM_SYSCOMMAND, SC_MONITORPOWER, 2); // -1: on, 1: low power, 2: off 
    }
    else {
        std::cerr << "Unrecognized power mode " << payload << std::endl;
        return;
    }
}

void publish_monitors_all(const std::string &payload) {
    queue.publish(topic_all_power_state, payload, &global_options);
}



enum VCPCode {
    None = 0,
    PowerMode = 0xD6
};

struct ParamSource {
    DWORD flag;
    VCPCode vcp;
    std::vector<uint8_t> vcp_domain;
};

static ParamSource PFlag(DWORD flag) {
    return ParamSource{ flag, VCPCode::None };
}
static ParamSource PVCP(VCPCode vcp) {
    return ParamSource{ 0, vcp };
}

struct Param {
    std::string name;
    ParamSource source;
    std::function<std::string(HANDLE)> get_state;
    std::function<void(HANDLE, std::string)> set_state;

    HANDLE monitor;
    std::string topic_state;
    std::string topic_command;
};


std::vector<Param> available_params{
    { "power", PVCP(VCPCode::PowerMode),
        [](HANDLE monitor) {
            MC_VCP_CODE_TYPE vcp_type;
            DWORD cur;
            DWORD max;
            if (!GetVCPFeatureAndVCPFeatureReply(
                monitor,
                VCPCode::PowerMode,
                &vcp_type,
                &cur,
                &max
            )) return std::string("OFF");
            return std::string(cur == 0x01 ? "ON" : "OFF");
        },
        [](HANDLE monitor, std::string payload) {
            uint8_t power;
            if (payload == "ON") power = 0x01;
            else if (payload == "OFF") power = 0x04;
            else {
                std::cerr << "Unrecognized power mode " << payload << std::endl;
                return;
            }

            if (!SetVCPFeature(
                monitor,
                VCPCode::PowerMode,
                power
            )) std::cerr << "Unable to set power mode to " << (int)power << std::endl;
        }
    },
    { "brightness", PFlag(MC_CAPS_BRIGHTNESS),
        [](HANDLE monitor) {
            DWORD min;
            DWORD cur;
            DWORD max;
            if (!GetMonitorBrightness(
                monitor, &min, &cur, &max 
            )) return std::string("-1");
            std::ostringstream stream;
            stream << cur*0xFF/100;
            return stream.str();
        },
        [](HANDLE monitor, std::string payload) {
            int brightness = std::stoi(payload)*100/0xFF;
            if (!SetMonitorBrightness(monitor, brightness)) {
                std::cerr << "Unable to set brightness to " << brightness << std::endl;
            }
        },
    },
    { "temperature", PFlag(MC_CAPS_COLOR_TEMPERATURE),
        [](HANDLE monitor) {
            MC_COLOR_TEMPERATURE temp;
            if (!GetMonitorColorTemperature(
                monitor, &temp
            )) return std::string("-3");
            
            int kelvin = -2;

            switch (temp) {
            case MC_COLOR_TEMPERATURE_UNKNOWN: kelvin = -1; break;
            case MC_COLOR_TEMPERATURE_4000K: kelvin = 4000; break;
            case MC_COLOR_TEMPERATURE_5000K: kelvin = 5000; break;
            case MC_COLOR_TEMPERATURE_6500K: kelvin = 6500; break;
            case MC_COLOR_TEMPERATURE_7500K: kelvin = 7500; break;
            case MC_COLOR_TEMPERATURE_8200K: kelvin = 8200; break;
            case MC_COLOR_TEMPERATURE_9300K: kelvin = 9300; break;
            case MC_COLOR_TEMPERATURE_10000K: kelvin = 10000; break;
            case MC_COLOR_TEMPERATURE_11500K: kelvin = 11500; break;
            }

            int mired = 1000000 / kelvin;

            std::ostringstream stream;
            stream << mired;
            return stream.str();
        },
        [](HANDLE monitor, std::string payload) {
            int mired = std::stoi(payload);
            int kelvin = 1000000 / mired;

            MC_COLOR_TEMPERATURE temp;
            
            if (kelvin < 0) temp = MC_COLOR_TEMPERATURE_UNKNOWN;
            else if (kelvin < 4500) temp = MC_COLOR_TEMPERATURE_4000K;
            else if (kelvin < 5750) temp = MC_COLOR_TEMPERATURE_5000K;
            else if (kelvin < 7000) temp = MC_COLOR_TEMPERATURE_6500K;
            else if (kelvin < 7850) temp = MC_COLOR_TEMPERATURE_7500K;
            else if (kelvin < 8750) temp = MC_COLOR_TEMPERATURE_8200K;
            else if (kelvin < 9300) temp = MC_COLOR_TEMPERATURE_9300K;
            else if (kelvin < 9650) temp = MC_COLOR_TEMPERATURE_10000K;
            else temp = MC_COLOR_TEMPERATURE_11500K;

            if (temp == MC_COLOR_TEMPERATURE_UNKNOWN) return;

            if (!SetMonitorColorTemperature(monitor, temp)) {
                std::cerr << "Unable to set color temperature to " << temp << std::endl;
            }
        }
    },
    { "contrast", PFlag(MC_CAPS_CONTRAST),
        [](HANDLE monitor) {
            DWORD min;
            DWORD cur;
            DWORD max;
            if (!GetMonitorContrast(
                monitor, &min, &cur, &max
            )) return std::string("-1");
            std::ostringstream stream;
            stream << cur * 0xFF / 100;
            return stream.str();
        },
            [](HANDLE monitor, std::string payload) {
            int contrast = std::stoi(payload) * 100 / 0xFF;
            if (!SetMonitorContrast(monitor, contrast)) {
                std::cerr << "Unable to set contrast to " << contrast << std::endl;
            }
        }
    },
    { "degauss", PFlag(MC_CAPS_DEGAUSS) },
    { "technology", PFlag(MC_CAPS_MONITOR_TECHNOLOGY_TYPE),
        [](HANDLE monitor) {
            MC_DISPLAY_TECHNOLOGY_TYPE type;

            if (!GetMonitorTechnologyType(
                monitor, &type
            )) return std::string("N/A");

            switch (type)
            {

            case MC_SHADOW_MASK_CATHODE_RAY_TUBE:
                return std::string("Shadow Mask Cathode Ray Tube (CRT)");

            case MC_APERTURE_GRILL_CATHODE_RAY_TUBE:
                return std::string("Aperture Grill Cathode Ray Tube (CRT)");

            case MC_THIN_FILM_TRANSISTOR:
                return std::string("Thin Film Transistor (TFT)");

            case MC_LIQUID_CRYSTAL_ON_SILICON:
                return std::string("Liquid Crystal on Silicon (LCoS)");

            case MC_PLASMA:
                return std::string("Plasma");

            case MC_ORGANIC_LIGHT_EMITTING_DIODE:
                return std::string("Organic Light Emitting Diode (OLED)");

            case MC_ELECTROLUMINESCENT:
                return std::string("Electroluminescent (ELD)");

            case MC_MICROELECTROMECHANICAL:
                return std::string("Microelectromechanical");

            case MC_FIELD_EMISSION_DEVICE:
                return std::string("Field Emission Display (FED)");

            }
            return std::string("N/A");
        }
    },
    { "rgb-drive", PFlag(MC_CAPS_RED_GREEN_BLUE_DRIVE) },
    { "rgb-gain", PFlag(MC_CAPS_RED_GREEN_BLUE_GAIN) }
};

struct Cap {
    std::string name;
    std::vector<Cap> list;
};

static bool is_whitespace(const std::string &str)
{
    for (std::string::const_iterator it = str.begin(); it != str.end(); ++it)
    {
        if (*it != ' ' &&
            *it != '\t' &&
            *it != '\r' &&
            *it != '\n') return false;
    }
    return true;
}

static Cap parse_capstring(const char **pos) {
    Cap cap;
    std::string str;
    while (**pos > 0) {
        switch (**pos) {
            case '(':
                {
                    (*pos)++;
                    Cap sub = parse_capstring(pos);
                    if (**pos != ')') goto break_loop;
                    sub.name = str;
                    str.clear();
                    cap.list.push_back(sub);
                }
                break;
            case ')':
                cap.list.push_back(Cap{str});
                goto break_loop;
            case ' ':
            case '\t':
            case '\r':
            case '\n':
                if (!is_whitespace(str)) {
                    cap.list.push_back(Cap{str});
                }
                str.clear();
                break;
            default:
                str += **pos;
        }
        (*pos)++;
    }
break_loop:
    return cap;
}

static void dump_cap(const Cap &cap, int level = 0) {
    for (size_t i = 0; i < cap.list.size(); i++) {
        std::cout << std::string(level*2, ' ') << i << ": " << cap.list[i].name << std::endl;
        dump_cap(cap.list[i], level + 1);
    }
}

static Cap parse_capabilities(const char *capabilities) {
    const char *pos = capabilities;
    if (*pos != '(') return Cap{};
    pos++;
    Cap cap = parse_capstring(&pos);
    if (*pos != ')') return Cap{};
    return cap;
}

struct Screen {
    int index;
    HANDLE monitor;
    std::wstring desc;

    DWORD cap_flags;
    DWORD cap_color_temp;
    Cap capabilities;
    std::string model;
    std::vector<Param> supported_params;

    MessageQueue *queue;
    std::string topic_prefix;
    int qos;

    Screen(int index, HANDLE monitor, std::wstring desc) : index(index), monitor(monitor), desc(desc) {
        updateInfo();
    };
    ~Screen() {
        DestroyPhysicalMonitor(monitor);
        monitor = nullptr;
    }

    void printError(const char *msg) {
        DWORD lastError = GetLastError();
        wchar_t lastErrorMsg[1024];
        bool lastErrorValid = FormatMessageW(
            FORMAT_MESSAGE_FROM_SYSTEM,
            NULL,
            lastError,
            MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
            lastErrorMsg,
            sizeof(lastErrorMsg),
            NULL
        ) > 0;
        std::wcerr << index << ": " << desc << ": " << msg;
        if (lastErrorValid) std::wcerr << ": " << lastErrorMsg;
        std::wcerr << " (" << GetLastError() << ")" << std::endl;
    }

    void updateInfo() {
        std::wcout << index << ": " << desc << " (" << monitor << ")" << std::endl;

        {
            printf("  flags\n");
            if (!GetMonitorCapabilities(
                monitor,
                &cap_flags,
                &cap_color_temp
            )) {
                printError("Error updating flags");
                return;
            }

            for (auto param : available_params) {
                if (cap_flags & param.source.flag) {
                    std::cout << "    " << param.name << std::endl;
                    supported_params.push_back(param);
                }
            }
        }

        {
            printf("  low level\n");
            printf("    length\n");
            DWORD cap_len;
            if (!GetCapabilitiesStringLength(monitor, &cap_len)) {
                printError("Error getting length");
                return;
            }
            std::string cap;
            cap.resize(cap_len);

            printf("    request\n");
            if (!CapabilitiesRequestAndCapabilitiesReply(
                monitor,
                (LPSTR)&cap[0],
                cap_len
            )) {
                printError("Request failed");
                return;
            }

            printf("    parse\n");
            capabilities = parse_capabilities(&cap[0]);

            for (Cap cap_main : capabilities.list) {
                if (cap_main.name == "model") {
                    if (cap_main.list.size() > 0) {
                        model = cap_main.list[0].name;
                    }
                }
            }
            printf("    model\n");
            std::cout << "      " << model << std::endl;

            for (Cap cap_main : capabilities.list) {
                if (cap_main.name == "vcp") {
                    for (Cap vcp : cap_main.list) {
                        if (vcp.name.empty()) continue;
                        uint8_t vcp_code = std::stoi(vcp.name, 0, 16);
                        for (auto param : available_params) {
                            if (param.source.vcp == vcp_code) {
                                std::cout << "    " << param.name << std::endl;
                                supported_params.push_back(param);
                                Param &added = supported_params.back();
                                for (auto cap_domain : vcp.list) {
                                    uint8_t dom_code = std::stoi(cap_domain.name, 0, 16);
                                    std::cout << "      " << (int)dom_code << std::endl;
                                    param.source.vcp_domain.push_back(dom_code);
                                }
                            }
                        }
                    }
                }
            }

            // Uncomment to print all the capabilities
            //dump_cap(capabilities);
        }

        printf("  done\n\n");
    }

    void publish(const Param &param, std::string state_override = "")
    {
        std::string state = state_override.empty() ? param.get_state(monitor) : state_override;
        printf("< %s: %s\n", param.topic_state.c_str(), state.c_str());
        queue->publish(param.topic_state, state, &global_options);
    }

    void publishAll()
    {
        for (auto &param : supported_params) {
            if (!param.topic_state.empty()) {
                publish(param);
            }
        }
    }

    bool receive(const std::string &topic, const std::string &payload)
    {
        for (auto &param : supported_params) {
            if (param.topic_command == topic) {
                param.set_state(monitor, payload);
                publish(param, payload);
                // Maybe update the real state in a bit?
                //std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                //publish(param);
                return true;
            }
        }
        return false;
    }

    void connect(MessageQueue *queue)
    {
        if (queue == nullptr) return;

        this->queue = queue;
        this->topic_prefix = global_topic_prefix + model + "/";
        
        for (auto &param : supported_params) {
            if (param.get_state) {
                param.topic_state = topic_prefix + param.name + "/state";
                std::cout << "Publishing to " << param.topic_state << std::endl;
            }
            if (param.set_state) {
                param.topic_command = topic_prefix + param.name + "/command";
                queue->subscribe(param.topic_command);
            }
        }
        publishAll();
    }

};

std::vector<std::shared_ptr<Screen>> screens;

// Might be useful, but not right now.
/*
bool get_monitor_device(int index, DISPLAY_DEVICE &device) {
    bool valid = true;

    ZeroMemory(&device, sizeof(DISPLAY_DEVICE));
    device.cb = sizeof(DISPLAY_DEVICE);

    if (EnumDisplayDevices(NULL, index, &device, 0)) {

        CHAR deviceName[32];
        // DeviceName is first the gpu name
        lstrcpy(deviceName, device.DeviceName);

        std::cout << "device " << deviceName << std::endl;

        // DeviceName is now the monitor name
        EnumDisplayDevices(deviceName, 0, &device, 0);
    }
    else {
        valid = false;
    }

    return valid;
}
//*/

BOOL CALLBACK EnumDisplayCallback(
    _In_ HMONITOR hMonitor,
    _In_ HDC      hdcMonitor,
    _In_ LPRECT   lprcMonitor,
    _In_ LPARAM   dwData
) {
    DWORD physical_num;

    if (!GetNumberOfPhysicalMonitorsFromHMONITOR(hMonitor, &physical_num)) {
        std::cerr << "Unable to get number of physical monitors" << std::endl;
        return true;
    }

    std::vector<PHYSICAL_MONITOR> monitors;

    monitors.resize(physical_num);

    bool success = GetPhysicalMonitorsFromHMONITOR(hMonitor, physical_num, &monitors[0]) == TRUE;
    if (!success) {
        std::cerr << "Unable to get physical monitors" << std::endl;
        return true;
    }

    for (DWORD i = 0; i < physical_num; i++) {
        PHYSICAL_MONITOR *mon = &monitors[i];
        
        std::shared_ptr<Screen> ptr = std::make_shared<Screen>(
            screens.size(),
            mon->hPhysicalMonitor,
            std::wstring(mon->szPhysicalMonitorDescription)
        );
        Screen &screen = *ptr;
        screens.push_back(ptr);
    }

    return true;
}




void receive_monitors(const std::string &topic, const std::string &payload) {
    if (topic == topic_all_power_command) {
        receive_monitors_all(payload);
        return;
    }
    for (auto ptr : screens) {
        Screen &screen = *ptr;
        bool handled = screen.receive(topic, payload);
        if (handled) break;
    }
}

void connect_monitors() {
    for (auto ptr : screens) {
        Screen &screen = *ptr;
        screen.connect(&queue);
    }
}

void update_monitors() {

    /* Might be useful in a bit.
    DISPLAY_DEVICE monitor;
    for (int i = 0; get_monitor_device(i, monitor); i++) {
        std::cout << i << ":" << std::endl;
        std::cout << monitor.DeviceID << std::endl;
        std::cout << monitor.DeviceKey << std::endl;
        std::cout << monitor.DeviceName << std::endl;
        std::cout << monitor.DeviceString << std::endl;
        std::cout << monitor.StateFlags << std::endl;
    }
    */

    screens.clear();
    if (!EnumDisplayMonitors(NULL, NULL, EnumDisplayCallback, 0)) {
        std::cerr << "Unable to enumerate display monitors" << std::endl;
    }
    connect_monitors();
}

long __stdcall WindowProcedure(HWND window, unsigned int msg, WPARAM wp, LPARAM lp)
{
    switch (msg)
    {
    case WM_DISPLAYCHANGE:
        std::cout << "Detected display change " << wp << " " << lp << std::endl;
        update_monitors();
        return 0L;
    case WM_POWERBROADCAST:
        switch (wp) {
        case PBT_POWERSETTINGCHANGE:
            auto setting = (POWERBROADCAST_SETTING*)lp;
            if (setting->PowerSetting == GUID_MONITOR_POWER_ON) {
                DWORD state = *(DWORD*)&setting->Data[0];
                publish_monitors_all(state == 1 ? "ON" : "OFF");
            }
            // This is the newer API to use, but that just seems to make it less compatible?
            /* else if (setting->PowerSetting == GUID_CONSOLE_DISPLAY_STATE) {
                DWORD state = *(DWORD*)&setting->Data[0];
                std::cout << "GUID_CONSOLE_DISPLAY_STATE " << state << std::endl;
            }*/
            break;
        }

        return 0L;
    case WM_DESTROY:
        std::cout << "\nDestroying window\n";
        PostQuitMessage(0);
        return 0L;
    default:
        return DefWindowProc(window, msg, wp, lp);
    }
}

void windows_run() {
    update_monitors();
    WNDCLASSEX wndclass = {
        sizeof(WNDCLASSEX), 0, WindowProcedure,
        0, 0, GetModuleHandle(0), nullptr,
        nullptr, nullptr,
        0, name.c_str(), nullptr };
    if (RegisterClassEx(&wndclass))
    {
        window = CreateWindowEx(0, name.c_str(), "Spooky hidden ghost title!",
            WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT,
            CW_USEDEFAULT, CW_USEDEFAULT, 0, 0, GetModuleHandle(0), 0);
        if (window)
        {
            MSG msg;
            RegisterPowerSettingNotification(window, &GUID_MONITOR_POWER_ON, 0);
            // Alternative newer API
            //RegisterPowerSettingNotification(window, &GUID_CONSOLE_DISPLAY_STATE, 0);
            while (GetMessage(&msg, 0, 0, 0)) DispatchMessage(&msg);
        }
    }
}

std::string get_default_topic_prefix() {
    std::string topic_prefix = "pc/";
    char comp_name[MAX_COMPUTERNAME_LENGTH];
    DWORD comp_name_len = MAX_COMPUTERNAME_LENGTH;
    if (GetComputerName(comp_name, &comp_name_len) != 0) {
        topic_prefix.resize(comp_name_len);
        memcpy(&topic_prefix[0], comp_name, comp_name_len);
        std::transform(topic_prefix.begin(), topic_prefix.end(), topic_prefix.begin(), ::tolower);
        topic_prefix += "/";
    }
    topic_prefix += "monitor/";
    return topic_prefix;
}

template <class T>
bool initOptionSilent(const YAML::Node &yaml, std::string prop, std::string name, T* config) {
    auto value = yaml[prop];
    if (value) {
        *config = value.as<T>();
        return true;
    }
    return false;
}

template <class T>
bool initOption(const YAML::Node &yaml, std::string prop, std::string name, T* config) {
    bool exists = initOptionSilent(yaml, prop, name, config);
    if (exists) std::cout << name << ": " << *config << std::boolalpha << std::endl;
    return exists;
}

template <class T>
bool initOptionHidden(const YAML::Node &yaml, std::string prop, std::string name, T* config) {
    bool exists = initOptionSilent(yaml, prop, name, config);
    if (exists) std::cout << name << ": *****" << std::endl;
    return exists;
}

void queueDisconnected(std::string cause) {
    if (!queue.config.reconnect) exit(3);
}

void queueReceived(std::string topic, std::string payload) {
    receive_monitors(topic, payload);
}

int main(int argc, char *argv[]) {

    std::cout << name << "\n\n";

    global_options.retained = true;

    queue.config.client_id = name;
    queue.onReceived = std::bind(&queueReceived, std::placeholders::_1, std::placeholders::_2);
    queue.onDisconnected = std::bind(&queueDisconnected, std::placeholders::_1);

    global_topic_prefix = get_default_topic_prefix();

    try {
        YAML::Node yaml = YAML::LoadFile("config.yaml");

        auto broker = yaml["broker"];
        if (!broker) throw "'broker' not found";
        queue.config.uri = broker.as<std::string>();
        std::cout << "Broker: " << queue.config.uri << std::endl;

        initOption(yaml, "username", "Username", &queue.config.username);
        initOptionHidden(yaml, "password", "Password", &queue.config.password);
        initOption(yaml, "keepalive", "Keep alive", &queue.config.keep_alive_interval);
        initOption(yaml, "prefix", "Topic prefix", &global_topic_prefix);
        initOption(yaml, "reconnect", "Reconnect", &queue.config.reconnect);
        initOption(yaml, "connect timeout", "Connect timeout", &queue.config.connectTimeout);
        initOption(yaml, "disconnect timeout", "Disconnect timeout", &queue.config.disconnectTimeout);
        initOption(yaml, "backoff min", "Backoff minimum", &queue.config.backoff_min);
        initOption(yaml, "backoff max", "Backoff maximum", &queue.config.backoff_max);

        int error = queue.connect();
        if (error) {
            return 3;
        }

    } catch (YAML::BadFile badFile) {
        std::cerr << "Error: config.yaml not found (" << badFile.msg << ")" << std::endl;
        return 1;
    } catch (const char* msg) {
        std::cerr << "Error: " << msg << std::endl;
        return 2;
    }

    std::string topic_prefix_all = global_topic_prefix + "all/";
    topic_all_power_state = topic_prefix_all + "power/state";
    topic_all_power_command = topic_prefix_all + "power/command";
    std::cout << "Publishing to " << topic_all_power_state << std::endl;
    queue.subscribe(topic_all_power_command);
    
    std::thread windows_thread(windows_run);

    int ch;
    do
    {
        ch = _getch();
    } while (ch != 'Q' && ch != 'q' && ch != 3 /* Ctrl-C */);

    {
        std::lock_guard<std::mutex> lock(mutex);
        done = true;
    }
    cv.notify_all();
    
    windows_thread.detach();

    queue.disconnect();

    return 0;
}