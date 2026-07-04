#include "app.h"

#include "collective_drive.h"
#include "config.h"
#include "dcs_bios_refs.h"
#include "dcs_bios_state.h"
#include "f14_roll_assist.h"
#include "logger.h"
#include "power_feedforward.h"
#include "retro_music_guard.h"
#include "retro_xm_player.h"
#include "retro_toggle.h"
#include "rudder_input.h"
#include "tune_session.h"
#include "windows/dcs_bios_udp_client.h"
#include "windows/directinput_axis.h"
#include "windows/fast_export_udp_client.h"
#include "windows/hotkey.h"
#include "windows/vjoy_device.h"
#include "windows/xinput_axis.h"
#include "yaw_damper.h"

#include <array>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <windows.h>
#include <windowsx.h>
#include <tlhelp32.h>
#include <mmsystem.h>

namespace autorudder {
namespace {

#ifndef AUTORUDDER_DEFAULT_PROFILE
#define AUTORUDDER_DEFAULT_PROFILE ""
#endif

std::atomic_bool g_running{true};

BOOL WINAPI console_handler(DWORD control_type) {
    if (control_type == CTRL_C_EVENT || control_type == CTRL_CLOSE_EVENT || control_type == CTRL_BREAK_EVENT) {
        g_running = false;
        return TRUE;
    }
    return FALSE;
}

struct CliOptions {
    std::filesystem::path config_path = "config.ini";
    bool config_explicit = false;
    std::string profile = AUTORUDDER_DEFAULT_PROFILE;
    bool dry_run = false;
    bool list_devices = false;
    bool calibrate_sign = false;
    bool test_output = false;
    std::optional<double> hold_output;
    bool reset_output = false;
    bool drive_collective = false;
    bool probe_power = false;
    bool tune_session = false;
    bool auto_tune = false;
    bool menu = false;
    bool help = false;
};

void print_help() {
    std::cout
        << "AH-64D external yaw FBW / F-14 high-AoA rudder roll assist\n"
        << "Usage: ah64d_auto_rudder [--config PATH] [--dry-run] [--list-devices] [--calibrate-sign] [--tune-session] [--auto-tune]\n\n"
        << "  --list-devices    Print DirectInput and vJoy devices, then exit.\n"
        << "  --dry-run         Decode DCS-BIOS and pedals without writing vJoy.\n"
        << "  --calibrate-sign  Run a low-authority sign comparison on vJoy output.\n"
        << "  --tune-session    Fly normally while logging feedforward and yaw-damping tuning advice.\n"
        << "  --auto-tune       Fly normally while applying conservative tuning changes in memory.\n"
        << "  --test-output     Sweep the configured output vJoy axis for binding diagnostics.\n"
        << "  --hold-output V   Hold the configured output vJoy axis at V in [-1, 1].\n"
        << "  --reset-output   Center rudder and pass current physical collective to vJoy once.\n"
        << "  --probe-power    Log raw engine/FM power probe fields from the fast export script.\n"
        << "  --drive-collective\n"
        << "                    In tune/auto-tune, write vJoy collective passthrough plus small scripted moves.\n"
        << "  --f14             Run the F-14 high-AoA roll assist profile.\n"
        << "  --ah64d           Run the AH-64D auto-rudder profile.\n"
        << "  --profile NAME    Run profile ah64d or f14.\n"
        << "  --menu            Show the retro profile selector.\n"
        << "  --config PATH     Use another config.ini path.\n"
        << "  --help            Print this help.\n";
}

CliOptions parse_args(const std::vector<std::string>& args) {
    CliOptions options;
    for (std::size_t i = 0; i < args.size(); ++i) {
        const std::string& arg = args[i];
        if (arg == "--help" || arg == "-h") {
            options.help = true;
        } else if (arg == "--dry-run") {
            options.dry_run = true;
        } else if (arg == "--list-devices") {
            options.list_devices = true;
        } else if (arg == "--calibrate-sign") {
            options.calibrate_sign = true;
        } else if (arg == "--test-output") {
            options.test_output = true;
        } else if (arg == "--hold-output") {
            if (i + 1 >= args.size()) {
                throw std::runtime_error("--hold-output requires a value in [-1, 1]");
            }
            options.hold_output = std::stod(args[++i]);
            if (*options.hold_output < -1.0 || *options.hold_output > 1.0) {
                throw std::runtime_error("--hold-output value must be in [-1, 1]");
            }
        } else if (arg == "--reset-output") {
            options.reset_output = true;
        } else if (arg == "--probe-power") {
            options.probe_power = true;
        } else if (arg == "--drive-collective") {
            options.drive_collective = true;
        } else if (arg == "--tune-session") {
            options.tune_session = true;
        } else if (arg == "--auto-tune") {
            options.auto_tune = true;
        } else if (arg == "--menu") {
            options.menu = true;
        } else if (arg == "--f14") {
            options.profile = "f14";
        } else if (arg == "--ah64d") {
            options.profile = "ah64d";
        } else if (arg == "--profile") {
            if (i + 1 >= args.size()) {
                throw std::runtime_error("--profile requires ah64d or f14");
            }
            options.profile = args[++i];
        } else if (arg == "--config") {
            if (i + 1 >= args.size()) {
                throw std::runtime_error("--config requires a path");
            }
            options.config_path = args[++i];
            options.config_explicit = true;
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }
    return options;
}

void apply_profile(AppConfig& cfg, const std::string& profile) {
    if (profile.empty()) {
        return;
    }
    if (profile == "f14") {
        cfg.control_mode = "f14_roll_assist";
        cfg.telemetry_source = "fast_export";
        return;
    }
    if (profile == "ah64d") {
        if (cfg.control_mode == "f14_roll_assist") {
            cfg.control_mode = "heading_hold";
        }
        return;
    }
    throw std::runtime_error("--profile must be ah64d or f14");
}

std::string trim(std::string value) {
    const auto not_space = [](unsigned char c) { return !std::isspace(c); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
    return value;
}

std::string uppercase(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    return value;
}

bool contains_case_insensitive(const std::string& text, const std::string& needle) {
    if (needle.empty()) {
        return true;
    }
    return uppercase(text).find(uppercase(needle)) != std::string::npos;
}

std::filesystem::path executable_path() {
    std::wstring buffer(MAX_PATH, L'\0');
    for (;;) {
        const DWORD written = GetModuleFileNameW(nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
        if (written == 0) {
            return {};
        }
        if (written < buffer.size() - 1) {
            buffer.resize(written);
            return std::filesystem::path(buffer);
        }
        buffer.resize(buffer.size() * 2);
    }
}

std::filesystem::path resolve_config_path(const CliOptions& options) {
    if (options.config_explicit || options.config_path.is_absolute() || std::filesystem::exists(options.config_path)) {
        return options.config_path;
    }

    const std::filesystem::path exe = executable_path();
    std::filesystem::path dir = exe.empty() ? std::filesystem::current_path() : exe.parent_path();
    for (int i = 0; i < 5 && !dir.empty(); ++i) {
        const auto candidate = dir / options.config_path;
        if (std::filesystem::exists(candidate)) {
            return candidate;
        }
        const auto parent = dir.parent_path();
        if (parent == dir) {
            break;
        }
        dir = parent;
    }
    return options.config_path;
}

std::optional<std::vector<std::uint8_t>> read_binary_file(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        return std::nullopt;
    }
    return std::vector<std::uint8_t>(
        std::istreambuf_iterator<char>(input),
        std::istreambuf_iterator<char>());
}

std::optional<std::vector<std::uint8_t>> load_retro_music_xm() {
    const std::filesystem::path exe = executable_path();
    const std::filesystem::path exe_dir = exe.empty() ? std::filesystem::current_path() : exe.parent_path();
    const std::array<std::filesystem::path, 4> candidates{
        exe_dir / "retro_music.xm",
        exe_dir / "assets" / "retro" / "retro_music.xm",
        std::filesystem::current_path() / "retro_music.xm",
        std::filesystem::current_path() / "assets" / "retro" / "retro_music.xm",
    };
    for (const auto& candidate : candidates) {
        if (auto bytes = read_binary_file(candidate)) {
            return bytes;
        }
    }
    return std::nullopt;
}

bool start_embedded_music() {
    static std::vector<std::uint8_t> rendered_music;
    static bool attempted_render = false;
    if (!attempted_render) {
        attempted_render = true;
        try {
            const auto music = load_retro_music_xm();
            if (!music) {
                return false;
            }
            rendered_music = render_xm_to_wav(std::span<const std::uint8_t>(*music), 22050, 300);
        } catch (const std::exception&) {
            rendered_music.clear();
        }
    }
    if (rendered_music.empty()) {
        return false;
    }
    return PlaySoundA(
        reinterpret_cast<LPCSTR>(rendered_music.data()),
        nullptr,
        SND_MEMORY | SND_ASYNC | SND_LOOP | SND_NODEFAULT) == TRUE;
}

void stop_embedded_music() {
    PlaySoundA(nullptr, nullptr, 0);
}

bool process_running(const wchar_t* process_name) {
    HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snapshot == INVALID_HANDLE_VALUE) {
        return false;
    }

    PROCESSENTRY32W entry{};
    entry.dwSize = sizeof(entry);
    bool found = false;
    if (Process32FirstW(snapshot, &entry)) {
        do {
            if (_wcsicmp(entry.szExeFile, process_name) == 0) {
                found = true;
                break;
            }
        } while (Process32NextW(snapshot, &entry));
    }
    CloseHandle(snapshot);
    return found;
}

enum RetroButtonId {
    kRetroAh64 = 1001,
    kRetroF14 = 1002,
    kRetroExit = 1003,
    kRetroSfx = 1004,
};

constexpr int kRetroWidth = 440;
constexpr int kRetroHeight = 280;
constexpr UINT kRetroWorkerDone = WM_APP + 1;
constexpr UINT_PTR kRetroTickTimer = 1;
constexpr COLORREF kRetroBg = RGB(15, 12, 34);
constexpr COLORREF kRetroPanel = RGB(27, 22, 58);
constexpr COLORREF kRetroPanelHover = RGB(47, 34, 92);
constexpr COLORREF kRetroCyan = RGB(96, 245, 255);
constexpr COLORREF kRetroPurple = RGB(166, 113, 255);
constexpr COLORREF kRetroPink = RGB(255, 96, 202);
constexpr COLORREF kRetroMuted = RGB(142, 134, 180);
constexpr COLORREF kRetroDim = RGB(88, 80, 128);
constexpr COLORREF kRetroAmber = RGB(255, 213, 104);

int run_normal(CliOptions options, AppConfig cfg);

struct RetroGuiState {
    CliOptions options;
    bool music_started = false;
    RECT ah64_rect{230, 62, 420, 88};
    RECT f14_rect{230, 106, 420, 132};
    RECT sfx_rect{330, 10, 396, 26};
    RECT exit_rect{400, 10, 434, 26};
    int hover_id = 0;
    bool tracking_mouse = false;
    HFONT mono_font = nullptr;
    HFONT small_font = nullptr;
    std::thread worker;
    std::mutex mutex;
    RetroMusicGuard music_guard;
    RetroToggleController toggle{1200};
    bool worker_started = false;
    bool worker_done = false;
    int worker_result = 0;
    std::wstring active_profile = L"NONE";
    std::wstring run_status = L"IDLE: CLICK A PROFILE";
    std::wstring last_log = L"LOG: auto_rudder.log";
    std::filesystem::path log_path = "auto_rudder.log";
};

void stop_retro_music(RetroGuiState& state, const std::wstring& reason) {
    if (!state.music_started) {
        return;
    }
    stop_embedded_music();
    state.music_started = false;
    std::lock_guard lock(state.mutex);
    state.run_status = reason;
}

void reset_run_options(CliOptions& options) {
    options.dry_run = false;
    options.list_devices = false;
    options.test_output = false;
    options.reset_output = false;
    options.hold_output.reset();
    options.calibrate_sign = false;
    options.tune_session = false;
    options.auto_tune = false;
    options.drive_collective = false;
}

void draw_text_line(HDC hdc, int x, int y, COLORREF color, const std::wstring& text) {
    SetTextColor(hdc, color);
    TextOutW(hdc, x, y, text.c_str(), static_cast<int>(text.size()));
}

void draw_dotted_hline(HDC hdc, int y, COLORREF color) {
    HPEN pen = CreatePen(PS_DOT, 1, color);
    HPEN old = static_cast<HPEN>(SelectObject(hdc, pen));
    MoveToEx(hdc, 0, y, nullptr);
    LineTo(hdc, kRetroWidth, y);
    SelectObject(hdc, old);
    DeleteObject(pen);
}

void draw_rect_outline(HDC hdc, const RECT& rect, COLORREF color) {
    HPEN pen = CreatePen(PS_SOLID, 1, color);
    HBRUSH brush = static_cast<HBRUSH>(GetStockObject(NULL_BRUSH));
    HPEN old_pen = static_cast<HPEN>(SelectObject(hdc, pen));
    HBRUSH old_brush = static_cast<HBRUSH>(SelectObject(hdc, brush));
    Rectangle(hdc, rect.left, rect.top, rect.right, rect.bottom);
    SelectObject(hdc, old_pen);
    SelectObject(hdc, old_brush);
    DeleteObject(pen);
}

bool point_in_rect(const RECT& rect, LPARAM lparam) {
    const POINT point{GET_X_LPARAM(lparam), GET_Y_LPARAM(lparam)};
    return PtInRect(&rect, point) == TRUE;
}

int hit_test_retro(const RetroGuiState& state, LPARAM lparam) {
    if (point_in_rect(state.exit_rect, lparam)) {
        return kRetroExit;
    }
    if (point_in_rect(state.sfx_rect, lparam)) {
        return kRetroSfx;
    }
    if (point_in_rect(state.ah64_rect, lparam)) {
        return kRetroAh64;
    }
    if (point_in_rect(state.f14_rect, lparam)) {
        return kRetroF14;
    }
    return 0;
}

void fill_rect_color(HDC hdc, const RECT& rect, COLORREF color) {
    HBRUSH brush = CreateSolidBrush(color);
    FillRect(hdc, &rect, brush);
    DeleteObject(brush);
}

std::wstring widen_utf8(std::string_view value) {
    if (value.empty()) {
        return {};
    }
    const int size = MultiByteToWideChar(
        CP_UTF8, MB_ERR_INVALID_CHARS, value.data(), static_cast<int>(value.size()), nullptr, 0);
    if (size <= 0) {
        return std::wstring(value.begin(), value.end());
    }
    std::wstring result(size, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, value.data(), static_cast<int>(value.size()), result.data(), size);
    return result;
}

std::wstring trim_to_width(std::wstring text, std::size_t max_chars) {
    if (text.size() <= max_chars) {
        return text;
    }
    if (max_chars <= 3) {
        return text.substr(0, max_chars);
    }
    return text.substr(0, max_chars - 3) + L"...";
}

std::string read_last_line(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return {};
    }
    in.seekg(0, std::ios::end);
    const std::streamoff end = in.tellg();
    if (end <= 0) {
        return {};
    }
    const std::streamoff size = std::min<std::streamoff>(end, 8192);
    in.seekg(end - size, std::ios::beg);
    std::string buffer(static_cast<std::size_t>(size), '\0');
    in.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    buffer.resize(static_cast<std::size_t>(in.gcount()));

    std::size_t last = buffer.find_last_not_of("\r\n");
    if (last == std::string::npos) {
        return {};
    }
    const std::size_t previous = buffer.find_last_of("\r\n", last);
    const std::size_t start = previous == std::string::npos ? 0 : previous + 1;
    return buffer.substr(start, last - start + 1);
}

void refresh_retro_log(RetroGuiState& state) {
    std::filesystem::path path;
    {
        std::lock_guard lock(state.mutex);
        path = state.log_path;
    }
    const std::string line = read_last_line(path);
    if (line.empty()) {
        return;
    }
    std::lock_guard lock(state.mutex);
    state.last_log = L"LOG: " + trim_to_width(widen_utf8(line), 52);
}

void set_retro_status(RetroGuiState& state, std::wstring status) {
    std::lock_guard lock(state.mutex);
    state.run_status = std::move(status);
}

std::int64_t steady_milliseconds() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

RetroProfile retro_profile_from_button(int id) {
    if (id == kRetroAh64) {
        return RetroProfile::Ah64d;
    }
    if (id == kRetroF14) {
        return RetroProfile::F14;
    }
    return RetroProfile::None;
}

std::wstring retro_profile_name(RetroProfile profile) {
    switch (profile) {
    case RetroProfile::Ah64d: return L"AH-64D";
    case RetroProfile::F14: return L"F-14";
    case RetroProfile::None: break;
    }
    return L"NONE";
}

std::string retro_profile_arg(RetroProfile profile) {
    switch (profile) {
    case RetroProfile::Ah64d: return "ah64d";
    case RetroProfile::F14: return "f14";
    case RetroProfile::None: break;
    }
    return {};
}

void start_retro_profile(HWND hwnd, RetroGuiState& state, int id) {
    if (id == kRetroExit) {
        DestroyWindow(hwnd);
        return;
    }
    const RetroProfile clicked_profile = retro_profile_from_button(id);
    const std::wstring profile_name = retro_profile_name(clicked_profile);

    bool join_finished = false;
    {
        std::lock_guard lock(state.mutex);
        join_finished = state.worker_started && state.worker_done;
    }
    if (join_finished && state.worker.joinable()) {
        state.worker.join();
    }

    RetroToggleAction action;
    {
        std::lock_guard lock(state.mutex);
        action = state.toggle.click_profile(clicked_profile, steady_milliseconds());
        if (action.kind == RetroToggleActionKind::Stop) {
            g_running = false;
            state.run_status = L"STOPPING: " + retro_profile_name(action.profile);
            InvalidateRect(hwnd, nullptr, FALSE);
            return;
        }
        if (action.kind == RetroToggleActionKind::WaitForRelease) {
            state.run_status = L"WAIT: vJoy RELEASE";
            InvalidateRect(hwnd, nullptr, FALSE);
            return;
        }
        if (action.kind == RetroToggleActionKind::BlockedByActiveProfile) {
            state.run_status = L"ACTIVE: STOP " + retro_profile_name(action.profile) + L" FIRST";
            InvalidateRect(hwnd, nullptr, FALSE);
            return;
        }
        if (action.kind != RetroToggleActionKind::Start) {
            return;
        }
    }

    CliOptions run_options = state.options;
    reset_run_options(run_options);
    run_options.profile = retro_profile_arg(clicked_profile);
    if (state.music_guard.profile_start(state.music_started) == RetroMusicAction::StopForProfileStart) {
        stop_retro_music(state, L"SFX OFF: PROFILE STARTED");
    }

    {
        std::lock_guard lock(state.mutex);
        state.worker_started = true;
        state.worker_done = false;
        state.worker_result = 0;
        state.active_profile = profile_name;
        state.run_status = L"STARTING: " + profile_name;
        state.last_log = L"LOG: starting...";
    }
    g_running = true;
    InvalidateRect(hwnd, nullptr, FALSE);

    state.worker = std::thread([hwnd, &state, run_options, clicked_profile, profile_name] {
        int result = 1;
        std::wstring final_status;
        try {
            const std::filesystem::path config_path = resolve_config_path(run_options);
            if (!std::filesystem::exists(config_path)) {
                write_default_config(config_path);
            }
            AppConfig cfg = load_config(config_path);
            apply_profile(cfg, run_options.profile);
            bool cancelled_before_run = false;
            {
                std::lock_guard lock(state.mutex);
                state.toggle.mark_running(clicked_profile);
                cancelled_before_run = state.toggle.stopping() || !g_running.load();
                state.log_path = cfg.log_path;
                state.run_status = cancelled_before_run
                    ? L"STOPPING: " + profile_name
                    : L"RUNNING: " + profile_name + L" ENABLED";
            }
            PostMessageW(hwnd, kRetroWorkerDone, 0, 0);
            if (cancelled_before_run) {
                result = 0;
                final_status = L"STOPPED: " + profile_name;
            } else {
                result = run_normal(run_options, std::move(cfg));
                final_status = result == 0
                    ? L"STOPPED: " + profile_name
                    : L"STOPPED: " + profile_name + L" EXIT=" + std::to_wstring(result);
            }
        } catch (const std::exception& ex) {
            final_status = L"ERROR: " + trim_to_width(widen_utf8(ex.what()), 48);
        }
        {
            std::lock_guard lock(state.mutex);
            state.toggle.mark_stopped(steady_milliseconds());
            state.worker_result = result;
            state.worker_done = true;
            state.run_status = std::move(final_status);
        }
        PostMessageW(hwnd, kRetroWorkerDone, 0, 0);
    });
}

void toggle_retro_sfx(HWND hwnd, RetroGuiState& state) {
    bool profile_active = false;
    {
        std::lock_guard lock(state.mutex);
        profile_active = state.toggle.active();
    }
    const bool dcs_running = process_running(L"DCS.exe");
    const RetroSfxAction action = retro_sfx_click(state.music_started, dcs_running, profile_active);
    if (action == RetroSfxAction::Stop) {
        stop_retro_music(state, L"SFX OFF: CLICKED");
    } else if (action == RetroSfxAction::Start) {
        const bool started = start_embedded_music();
        {
            std::lock_guard lock(state.mutex);
            state.music_started = started;
            state.run_status = started ? L"SFX ON: XM MODULE" : L"SFX OFF: XM PLAYBACK FAILED";
        }
    } else {
        std::lock_guard lock(state.mutex);
        state.run_status = dcs_running ? L"SFX LOCKED: DCS RUNNING" : L"SFX LOCKED: PROFILE ACTIVE";
    }
    InvalidateRect(hwnd, nullptr, FALSE);
}

void paint_retro_gui(HWND hwnd, RetroGuiState& state) {
    PAINTSTRUCT ps{};
    HDC hdc = BeginPaint(hwnd, &ps);
    RECT client{};
    GetClientRect(hwnd, &client);
    std::wstring run_status;
    std::wstring active_profile;
    std::wstring last_log;
    bool active = false;
    bool stopping = false;
    {
        std::lock_guard lock(state.mutex);
        run_status = state.run_status;
        active_profile = state.active_profile;
        last_log = state.last_log;
        active = state.worker_started && !state.worker_done;
        stopping = active && run_status.rfind(L"STOPPING:", 0) == 0;
    }

    HBRUSH background = CreateSolidBrush(kRetroBg);
    FillRect(hdc, &client, background);
    DeleteObject(background);
    SetBkMode(hdc, TRANSPARENT);

    SelectObject(hdc, state.mono_font);
    draw_text_line(hdc, 8, 10, kRetroCyan, L"rudder helper");
    if (state.hover_id == kRetroSfx) {
        fill_rect_color(hdc, state.sfx_rect, kRetroPanelHover);
    }
    draw_rect_outline(hdc, state.sfx_rect, state.hover_id == kRetroSfx ? kRetroPink : kRetroDim);
    draw_text_line(hdc, 336, 10, state.hover_id == kRetroSfx ? kRetroPink : kRetroMuted,
        state.music_started ? L"SFX ON" : L"SFX OFF");
    draw_text_line(hdc, 402, 10, state.hover_id == kRetroExit ? kRetroPink : kRetroCyan, L"EXIT");
    draw_dotted_hline(hdc, 30, kRetroPurple);

    RECT art_box{50, 50, 182, 182};
    HBRUSH art_fill = CreateSolidBrush(RGB(22, 18, 52));
    FillRect(hdc, &art_box, art_fill);
    DeleteObject(art_fill);
    draw_rect_outline(hdc, art_box, RGB(76, 68, 140));
    SelectObject(hdc, state.mono_font);
    draw_text_line(hdc, 63, 60, kRetroCyan, L"  ___   ___ ");
    draw_text_line(hdc, 63, 78, kRetroCyan, L" / _ \\ / __|");
    draw_text_line(hdc, 63, 96, kRetroPurple, L"| (_) | (__ ");
    draw_text_line(hdc, 63, 114, kRetroPurple, L" \\___/ \\___|");
    draw_text_line(hdc, 63, 136, kRetroAmber, L" AH64  F14 ");
    draw_text_line(hdc, 63, 154, RGB(214, 206, 255), L" vJoy MIXER");

    SelectObject(hdc, state.mono_font);
    if (state.hover_id == kRetroAh64) {
        fill_rect_color(hdc, state.ah64_rect, kRetroPanelHover);
    }
    if (state.hover_id == kRetroF14) {
        fill_rect_color(hdc, state.f14_rect, kRetroPanelHover);
    }
    draw_rect_outline(hdc, state.ah64_rect, state.hover_id == kRetroAh64 ? kRetroPink : kRetroDim);
    draw_rect_outline(hdc, state.f14_rect, state.hover_id == kRetroF14 ? kRetroPink : kRetroDim);
    draw_text_line(hdc, 236, 68, state.hover_id == kRetroAh64 ? kRetroPink : kRetroCyan,
        active && active_profile == L"AH-64D" ? L"ON/OFF - AH-64D" : L"CLICK  - AH-64D");
    draw_text_line(hdc, 236, 112, state.hover_id == kRetroF14 ? kRetroPink : kRetroCyan,
        active && active_profile == L"F-14" ? L"ON/OFF - F-14" : L"CLICK  - F-14");
    draw_text_line(hdc, 230, 150, kRetroMuted, L"AH-64D: COL2YAW / Heading Hold");
    draw_text_line(hdc, 230, 172, kRetroMuted, L"F-14: Roll Washout / Mixer");
    draw_text_line(hdc, 230, 198, active ? kRetroPink : kRetroMuted, trim_to_width(run_status, 27));

    draw_dotted_hline(hdc, 232, kRetroPurple);
    const wchar_t* status = active ? L"INTERACTION: CLICK ACTIVE PROFILE TO STOP" : L"INTERACTION: CLICK HOTSPOTS ENABLED";
    if (state.hover_id == kRetroAh64) {
        if (stopping) {
            status = L"STOPPING: WAITING FOR vJoy RELEASE";
        } else if (active && active_profile == L"AH-64D") {
            status = L"READY: CLICK AH-64D TO STOP";
        } else if (active) {
            status = L"ACTIVE: STOP CURRENT PROFILE FIRST";
        } else {
            status = L"READY: CLICK TO ACTIVATE AH-64D";
        }
    } else if (state.hover_id == kRetroF14) {
        if (stopping) {
            status = L"STOPPING: WAITING FOR vJoy RELEASE";
        } else if (active && active_profile == L"F-14") {
            status = L"READY: CLICK F-14 TO STOP";
        } else if (active) {
            status = L"ACTIVE: STOP CURRENT PROFILE FIRST";
        } else {
            status = L"READY: CLICK TO ACTIVATE F-14";
        }
    } else if (state.hover_id == kRetroExit) {
        status = L"READY: CLICK TO EXIT";
    } else if (state.hover_id == kRetroSfx) {
        status = state.music_started ? L"READY: CLICK SFX TO OFF" : L"READY: CLICK SFX TO ON";
    }
    draw_text_line(hdc, 60, 240, kRetroMuted, L"-- zombiesyard presents --");
    draw_dotted_hline(hdc, 255, kRetroPurple);
    draw_text_line(hdc, 8, 258, kRetroMuted, trim_to_width(last_log, 52));
    draw_text_line(hdc, 8, 270, active ? kRetroPink : kRetroMuted, trim_to_width(status, 52));

    EndPaint(hwnd, &ps);
}

void select_retro_option(HWND hwnd, RetroGuiState& state, int id) {
    if (id == kRetroSfx) {
        toggle_retro_sfx(hwnd, state);
        return;
    }
    start_retro_profile(hwnd, state, id);
}

LRESULT CALLBACK retro_wnd_proc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam) {
    auto* state = reinterpret_cast<RetroGuiState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    switch (message) {
    case WM_CREATE: {
        auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
        state = static_cast<RetroGuiState*>(create->lpCreateParams);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(state));
        state->mono_font = CreateFontW(14, 0, 0, 0, FW_BOLD, TRUE, FALSE, FALSE, ANSI_CHARSET,
            OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, DEFAULT_QUALITY, FF_MODERN, L"Consolas");
        state->small_font = CreateFontW(23, 0, 0, 0, FW_HEAVY, FALSE, FALSE, FALSE, ANSI_CHARSET,
            OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, DEFAULT_QUALITY, FF_MODERN, L"Consolas");
        SetTimer(hwnd, kRetroTickTimer, 500, nullptr);
        return 0;
    }
    case WM_NCHITTEST:
        if (state) {
            POINT point{GET_X_LPARAM(lparam), GET_Y_LPARAM(lparam)};
            ScreenToClient(hwnd, &point);
            if (PtInRect(&state->exit_rect, point) ||
                PtInRect(&state->sfx_rect, point) ||
                PtInRect(&state->ah64_rect, point) ||
                PtInRect(&state->f14_rect, point)) {
                return HTCLIENT;
            }
        }
        return HTCAPTION;
    case WM_LBUTTONDOWN:
        if (state) {
            const int hit = hit_test_retro(*state, lparam);
            if (hit != 0) {
                select_retro_option(hwnd, *state, hit);
                return 0;
            }
            SendMessageW(hwnd, WM_NCLBUTTONDOWN, HTCAPTION, 0);
            return 0;
        }
        break;
    case WM_MOUSEMOVE:
        if (state) {
            const int hit = hit_test_retro(*state, lparam);
            SetCursor(LoadCursorW(nullptr, hit != 0 ? MAKEINTRESOURCEW(32649) : MAKEINTRESOURCEW(32512)));
            if (hit != state->hover_id) {
                state->hover_id = hit;
                InvalidateRect(hwnd, nullptr, FALSE);
            }
            if (!state->tracking_mouse) {
                TRACKMOUSEEVENT track{};
                track.cbSize = sizeof(track);
                track.dwFlags = TME_LEAVE;
                track.hwndTrack = hwnd;
                if (TrackMouseEvent(&track)) {
                    state->tracking_mouse = true;
                }
            }
        }
        return 0;
    case WM_MOUSELEAVE:
        if (state) {
            state->tracking_mouse = false;
            if (state->hover_id != 0) {
                state->hover_id = 0;
                InvalidateRect(hwnd, nullptr, FALSE);
            }
        }
        return 0;
    case WM_PAINT:
        if (state) {
            paint_retro_gui(hwnd, *state);
            return 0;
        }
        break;
    case WM_TIMER:
        if (state && wparam == kRetroTickTimer) {
            if (state->music_guard.update(state->music_started, process_running(L"DCS.exe")) == RetroMusicAction::StopForDcs) {
                stop_retro_music(*state, L"SFX OFF: DCS DETECTED");
            }
            refresh_retro_log(*state);
            InvalidateRect(hwnd, nullptr, FALSE);
        }
        return 0;
    case kRetroWorkerDone:
        if (state) {
            refresh_retro_log(*state);
            InvalidateRect(hwnd, nullptr, FALSE);
        }
        return 0;
    case WM_DESTROY:
        if (state) {
            KillTimer(hwnd, kRetroTickTimer);
            g_running = false;
            if (state->worker.joinable()) {
                state->worker.join();
            }
            if (state->mono_font) DeleteObject(state->mono_font);
            if (state->small_font) DeleteObject(state->small_font);
        }
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(hwnd, message, wparam, lparam);
}

int show_retro_menu(CliOptions options) {
    RetroGuiState state;
    state.options = std::move(options);
    state.music_started = start_embedded_music();

    const wchar_t* class_name = L"AutoRudderRetroLauncher";
    WNDCLASSEXW wc{};
    wc.cbSize = sizeof(wc);
    wc.lpfnWndProc = retro_wnd_proc;
    wc.hInstance = GetModuleHandleW(nullptr);
    wc.hCursor = LoadCursorW(nullptr, MAKEINTRESOURCEW(32512));
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    wc.lpszClassName = class_name;
    RegisterClassExW(&wc);

    constexpr DWORD style = WS_POPUP;
    RECT rect{0, 0, kRetroWidth, kRetroHeight};
    AdjustWindowRectEx(&rect, style, FALSE, 0);
    const int window_width = rect.right - rect.left;
    const int window_height = rect.bottom - rect.top;
    const int screen_width = GetSystemMetrics(SM_CXSCREEN);
    const int screen_height = GetSystemMetrics(SM_CYSCREEN);
    const int window_x = std::max(0, (screen_width - window_width) / 2);
    const int window_y = std::max(0, (screen_height - window_height) / 2);

    HWND hwnd = CreateWindowExW(
        0,
        class_name,
        L"AH-64D / F-14 Rudder FCS",
        style,
        window_x,
        window_y,
        window_width,
        window_height,
        nullptr,
        nullptr,
        GetModuleHandleW(nullptr),
        &state);
    if (!hwnd) {
        stop_embedded_music();
        return 1;
    }

    ShowWindow(hwnd, SW_SHOWNORMAL);
    UpdateWindow(hwnd);

    MSG msg{};
    while (GetMessageW(&msg, nullptr, 0, 0) > 0) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }

    stop_embedded_music();
    return state.worker_result;
}

bool is_ah64(const std::string& aircraft_name) {
    return aircraft_name.find("AH-64D") != std::string::npos;
}

bool is_f14(const std::string& aircraft_name) {
    return aircraft_name.find("F-14") != std::string::npos ||
           aircraft_name.find("F14") != std::string::npos;
}

double parse_double_or_zero(const std::string& value) {
    try {
        return std::stod(trim(value));
    } catch (const std::exception&) {
        return 0.0;
    }
}

struct Telemetry {
    std::string aircraft_name;
    bool aircraft_is_ah64 = false;
    bool aircraft_is_f14 = false;
    double yaw_rate_z = 0.0;
    std::optional<double> yaw_rate_y;
    std::optional<double> slip_ball;
    std::optional<double> collective;
    std::optional<double> heading;
    std::optional<double> angle_of_attack;
    std::optional<double> roll_rate_x;
    std::optional<double> indicated_airspeed;
    std::optional<double> radar_altitude;
    std::optional<double> gear_position;
    std::optional<double> flaps_position;
    std::optional<double> engine_rpm_avg;
    std::optional<double> engine_fuel_flow_avg;
    std::optional<double> engine_torque_avg;
    std::optional<double> engine_torque_left;
    std::optional<double> engine_torque_right;
    std::optional<double> tail_rudder_left;
    std::optional<double> tail_rudder_right;
    std::optional<double> yaw_acceleration_z;
    std::optional<double> pitch;
    std::optional<double> bank;
    std::optional<double> attitude_yaw;
    std::optional<double> velocity_x;
    std::optional<double> velocity_y;
    std::optional<double> velocity_z;
    std::optional<double> speed_3d;
    std::optional<double> ground_speed;
    std::optional<double> vertical_velocity;
    std::optional<double> true_airspeed;
    std::optional<double> mach;
    std::optional<double> altitude_msl;
    std::optional<double> latitude;
    std::optional<double> longitude;
    std::optional<double> accel_x;
    std::optional<double> accel_y;
    std::optional<double> accel_z;
    std::optional<double> wind_x;
    std::optional<double> wind_y;
    std::optional<double> wind_z;
};

struct TelemetrySample {
    Telemetry telemetry;
    bool fresh = false;
};

Telemetry read_telemetry(const DcsBiosState& state, const TelemetryRefs& refs) {
    Telemetry telemetry;
    telemetry.aircraft_name = state.read_string(refs.aircraft_name.address, refs.aircraft_name.max_length);
    telemetry.aircraft_is_ah64 = is_ah64(telemetry.aircraft_name);
    telemetry.aircraft_is_f14 = is_f14(telemetry.aircraft_name);
    telemetry.yaw_rate_z = parse_double_or_zero(state.read_string(refs.yaw_rate_z.address, refs.yaw_rate_z.max_length));

    if (refs.slip_ball) {
        const auto raw = state.read_u16(refs.slip_ball->address);
        if (raw) {
            telemetry.slip_ball = (static_cast<double>(*raw) / 65535.0) * 2.0 - 1.0;
        }
    }
    return telemetry;
}

std::string fixed3(double value) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << value;
    return out.str();
}

std::string fixed6(double value) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(6) << value;
    return out.str();
}

std::string maybe3(const std::optional<double>& value) {
    return value ? fixed3(*value) : "NA";
}

std::string maybe6(const std::optional<double>& value) {
    return value ? fixed6(*value) : "NA";
}

double control_yaw_rate(const Telemetry& telemetry) {
    return telemetry.yaw_rate_y.value_or(telemetry.yaw_rate_z);
}

void print_device_lists() {
    std::cout << "DirectInput game controllers:\n";
    const auto devices = windows::DirectInputAxisInput::list_devices();
    if (devices.empty()) {
        std::cout << "  none\n";
    }
    for (const auto& device : devices) {
        std::cout << "  [" << device.index << "] " << device.product_name
                  << " / " << device.instance_name
                  << " instance=" << device.instance_guid
                  << " product=" << device.product_guid << '\n';
        std::cout << "      axes:";
        for (const char* axis : {"X", "Y", "Z", "RX", "RY", "RZ", "SLIDER0", "SLIDER1"}) {
            try {
                windows::DirectInputAxisInput input(device.index, "", axis);
                const auto value = input.read();
                std::cout << ' ' << axis << '=';
                if (value) {
                    std::cout << fixed3(*value);
                } else {
                    std::cout << "NA";
                }
            } catch (const std::exception&) {
                std::cout << ' ' << axis << "=NA";
            }
        }
        std::cout << '\n';
        std::vector<int> pressed_buttons;
        try {
            pressed_buttons = windows::DirectInputButtonInput::pressed_buttons(device.index, "", 32);
        } catch (const std::exception&) {
        }
        std::cout << "      pressed buttons:";
        if (pressed_buttons.empty()) {
            std::cout << " none";
        } else {
            for (const int button : pressed_buttons) {
                std::cout << ' ' << button;
            }
        }
        std::cout << '\n';
    }

    std::cout << "\nXInput controllers:\n";
    const auto xinput_devices = windows::XInputAxisInput::list_devices();
    if (xinput_devices.empty()) {
        std::cout << "  none\n";
    }
    for (const auto& device : xinput_devices) {
        std::cout << "  [" << device.index << "] XInput controller #" << device.index
                  << " packet=" << device.packet_number << '\n';
        std::cout << "      axes:";
        for (const char* axis : {"LX", "LY", "RX", "RY", "LT", "RT"}) {
            try {
                windows::XInputAxisInput input(device.index, axis);
                const auto value = input.read();
                std::cout << ' ' << axis << '=';
                if (value) {
                    std::cout << fixed3(*value);
                } else {
                    std::cout << "NA";
                }
            } catch (const std::exception&) {
                std::cout << ' ' << axis << "=NA";
            }
        }
        std::cout << '\n';
    }

    std::cout << "\nvJoy status:\n";
    for (const auto& status : windows::VJoyDevice::list_statuses()) {
        if (status.id == 0) {
            std::cout << "  " << status.status << '\n';
        } else {
            std::cout << "  vJoy #" << status.id << ": " << status.status << '\n';
        }
    }
}

struct Runtime {
    AppConfig cfg;
    Logger logger;
    TelemetryRefs refs;
    DcsBiosState bios_state;
    windows::DcsBiosUdpClient bios;
    std::optional<windows::FastExportUdpClient> fast_export;
    windows::DirectInputAxisInput pedals;
    std::optional<windows::DirectInputAxisInput> ah64_roll_input;
    std::optional<windows::DirectInputAxisInput> collective_directinput;
    std::optional<windows::XInputAxisInput> collective_xinput;
    std::optional<windows::DirectInputButtonInput> trim_guard_input;
    std::string ah64_roll_input_error;
    std::string collective_input_error;
    std::string trim_guard_input_error;

    Runtime(AppConfig config)
        : cfg(std::move(config)),
          logger(cfg.log_path),
          refs(load_telemetry_refs(cfg.dcs_bios_path)),
          bios_state(),
          bios(cfg.multicast_address, cfg.multicast_interface, cfg.udp_port, bios_state),
          pedals(cfg.input_vjoy_id, cfg.input_device_name_contains, cfg.axis_name) {
        if (cfg.telemetry_source == "fast_export") {
            fast_export.emplace(cfg.fast_export_bind_address, cfg.fast_export_port);
        }
        if (cfg.ah64_roll_enabled > 0.5) {
            try {
                ah64_roll_input.emplace(
                    cfg.ah64_roll_input_id,
                    cfg.ah64_roll_device_name_contains,
                    cfg.ah64_roll_axis_name);
            } catch (const std::exception& ex) {
                ah64_roll_input_error = ex.what();
            }
        }
        const auto init_directinput = [&] {
            collective_directinput.emplace(cfg.collective_input_id, cfg.collective_device_name_contains, cfg.collective_axis_name);
        };
        const auto init_xinput = [&] {
            collective_xinput.emplace(cfg.collective_input_id, cfg.collective_axis_name);
        };

        if (cfg.collective_source == "directinput") {
            init_directinput();
        } else if (cfg.collective_source == "xinput") {
            init_xinput();
        } else if (cfg.collective_source == "auto") {
            try {
                if (cfg.collective_device_name_contains.empty()) {
                    collective_input_error =
                        "collective_device_name_contains is empty; set it to your collective device name or use collective_source=xinput/directinput";
                } else if (contains_case_insensitive(cfg.collective_device_name_contains, "xinput") ||
                           contains_case_insensitive(cfg.collective_device_name_contains, "xbox")) {
                    init_xinput();
                } else {
                    init_directinput();
                }
            } catch (const std::exception& ex) {
                collective_input_error = ex.what();
            }
        }
        if (cfg.trim_guard_enabled > 0.5 && cfg.trim_guard_input_button > 0) {
            try {
                trim_guard_input.emplace(
                    cfg.trim_guard_input_id,
                    cfg.trim_guard_input_device_name_contains,
                    cfg.trim_guard_input_button);
            } catch (const std::exception& ex) {
                trim_guard_input_error = ex.what();
            }
        }
    }

    std::string collective_input_name() const {
        if (collective_xinput) {
            return collective_xinput->selected_name();
        }
        if (collective_directinput) {
            return collective_directinput->selected_name();
        }
        return {};
    }

    std::string ah64_roll_input_name() const {
        if (ah64_roll_input) {
            return ah64_roll_input->selected_name();
        }
        return {};
    }

    std::string trim_guard_input_name() const {
        if (trim_guard_input) {
            return trim_guard_input->selected_name();
        }
        return {};
    }
};

struct F14Runtime {
    AppConfig cfg;
    Logger logger;
    windows::FastExportUdpClient fast_export;
    windows::DirectInputAxisInput roll_input;
    windows::DirectInputAxisInput rudder_input;

    F14Runtime(AppConfig config)
        : cfg(std::move(config)),
          logger(cfg.log_path),
          fast_export(cfg.fast_export_bind_address, cfg.fast_export_port),
          roll_input(
              cfg.f14_roll_input_id > 0 ? cfg.f14_roll_input_id : cfg.input_vjoy_id,
              cfg.f14_roll_device_name_contains.empty() ? cfg.input_device_name_contains : cfg.f14_roll_device_name_contains,
              cfg.f14_roll_axis_name),
          rudder_input(
              cfg.f14_rudder_input_id > 0 ? cfg.f14_rudder_input_id : cfg.input_vjoy_id,
              cfg.f14_rudder_device_name_contains.empty() ? cfg.input_device_name_contains : cfg.f14_rudder_device_name_contains,
              cfg.f14_rudder_axis_name) {}
};

double clamp01(double value) {
    return std::max(0.0, std::min(1.0, value));
}

double clamp_unit(double value) {
    return std::max(-1.0, std::min(1.0, value));
}

double apply_centered_axis_calibration(double raw, double center, double deadzone, double scale) {
    const double clamped_center = std::clamp(center, -0.95, 0.95);
    const double span = raw >= clamped_center ? (1.0 - clamped_center) : (clamped_center + 1.0);
    const double centered = span > 0.000001 ? (raw - clamped_center) / span : 0.0;
    const double dz = std::clamp(deadzone, 0.0, 0.95);
    const double magnitude = std::abs(centered);
    double with_deadzone = 0.0;
    if (magnitude > dz) {
        with_deadzone = std::copysign((magnitude - dz) / (1.0 - dz), centered);
    }
    return clamp_unit(with_deadzone * scale);
}

double move_toward(double current, double target, double max_delta) {
    if (max_delta <= 0.0) {
        return target;
    }
    if (current < target) {
        return std::min(target, current + max_delta);
    }
    return std::max(target, current - max_delta);
}

double apply_symmetric_deadband(double value, double deadband) {
    const double dz = std::clamp(deadband, 0.0, 1.0);
    const double magnitude = std::abs(value);
    if (magnitude <= dz) {
        return 0.0;
    }
    return std::copysign(magnitude - dz, value);
}

double directinput_axis_to_collective(double raw_axis, const AppConfig& cfg) {
    double value = (raw_axis + 1.0) * 0.5;
    if (cfg.collective_invert > 0.5) {
        value = 1.0 - value;
    }
    return clamp01(value);
}

double collective_to_output_axis(double collective, const AppConfig& cfg) {
    double axis = clamp01(collective) * 2.0 - 1.0;
    if (cfg.collective_output_invert > 0.5) {
        axis = -axis;
    }
    return axis;
}

CollectiveDriveConfig make_collective_drive_config(const AppConfig& cfg) {
    CollectiveDriveConfig drive;
    drive.amplitude = cfg.auto_tune_collective_amplitude;
    drive.period = cfg.auto_tune_collective_period;
    drive.settle_time = cfg.auto_tune_collective_settle_time;
    drive.rate_limit = cfg.auto_tune_collective_rate_limit;
    return drive;
}

PowerFeedforwardConfig make_power_feedforward_config(const AppConfig& cfg) {
    PowerFeedforwardConfig power;
    power.source = cfg.power_feedforward_source;
    power.fuel_flow_min = cfg.fuel_flow_min;
    power.fuel_flow_max = cfg.fuel_flow_max;
    power.rpm_nominal = cfg.rpm_nominal;
    power.rpm_drop_full_scale = cfg.rpm_drop_full_scale;
    power.rpm_power_gain = cfg.rpm_power_gain;
    power.collective_lead_gain = cfg.power_collective_lead_gain;
    power.collective_lead_invert = cfg.power_collective_lead_invert;
    power.collective_lead_deadband = cfg.power_collective_lead_deadband;
    return power;
}

PowerFeedforwardOutput read_power_feedforward(
    const AppConfig& cfg,
    const Telemetry& telemetry,
    const std::optional<double>& physical_collective) {
    PowerFeedforwardInput input;
    input.collective = physical_collective;
    input.fuel_flow = telemetry.engine_fuel_flow_avg;
    input.rpm = telemetry.engine_rpm_avg;
    return compute_power_feedforward_input(make_power_feedforward_config(cfg), input);
}

TuneConfig make_tune_config(const AppConfig& cfg) {
    TuneConfig tune;
    tune.kp = cfg.kp;
    tune.heading_hold_max_assist = cfg.heading_hold_max_assist;
    tune.collective_feedforward_mode = cfg.collective_feedforward_mode;
    tune.collective_gain = cfg.collective_gain;
    tune.collective_zero_thrust = cfg.collective_zero_thrust;
    tune.collective_power_exponent = cfg.collective_power_exponent;
    tune.collective_rate_gain = cfg.collective_rate_gain;
    tune.collective_rate_limit = cfg.collective_rate_limit;
    tune.collective_sign = cfg.collective_sign;
    return tune;
}

void apply_tune_config(AppConfig& cfg, const TuneConfig& tune) {
    cfg.kp = tune.kp;
    cfg.heading_hold_max_assist = tune.heading_hold_max_assist;
    cfg.collective_feedforward_mode = tune.collective_feedforward_mode;
    cfg.collective_gain = tune.collective_gain;
    cfg.collective_zero_thrust = tune.collective_zero_thrust;
    cfg.collective_power_exponent = tune.collective_power_exponent;
    cfg.collective_rate_gain = tune.collective_rate_gain;
    cfg.collective_sign = tune.collective_sign;
}

std::string tune_config_summary(const AppConfig& cfg) {
    std::ostringstream out;
    out << "collective_mode=" << cfg.collective_feedforward_mode
        << " ff_source=" << cfg.power_feedforward_source
        << " collective_gain=" << fixed3(cfg.collective_gain)
        << " collective_zero=" << fixed3(cfg.collective_zero_thrust)
        << " collective_exp=" << fixed3(cfg.collective_power_exponent)
        << " collective_rate_gain=" << fixed3(cfg.collective_rate_gain)
        << " collective_rate_limit=" << fixed3(cfg.collective_rate_limit)
        << " kp=" << fixed3(cfg.kp)
        << " heading_hold_max_assist=" << fixed3(cfg.heading_hold_max_assist);
    return out.str();
}

void add_tune_sample(
    TuneSessionAnalyzer& analyzer,
    double dt,
    const std::optional<double>& pedal,
    const TelemetrySample& sample,
    const std::optional<double>& collective,
    const YawDamperOutput& result,
    bool collective_drive_active) {
    TuneSample tune_sample;
    tune_sample.dt = dt;
    tune_sample.pedal = pedal.value_or(0.0);
    tune_sample.heading_rate = result.heading_rate;
    tune_sample.heading_error = result.heading_error;
    tune_sample.collective = collective.value_or(0.0);
    tune_sample.collective_rate = result.collective_rate;
    tune_sample.collective_feedforward = result.collective_feedforward;
    tune_sample.final_rudder = result.final_rudder;
    tune_sample.fresh = sample.fresh;
    tune_sample.aircraft_is_ah64 = sample.telemetry.aircraft_is_ah64;
    tune_sample.input_valid = pedal.has_value();
    tune_sample.heading_valid = sample.telemetry.heading.has_value();
    tune_sample.collective_valid = collective.has_value();
    tune_sample.collective_drive_active = collective_drive_active;
    tune_sample.heading_hold_mode =
        result.reason == "heading hold" || result.reason == "rate hold no heading";
    tune_sample.closed_loop_heading_hold = result.reason == "heading hold";
    analyzer.add(tune_sample);
}

void log_tune_report(Logger& logger, const std::string& label, const TuneReport& report) {
    std::ostringstream line;
    line << label
         << " usable=" << fixed3(report.usable_seconds) << "s"
         << " drive=" << fixed3(report.collective_drive_seconds) << "s"
         << " normal=" << fixed3(report.normal_seconds) << "s"
         << " hRateRms=" << fixed3(report.heading_rate_rms)
         << " hRatePeak=" << fixed3(report.heading_rate_peak)
         << " finalMean=" << fixed3(report.final_abs_mean)
         << " sat=" << fixed3(report.saturation_ratio * 100.0) << "%"
         << " osc/s=" << fixed3(report.oscillation_rate)
         << " staticColl=" << fixed3(report.static_collective_seconds) << "s"
         << " staticFit=" << fixed3(report.static_gain_fit_seconds) << "s"
         << " transColl=" << fixed3(report.collective_transient_seconds) << "s"
         << " rateLim=" << fixed3(report.collective_rate_limited_ratio * 100.0) << "%";
    logger.info(line.str());
    for (const auto& recommendation : report.recommendations) {
        logger.info(label + " recommend: " + recommendation);
    }
}

std::optional<double> read_collective(Runtime& runtime, const Telemetry& telemetry) {
    const auto read_configured_input = [&]() -> std::optional<double> {
        if (runtime.collective_xinput) {
            if (const auto raw = runtime.collective_xinput->read()) {
                return directinput_axis_to_collective(*raw, runtime.cfg);
            }
        }
        if (runtime.collective_directinput) {
            if (const auto raw = runtime.collective_directinput->read()) {
                return directinput_axis_to_collective(*raw, runtime.cfg);
            }
        }
        return std::nullopt;
    };

    if (runtime.cfg.collective_source == "auto") {
        if (telemetry.collective) {
            return telemetry.collective;
        }
        return read_configured_input();
    }
    if (runtime.cfg.collective_source == "xinput") {
        return read_configured_input();
    }
    if (runtime.cfg.collective_source == "directinput") {
        return read_configured_input();
    }
    if (runtime.cfg.collective_source == "fast_export") {
        return telemetry.collective;
    }
    return std::nullopt;
}

std::optional<double> read_ah64_roll(Runtime& runtime, std::optional<double>& raw_roll) {
    raw_roll = std::nullopt;
    if (runtime.cfg.ah64_roll_enabled <= 0.5 || !runtime.ah64_roll_input) {
        return std::nullopt;
    }
    raw_roll = runtime.ah64_roll_input->read();
    if (!raw_roll) {
        return std::nullopt;
    }
    return apply_centered_axis_calibration(
        *raw_roll,
        runtime.cfg.ah64_roll_input_center,
        runtime.cfg.ah64_roll_input_deadzone,
        runtime.cfg.ah64_roll_input_scale);
}

void log_collective_input(Logger& logger, const Runtime& runtime) {
    const std::string name = runtime.collective_input_name();
    if (!name.empty()) {
        logger.info("Selected collective input: " + name);
    } else if (!runtime.collective_input_error.empty()) {
        logger.warn("Collective input unavailable: " + runtime.collective_input_error);
    } else {
        logger.info("Collective source: " + runtime.cfg.collective_source);
    }
}

void log_ah64_roll_input(Logger& logger, const Runtime& runtime) {
    if (runtime.cfg.ah64_roll_enabled <= 0.5) {
        logger.info("AH-64D cyclic roll passthrough disabled");
        return;
    }
    const std::string name = runtime.ah64_roll_input_name();
    if (!name.empty()) {
        logger.info(
            "Selected AH-64D cyclic roll input: " + name +
            " axis " + runtime.cfg.ah64_roll_axis_name +
            " -> vJoy #" + std::to_string(runtime.cfg.output_vjoy_id) +
            " axis " + runtime.cfg.ah64_roll_output_axis_name);
    } else if (!runtime.ah64_roll_input_error.empty()) {
        logger.warn("AH-64D cyclic roll input unavailable: " + runtime.ah64_roll_input_error);
    } else {
        logger.warn("AH-64D cyclic roll input unavailable");
    }
}

void log_trim_guard_input(Logger& logger, const Runtime& runtime) {
    if (runtime.cfg.trim_guard_enabled <= 0.5) {
        return;
    }
    if (runtime.cfg.trim_guard_input_button <= 0) {
        logger.warn("Trim guard enabled but trim_guard_input_button is 0");
        return;
    }
    const std::string name = runtime.trim_guard_input_name();
    if (!name.empty()) {
        std::ostringstream line;
        line << "Trim guard input: " << name
             << " button " << runtime.cfg.trim_guard_input_button;
        if (runtime.cfg.trim_guard_output_button > 0) {
            line << "; forwarding to vJoy #" << runtime.cfg.output_vjoy_id
                 << " button " << runtime.cfg.trim_guard_output_button;
        } else {
            line << "; guard-only mode, DCS trim binding must not sample before suppression";
        }
        logger.info(line.str());
    } else if (!runtime.trim_guard_input_error.empty()) {
        logger.warn("Trim guard input unavailable: " + runtime.trim_guard_input_error);
    }
}

TelemetrySample sample_telemetry(Runtime& runtime) {
    runtime.bios.pump();
    if (runtime.fast_export) {
        runtime.fast_export->pump();
        TelemetrySample sample;
        if (const auto latest = runtime.fast_export->latest()) {
            sample.telemetry.aircraft_name = latest->aircraft_name;
            sample.telemetry.aircraft_is_ah64 = is_ah64(latest->aircraft_name);
            sample.telemetry.aircraft_is_f14 = is_f14(latest->aircraft_name);
            sample.telemetry.yaw_rate_z = latest->yaw_rate_z;
            sample.telemetry.yaw_rate_y = latest->yaw_rate_y;
            sample.telemetry.slip_ball = latest->slip_ball;
            sample.telemetry.collective = latest->collective;
            sample.telemetry.heading = latest->heading;
            sample.telemetry.angle_of_attack = latest->angle_of_attack;
            sample.telemetry.roll_rate_x = latest->roll_rate_x;
            sample.telemetry.indicated_airspeed = latest->indicated_airspeed;
            sample.telemetry.radar_altitude = latest->radar_altitude;
            sample.telemetry.gear_position = latest->gear_position;
            sample.telemetry.flaps_position = latest->flaps_position;
            sample.telemetry.engine_rpm_avg = latest->engine_rpm_avg;
            sample.telemetry.engine_fuel_flow_avg = latest->engine_fuel_flow_avg;
            sample.telemetry.engine_torque_avg = latest->engine_torque_avg;
            sample.telemetry.engine_torque_left = latest->engine_torque_left;
            sample.telemetry.engine_torque_right = latest->engine_torque_right;
            sample.telemetry.tail_rudder_left = latest->tail_rudder_left;
            sample.telemetry.tail_rudder_right = latest->tail_rudder_right;
            sample.telemetry.yaw_acceleration_z = latest->yaw_acceleration_z;
            sample.telemetry.pitch = latest->pitch;
            sample.telemetry.bank = latest->bank;
            sample.telemetry.attitude_yaw = latest->attitude_yaw;
            sample.telemetry.velocity_x = latest->velocity_x;
            sample.telemetry.velocity_y = latest->velocity_y;
            sample.telemetry.velocity_z = latest->velocity_z;
            sample.telemetry.speed_3d = latest->speed_3d;
            sample.telemetry.ground_speed = latest->ground_speed;
            sample.telemetry.vertical_velocity = latest->vertical_velocity;
            sample.telemetry.true_airspeed = latest->true_airspeed;
            sample.telemetry.mach = latest->mach;
            sample.telemetry.altitude_msl = latest->altitude_msl;
            sample.telemetry.latitude = latest->latitude;
            sample.telemetry.longitude = latest->longitude;
            sample.telemetry.accel_x = latest->accel_x;
            sample.telemetry.accel_y = latest->accel_y;
            sample.telemetry.accel_z = latest->accel_z;
            sample.telemetry.wind_x = latest->wind_x;
            sample.telemetry.wind_y = latest->wind_y;
            sample.telemetry.wind_z = latest->wind_z;
        }
        sample.fresh = runtime.fast_export->has_recent_frame(runtime.cfg.stale_timeout);
        return sample;
    }

    TelemetrySample sample;
    sample.telemetry = read_telemetry(runtime.bios_state, runtime.refs);
    sample.fresh = runtime.bios.has_recent_frame(runtime.cfg.stale_timeout);
    return sample;
}

TelemetrySample sample_f14_telemetry(F14Runtime& runtime) {
    runtime.fast_export.pump();
    TelemetrySample sample;
    if (const auto latest = runtime.fast_export.latest()) {
        sample.telemetry.aircraft_name = latest->aircraft_name;
        sample.telemetry.aircraft_is_ah64 = is_ah64(latest->aircraft_name);
        sample.telemetry.aircraft_is_f14 = is_f14(latest->aircraft_name);
        sample.telemetry.yaw_rate_z = latest->yaw_rate_z;
        sample.telemetry.yaw_rate_y = latest->yaw_rate_y;
        sample.telemetry.slip_ball = latest->slip_ball;
        sample.telemetry.heading = latest->heading;
        sample.telemetry.angle_of_attack = latest->angle_of_attack;
        sample.telemetry.roll_rate_x = latest->roll_rate_x;
        sample.telemetry.indicated_airspeed = latest->indicated_airspeed;
        sample.telemetry.radar_altitude = latest->radar_altitude;
        sample.telemetry.gear_position = latest->gear_position;
        sample.telemetry.flaps_position = latest->flaps_position;
        sample.telemetry.engine_rpm_avg = latest->engine_rpm_avg;
        sample.telemetry.engine_fuel_flow_avg = latest->engine_fuel_flow_avg;
        sample.telemetry.engine_torque_avg = latest->engine_torque_avg;
        sample.telemetry.engine_torque_left = latest->engine_torque_left;
        sample.telemetry.engine_torque_right = latest->engine_torque_right;
        sample.telemetry.tail_rudder_left = latest->tail_rudder_left;
        sample.telemetry.tail_rudder_right = latest->tail_rudder_right;
        sample.telemetry.yaw_acceleration_z = latest->yaw_acceleration_z;
        sample.telemetry.pitch = latest->pitch;
        sample.telemetry.bank = latest->bank;
        sample.telemetry.attitude_yaw = latest->attitude_yaw;
        sample.telemetry.velocity_x = latest->velocity_x;
        sample.telemetry.velocity_y = latest->velocity_y;
        sample.telemetry.velocity_z = latest->velocity_z;
        sample.telemetry.speed_3d = latest->speed_3d;
        sample.telemetry.ground_speed = latest->ground_speed;
        sample.telemetry.vertical_velocity = latest->vertical_velocity;
        sample.telemetry.true_airspeed = latest->true_airspeed;
        sample.telemetry.mach = latest->mach;
        sample.telemetry.altitude_msl = latest->altitude_msl;
        sample.telemetry.latitude = latest->latitude;
        sample.telemetry.longitude = latest->longitude;
        sample.telemetry.accel_x = latest->accel_x;
        sample.telemetry.accel_y = latest->accel_y;
        sample.telemetry.accel_z = latest->accel_z;
        sample.telemetry.wind_x = latest->wind_x;
        sample.telemetry.wind_y = latest->wind_y;
        sample.telemetry.wind_z = latest->wind_z;
    }
    sample.fresh = runtime.fast_export.has_recent_frame(runtime.cfg.stale_timeout);
    return sample;
}

std::optional<double> f14_aoa_units(const Telemetry& telemetry, const AppConfig& cfg) {
    if (!telemetry.angle_of_attack) {
        return std::nullopt;
    }
    return cfg.f14_aoa_units_offset + cfg.f14_aoa_units_per_radian * *telemetry.angle_of_attack;
}

int run_f14_roll_assist(CliOptions options, AppConfig cfg) {
    if (cfg.telemetry_source != "fast_export") {
        throw std::runtime_error("f14_roll_assist requires telemetry_source=fast_export");
    }

    F14Runtime runtime(std::move(cfg));
    windows::Hotkey hotkey(runtime.cfg.hotkey);
    std::optional<windows::VJoyDevice> output;
    if (!options.dry_run) {
        output.emplace(runtime.cfg.output_vjoy_id, runtime.cfg.f14_output_rudder_axis_name);
    }

    runtime.logger.info("Selected F-14 roll input: " + runtime.roll_input.selected_name() +
                        " axis " + runtime.cfg.f14_roll_axis_name);
    runtime.logger.info("Selected F-14 rudder input: " + runtime.rudder_input.selected_name() +
                        " axis " + runtime.cfg.f14_rudder_axis_name);
    runtime.logger.info("Telemetry source: fast_export");
    runtime.logger.info(
        options.dry_run
            ? "Running F-14 assist in dry-run mode"
            : "Writing F-14 roll/rudder to vJoy #" + std::to_string(runtime.cfg.output_vjoy_id) +
                  " axes " + runtime.cfg.f14_output_roll_axis_name + "/" + runtime.cfg.f14_output_rudder_axis_name);
    runtime.logger.info("Press " + hotkey.name() + " to toggle assist; Ctrl+C to exit.");

    F14RollAssist assist(runtime.cfg);
    bool assist_enabled = true;
    auto last = std::chrono::steady_clock::now();
    auto next_log = last;
    const auto loop_period = std::chrono::duration<double>(1.0 / static_cast<double>(runtime.cfg.loop_hz));

    while (g_running) {
        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now - last).count();
        last = now;

        if (hotkey.pressed_edge()) {
            assist_enabled = !assist_enabled;
            runtime.logger.info(std::string("F-14 assist ") + (assist_enabled ? "enabled" : "disabled"));
        }

        const auto roll = runtime.roll_input.read();
        const auto rudder = runtime.rudder_input.read();
        const TelemetrySample sample = sample_f14_telemetry(runtime);
        const Telemetry& telemetry = sample.telemetry;
        const auto aoa_units = f14_aoa_units(telemetry, runtime.cfg);

        F14RollAssistInput assist_input;
        assist_input.dt = dt;
        assist_input.physical_roll = roll.value_or(0.0);
        assist_input.physical_rudder = rudder.value_or(0.0);
        assist_input.yaw_rate_z = telemetry.yaw_rate_z;
        assist_input.roll_rate_x = telemetry.roll_rate_x.value_or(0.0);
        assist_input.slip_ball = telemetry.slip_ball.value_or(0.0);
        assist_input.aoa_units = aoa_units.value_or(0.0);
        assist_input.indicated_airspeed = telemetry.indicated_airspeed.value_or(0.0);
        assist_input.radar_altitude = telemetry.radar_altitude.value_or(0.0);
        assist_input.gear_position = telemetry.gear_position.value_or(0.0);
        assist_input.flaps_position = telemetry.flaps_position.value_or(0.0);
        assist_input.roll_rate_valid = telemetry.roll_rate_x.has_value();
        assist_input.slip_valid = telemetry.slip_ball.has_value();
        assist_input.aoa_valid = aoa_units.has_value();
        assist_input.indicated_airspeed_valid = telemetry.indicated_airspeed.has_value();
        assist_input.radar_altitude_valid = telemetry.radar_altitude.has_value();
        assist_input.gear_valid = telemetry.gear_position.has_value();
        assist_input.flaps_valid = telemetry.flaps_position.has_value();
        assist_input.telemetry_fresh = sample.fresh;
        assist_input.aircraft_is_f14 = telemetry.aircraft_is_f14;
        assist_input.roll_input_valid = roll.has_value();
        assist_input.rudder_input_valid = rudder.has_value();
        assist_input.assist_enabled = assist_enabled;
        const auto result = assist.update(assist_input);

        if (output) {
            output->set_axis(runtime.cfg.f14_output_roll_axis_name, result.final_roll);
            output->set_axis(runtime.cfg.f14_output_rudder_axis_name, result.final_rudder);
        }

        if (now >= next_log) {
            std::ostringstream line;
            line << "f14 acft=" << (telemetry.aircraft_name.empty() ? "NONE" : telemetry.aircraft_name)
                 << " fresh=" << (sample.fresh ? "yes" : "no")
                 << " roll=" << fixed3(roll.value_or(0.0))
                 << " rudder=" << fixed3(rudder.value_or(0.0))
                 << " rawAoa=" << (telemetry.angle_of_attack ? fixed3(*telemetry.angle_of_attack) : "NA")
                 << " aoaU=" << (aoa_units ? fixed3(*aoa_units) : "NA")
                 << " state=" << result.flight_mode
                 << " ias=" << (telemetry.indicated_airspeed ? fixed3(*telemetry.indicated_airspeed) : "NA")
                 << " agl=" << (telemetry.radar_altitude ? fixed3(*telemetry.radar_altitude) : "NA")
                 << " gear=" << (telemetry.gear_position ? fixed3(*telemetry.gear_position) : "NA")
                 << " flaps=" << (telemetry.flaps_position ? fixed3(*telemetry.flaps_position) : "NA")
                 << " w=" << fixed3(result.aoa_weight)
                 << " deep=" << fixed3(result.deep_aoa_weight)
                 << " rollScale=" << fixed3(result.roll_scale)
                 << " yawZ=" << fixed3(telemetry.yaw_rate_z)
                 << " angY=" << (telemetry.yaw_rate_y ? fixed3(*telemetry.yaw_rate_y) : "NA")
                 << " p=" << (telemetry.roll_rate_x ? fixed3(*telemetry.roll_rate_x) : "NA")
                 << " mix=" << fixed3(result.rudder_from_roll)
                 << " damp=" << fixed3(result.yaw_damping)
                 << " guard=" << fixed3(result.reversal_guard)
                 << " finalRoll=" << fixed3(result.final_roll)
                 << " finalRudder=" << fixed3(result.final_rudder)
                 << " mode=" << result.reason;
            if (telemetry.slip_ball) {
                line << " slip=" << fixed3(*telemetry.slip_ball);
            }
            runtime.logger.info(line.str());
            next_log = now + std::chrono::milliseconds(250);
        }

        std::this_thread::sleep_until(now + loop_period);
    }

    if (output) {
        output->set_axis(runtime.cfg.f14_output_roll_axis_name, 0.0);
        output->set_axis(runtime.cfg.f14_output_rudder_axis_name, 0.0);
    }
    runtime.logger.info("Stopped F-14 assist");
    return 0;
}

int run_normal(CliOptions options, AppConfig cfg) {
    if (cfg.control_mode == "f14_roll_assist") {
        return run_f14_roll_assist(std::move(options), std::move(cfg));
    }

    Runtime runtime(std::move(cfg));
    windows::Hotkey hotkey(runtime.cfg.hotkey);
    std::optional<windows::VJoyDevice> output;
    if (!options.dry_run) {
        output.emplace(runtime.cfg.output_vjoy_id, runtime.cfg.axis_name);
    }

    runtime.logger.info("Selected pedal input: " + runtime.pedals.selected_name());
    log_ah64_roll_input(runtime.logger, runtime);
    log_collective_input(runtime.logger, runtime);
    log_trim_guard_input(runtime.logger, runtime);
    runtime.logger.info("Telemetry source: " + runtime.cfg.telemetry_source);
    runtime.logger.info("Yaw-rate source: " + runtime.cfg.yaw_rate_source);
    runtime.logger.info(
        "Power feedforward source: " + runtime.cfg.power_feedforward_source +
        " fuel=[" + fixed3(runtime.cfg.fuel_flow_min) + "," + fixed3(runtime.cfg.fuel_flow_max) + "]" +
        " rpm_nominal=" + fixed3(runtime.cfg.rpm_nominal) +
        " rpm_gain=" + fixed3(runtime.cfg.rpm_power_gain) +
        " proxy_slew_up=" + fixed3(runtime.cfg.power_proxy_rise_rate_limit) +
        " proxy_slew_down=" + fixed3(runtime.cfg.power_proxy_fall_rate_limit) +
        " coll_lead_gain=" + fixed3(runtime.cfg.power_collective_lead_gain) +
        " coll_lead_invert=" + fixed3(runtime.cfg.power_collective_lead_invert) +
        " coll_lead_db=" + fixed3(runtime.cfg.power_collective_lead_deadband));
    runtime.logger.info(
        options.dry_run
            ? "Running in dry-run mode"
            : "Writing final rudder/roll to vJoy #" + std::to_string(runtime.cfg.output_vjoy_id));
    runtime.logger.info("Press " + hotkey.name() + " to toggle assist; Ctrl+C to exit.");

    YawDamper damper(runtime.cfg);
    bool assist_enabled = true;
    double ah64_roll_counter = 0.0;
    auto last = std::chrono::steady_clock::now();
    auto next_log = last;
    const auto loop_period = std::chrono::duration<double>(1.0 / static_cast<double>(runtime.cfg.loop_hz));
    enum class TrimGuardState {
        Idle,
        PrePress,
        Pressing,
        PostPress,
    };
    TrimGuardState trim_guard_state = TrimGuardState::Idle;
    double trim_guard_timer = 0.0;
    bool trim_guard_previous_button = false;
    bool trim_guard_output_pressed = false;
    const auto trim_guard_state_name = [&] {
        switch (trim_guard_state) {
        case TrimGuardState::Idle: return "idle";
        case TrimGuardState::PrePress: return "pre";
        case TrimGuardState::Pressing: return "press";
        case TrimGuardState::PostPress: return "post";
        }
        return "idle";
    };
    const auto release_trim_guard_button = [&] {
        if (output && trim_guard_output_pressed && runtime.cfg.trim_guard_output_button > 0) {
            output->set_button(runtime.cfg.trim_guard_output_button, false);
        }
        trim_guard_output_pressed = false;
    };

    while (g_running) {
        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now - last).count();
        last = now;

        if (hotkey.pressed_edge()) {
            assist_enabled = !assist_enabled;
            runtime.logger.info(std::string("Assist ") + (assist_enabled ? "enabled" : "disabled"));
        }

        const auto raw_pedal = runtime.pedals.read();
        const std::optional<double> pedal =
            raw_pedal ? std::optional<double>(apply_rudder_input_calibration(*raw_pedal, runtime.cfg)) : std::nullopt;
        std::optional<double> raw_roll;
        const std::optional<double> roll = read_ah64_roll(runtime, raw_roll);
        const TelemetrySample sample = sample_telemetry(runtime);
        const Telemetry& telemetry = sample.telemetry;
        const auto collective = read_collective(runtime, telemetry);
        const auto power_ff = read_power_feedforward(runtime.cfg, telemetry, collective);
        bool trim_guard_button = false;
        if (runtime.trim_guard_input) {
            trim_guard_button = runtime.trim_guard_input->read().value_or(false);
        }
        const bool trim_guard_edge = trim_guard_button && !trim_guard_previous_button;
        trim_guard_previous_button = trim_guard_button;
        bool trim_guard_press_output_this_frame = false;

        if (runtime.cfg.trim_guard_enabled > 0.5 && runtime.trim_guard_input) {
            if (runtime.cfg.trim_guard_output_button > 0) {
                if (trim_guard_state == TrimGuardState::Idle && trim_guard_edge) {
                    trim_guard_state = TrimGuardState::PrePress;
                    trim_guard_timer = runtime.cfg.trim_guard_pre_time;
                    runtime.logger.info("Trim guard started: suppressing auto rudder before forwarded trim");
                }
                if (trim_guard_state == TrimGuardState::PrePress) {
                    trim_guard_timer -= dt;
                    if (trim_guard_timer <= 0.0) {
                        trim_guard_press_output_this_frame = true;
                        trim_guard_state = TrimGuardState::Pressing;
                        trim_guard_timer = runtime.cfg.trim_guard_press_time;
                    }
                } else if (trim_guard_state == TrimGuardState::Pressing) {
                    trim_guard_timer -= dt;
                    if (trim_guard_timer <= 0.0) {
                        release_trim_guard_button();
                        trim_guard_state = TrimGuardState::PostPress;
                        trim_guard_timer = runtime.cfg.trim_guard_post_time;
                    }
                } else if (trim_guard_state == TrimGuardState::PostPress) {
                    trim_guard_timer -= dt;
                    if (trim_guard_timer <= 0.0) {
                        trim_guard_state = TrimGuardState::Idle;
                    }
                }
            } else {
                if (trim_guard_edge) {
                    trim_guard_state = TrimGuardState::PostPress;
                    trim_guard_timer = runtime.cfg.trim_guard_post_time;
                    runtime.logger.info("Trim guard started: guard-only suppression");
                } else if (trim_guard_state == TrimGuardState::PostPress && !trim_guard_button) {
                    trim_guard_timer -= dt;
                    if (trim_guard_timer <= 0.0) {
                        trim_guard_state = TrimGuardState::Idle;
                    }
                }
            }
        }
        const bool trim_guard_active =
            runtime.cfg.trim_guard_enabled > 0.5 &&
            runtime.trim_guard_input.has_value() &&
            (trim_guard_button || trim_guard_state != TrimGuardState::Idle);

        YawDamperInput damper_input;
        damper_input.dt = dt;
        damper_input.physical_rudder = pedal.value_or(0.0);
        damper_input.yaw_rate_z = control_yaw_rate(telemetry);
        damper_input.yaw_acceleration_z = telemetry.yaw_acceleration_z.value_or(0.0);
        damper_input.heading = telemetry.heading.value_or(0.0);
        damper_input.collective = power_ff.value.value_or(0.0);
        damper_input.heading_valid = telemetry.heading.has_value();
        damper_input.yaw_acceleration_valid = telemetry.yaw_acceleration_z.has_value();
        damper_input.collective_valid = power_ff.value.has_value();
        damper_input.telemetry_fresh = sample.fresh;
        damper_input.aircraft_is_ah64 = telemetry.aircraft_is_ah64;
        damper_input.input_valid = pedal.has_value();
        damper_input.assist_enabled = assist_enabled;
        damper_input.trim_guard_active = trim_guard_active;
        const auto result = damper.update(damper_input);

        double ah64_roll_target = 0.0;
        const bool ah64_roll_counter_allowed =
            runtime.cfg.ah64_roll_enabled > 0.5 &&
            roll.has_value() &&
            sample.fresh &&
            telemetry.aircraft_is_ah64 &&
            assist_enabled &&
            !trim_guard_active &&
            std::abs(*roll) <= runtime.cfg.ah64_roll_override_threshold;
        if (ah64_roll_counter_allowed) {
            const double auto_rudder =
                apply_symmetric_deadband(result.assist_offset, runtime.cfg.ah64_roll_counter_deadband);
            ah64_roll_target = std::clamp(
                runtime.cfg.ah64_roll_counter_sign * runtime.cfg.ah64_roll_counter_gain * auto_rudder,
                -runtime.cfg.ah64_roll_counter_max,
                runtime.cfg.ah64_roll_counter_max);
        }
        const double roll_step =
            runtime.cfg.ah64_roll_counter_fade_time <= 0.0
                ? runtime.cfg.ah64_roll_counter_max
                : std::max(runtime.cfg.ah64_roll_counter_max, 0.001) * dt / runtime.cfg.ah64_roll_counter_fade_time;
        ah64_roll_counter = move_toward(ah64_roll_counter, ah64_roll_target, roll_step);
        const double final_roll = clamp_unit(roll.value_or(0.0) + ah64_roll_counter);

        if (output) {
            output->set_axis(result.final_rudder);
            if (runtime.cfg.ah64_roll_enabled > 0.5) {
                output->set_axis(runtime.cfg.ah64_roll_output_axis_name, final_roll);
            }
            if (trim_guard_press_output_this_frame && runtime.cfg.trim_guard_output_button > 0) {
                output->set_button(runtime.cfg.trim_guard_output_button, true);
                trim_guard_output_pressed = true;
            }
        }

        if (now >= next_log) {
            std::ostringstream line;
            line << "acft=" << (telemetry.aircraft_name.empty() ? "NONE" : telemetry.aircraft_name)
                 << " fresh=" << (sample.fresh ? "yes" : "no")
                 << " rawPedal=" << fixed3(raw_pedal.value_or(0.0))
                 << " pedal=" << fixed3(pedal.value_or(0.0))
                 << " rawRoll=" << (raw_roll ? fixed3(*raw_roll) : "NA")
                 << " roll=" << (roll ? fixed3(*roll) : "NA")
                 << " yawZ=" << fixed3(telemetry.yaw_rate_z)
                 << " angY=" << (telemetry.yaw_rate_y ? fixed3(*telemetry.yaw_rate_y) : "NA")
                 << " p=" << (telemetry.roll_rate_x ? fixed3(*telemetry.roll_rate_x) : "NA")
                 << " yawCtl=" << fixed3(control_yaw_rate(telemetry))
                 << " yawSrc=" << runtime.cfg.yaw_rate_source
                 << " yawAccZ=" << (telemetry.yaw_acceleration_z ? fixed3(*telemetry.yaw_acceleration_z) : "NA")
                 << " pitch=" << maybe3(telemetry.pitch)
                 << " bank=" << maybe3(telemetry.bank)
                 << " attYaw=" << maybe3(telemetry.attitude_yaw)
                 << " vx=" << maybe3(telemetry.velocity_x)
                 << " vy=" << maybe3(telemetry.velocity_y)
                 << " vz=" << maybe3(telemetry.velocity_z)
                 << " spd3=" << maybe3(telemetry.speed_3d)
                 << " gs=" << maybe3(telemetry.ground_speed)
                 << " vs=" << maybe3(telemetry.vertical_velocity)
                 << " ias=" << maybe3(telemetry.indicated_airspeed)
                 << " tas=" << maybe3(telemetry.true_airspeed)
                 << " mach=" << maybe3(telemetry.mach)
                 << " agl=" << maybe3(telemetry.radar_altitude)
                 << " msl=" << maybe3(telemetry.altitude_msl)
                 << " lat=" << maybe6(telemetry.latitude)
                 << " lon=" << maybe6(telemetry.longitude)
                 << " ax=" << maybe3(telemetry.accel_x)
                 << " ay=" << maybe3(telemetry.accel_y)
                 << " az=" << maybe3(telemetry.accel_z)
                 << " windX=" << maybe3(telemetry.wind_x)
                 << " windY=" << maybe3(telemetry.wind_y)
                 << " windZ=" << maybe3(telemetry.wind_z)
                 << " rCmd=" << fixed3(result.yaw_rate_command)
                 << " hRate=" << fixed3(result.heading_rate)
                 << " hdg=" << (telemetry.heading ? fixed3(*telemetry.heading) : "NA")
                 << " href=" << fixed3(result.heading_ref)
                 << " hErr=" << fixed3(result.heading_error)
                 << " coll=" << (collective ? fixed3(*collective) : "NA")
                 << " ffSrc=" << power_ff.source
                 << " ffIn=" << (power_ff.value ? fixed3(*power_ff.value) : "NA")
                 << " ffCtl=" << (power_ff.value ? fixed3(result.collective) : "NA")
                 << " rpm=" << (telemetry.engine_rpm_avg ? fixed3(*telemetry.engine_rpm_avg) : "NA")
                 << " fuel=" << (telemetry.engine_fuel_flow_avg ? fixed3(*telemetry.engine_fuel_flow_avg) : "NA")
                 << " tq=" << (telemetry.engine_torque_avg ? fixed3(*telemetry.engine_torque_avg) : "NA")
                 << " tqL=" << (telemetry.engine_torque_left ? fixed3(*telemetry.engine_torque_left) : "NA")
                 << " tqR=" << (telemetry.engine_torque_right ? fixed3(*telemetry.engine_torque_right) : "NA")
                 << " tailL=" << (telemetry.tail_rudder_left ? fixed3(*telemetry.tail_rudder_left) : "NA")
                 << " tailR=" << (telemetry.tail_rudder_right ? fixed3(*telemetry.tail_rudder_right) : "NA")
                 << " cdot=" << fixed3(result.collective_rate)
                 << " cff=" << fixed3(result.collective_feedforward)
                 << " filt=" << fixed3(result.filtered_yaw_rate)
                 << " aFilt=" << fixed3(result.filtered_yaw_acceleration)
                 << " aAssist=" << fixed3(result.yaw_acceleration_assist)
                 << " assist=" << fixed3(result.assist_offset)
                 << " hold=" << fixed3(result.integral_assist)
                 << " trim=" << fixed3(result.trim_bias)
                 << " final=" << fixed3(result.final_rudder)
                 << " rollCtr=" << fixed3(ah64_roll_counter)
                 << " finalRoll=" << fixed3(final_roll)
                 << " trimGuard=" << (trim_guard_active ? trim_guard_state_name() : "off")
                 << " mode=" << result.reason;
            if (telemetry.slip_ball) {
                line << " slip=" << fixed3(*telemetry.slip_ball);
            }
            runtime.logger.info(line.str());
            next_log = now + std::chrono::milliseconds(250);
        }

        std::this_thread::sleep_until(now + loop_period);
    }

    if (output) {
        release_trim_guard_button();
        output->set_axis(0.0);
        if (runtime.cfg.ah64_roll_enabled > 0.5) {
            output->set_axis(runtime.cfg.ah64_roll_output_axis_name, 0.0);
        }
    }
    runtime.logger.info("Stopped");
    return 0;
}

int run_tune_session(AppConfig cfg, bool auto_apply, bool drive_collective) {
    Logger startup_logger(cfg.log_path);
    startup_logger.info(std::string(auto_apply ? "Auto tune" : "Tune session") + " startup: constructing runtime");
    Runtime runtime(std::move(cfg));
    runtime.logger.info(std::string(auto_apply ? "Auto tune" : "Tune session") + " startup: runtime ready");
    windows::Hotkey hotkey(runtime.cfg.hotkey);
    runtime.logger.info("Opening vJoy #" + std::to_string(runtime.cfg.output_vjoy_id) +
                        " rudder axis " + runtime.cfg.axis_name);
    windows::VJoyDevice output(runtime.cfg.output_vjoy_id, runtime.cfg.axis_name);
    runtime.logger.info("vJoy output ready");

    AppConfig active_cfg = runtime.cfg;
    const bool collective_drive_enabled =
        drive_collective || (auto_apply && active_cfg.auto_tune_collective_drive > 0.5);
    runtime.logger.info(std::string(auto_apply ? "Auto tune" : "Tune session") +
                        " writes final rudder to vJoy #" + std::to_string(runtime.cfg.output_vjoy_id));
    if (collective_drive_enabled) {
        runtime.logger.info("Collective drive writes vJoy #" + std::to_string(runtime.cfg.output_vjoy_id) +
                            " axis " + runtime.cfg.collective_output_axis_name +
                            " as physical collective passthrough plus scripted tune moves.");
    }
    runtime.logger.info("Goal order: tune collective feedforward first, then yaw-rate P damping.");
    runtime.logger.info("Fly centered-pedal hover/low-speed segments. Move collective up/down several times, then hold steady.");
    runtime.logger.info("Pedal turn commands, stale telemetry, non-AH-64D data, and unstable/VRS-like segments are ignored.");
    if (auto_apply) {
        runtime.logger.info("Auto tune applies valid in-memory parameter changes every 10-second window.");
        runtime.logger.info("Auto tune does not edit config.ini; Ctrl+C prints the final active values.");
    }
    runtime.logger.info("Active tune config: " + tune_config_summary(active_cfg));
    log_ah64_roll_input(runtime.logger, runtime);
    log_collective_input(runtime.logger, runtime);
    runtime.logger.info("Yaw-rate source: " + active_cfg.yaw_rate_source);
    runtime.logger.info(
        "Power feedforward source: " + active_cfg.power_feedforward_source +
        " fuel=[" + fixed3(active_cfg.fuel_flow_min) + "," + fixed3(active_cfg.fuel_flow_max) + "]" +
        " rpm_nominal=" + fixed3(active_cfg.rpm_nominal) +
        " rpm_gain=" + fixed3(active_cfg.rpm_power_gain) +
        " proxy_slew_up=" + fixed3(active_cfg.power_proxy_rise_rate_limit) +
        " proxy_slew_down=" + fixed3(active_cfg.power_proxy_fall_rate_limit) +
        " coll_lead_gain=" + fixed3(active_cfg.power_collective_lead_gain) +
        " coll_lead_invert=" + fixed3(active_cfg.power_collective_lead_invert) +
        " coll_lead_db=" + fixed3(active_cfg.power_collective_lead_deadband));
    runtime.logger.info("Press " + hotkey.name() + " to toggle assist; Ctrl+C to finish.");

    YawDamper damper(active_cfg);
    CollectiveDrive collective_drive(make_collective_drive_config(active_cfg));
    TuneSessionAnalyzer total_analyzer(make_tune_config(active_cfg));
    TuneSessionAnalyzer window_analyzer(make_tune_config(active_cfg));
    bool assist_enabled = true;
    auto last = std::chrono::steady_clock::now();
    auto next_log = last;
    auto next_tune_report = last + std::chrono::seconds(10);
    const auto loop_period = std::chrono::duration<double>(1.0 / static_cast<double>(runtime.cfg.loop_hz));

    while (g_running) {
        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now - last).count();
        last = now;

        if (hotkey.pressed_edge()) {
            assist_enabled = !assist_enabled;
            runtime.logger.info(std::string("Assist ") + (assist_enabled ? "enabled" : "disabled"));
        }

        const auto pedal = runtime.pedals.read();
        std::optional<double> raw_roll;
        const auto roll = read_ah64_roll(runtime, raw_roll);
        const TelemetrySample sample = sample_telemetry(runtime);
        const Telemetry& telemetry = sample.telemetry;
        const auto physical_collective = read_collective(runtime, telemetry);
        std::optional<double> collective = physical_collective;
        CollectiveDriveOutput drive_output;
        bool drive_output_valid = false;

        if (collective_drive_enabled && physical_collective) {
            drive_output = collective_drive.update(dt, *physical_collective, true);
            drive_output_valid = true;
            collective = drive_output.collective;
            output.set_axis(
                active_cfg.collective_output_axis_name,
                collective_to_output_axis(drive_output.collective, active_cfg));
        }
        const auto power_ff = read_power_feedforward(active_cfg, telemetry, collective);

        YawDamperInput damper_input;
        damper_input.dt = dt;
        damper_input.physical_rudder = pedal.value_or(0.0);
        damper_input.yaw_rate_z = control_yaw_rate(telemetry);
        damper_input.yaw_acceleration_z = telemetry.yaw_acceleration_z.value_or(0.0);
        damper_input.heading = telemetry.heading.value_or(0.0);
        damper_input.collective = power_ff.value.value_or(0.0);
        const bool tune_rate_hold = collective_drive_enabled;
        damper_input.heading_valid = telemetry.heading.has_value() && !tune_rate_hold;
        damper_input.yaw_acceleration_valid = telemetry.yaw_acceleration_z.has_value();
        damper_input.collective_valid = power_ff.value.has_value();
        damper_input.telemetry_fresh = sample.fresh;
        damper_input.aircraft_is_ah64 = telemetry.aircraft_is_ah64;
        damper_input.input_valid = pedal.has_value();
        damper_input.assist_enabled = assist_enabled;
        const auto result = damper.update(damper_input);
        output.set_axis(result.final_rudder);
        if (active_cfg.ah64_roll_enabled > 0.5) {
            output.set_axis(active_cfg.ah64_roll_output_axis_name, roll.value_or(0.0));
        }

        const bool collective_drive_active = drive_output_valid && drive_output.driving;
        add_tune_sample(window_analyzer, dt, pedal, sample, power_ff.value, result, collective_drive_active);
        add_tune_sample(total_analyzer, dt, pedal, sample, power_ff.value, result, collective_drive_active);

        if (now >= next_log) {
            std::ostringstream line;
            line << "tune acft=" << (telemetry.aircraft_name.empty() ? "NONE" : telemetry.aircraft_name)
                 << " fresh=" << (sample.fresh ? "yes" : "no")
                 << " pedal=" << fixed3(pedal.value_or(0.0))
                 << " rawRoll=" << (raw_roll ? fixed3(*raw_roll) : "NA")
                 << " roll=" << (roll ? fixed3(*roll) : "NA")
                 << " yawZ=" << fixed3(telemetry.yaw_rate_z)
                 << " angY=" << (telemetry.yaw_rate_y ? fixed3(*telemetry.yaw_rate_y) : "NA")
                 << " p=" << (telemetry.roll_rate_x ? fixed3(*telemetry.roll_rate_x) : "NA")
                 << " yawCtl=" << fixed3(control_yaw_rate(telemetry))
                 << " yawSrc=" << active_cfg.yaw_rate_source
                 << " yawAccZ=" << (telemetry.yaw_acceleration_z ? fixed3(*telemetry.yaw_acceleration_z) : "NA")
                 << " pitch=" << maybe3(telemetry.pitch)
                 << " bank=" << maybe3(telemetry.bank)
                 << " attYaw=" << maybe3(telemetry.attitude_yaw)
                 << " vx=" << maybe3(telemetry.velocity_x)
                 << " vy=" << maybe3(telemetry.velocity_y)
                 << " vz=" << maybe3(telemetry.velocity_z)
                 << " spd3=" << maybe3(telemetry.speed_3d)
                 << " gs=" << maybe3(telemetry.ground_speed)
                 << " vs=" << maybe3(telemetry.vertical_velocity)
                 << " ias=" << maybe3(telemetry.indicated_airspeed)
                 << " tas=" << maybe3(telemetry.true_airspeed)
                 << " mach=" << maybe3(telemetry.mach)
                 << " agl=" << maybe3(telemetry.radar_altitude)
                 << " msl=" << maybe3(telemetry.altitude_msl)
                 << " lat=" << maybe6(telemetry.latitude)
                 << " lon=" << maybe6(telemetry.longitude)
                 << " ax=" << maybe3(telemetry.accel_x)
                 << " ay=" << maybe3(telemetry.accel_y)
                 << " az=" << maybe3(telemetry.accel_z)
                 << " windX=" << maybe3(telemetry.wind_x)
                 << " windY=" << maybe3(telemetry.wind_y)
                 << " windZ=" << maybe3(telemetry.wind_z)
                 << " hRate=" << fixed3(result.heading_rate)
                 << " hErr=" << fixed3(result.heading_error)
                 << " coll=" << (collective ? fixed3(*collective) : "NA")
                 << " ffSrc=" << power_ff.source
                 << " ffIn=" << (power_ff.value ? fixed3(*power_ff.value) : "NA")
                 << " ffCtl=" << (power_ff.value ? fixed3(result.collective) : "NA")
                 << " rpm=" << (telemetry.engine_rpm_avg ? fixed3(*telemetry.engine_rpm_avg) : "NA")
                 << " fuel=" << (telemetry.engine_fuel_flow_avg ? fixed3(*telemetry.engine_fuel_flow_avg) : "NA")
                 << " tq=" << (telemetry.engine_torque_avg ? fixed3(*telemetry.engine_torque_avg) : "NA")
                 << " tqL=" << (telemetry.engine_torque_left ? fixed3(*telemetry.engine_torque_left) : "NA")
                 << " tqR=" << (telemetry.engine_torque_right ? fixed3(*telemetry.engine_torque_right) : "NA")
                 << " tailL=" << (telemetry.tail_rudder_left ? fixed3(*telemetry.tail_rudder_left) : "NA")
                 << " tailR=" << (telemetry.tail_rudder_right ? fixed3(*telemetry.tail_rudder_right) : "NA");
            if (collective_drive_enabled) {
                line << " physColl=" << (physical_collective ? fixed3(*physical_collective) : "NA")
                     << " cDrive=" << (drive_output_valid ? fixed3(drive_output.offset) : "NA")
                     << " cDriveOn=" << (drive_output_valid && drive_output.driving ? "yes" : "no");
            }
            line
                 << " cdot=" << fixed3(result.collective_rate)
                 << " cff=" << fixed3(result.collective_feedforward)
                 << " aAssist=" << fixed3(result.yaw_acceleration_assist)
                 << " final=" << fixed3(result.final_rudder)
                 << " finalRoll=" << fixed3(roll.value_or(0.0))
                 << " mode=" << result.reason;
            runtime.logger.info(line.str());
            next_log = now + std::chrono::milliseconds(250);
        }

        if (now >= next_tune_report) {
            const TuneReport report = window_analyzer.report();
            log_tune_report(runtime.logger, auto_apply ? "Auto tune window" : "Tune window", report);
            if (auto_apply &&
                report.usable_seconds >= 5.0 &&
                report.static_collective_seconds <= 0.0 &&
                report.collective_transient_seconds <= 0.0) {
                runtime.logger.warn(
                    "Auto tune has no collective input; collective feedforward cannot tune. "
                    "Run --list-devices while moving the collective, then set collective_device_name_contains and collective_axis_name.");
            }
            if (auto_apply) {
                const TuneUpdate update = choose_tune_update(make_tune_config(active_cfg), report);
                if (update.changed) {
                    apply_tune_config(active_cfg, update.config);
                    runtime.cfg = active_cfg;
                    damper.set_config(active_cfg);
                    runtime.logger.info("Auto tune applied: " + update.message);
                    runtime.logger.info("Active tune config: " + tune_config_summary(active_cfg));
                } else {
                    runtime.logger.info("Auto tune held parameters: " + update.message);
                }
            }
            window_analyzer = TuneSessionAnalyzer(make_tune_config(active_cfg));
            next_tune_report = now + std::chrono::seconds(10);
        }

        std::this_thread::sleep_until(now + loop_period);
    }

    if (collective_drive_enabled) {
        const TelemetrySample sample = sample_telemetry(runtime);
        const auto physical_collective = read_collective(runtime, sample.telemetry);
        if (physical_collective) {
            output.set_axis(
                active_cfg.collective_output_axis_name,
                collective_to_output_axis(*physical_collective, active_cfg));
        }
    }
    output.set_axis(0.0);
    if (active_cfg.ah64_roll_enabled > 0.5) {
        output.set_axis(active_cfg.ah64_roll_output_axis_name, 0.0);
    }
    log_tune_report(runtime.logger, auto_apply ? "Auto tune final" : "Tune final", total_analyzer.report());
    runtime.logger.info("Final active tune config: " + tune_config_summary(active_cfg));
    runtime.logger.info(std::string("Stopped ") + (auto_apply ? "auto tune" : "tune session"));
    return 0;
}

double run_calibration_phase(Runtime& runtime, windows::VJoyDevice& output, double sign, double seconds) {
    AppConfig cfg = runtime.cfg;
    cfg.assist_sign = sign;
    cfg.yaw_response_sign = sign;
    cfg.max_assist = cfg.calibration_max_assist;
    cfg.fade_in_time = 0.30;
    cfg.fade_out_time = 0.10;
    YawDamper damper(cfg);

    runtime.logger.info("Calibration phase assist_sign=" + fixed3(sign));
    const auto started = std::chrono::steady_clock::now();
    auto last = started;
    double sum_abs_yaw = 0.0;
    int samples = 0;
    const auto loop_period = std::chrono::duration<double>(1.0 / static_cast<double>(cfg.loop_hz));

    while (g_running) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - started).count();
        if (elapsed >= seconds) {
            break;
        }
        const double dt = std::chrono::duration<double>(now - last).count();
        last = now;

        const auto pedal = runtime.pedals.read();
        const TelemetrySample sample = sample_telemetry(runtime);
        const Telemetry& telemetry = sample.telemetry;
        const auto collective = read_collective(runtime, telemetry);
        const auto power_ff = read_power_feedforward(cfg, telemetry, collective);
        YawDamperInput damper_input;
        damper_input.dt = dt;
        damper_input.physical_rudder = pedal.value_or(0.0);
        damper_input.yaw_rate_z = control_yaw_rate(telemetry);
        damper_input.heading = telemetry.heading.value_or(0.0);
        damper_input.collective = power_ff.value.value_or(0.0);
        damper_input.heading_valid = telemetry.heading.has_value();
        damper_input.collective_valid = power_ff.value.has_value();
        damper_input.telemetry_fresh = sample.fresh;
        damper_input.aircraft_is_ah64 = telemetry.aircraft_is_ah64;
        damper_input.input_valid = pedal.has_value();
        damper_input.assist_enabled = true;
        const auto result = damper.update(damper_input);
        output.set_axis(result.final_rudder);

        if (elapsed > 2.0 && sample.fresh && telemetry.aircraft_is_ah64 && !result.user_override) {
            sum_abs_yaw += std::abs(control_yaw_rate(telemetry));
            ++samples;
        }
        std::this_thread::sleep_until(now + loop_period);
    }

    output.set_axis(0.0);
    if (samples == 0) {
        return std::numeric_limits<double>::infinity();
    }
    return sum_abs_yaw / static_cast<double>(samples);
}

int run_power_probe(AppConfig cfg) {
    Logger logger(cfg.log_path);
    windows::FastExportUdpClient fast_export(cfg.fast_export_bind_address, cfg.fast_export_port);
    logger.info("Power probe listening on " + cfg.fast_export_bind_address + ":" + std::to_string(cfg.fast_export_port));
    logger.info("This mode does not read controls or write vJoy. Start an AH-64D mission and watch for ARP engine/heli fields.");
    logger.info("Useful candidates are fields that move with torque/power/RPM changes before yaw starts. Ctrl+C to exit.");

    while (g_running) {
        fast_export.pump();
        for (const auto& value : fast_export.drain_probe_values()) {
            std::ostringstream line;
            line << "probe acft=" << (value.aircraft_name.empty() ? "NONE" : value.aircraft_name)
                 << " source=" << value.source
                 << " key=" << value.key
                 << " value=" << std::fixed << std::setprecision(6) << value.value;
            logger.info(line.str());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    logger.info("Stopped power probe");
    return 0;
}

int run_calibration(AppConfig cfg) {
    Runtime runtime(std::move(cfg));
    windows::VJoyDevice output(runtime.cfg.output_vjoy_id, runtime.cfg.axis_name);
    runtime.logger.info("Calibration writes low-authority rudder to vJoy #" + std::to_string(runtime.cfg.output_vjoy_id));
    runtime.logger.info("Keep pedals centered. Move pedals to force override, Ctrl+C to abort.");

    const double score_positive = run_calibration_phase(runtime, output, 1.0, 8.0);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    const double score_negative = run_calibration_phase(runtime, output, -1.0, 8.0);

    output.set_axis(0.0);
    runtime.logger.info("Calibration score sign=+1 mean_abs_yaw=" + fixed3(score_positive));
    runtime.logger.info("Calibration score sign=-1 mean_abs_yaw=" + fixed3(score_negative));
    if (std::isfinite(score_positive) && std::isfinite(score_negative)) {
        const double recommended = score_positive <= score_negative ? 1.0 : -1.0;
        runtime.logger.info("Recommended config: assist_sign=" + fixed3(recommended));
        runtime.logger.info("Recommended config: yaw_response_sign=" + fixed3(recommended));
    } else {
        runtime.logger.warn("Calibration did not collect enough AH-64D fresh telemetry samples");
    }
    return 0;
}

int run_test_output(AppConfig cfg) {
    Logger logger(cfg.log_path);
    const bool f14_mode = cfg.control_mode == "f14_roll_assist";
    const bool ah64_roll_mode = !f14_mode && cfg.ah64_roll_enabled > 0.5;
    const std::string primary_axis = f14_mode ? cfg.f14_output_rudder_axis_name : cfg.axis_name;
    windows::VJoyDevice output(cfg.output_vjoy_id, primary_axis);
    if (f14_mode) {
        logger.info("Sweeping vJoy #" + std::to_string(cfg.output_vjoy_id) +
                    " F-14 axes roll=" + cfg.f14_output_roll_axis_name +
                    " rudder=" + cfg.f14_output_rudder_axis_name + " for diagnostics");
    } else if (ah64_roll_mode) {
        logger.info("Sweeping vJoy #" + std::to_string(cfg.output_vjoy_id) +
                    " AH-64D axes rudder=" + cfg.axis_name +
                    " roll=" + cfg.ah64_roll_output_axis_name + " for diagnostics");
    } else {
        logger.info("Sweeping vJoy #" + std::to_string(cfg.output_vjoy_id) + " axis " + cfg.axis_name + " for diagnostics");
    }
    logger.info("Open DCS axis controls or RCtrl+Enter. Stop with Ctrl+C.");

    const auto started = std::chrono::steady_clock::now();
    auto next_log = started;
    constexpr double pi = 3.14159265358979323846;
    while (g_running) {
        const auto now = std::chrono::steady_clock::now();
        const double t = std::chrono::duration<double>(now - started).count();
        const double value = 0.80 * std::sin(2.0 * pi * 0.20 * t);
        if (f14_mode) {
            output.set_axis(cfg.f14_output_roll_axis_name, value);
            output.set_axis(cfg.f14_output_rudder_axis_name, -value);
        } else if (ah64_roll_mode) {
            output.set_axis(value);
            output.set_axis(cfg.ah64_roll_output_axis_name, -value);
        } else {
            output.set_axis(value);
        }

        if (now >= next_log) {
            if (f14_mode) {
                logger.info("test_output roll=" + fixed3(value) + " rudder=" + fixed3(-value));
            } else if (ah64_roll_mode) {
                logger.info("test_output rudder=" + fixed3(value) + " roll=" + fixed3(-value));
            } else {
                logger.info("test_output final=" + fixed3(value));
            }
            next_log = now + std::chrono::milliseconds(500);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (f14_mode) {
        output.set_axis(cfg.f14_output_roll_axis_name, 0.0);
        output.set_axis(cfg.f14_output_rudder_axis_name, 0.0);
    } else if (ah64_roll_mode) {
        output.set_axis(0.0);
        output.set_axis(cfg.ah64_roll_output_axis_name, 0.0);
    } else {
        output.set_axis(0.0);
    }
    logger.info("Stopped test output");
    return 0;
}

int run_hold_output(AppConfig cfg, double value) {
    Logger logger(cfg.log_path);
    const bool f14_mode = cfg.control_mode == "f14_roll_assist";
    const std::string primary_axis = f14_mode ? cfg.f14_output_rudder_axis_name : cfg.axis_name;
    windows::VJoyDevice output(cfg.output_vjoy_id, primary_axis);
    if (f14_mode) {
        output.set_axis(cfg.f14_output_roll_axis_name, 0.0);
        output.set_axis(cfg.f14_output_rudder_axis_name, value);
        logger.info("Holding vJoy #" + std::to_string(cfg.output_vjoy_id) +
                    " F-14 rudder axis " + cfg.f14_output_rudder_axis_name +
                    " at " + fixed3(value));
    } else {
        output.set_axis(value);
        logger.info("Holding vJoy #" + std::to_string(cfg.output_vjoy_id) +
                    " axis " + cfg.axis_name + " at " + fixed3(value));
    }
    logger.info("Open DCS RCtrl+Enter or axis controls. Stop with Ctrl+C.");

    while (g_running) {
        if (f14_mode) {
            output.set_axis(cfg.f14_output_rudder_axis_name, value);
        } else {
            output.set_axis(value);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (f14_mode) {
        output.set_axis(cfg.f14_output_roll_axis_name, 0.0);
        output.set_axis(cfg.f14_output_rudder_axis_name, 0.0);
    } else {
        output.set_axis(0.0);
    }
    logger.info("Stopped hold output");
    return 0;
}

int run_reset_output(AppConfig cfg) {
    Runtime runtime(std::move(cfg));
    runtime.logger.info("Resetting vJoy #" + std::to_string(runtime.cfg.output_vjoy_id) +
                        " rudder axis " + runtime.cfg.axis_name +
                        " and collective axis " + runtime.cfg.collective_output_axis_name);
    windows::VJoyDevice output(runtime.cfg.output_vjoy_id, runtime.cfg.axis_name);

    const TelemetrySample sample = sample_telemetry(runtime);
    const auto physical_collective = read_collective(runtime, sample.telemetry);
    if (physical_collective) {
        output.set_axis(
            runtime.cfg.collective_output_axis_name,
            collective_to_output_axis(*physical_collective, runtime.cfg));
        runtime.logger.info("Collective output set from physical collective " + fixed3(*physical_collective));
    } else {
        runtime.logger.warn("Collective input unavailable; collective output was not changed");
    }

    output.set_axis(0.0);
    runtime.logger.info("Rudder output centered");
    return 0;
}

}  // namespace

int run_app(const std::vector<std::string>& args) {
    bool used_gui_menu = false;
    try {
        SetConsoleCtrlHandler(console_handler, TRUE);
        CliOptions options = parse_args(args);
        if (options.help) {
            print_help();
            return 0;
        }
        if (options.menu
#ifndef AUTORUDDER_DISABLE_GUI_MENU
            || args.empty()
#endif
        ) {
            used_gui_menu = true;
            return show_retro_menu(options);
        }
        if (options.list_devices) {
            print_device_lists();
            return 0;
        }

        const std::filesystem::path config_path = resolve_config_path(options);
        if (!std::filesystem::exists(config_path)) {
            write_default_config(config_path);
            std::cout << "Wrote default config: " << config_path << '\n';
        }
        AppConfig cfg = load_config(config_path);
        apply_profile(cfg, options.profile);
        if (cfg.control_mode == "f14_roll_assist" &&
            (options.calibrate_sign || options.tune_session || options.auto_tune)) {
            throw std::runtime_error("--calibrate-sign, --tune-session, and --auto-tune only support AH-64D modes");
        }
        if (options.drive_collective && !options.tune_session && !options.auto_tune) {
            throw std::runtime_error("--drive-collective requires --tune-session or --auto-tune");
        }

        if (options.probe_power) {
            return run_power_probe(std::move(cfg));
        }
        if (options.test_output) {
            return run_test_output(std::move(cfg));
        }
        if (options.hold_output) {
            return run_hold_output(std::move(cfg), *options.hold_output);
        }
        if (options.reset_output) {
            return run_reset_output(std::move(cfg));
        }
        if (options.calibrate_sign) {
            return run_calibration(std::move(cfg));
        }
        if (options.tune_session || options.auto_tune) {
            return run_tune_session(std::move(cfg), options.auto_tune, options.drive_collective);
        }
        return run_normal(options, std::move(cfg));
    } catch (const std::exception& ex) {
        if (used_gui_menu) {
            MessageBoxA(nullptr, ex.what(), "Auto Rudder Error", MB_ICONERROR | MB_OK);
        } else {
            std::cerr << "ERROR: " << ex.what() << '\n';
        }
        return 1;
    }
}

}  // namespace autorudder
