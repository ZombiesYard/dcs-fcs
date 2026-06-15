-- Optional high-rate telemetry export for ah64d_auto_rudder.
-- Install by adding this line after DCS-BIOS in Saved Games\DCS\Scripts\Export.lua:
-- dofile(lfs.writedir() .. [[Scripts\AH64DAutoRudder\ah64d_auto_rudder_export.lua]])

local AutoRudderExport = AutoRudderExport or {}

AutoRudderExport.host = "127.0.0.1"
AutoRudderExport.port = 34380
AutoRudderExport.rate_hz = 120
AutoRudderExport.last_time = 0
AutoRudderExport.socket = nil
AutoRudderExport.udp = nil

local function ensure_socket()
    if AutoRudderExport.udp then
        return true
    end

    local ok, socket = pcall(require, "socket")
    if not ok or not socket then
        return false
    end

    local udp = socket.udp()
    if not udp then
        return false
    end

    udp:settimeout(0)
    udp:setpeername(AutoRudderExport.host, AutoRudderExport.port)
    AutoRudderExport.socket = socket
    AutoRudderExport.udp = udp
    return true
end

local function safe_number(value)
    if value == nil then
        return 0
    end
    return value
end

local function export_frame()
    if not ensure_socket() then
        return
    end

    if LoIsOwnshipExportAllowed and not LoIsOwnshipExportAllowed() then
        return
    end

    local now = LoGetModelTime and LoGetModelTime() or 0
    if now - AutoRudderExport.last_time < (1 / AutoRudderExport.rate_hz) then
        return
    end
    AutoRudderExport.last_time = now

    local self_data = LoGetSelfData and LoGetSelfData() or nil
    local aircraft = "NONE"
    if self_data and self_data.Name then
        aircraft = tostring(self_data.Name)
    end

    local angular = LoGetAngularVelocity and LoGetAngularVelocity() or nil
    local yaw_z = angular and safe_number(angular.z) or 0
    local slip = LoGetSlipBallPosition and safe_number(LoGetSlipBallPosition()) or 0

    AutoRudderExport.udp:send(string.format("AR1,%.3f,%s,%.8f,%.8f\n", now, aircraft, yaw_z, slip))
end

local previous_after_next_frame = LuaExportAfterNextFrame
LuaExportAfterNextFrame = function()
    if previous_after_next_frame then
        previous_after_next_frame()
    end
    pcall(export_frame)
end
