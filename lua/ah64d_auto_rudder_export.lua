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

local function find_collective()
    local fm = LoGetHelicopterFMData and LoGetHelicopterFMData() or nil
    if not fm then
        return nil
    end

    local direct_keys = {
        "collective",
        "Collective",
        "collective_position",
        "collectivePosition",
        "mainRotorCollective",
        "main_rotor_collective",
    }
    for _, key in ipairs(direct_keys) do
        if type(fm[key]) == "number" then
            return fm[key]
        end
    end

    for key, value in pairs(fm) do
        if type(value) == "number" and string.find(string.lower(tostring(key)), "collect") then
            return value
        end
    end
    return nil
end

local function find_heading(self_data)
    if self_data and type(self_data.Heading) == "number" then
        return self_data.Heading
    end

    if LoGetADIPitchBankYaw then
        local _, _, yaw = LoGetADIPitchBankYaw()
        if type(yaw) == "number" then
            return yaw
        end
    end

    local fm = LoGetHelicopterFMData and LoGetHelicopterFMData() or nil
    if fm then
        local keys = { "yaw", "Yaw", "heading", "Heading" }
        for _, key in ipairs(keys) do
            if type(fm[key]) == "number" then
                return fm[key]
            end
        end
    end
    return nil
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
    local collective = find_collective()
    local heading = find_heading(self_data)
    local collective_text = ""
    local heading_text = ""
    if collective ~= nil then
        if collective < 0 then collective = 0 end
        if collective > 1 then collective = 1 end
        collective_text = string.format("%.8f", collective)
    end
    if heading ~= nil then
        heading_text = string.format("%.8f", heading)
    end

    AutoRudderExport.udp:send(string.format("AR1,%.3f,%s,%.8f,%.8f,%s,%s\n", now, aircraft, yaw_z, slip, collective_text, heading_text))
end

local previous_after_next_frame = LuaExportAfterNextFrame
LuaExportAfterNextFrame = function()
    if previous_after_next_frame then
        previous_after_next_frame()
    end
    pcall(export_frame)
end
