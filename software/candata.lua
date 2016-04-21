--[[ CAN data structure
  0 1 2 3 |  4   |  5  | 6 7 8 9 10 11 12 13 14 | 15 16 17 18 | 19 20 21 22  | ...  32  |
     ID   | type | len |         data           | timestamp s | timestamp us | reserved |
          76543210
                 0 - extend
                 1 - remote
--]]

require("util")
class "CanData"(QObject)
function CanData:__init(id, data, extend, remote , length, tsHigh, tsLow)
    QObject.__init(self)
    self.id = id or 0
    self.data = data or {}
    self.remote = remote or false
    self.extend = extend or false
    self.len = length or #self.data
    local tm = QDateTime.currentDateTime()
    self.tsHigh = tsHigh or tm:toTime_t()
    self.tsLow = tsLow or tm.time.msec
    self.ts_s = 0
    self.ts_ms = 0
    self.ts_bs = 0
end

function CanData:toData(d)
   d = d or {}
   local t1 = self.extend and 1 or 0
   local t2 = self.remote and 2 or 0
   d = QUtil.fromUint32(d, self.id)
   d = QUtil.fromUint8(d, t1+t2)
   d = QUtil.fromUint8(d, self.len)
   for i=1,8 do
    d[#d+1] = self.data[i] or 0
   end
   d = QUtil.fromUint32(d, self.ts_s)
   d = QUtil.fromUint16(d, self.ts_ms)
   d = QUtil.fromUint16(d, self.ts_bs)
   
   --d = QUtil.fromUint32(d, self.tsHigh)
   --d = QUtil.fromUint32(d, self.tsLow)
   
   return d
end

function CanData:fromData(d ,offset)
   offset = offset or 1
   self.id = QUtil.toUint32(d, offset)
   local t = QUtil.toUint8(d, offset + 4)
   self.extend = bitand(1,t) == 1
   self.remote = bitand(2,t) == 2
   self.len = QUtil.toUint8(d, offset + 5)
   for i=1,8 do
       self.data[i] = d[offset+5+i] or 0
   end
   self.ts_s = QUtil.toUint32(d, offset + 14)
   self.ts_ms = QUtil.toUint16(d, offset + 18)
   self.ts_bs = QUtil.toUint16(d, offset + 20)
   
   --self.tsHigh = QUtil.toUint32(d, offset + 14)
   --self.tsLow = QUtil.toUint32(d, offset + 18)
end

function CanData:__tostring()
    local d = {}
    for i=1,self.len do  d[i] = self.data[i] or 0 end
    return string.format("ID%s: 0x%X ", (self.extend and "x" or "s"), self.id)
    .. (self.remote and "Remote"  or "Data:" .. QUtil.showBytes(d)) .. " Len:" .. self.len
    .. " timestamp: " .. self.ts_s .. ":" .. self.ts_ms .. ":" .. self.ts_bs
end
--[[
local x = CanData(0x312,{1,2,3},true, false, 6)
local y = CanData()
log(x)
local dx = x:toData({1,2,3})
log(QUtil.showBytes(dx))
y:fromData(dx,4)
log(y)
--]]