-- CAN Baudrate configure tool 

function get_value(v, t)
    if type(v) == "number" then return v end
        local factor_table = t or {
            k = 1000, K = 1000,
            m = 1000*1000, M = 1000*1000,
            g = 1000*1000*1000, G = 1000*1000*1000,
        }
        local r = 0
        string.gsub(v,"([%d.]+)([kKMmGg]?)", function(a,b)
        local factor = factor_table[b] or 1
        r = factor * a
        end)
    return r
end

function find_divide_down(a,b)
    local x,y = math.modf(a/b)
    while y ~= 0 and b > 1 do
        b = b - 1
        x,y = math.modf(a/b)
    end
    if b == 1 then return a end
    return x
end

function find_divide_up(a,b)
    local x,y = math.modf(a/b)
    while y ~= 0 and b < a do
        b = b + 1
        x,y = math.modf(a/b)
    end
    if b >= a then return 1 end
    return x
end

function calc_freq(sysfreq, baudrate, sample_point, tq_num, bs1max, bs2max)
    local pres, bs1, bs2
    sysfreq = get_value(sysfreq)
    baudrate = get_value(baudrate)
    sample_point = sample_point or 14
    tq_num = tq_num or 16
    bs1max = bs1max or 32
    bs2max = bs2max or 16
    local samp = sample_point/tq_num
    local a,b = math.modf(sysfreq/baudrate)
    if b ~= 0 then
        log("SysFreq can not divide by baudrate, round up")
    end
    --log(a, b)
    pres, b = math.modf(a/tq_num)
    --log(pres,b,a)
    if b~= 0 then
        -- select a nearest one
        local t1 = find_divide_down(a, pres)
        local t2 = find_divide_up(a, pres)
        local p1 = t1 * samp + 0.5
        local p2 = t2 * samp + 0.5
        p1 = math.floor(p1)
        p2 = math.floor(p2)
        if samp - p1/t1 > p2/t2 - samp then
            pres = pres + 1
            bs1 = p2 - 1
            tq_num = t2
        else
            bs1 = p1 - 1
            tq_num = t1
        end
        if p1/t1>samp then bs1 = bs1 - 1 end
            --log(p1/t1,p2/t2,samp)
    else
        bs1 = tq_num*samp - 1
    end
    bs2 = tq_num - 1 - bs1
    local e1 = (baudrate - (sysfreq/pres/(bs1+bs2+1)))/baudrate
    local e2 = ( (bs1+1)/(bs1+1+bs2) - samp) / samp
    return pres, bs1, bs2, e1, e2
end

class "CanBaudView"(QFrame)

function CanBaudView:__init()
    QFrame.__init(self)
    self.sysFreq = QComboBox(){ {"16M", "36M", "72M"}}
    self.baudRate = QComboBox(){ {"1M", "800K", "500K", "250K", "125K", "50K", "20K", "10K"}}
    self.mode = QComboBox(){ {"Normal", "Silent", "Loopback", "Si-Loop"}}
    self.sysFreq.editable = true
    self.pres = QLineEdit("1"){ inputMask = "DDDD"}
    self.bs1 = QLineEdit("13"){ inputMask = "DD"}
    self.bs2 = QLineEdit("2"){ inputMask = "DD"}
    self.sjw = QLineEdit("1"){ inputMask = "DD"}
    
    self.resistor = QCheckBox("Resistor"){ checked = true }
    self.status = QLineEdit("Status:"){ readonly = true }
    self.btnSet = QPushButton("Set")
    
    self.layout = QVBoxLayout{
        QHBoxLayout{
            QLabel("SysFreq:"), self.sysFreq,
            QLabel("Baudrate:"),self.baudRate,
            QLabel("Prescaler:"),self.pres,
            QLabel("BS1:"), self.bs1,
            QLabel("BS2:"), self.bs2,
            QLabel("SJW:"), self.sjw,
        },
        QHBoxLayout{
            QLabel("Mode:"), self.mode,
            self.resistor,
            self.status,
            self.btnSet,
        },
    }
    self.sysFreq.editTextChanged = function(s)
        local pres,bs1,bs2,e1,e2 = calc_freq(self.sysFreq.currentText, self.baudRate.currentText)
        self.pres.text = pres .. ""
        self.bs1.text = bs1 .. ""
        self.bs2.text = bs2 .. ""
        self.status.text = string.format("Tq count %d, Baudrate Err:%.2f%%, Sample Pt Err:%.2f%%", bs1+bs2+1,e1,e2)
    end
    self.baudRate.currentIndexChanged = function(s)
        local pres,bs1,bs2,e1,e2 = calc_freq(self.sysFreq.currentText, self.baudRate.currentText)
        self.pres.text = pres .. ""
        self.bs1.text = bs1 .. ""
        self.bs2.text = bs2 .. ""
        self.status.text = string.format("Tq count %d, Baudrate Err:%.2f%%, Sample Pt Err:%.2f%%", bs1+bs2+1,e1,e2)
    end
    
    self.btnSet.clicked = function()
        if self.setData then
            local baud = get_value(self.baudRate.currentText)
            local pres = tonumber(self.pres.text)
            local bs1 = tonumber(self.bs1.text)
            local bs2 = tonumber(self.bs2.text)
            local sjw = tonumber(self.sjw.text)
            self:setData(baud, pres, bs1, bs2, sjw, self.mode.currentIndex, self.resistor.checked)
        end
    end
end
--mdiArea:addSubWindow(CanBaudView()):show()
--[[
local pres,bs1,bs2,e1,e2 = calc_freq("36M","800K")
local tq_num = bs1+bs2+1
log(string.format("Pres = %d, BS1 = %d, BS2 = %d, Ebaud = %.3f%%, Esmpl = %.3f%%", pres, bs1, bs2, e1*100, e2*100))
log("Actual baudrate = ".. (get_value("36M")/pres/tq_num))
log(string.format("Sample point = %.3f(%.3f)",(1-bs2/tq_num),14/16))
log("Tq count = " .. tq_num)
--]]
